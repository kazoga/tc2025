#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Route manager Phase2 implementation.

The node sits between ``route_planner`` and ``route_follower`` and decides how to
recover from stagnation reports.  The logic follows the Phase2 design:

* Maintain four observable states (idle/running/updating_route/holding).
* Handle ``/report_stuck`` requests with three layered decisions
  (lateral offset replan → short-range skip → full replan or hold).
* Publish ``/manager_status`` alongside ``/active_route`` updates so that the
  surrounding nodes can keep a consistent view of the scenario.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image

from route_msgs.msg import (
    FollowerState,
    ManagerStatus,
    MissionInfo,
    Route,
    RouteState,
    Waypoint,
)
from route_msgs.srv import GetRoute, ReportStuck, UpdateRoute


def _qos_transient_local() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _qos_volatile(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _distance_xy(a: Pose, b: Pose) -> float:
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


@dataclass
class VersionMM:
    major: int = 0
    minor: int = 0

    def to_int(self) -> int:
        return int(self.major) * 1000 + int(self.minor)


DECISION_REPLAN = 1
DECISION_SKIP = 2
DECISION_FAILED = 3


class RouteManager(Node):
    """Route manager node aligned with the Phase2 specification."""

    def __init__(self) -> None:
        super().__init__("route_manager")

        self.qos_tl = _qos_transient_local()
        self.qos_vol = _qos_volatile()

        # Publishers
        self.pub_active_route = self.create_publisher(Route, "/active_route", self.qos_tl)
        self.pub_route_state = self.create_publisher(RouteState, "/route_state", self.qos_vol)
        self.pub_mission_info = self.create_publisher(MissionInfo, "/mission_info", self.qos_tl)
        self.pub_manager_status = self.create_publisher(
            ManagerStatus, "/manager_status", self.qos_vol
        )

        # Subscriber
        self.sub_follower = self.create_subscription(
            FollowerState, "/follower_state", self._on_follower_state, self.qos_vol
        )

        # Services
        self.cb_srv = MutuallyExclusiveCallbackGroup()
        self.cb_cli = MutuallyExclusiveCallbackGroup()
        self.srv_report_stuck = self.create_service(
            ReportStuck, "/report_stuck", self._on_report_stuck, callback_group=self.cb_srv
        )
        self.cli_get = self.create_client(GetRoute, "/get_route", callback_group=self.cb_cli)
        self.cli_update = self.create_client(UpdateRoute, "/update_route", callback_group=self.cb_cli)

        # Parameters
        self.declare_parameter("auto_request_on_startup", True)
        self.declare_parameter("planner_timeout_sec", 5.0)
        self.declare_parameter("waiting_deadline_sec", 8.0)
        self.declare_parameter("skip_threshold_m", 0.8)
        self.declare_parameter("avoid_max_retry", 3)
        self.declare_parameter("offset_step_m", 0.5)
        self.declare_parameter("state_publish_rate_hz", 1.0)

        self.auto_request = self.get_parameter("auto_request_on_startup").value
        self.planner_timeout_sec = float(self.get_parameter("planner_timeout_sec").value)
        self.waiting_deadline_sec = float(self.get_parameter("waiting_deadline_sec").value)
        self.skip_threshold_m = float(self.get_parameter("skip_threshold_m").value)
        self.avoid_max_retry = int(self.get_parameter("avoid_max_retry").value)
        self.offset_step_m = float(self.get_parameter("offset_step_m").value)
        self.state_publish_rate_hz = float(self.get_parameter("state_publish_rate_hz").value)

        # Internal state
        self.version = VersionMM(major=0, minor=0)
        self.active_route: Optional[Route] = None
        self.follow_state: Optional[FollowerState] = None

        self.manager_state: str = "idle"
        self.last_decision: str = "none"
        self.last_cause: str = "none"
        self.current_index: int = -1
        self.current_label: str = ""
        self.total_waypoints: int = 0

        if self.auto_request:
            self._request_initial_route()

        self.create_timer(1.0 / max(0.1, self.state_publish_rate_hz), self._publish_route_state)
        self.create_timer(1.0, self._publish_manager_status)

        self.get_logger().info("route_manager Phase2 started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_follower_state(self, msg: FollowerState) -> None:
        self.follow_state = msg
        try:
            self.current_index = int(msg.current_index)
        except Exception:
            pass
        try:
            self.current_label = str(msg.current_waypoint_label)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # /report_stuck handling
    # ------------------------------------------------------------------
    def _on_report_stuck(
        self, req: ReportStuck.Request, res: ReportStuck.Response
    ) -> ReportStuck.Response:
        res.decision = DECISION_FAILED
        res.offset_hint = 0.0
        res.waiting_deadline = self._duration_from_seconds(self.waiting_deadline_sec)
        res.note = ""

        if self.active_route is None or len(self.active_route.waypoints) < 2:
            self._set_manager_state("holding")
            self._set_last_decision("failed", req.reason or "invalid_route")
            res.note = "active_route is not available"
            return res

        wps = self.active_route.waypoints
        current_index = int(req.current_index)
        if current_index < 1 or current_index >= len(wps):
            self._set_manager_state("holding")
            self._set_last_decision("failed", "invalid_index")
            res.note = f"invalid current_index={current_index}"
            return res

        prev_wp = wps[current_index - 1]
        next_wp = wps[current_index]

        self.current_index = current_index
        self.current_label = next_wp.label

        # ---- Layer 1: lateral offset replan ---------------------------------
        offset_hint = self._compute_offset_hint(req, next_wp)
        if offset_hint is not None:
            note = f"replan with lateral offset ({offset_hint:+.2f} m)"
            if self._try_update_route(reason=f"offset_replan:{offset_hint:+.2f}"):
                res.decision = DECISION_REPLAN
                res.offset_hint = float(offset_hint)
                res.note = note
                self._set_last_decision("replan", req.reason or "stagnation")
                return res
            self.get_logger().warn("offset replan failed. Trying next recovery step.")

        # ---- Layer 2: skip ---------------------------------------------------
        if self._can_skip(req.reason, current_wp, wps, current_index):
            if self._apply_skip(current_index):
                res.decision = DECISION_SKIP
                res.note = "skipped obstructed waypoint"
                self._set_last_decision("skip", req.reason or "skip")
                return res

        # ---- Layer 3: fallback replan ---------------------------------------
        if self._try_update_route(reason=req.reason or "replan"):
            res.decision = DECISION_REPLAN
            res.offset_hint = 0.0
            res.note = "route updated via planner"
            self._set_last_decision("replan", req.reason or "replan")
            return res

        # ---- Failed ---------------------------------------------------------
        self._set_manager_state("holding")
        self._set_last_decision("failed", req.reason or "failed")
        res.decision = DECISION_FAILED
        res.note = "no valid recovery action"
        return res

    # ------------------------------------------------------------------
    # Decision helpers
    # ------------------------------------------------------------------
    def _compute_offset_hint(
        self, req: ReportStuck.Request, target_wp: Waypoint
    ) -> Optional[float]:
        reason = (req.reason or "").lower()
        if reason not in {"stagnation", "avoidance_failed"}:
            return None
        if int(req.avoid_trial_count) >= self.avoid_max_retry:
            return None
        if not bool(req.last_hint_blocked):
            return None

        left_open = max(float(getattr(next_wp, "left_open", 0.0)), 0.0)
        right_open = max(float(getattr(next_wp, "right_open", 0.0)), 0.0)
        if left_open <= 0.0 and right_open <= 0.0:
            return None

        last_offset = float(req.last_applied_offset_m)
        # Prefer alternating sides depending on the last applied offset.
        if last_offset <= 0.0 and left_open > 0.0:
            return -float(self.offset_step_m)
        if last_offset >= 0.0 and right_open > 0.0:
            return float(self.offset_step_m)

        # Fallback: pick the side with wider opening.
        if left_open >= right_open and left_open > 0.0:
            return -float(self.offset_step_m)
        if right_open > 0.0:
            return float(self.offset_step_m)
        return None

    def _can_skip(
        self, reason: str, current_wp: Waypoint, wps: Sequence[Waypoint], current_index: int
    ) -> bool:
        reason_l = (reason or "").lower()
        if reason_l not in {"no_hint", "no_space", "avoidance_failed"}:
            return False
        if current_index + 1 >= len(wps):
            return False
        if bool(getattr(current_wp, "not_skip", False)):
            return False
        next_wp = wps[current_index + 1]
        dist = _distance_xy(current_wp.pose, next_wp.pose)
        return dist <= self.skip_threshold_m

    def _apply_skip(self, current_index: int) -> bool:
        if self.active_route is None:
            return False
        wps = self.active_route.waypoints
        if current_index + 1 >= len(wps):
            return False

        new_route = Route()
        new_route.header = Header()
        new_route.header.stamp = self.get_clock().now().to_msg()
        new_route.header.frame_id = "map"
        self.version.minor += 1
        new_route.version = self.version.to_int()
        new_route.total_distance = getattr(self.active_route, "total_distance", 0.0)
        new_route.route_image = Image()
        new_route.waypoints = list(wps[current_index + 1 :])

        if not new_route.waypoints:
            return False

        self.active_route = new_route
        self.total_waypoints = len(new_route.waypoints)
        self.current_index = 0
        self.current_label = new_route.waypoints[0].label

        self._set_manager_state("updating_route")
        self.pub_active_route.publish(new_route)
        self._set_manager_state("running")
        return True

    # ------------------------------------------------------------------
    # Planner interaction
    # ------------------------------------------------------------------
    def _try_update_route(self, reason: str) -> bool:
        if not self.cli_update.wait_for_service(timeout_sec=self.planner_timeout_sec):
            self.get_logger().warn("/update_route service unavailable.")
            self._set_manager_state("running")
            return False

        req = UpdateRoute.Request()
        if self.active_route is not None and self.current_index >= 0:
            prev_idx = max(self.current_index - 1, 0)
            next_idx = min(self.current_index, len(self.active_route.waypoints) - 1)
            req.prev_index = int(prev_idx)
            req.next_index = int(next_idx)
            req.prev_wp_label = self.active_route.waypoints[prev_idx].label
            req.next_wp_label = self.active_route.waypoints[next_idx].label

        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        if self.follow_state is not None:
            try:
                pose.pose = self.follow_state.current_pose
            except Exception:
                pose.pose = Pose()
        req.current_pose = pose
        req.route_version = self.version.to_int()
        req.reason = reason

        future = self.cli_update.call_async(req)
        self._set_manager_state("updating_route")
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.planner_timeout_sec)
        if not future.done():
            self.get_logger().warn("/update_route timeout.")
            self._set_manager_state("running")
            return False
        resp = future.result()
        if resp is None or not getattr(resp, "success", False):
            self.get_logger().warn("/update_route failed.")
            self._set_manager_state("running")
            return False

        route = resp.route
        self.version.major += 1
        self.version.minor = 0
        route.version = self.version.to_int()
        route.header = route.header or Header()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = "map"

        self.active_route = route
        self.total_waypoints = len(route.waypoints)
        self.current_index = 0
        self.current_label = route.waypoints[0].label if route.waypoints else ""

        self.pub_active_route.publish(route)
        self._set_manager_state("running")
        return True

    # ------------------------------------------------------------------
    # Initial route request
    # ------------------------------------------------------------------
    def _request_initial_route(self) -> None:
        if not self.cli_get.wait_for_service(timeout_sec=self.planner_timeout_sec):
            self.get_logger().warn("/get_route service unavailable.")
            self._set_manager_state("holding")
            return

        future = self.cli_get.call_async(GetRoute.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.planner_timeout_sec)
        if not future.done():
            self.get_logger().warn("/get_route timeout.")
            self._set_manager_state("holding")
            return

        resp = future.result()
        if resp is None or not hasattr(resp, "route"):
            self.get_logger().warn("/get_route failed.")
            self._set_manager_state("holding")
            return

        route: Route = resp.route
        self.version.major = max(1, self.version.major or 0)
        self.version.minor = 0
        route.version = self.version.to_int()
        route.header = route.header or Header()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = "map"

        self.active_route = route
        self.total_waypoints = len(route.waypoints)
        self.current_index = 0
        self.current_label = route.waypoints[0].label if route.waypoints else ""

        self.pub_active_route.publish(route)
        self._publish_mission_info()
        self._set_manager_state("running")

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_mission_info(self) -> None:
        msg = MissionInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.pub_mission_info.publish(msg)

    def _publish_route_state(self) -> None:
        msg = RouteState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.route_version = self.version.to_int()
        msg.total_waypoints = int(self.total_waypoints)
        msg.current_index = int(self.current_index)
        msg.current_label = str(self.current_label)
        msg.status = self.manager_state.upper()
        msg.message = self.last_decision
        self.pub_route_state.publish(msg)

    def _publish_manager_status(self) -> None:
        msg = ManagerStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.state = self.manager_state
        msg.decision = self.last_decision
        msg.last_cause = self.last_cause
        msg.route_version = self.version.to_int()
        self.pub_manager_status.publish(msg)

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------
    def _set_manager_state(self, state: str) -> None:
        if self.manager_state != state:
            self.get_logger().info(f"manager_state: {self.manager_state} -> {state}")
        self.manager_state = state

    def _set_last_decision(self, decision: str, cause: str) -> None:
        self.last_decision = decision
        self.last_cause = cause

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        msg = Duration()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteManager()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
