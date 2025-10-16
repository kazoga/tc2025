"""ROS2との通信層を担うRouteManagerノード."""

from __future__ import annotations

import asyncio
import threading
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Optional

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from route_msgs.msg import FollowerState, ManagerStatus, MissionInfo, Route, RouteState
from route_msgs.srv import ReportStuck

from .route_manager_fsm import RouteManagerFSM


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


class _PublisherStub:
    """スタンドアロン試験で使うPublisher代替."""

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.messages.append(msg)


@dataclass
class VersionMM:
    """経路バージョン管理用のMM表現."""

    major: int = 0
    minor: int = 0

    def to_int(self) -> int:
        return int(self.major) * 1000 + int(self.minor)


DECISION_REPLAN = 1
DECISION_SKIP = 2
DECISION_FAILED = 3


class RouteManagerNode(Node):
    """FSMをラップしROS I/Fを提供するノード."""

    def __init__(self) -> None:
        super().__init__("route_manager")

        self.qos_tl = _qos_transient_local()
        self.qos_vol = _qos_volatile()

        self.pub_active_route = self._create_publisher(Route, "/active_route", self.qos_tl)
        self.pub_route_state = self._create_publisher(RouteState, "/route_state", self.qos_vol)
        self.pub_mission_info = self._create_publisher(MissionInfo, "/mission_info", self.qos_tl)
        self.pub_manager_status = self._create_publisher(ManagerStatus, "/manager_status", self.qos_vol)

        self.sub_follower = self._create_subscription(
            FollowerState, "/follower_state", self._on_follower_state, self.qos_vol
        )

        self.cb_srv = MutuallyExclusiveCallbackGroup()
        self.srv_report_stuck = self._create_service(
            ReportStuck, "/report_stuck", self._on_report_stuck, callback_group=self.cb_srv
        )

        self.report_stuck_timeout_sec = 5.0

        self.fsm = RouteManagerFSM(self.get_logger().info)
        self.fsm.set_replan_callback(self._request_replan)

        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

        self.version = VersionMM()
        self.manager_state = "IDLE"

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _create_publisher(self, msg_type: Any, topic: str, qos: QoSProfile) -> Any:
        creator: Optional[Callable[..., Any]] = getattr(super(), "create_publisher", None)
        if callable(creator):
            return creator(msg_type, topic, qos)
        return _PublisherStub(topic)

    def _create_subscription(
        self,
        msg_type: Any,
        topic: str,
        callback: Callable[[Any], None],
        qos: QoSProfile,
    ) -> Any:
        creator: Optional[Callable[..., Any]] = getattr(super(), "create_subscription", None)
        if callable(creator):
            return creator(msg_type, topic, callback, qos)
        # オフライン試験では単にコールバック参照を保持する
        setattr(self, f"_sub_{topic.strip('/').replace('/', '_')}", callback)
        return None

    def _create_service(
        self,
        srv_type: Any,
        name: str,
        callback: Callable[[Any, Any], Any],
        callback_group: Any,
    ) -> Any:
        creator: Optional[Callable[..., Any]] = getattr(super(), "create_service", None)
        if callable(creator):
            return creator(srv_type, name, callback, callback_group=callback_group)
        setattr(self, f"_srv_{name.strip('/').replace('/', '_')}", callback)
        return callback

    def _submit(self, coro: Awaitable[Any]) -> asyncio.Future:
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    async def _request_replan(self, _data: Optional[Any]) -> bool:
        await asyncio.sleep(0.01)
        return True

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_follower_state(self, msg: FollowerState) -> None:
        raw_state = getattr(msg, "state", None)
        if raw_state is None:
            raw_state = getattr(msg, "status", "")

        state = str(raw_state).upper()
        if not state:
            return

        if state == "RUNNING":
            self._submit(self.fsm.transition("RUNNING"))
        elif state == "IDLE":
            self._submit(self.fsm.transition("IDLE"))

    def _on_report_stuck(self, request: ReportStuck.Request, response: ReportStuck.Response) -> ReportStuck.Response:
        future = self._submit(self.fsm.handle_event("REPORT_STUCK", request))
        try:
            result = future.result(timeout=self.report_stuck_timeout_sec)
        except Exception as exc:  # pragma: no cover - 異常系
            self.get_logger().warn(f"FSM execution failed: {exc}")
            response.decision = DECISION_FAILED
            response.note = str(exc)
            return response

        response.decision = DECISION_REPLAN if result.get("success") else DECISION_FAILED
        response.note = result.get("message", "")
        response.waiting_deadline = Duration()
        response.waiting_deadline.sec = int(self.report_stuck_timeout_sec)
        response.waiting_deadline.nanosec = 0
        return response

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        if hasattr(self, "_loop") and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._loop_thread.join(timeout=1.0)
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RouteManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
