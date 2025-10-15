"""ROS2 Node ラッパー."""

from __future__ import annotations

import asyncio
from typing import Any, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped  # type: ignore
from rclpy.node import Node
from std_msgs.msg import Bool, Int32  # type: ignore

from route_msgs.msg import ObstacleAvoidanceHint, Route  # type: ignore
from route_msgs.srv import ReportStuck  # type: ignore

from .route_follower_fsm import Event, ReportStuckRequest, RouteFollowerFSM, RouteFollowerHooks


class RouteFollowerNode(Node):
    """route_follower ROSノード."""

    def __init__(self) -> None:
        super().__init__("route_follower")
        self._loop = asyncio.get_event_loop()
        self._queue: asyncio.Queue[Tuple[Event, Any]] = asyncio.Queue()

        hooks = RouteFollowerHooks(report_stuck=self._report_stuck)
        self.fsm = RouteFollowerFSM(self.get_logger(), hooks=hooks)

        self.create_subscription(Route, "/active_route", self._on_route, 10)
        self.create_subscription(PoseStamped, "/amcl_pose", self._on_pose, 10)
        self.create_subscription(
            ObstacleAvoidanceHint,
            "/obstacle_avoidance_hint",
            self._on_hint,
            10,
        )
        self.create_subscription(Bool, "/manual_start", self._on_manual_start, 10)
        self.create_subscription(Int32, "/sig_recog", self._on_sig_recog, 10)

        self.cli_report_stuck = self.create_client(ReportStuck, "/report_stuck")

        self.create_timer(0.05, self._on_timer)

        self._loop.create_task(self._event_loop())

    def _enqueue(self, event: Event, data: Any | None = None) -> None:
        """イベントを非同期キューへ投入."""

        asyncio.run_coroutine_threadsafe(self._queue.put((event, data)), self._loop)

    def _on_route(self, msg: Route) -> None:
        self._enqueue(Event.ROUTE_RECEIVED, msg)

    def _on_pose(self, msg: PoseStamped) -> None:
        self._enqueue(Event.POSE_UPDATE, msg)

    def _on_hint(self, msg: ObstacleAvoidanceHint) -> None:
        self._enqueue(Event.HINT_RECEIVED, msg)

    def _on_manual_start(self, msg: Bool) -> None:
        if msg.data:
            self._enqueue(Event.MANUAL_START, msg)

    def _on_sig_recog(self, msg: Int32) -> None:
        self._enqueue(Event.SIG_RECOG, msg)

    def _on_timer(self) -> None:
        self._enqueue(Event.TIMER_TICK, None)

    async def _event_loop(self) -> None:
        """FSMイベントループ."""

        while rclpy.ok():
            event, data = await self._queue.get()
            await self.fsm.handle_event(event, data)

    async def _report_stuck(self, request: ReportStuckRequest) -> None:
        """report_stuck 呼び出し."""

        if not self.cli_report_stuck.service_is_ready():
            self.get_logger().warning("/report_stuck service not ready.")
            return

        ros_req = ReportStuck.Request()
        ros_req.reason = request.reason
        future = self.cli_report_stuck.call_async(ros_req)
        while not future.done():
            await asyncio.sleep(0.01)


def main(args: list[str] | None = None) -> None:
    """ノードエントリポイント."""

    rclpy.init(args=args)
    node = RouteFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["RouteFollowerNode", "main"]

