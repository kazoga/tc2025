"""ROS2 node wrapper for the route planner core logic."""

from __future__ import annotations

import sys
from typing import List, Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from route_msgs.msg import Route
from route_msgs.srv import GetRoute, UpdateRoute

from .route_builder import LoggerProtocol, RouteBuilder


class _NodeLogger(LoggerProtocol):
    """rclpyのロガーをラップする。"""

    def __init__(self, node: Node) -> None:
        self._node = node

    def info(self, message: str) -> None:
        """情報ログを出力する。"""
        self._node.get_logger().info(message)

    def warn(self, message: str) -> None:
        """警告ログを出力する。"""
        self._node.get_logger().warn(message)

    def error(self, message: str) -> None:
        """エラーログを出力する。"""
        self._node.get_logger().error(message)


class RoutePlannerNode(Node):
    """RouteBuilderをサービスで公開するROS2ノード。"""

    def __init__(self) -> None:
        super().__init__("route_planner")
        self.declare_parameter("config_yaml_path", "")
        self.declare_parameter("csv_base_dir", "")
        config_yaml_path = (
            self.get_parameter("config_yaml_path").get_parameter_value().string_value
        )
        csv_base_dir = (
            self.get_parameter("csv_base_dir").get_parameter_value().string_value
        )
        self.get_logger().info(f"Using config_yaml_path: {config_yaml_path}")
        self._builder = RouteBuilder(
            config_yaml_path=config_yaml_path,
            csv_base_dir=csv_base_dir,
            logger=_NodeLogger(self),
        )
        cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_get = self.create_service(
            GetRoute, "/get_route", self.handle_get_route, callback_group=cb_group
        )
        self._srv_update = self.create_service(
            UpdateRoute, "/update_route", self.handle_update_route, callback_group=cb_group
        )
        self.get_logger().info("route_planner is ready.")

    def handle_get_route(
        self, request: GetRoute.Request, response: GetRoute.Response
    ) -> GetRoute.Response:
        """GetRouteサービスリクエストを処理する。"""
        try:
            route = self._builder.build_initial_route(
                start_label=getattr(request, "start_label", None),
                goal_label=getattr(request, "goal_label", None),
                checkpoint_labels=getattr(request, "checkpoint_labels", []),
            )
            response.success = True
            response.message = ""
            response.route = route
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[GetRoute] {exc}")
            response.success = False
            response.message = str(exc)
            response.route = Route()
        return response

    def handle_update_route(
        self, request: UpdateRoute.Request, response: UpdateRoute.Response
    ) -> UpdateRoute.Response:
        """UpdateRouteサービスリクエストを処理する。"""
        try:
            route = self._builder.update_route(
                route_version=request.route_version,
                prev_index=request.prev_index,
                next_index=request.next_index,
                prev_wp_label=request.prev_wp_label,
                next_wp_label=request.next_wp_label,
                block_name_hint=request.block_name,
                current_pose=request.current_pose,
            )
            response.success = True
            response.message = ""
            response.route = route
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[UpdateRoute] {exc}")
            response.success = False
            response.message = str(exc)
            response.route = self._builder.current_route or Route()
        return response


def main(argv: Optional[List[str]] = None) -> None:
    """ノード起動エントリポイント。"""
    rclpy.init(args=argv)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
