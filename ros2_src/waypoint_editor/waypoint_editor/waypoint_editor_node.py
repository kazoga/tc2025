"""CSVベースのウェイポイント編集ノード。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray

from waypoint_editor.csv_io import Waypoint, read_waypoints, update_waypoint, write_waypoints
from waypoint_editor.srv import (
    DeleteWaypoint,
    ResetWaypoints,
    RotateWaypoint,
    SaveWaypoints,
    SearchLabel,
    SelectInRviz,
    ShiftWaypoint,
    UpdateFlags,
    UpdateNode,
    UpdatePose,
)


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """平面yawからq1..q4を生成する。"""
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _quaternion_to_yaw(q3: float, q4: float) -> float:
    return math.atan2(q3, q4) * 2.0


class WaypointEditorNode(Node):
    """GUIと連携しウェイポイント操作を行うノード。"""

    def __init__(self) -> None:
        super().__init__("waypoint_editor")
        param = self.declare_parameter("waypoint_path", "waypoints.csv")
        self.waypoint_path = Path(param.get_parameter_value().string_value)
        self.waypoints: List[Waypoint] = []
        self.header_order: List[str] = []
        self.selected_index: Optional[int] = None

        self.selected_pub = self.create_publisher(Int32, "selected_label", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "markers", 10)

        self.create_service(SearchLabel, "search_label", self.handle_search_label)
        self.create_service(UpdatePose, "update_pose", self.handle_update_pose)
        self.create_service(UpdateFlags, "update_flags", self.handle_update_flags)
        self.create_service(UpdateNode, "update_node", self.handle_update_node)
        self.create_service(ShiftWaypoint, "shift_waypoint", self.handle_shift_waypoint)
        self.create_service(RotateWaypoint, "rotate_waypoint", self.handle_rotate_waypoint)
        self.create_service(SaveWaypoints, "save_waypoints", self.handle_save_waypoints)
        self.create_service(ResetWaypoints, "reset_waypoints", self.handle_reset_waypoints)
        self.create_service(DeleteWaypoint, "delete_waypoint", self.handle_delete_waypoint)
        self.create_service(SelectInRviz, "select_in_rviz", self.handle_select_in_rviz)

        self._load_waypoints()
        self.get_logger().info("waypoint_editorノードを起動しました。")

    def _load_waypoints(self) -> None:
        self.waypoints, self.header_order = read_waypoints(self.waypoint_path)
        if not self.waypoints:
            self.get_logger().warn("ウェイポイントCSVが空か見つかりません。")

    def _find_index(self, label: int) -> Optional[int]:
        for idx, waypoint in enumerate(self.waypoints):
            if waypoint.label == label:
                return idx
        return None

    def _publish_selected_label(self, label: int) -> None:
        msg = Int32()
        msg.data = label
        self.selected_pub.publish(msg)

    def _update_selection(self, index: Optional[int]) -> None:
        self.selected_index = index
        if index is not None:
            self._publish_selected_label(self.waypoints[index].label)

    def _update_waypoint_pose(self, waypoint: Waypoint, x: float, y: float, yaw: float) -> Waypoint:
        q1, q2, q3, q4 = _yaw_to_quaternion(yaw)
        return update_waypoint(waypoint, x=x, y=y, q1=q1, q2=q2, q3=q3, q4=q4)

    def _response_common(self, success: bool, message: str):
        return {"success": success, "message": message}

    def handle_search_label(self, request: SearchLabel.Request, response: SearchLabel.Response
                            ) -> SearchLabel.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "指定labelが見つかりませんでした。"
            return response
        wp = self.waypoints[index]
        self._update_selection(index)

        response.success = True
        response.message = "OK"
        response.x = wp.x
        response.y = wp.y
        response.z = wp.z
        response.yaw = _quaternion_to_yaw(wp.q3, wp.q4)
        response.right_is_open = wp.right_is_open
        response.left_is_open = wp.left_is_open
        response.line_is_stop = int(wp.line_is_stop)
        response.signal_is_stop = int(wp.signal_is_stop)
        response.isnot_skipnum = int(wp.isnot_skipnum)
        response.node = wp.node
        return response

    def handle_update_pose(self, request: UpdatePose.Request, response: UpdatePose.Response
                           ) -> UpdatePose.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "更新対象が見つかりません。"
            return response
        waypoint = self._update_waypoint_pose(
            self.waypoints[index], request.x, request.y, request.yaw
        )
        self.waypoints[index] = waypoint
        self._update_selection(index)
        response.success = True
        response.message = "姿勢を更新しました。"
        return response

    def handle_update_flags(self, request: UpdateFlags.Request, response: UpdateFlags.Response
                            ) -> UpdateFlags.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "更新対象が見つかりません。"
            return response
        waypoint = update_waypoint(
            self.waypoints[index],
            right_is_open=request.right_is_open,
            left_is_open=request.left_is_open,
            line_is_stop=int(request.line_is_stop),
            signal_is_stop=int(request.signal_is_stop),
            isnot_skipnum=int(request.isnot_skipnum),
        )
        self.waypoints[index] = waypoint
        self._update_selection(index)
        response.success = True
        response.message = "フラグを更新しました。"
        return response

    def handle_update_node(self, request: UpdateNode.Request, response: UpdateNode.Response
                           ) -> UpdateNode.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "更新対象が見つかりません。"
            return response
        self.waypoints[index] = update_waypoint(self.waypoints[index], node=request.node)
        self._update_selection(index)
        response.success = True
        response.message = "nodeを更新しました。"
        return response

    def handle_shift_waypoint(self, request: ShiftWaypoint.Request, response: ShiftWaypoint.Response
                              ) -> ShiftWaypoint.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "更新対象が見つかりません。"
            return response
        wp = self.waypoints[index]
        self.waypoints[index] = update_waypoint(
            wp, x=wp.x + request.shift_x, y=wp.y + request.shift_y
        )
        self._update_selection(index)
        response.success = True
        response.message = "シフトしました。"
        return response

    def handle_rotate_waypoint(
        self, request: RotateWaypoint.Request, response: RotateWaypoint.Response
    ) -> RotateWaypoint.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "更新対象が見つかりません。"
            return response
        wp = self.waypoints[index]
        yaw = _quaternion_to_yaw(wp.q3, wp.q4) + request.delta_yaw
        self.waypoints[index] = self._update_waypoint_pose(wp, wp.x, wp.y, yaw)
        self._update_selection(index)
        response.success = True
        response.message = "回転しました。"
        return response

    def handle_save_waypoints(self, request: SaveWaypoints.Request, response: SaveWaypoints.Response
                              ) -> SaveWaypoints.Response:
        write_waypoints(self.waypoint_path, self.waypoints, self.header_order)
        response.success = True
        response.message = "CSVを保存しました。"
        return response

    def handle_reset_waypoints(
        self, request: ResetWaypoints.Request, response: ResetWaypoints.Response
    ) -> ResetWaypoints.Response:
        self._load_waypoints()
        self._update_selection(None)
        response.success = True
        response.message = "CSVを再読み込みしました。"
        return response

    def handle_delete_waypoint(
        self, request: DeleteWaypoint.Request, response: DeleteWaypoint.Response
    ) -> DeleteWaypoint.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "削除対象が見つかりません。"
            return response
        self.waypoints.pop(index)
        self._update_selection(None)
        response.success = True
        response.message = "削除しました。"
        return response

    def handle_select_in_rviz(self, request: SelectInRviz.Request, response: SelectInRviz.Response
                              ) -> SelectInRviz.Response:
        index = self._find_index(request.label)
        if index is None:
            response.success = False
            response.message = "対象が見つかりません。"
            return response
        self._update_selection(index)
        response.success = True
        response.message = "選択を更新しました。"
        return response


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointEditorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
