
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
route_manager_phase2.py

Phase2 対応版 route_manager ノード（最終確定仕様反映）
- /report_stuck サーバを実装（decision 自動判定：replan / shift / skip / failed）
- /get_route・/update_route クライアント（非ブロッキング、timeout管理）
- /active_route, /route_state, /mission_info をPublish
- /follower_state を購読（距離・状態・現在/次ラベル等をキャッシュ）
- バージョン体系：Route.version = major * 1000 + minor
    * replan 成功時: major += 1, minor = 0
    * shift / skip:  minor += 1
- 判定規則（最終）
    Step0: エラーチェック（入力健全性）→ failed（HOLDING）
    Step1: replan 試行（前後WP可変が明確／不明ならまず一度試す）
    Step2: shift（次WP 1点のみ、基準線=prev→next、垂直方向、距離=min(open,1.0)）
    Step3: skip（距離<=1.0m かつ not_skip==False）
    Step4: fallback replan 試行
    Step5: failed（HOLDING）

ROS: Foxy 想定／Google Python Style／型ヒント／日本語コメント
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, List, Tuple

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image

# === パッケージ依存（プロジェクト定義） ===
# NOTE: メッセージおよびサービスはユーザー定義パッケージ（route_msgs / route_srvs）に準拠
from route_msgs.msg import Route, Waypoint, MissionInfo, RouteState, FollowerState  # type: ignore
from route_msgs.srv import GetRoute, UpdateRoute  # type: ignore
try:
    # 既定：ReportStuck は route_srvs にある想定（仕様初期提示）
    from route_srvs.srv import ReportStuck  # type: ignore
except Exception:
    # ユーザー最終合意が route_msgs に集約される場合の互換
    from route_msgs.srv import ReportStuck  # type: ignore


# ==============================
# ユーティリティ
# ==============================

def _qos_tl() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _qos_vol() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def xy_of_pose(pose: Pose) -> Tuple[float, float]:
    return (pose.position.x, pose.position.y)


def seg_side(a: Tuple[float, float], b: Tuple[float, float], p: Tuple[float, float]) -> float:
    """有向線分 a->b に対する点 p の左右判定（+なら左、-なら右、0に近ければ線上）"""
    ax, ay = a
    bx, by = b
    px, py = p
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax)


def norm2(vx: float, vy: float) -> float:
    return math.hypot(vx, vy)


def perp_unit(ax: float, ay: float, bx: float, by: float, right_side: bool) -> Tuple[float, float]:
    """a->b の法線単位ベクトルを返す。right_side=True なら右向き、False なら左向き。"""
    dx, dy = (bx - ax), (by - ay)
    n = norm2(dx, dy)
    if n < 1e-9:
        # 退避：x軸基準の垂直
        return (0.0, -1.0) if right_side else (0.0, 1.0)
    ux, uy = dx / n, dy / n
    # 左法線 = (-uy, ux), 右法線 = (uy, -ux)
    if right_side:
        return (uy, -ux)
    else:
        return (-uy, ux)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ==============================
# 内部状態
# ==============================

@dataclass
class VersionMM:
    major: int = 0
    minor: int = 0

    def to_int(self) -> int:
        return int(self.major) * 1000 + int(self.minor)


# ==============================
# メインノード
# ==============================

class RouteManager(Node):
    """Phase2 route_manager 実装"""

    def __init__(self) -> None:
        super().__init__("route_manager")

        # ---- QoS ----
        self.qos_tl = _qos_tl()
        self.qos_vol = _qos_vol()

        # ---- Publisher ----
        self.pub_active_route = self.create_publisher(Route, "/active_route", self.qos_tl)
        self.pub_route_state = self.create_publisher(RouteState, "/route_state", self.qos_vol)
        self.pub_mission_info = self.create_publisher(MissionInfo, "/mission_info", self.qos_tl)

        # ---- Subscriber ----
        self.sub_follower = self.create_subscription(
            FollowerState, "/follower_state", self._on_follower_state, self.qos_vol
        )

        # ---- Services ----
        self.cb_srv = MutuallyExclusiveCallbackGroup()
        self.cb_cli = MutuallyExclusiveCallbackGroup()
        self.srv_report_stuck = self.create_service(
            ReportStuck, "/report_stuck", self._on_report_stuck, callback_group=self.cb_srv
        )
        self.cli_get = self.create_client(GetRoute, "/get_route", callback_group=self.cb_cli)
        self.cli_update = self.create_client(UpdateRoute, "/update_route", callback_group=self.cb_cli)

        # ---- Parameters ----
        self.declare_parameter("auto_request_on_startup", True)
        self.declare_parameter("planner_timeout_sec", 5.0)
        self.declare_parameter("waiting_deadline_sec", 8.0)
        self.declare_parameter("skip_threshold_m", 1.0)  # 仕様：1.0m 以内でスキップ可能
        self.declare_parameter("offset_step_m_max", 1.0)  # 仕様：shift の上限距離 1.0m
        self.declare_parameter("state_publish_rate_hz", 1.0)

        self.auto_request = self.get_parameter("auto_request_on_startup").get_parameter_value().bool_value
        self.planner_timeout_sec = float(self.get_parameter("planner_timeout_sec").value)
        self.waiting_deadline_sec = float(self.get_parameter("waiting_deadline_sec").value)
        self.skip_threshold_m = float(self.get_parameter("skip_threshold_m").value)
        self.offset_step_m_max = float(self.get_parameter("offset_step_m_max").value)
        self.state_publish_rate_hz = float(self.get_parameter("state_publish_rate_hz").value)

        # ---- 内部状態 ----
        self.version = VersionMM(major=0, minor=0)
        self.active_route: Optional[Route] = None

        # follower_state のキャッシュ
        self.follow_state: Optional[FollowerState] = None

        # route_state の現在値
        self.state_status: str = "IDLE"       # "IDLE","ACTIVE","UPDATING_ROUTE","HOLDING","COMPLETED","ERROR"
        self.state_message: str = ""
        self.current_index: int = -1
        self.current_label: str = ""
        self.total_waypoints: int = 0

        # 起動直後の初期リクエスト（任意）
        if self.auto_request:
            self._request_initial_route()

        # 状態定期配信
        self.create_timer(1.0 / max(0.1, self.state_publish_rate_hz), self._publish_route_state)

        self.get_logger().info("route_manager Phase2 started.")

    # ==============================
    # FollowerState 取り込み
    # ==============================

    def _on_follower_state(self, msg: FollowerState) -> None:
        self.follow_state = msg
        try:
            self.current_index = int(getattr(msg, "current_index", self.current_index))
        except Exception:
            pass
        try:
            self.current_label = str(getattr(msg, "current_waypoint_label", self.current_label) or "")
        except Exception:
            pass
        try:
            self.total_waypoints = int(getattr(msg, "route_version", 0))  # ダミー代入回避（下で上書き）
        except Exception:
            pass
        # 完了／異常のブリッジ
        st = (msg.state or "").upper()
        if st == "FINISHED":
            self.state_status = "COMPLETED"
            self.state_message = "mission complete"
        elif st == "ERROR":
            self.state_status = "ERROR"
            self.state_message = "follower error"

    # ==============================
    # /report_stuck サービス（判定）
    # ==============================

    def _on_report_stuck(self, req: ReportStuck.Request, res: ReportStuck.Response) -> ReportStuck.Response:
        # Step0: 入力・健全性チェック（破綻判定ではなくガード）
        if self.active_route is None or len(self.active_route.waypoints) < 2:
            return self._res_failed(res, "invalid route state (no active_route)")

        wps = self.active_route.waypoints
        current_index = int(req.current_index)
        if current_index < 0 or current_index >= len(wps):
            return self._res_failed(res, f"invalid current_index={current_index}")

        # prev / next の特定（current_index が「現在目標の次WP」）
        prev_idx = current_index - 1
        next_idx = current_index
        if prev_idx < 0:
            return self._res_failed(res, "no previous waypoint")
        prev_wp = wps[prev_idx]
        next_wp = wps[next_idx]

        # Follower 側の距離キャッシュ
        dist_to_next: Optional[float] = None
        if self.follow_state is not None:
            try:
                dist_to_next = float(self.follow_state.distance_to_target)
            except Exception:
                dist_to_next = None

        # ---- Step1: replan 試行（可変ブロック明確 or 不明でも一度試す）
        # Waypoint.msg に block_type が無い場合は「不明」扱い → まず試す方針
        replan_needed = False
        # block_type が存在する環境では true/false 判定を行う（後方互換）
        try:
            bt_prev = str(getattr(prev_wp, "block_type", "") or "")
            bt_next = str(getattr(next_wp, "block_type", "") or "")
            if bt_prev == "variable" and bt_next == "variable":
                replan_needed = True
        except Exception:
            replan_needed = False  # 不明

        if replan_needed or not hasattr(next_wp, "block_type"):
            if self._try_update_route(reason="replan_first"):
                res.accepted = True
                res.decision = "replan"
                res.note = "route updated via planner"
                return res
            # 失敗時は後段へ継続

        # ---- Step2: shift（次WP 1点のみ）
        # 条件：left_open>0 または right_open>0
        left_open = float(getattr(next_wp, "left_open", 0.0) or 0.0)
        right_open = float(getattr(next_wp, "right_open", 0.0) or 0.0)
        can_shift = (left_open > 0.0) or (right_open > 0.0)

        if can_shift:
            # 基準線 prev→next
            ax, ay = xy_of_pose(prev_wp.pose)
            bx, by = xy_of_pose(next_wp.pose)
            px, py = (req.current_pose_map.position.x, req.current_pose_map.position.y)

            side_val = seg_side((ax, ay), (bx, by), (px, py))
            # 正: 左側、負: 右側
            if left_open > 0.0 and right_open > 0.0:
                right_side = (side_val < 0.0)
                open_len = right_open if right_side else left_open
            elif right_open > 0.0:
                right_side = True
                open_len = right_open
            else:
                right_side = False
                open_len = left_open

            shift_d = clamp(open_len, 0.0, float(self.offset_step_m_max))
            nx, ny = bx, by
            vx, vy = perp_unit(ax, ay, bx, by, right_side=right_side)
            sx, sy = nx + vx * shift_d, ny + vy * shift_d

            # 次WPのみ変更（index/label/flags は維持）
            new_wp = Waypoint()
            new_wp.index = next_wp.index
            new_wp.label = next_wp.label
            new_wp.pose = Pose()
            new_wp.pose.position.x = sx
            new_wp.pose.position.y = sy
            new_wp.pose.position.z = next_wp.pose.position.z
            new_wp.pose.orientation = next_wp.pose.orientation  # yaw は維持
            # 開放度・フラグは元値維持
            for fld in ("right_open", "left_open", "line_stop", "signal_stop", "not_skip"):
                if hasattr(next_wp, fld):
                    setattr(new_wp, fld, getattr(next_wp, fld))

            # 新しい経路（次WP 1点のみ差し替え）
            new_route = Route()
            new_route.header = Header()
            new_route.header.stamp = self.get_clock().now().to_msg()
            new_route.header.frame_id = "map"
            self.version.minor += 1
            new_route.version = self.version.to_int()
            new_route.total_distance = getattr(self.active_route, "total_distance", 0.0)
            new_route.route_image = Image()  # 画像は空（Planner返却時のみ正値想定）
            new_route.waypoints = list(wps)
            new_route.waypoints[next_idx] = new_wp

            # 採用・配信
            self.active_route = new_route
            self.total_waypoints = len(new_route.waypoints)
            self._set_status("ACTIVE", "shifted next waypoint")
            self.pub_active_route.publish(new_route)

            res.accepted = True
            res.decision = "shift"
            res.note = "shifted next waypoint to open side"
            return res

        # ---- Step3: skip（距離<=1.0m かつ not_skip==False）
        can_skip = False
        if next_idx + 1 < len(wps):
            not_skip = bool(getattr(next_wp, "not_skip", False))
            if (dist_to_next is not None) and (dist_to_next <= self.skip_threshold_m) and (not not_skip):
                can_skip = True

        if can_skip:
            # current_index+1 以降を採用
            new_route = Route()
            new_route.header = Header()
            new_route.header.stamp = self.get_clock().now().to_msg()
            new_route.header.frame_id = "map"
            self.version.minor += 1
            new_route.version = self.version.to_int()
            new_route.total_distance = getattr(self.active_route, "total_distance", 0.0)
            new_route.route_image = Image()
            new_route.waypoints = list(wps[next_idx + 1:])

            # 採用・配信
            self.active_route = new_route
            self.total_waypoints = len(new_route.waypoints)
            self.current_index = 0  # 先頭へ
            self.current_label = new_route.waypoints[0].label if new_route.waypoints else ""
            self._set_status("ACTIVE", "skipped current waypoint")
            self.pub_active_route.publish(new_route)

            res.accepted = True
            res.decision = "skip"
            res.note = "skipped current waypoint"
            return res

        # ---- Step4: fallback replan
        if self._try_update_route(reason="fallback_replan"):
            res.accepted = True
            res.decision = "replan"
            res.note = "route updated via planner (fallback)"
            return res

        # ---- Step5: failed
        self._set_status("HOLDING", "no valid recovery action")
        res.accepted = True
        res.decision = "failed"
        res.note = "no valid recovery action"
        return res

    # ==============================
    # Planner 呼出（/update_route）
    # ==============================

    def _try_update_route(self, reason: str) -> bool:
        """非同期だが、この関数は結果（True/False）を返すまで待機する簡易実装。
        実運用では Future とタイマでタイムアウト制御するが、ここでは簡潔性を優先。
        """
        if not self.cli_update.wait_for_service(timeout_sec=self.planner_timeout_sec):
            self._set_status("HOLDING", "planner unavailable")
            return False

        req = UpdateRoute.Request()
        # prev/next の index/label を可能な限り埋める（整合性のため）
        if self.active_route is not None and len(self.active_route.waypoints) >= 2 and self.current_index >= 0:
            prev_idx = self.current_index - 1
            next_idx = self.current_index
            if 0 <= prev_idx < len(self.active_route.waypoints):
                req.prev_index = int(prev_idx)
                req.prev_wp_label = self.active_route.waypoints[prev_idx].label
            if 0 <= next_idx < len(self.active_route.waypoints):
                req.next_index = int(next_idx)
                req.next_wp_label = self.active_route.waypoints[next_idx].label

        # 位置（PoseStamped）は follower キャッシュがあれば使用
        ps = PoseStamped()
        ps.header = Header()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        if self.follow_state is not None:
            try:
                ps.pose = self.follow_state.current_pose
            except Exception:
                ps.pose = Pose()
        req.current_pose = ps
        req.route_version = int(self.version.to_int())
        req.reason = reason

        self._set_status("UPDATING_ROUTE", f"replanning: {reason}")
        future = self.cli_update.call_async(req)

        # 簡易：スピン待ち（Foxy互換）
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.planner_timeout_sec)
        if not future.done():
            self._set_status("HOLDING", "planner timeout")
            return False
        resp = future.result()
        if resp is None or not getattr(resp, "success", False):
            self._set_status("HOLDING", "planner failed")
            return False

        route: Route = resp.route
        # major += 1, minor = 0
        self.version.major += 1
        self.version.minor = 0
        # 採用前に version を上書き（整数合算方式）
        route.version = self.version.to_int()
        route.header = route.header or Header()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = "map"

        self.active_route = route
        self.total_waypoints = len(route.waypoints)
        self.current_index = 0
        self.current_label = route.waypoints[0].label if route.waypoints else ""
        self._set_status("ACTIVE", "route updated")
        self.pub_active_route.publish(route)
        return True

    # ==============================
    # 初期ルート取得（/get_route）
    # ==============================

    def _request_initial_route(self) -> None:
        if not self.cli_get.wait_for_service(timeout_sec=self.planner_timeout_sec):
            self._set_status("HOLDING", "get_route unavailable")
            return

        req = GetRoute.Request()  # 具体フィールドはプロジェクト定義に依存
        future = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.planner_timeout_sec)
        if not future.done():
            self._set_status("HOLDING", "get_route timeout")
            return
        resp = future.result()
        if resp is None or not hasattr(resp, "route"):
            self._set_status("HOLDING", "get_route failed")
            return

        route: Route = resp.route
        # 初回：major=1, minor=0 扱い
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
        self._set_status("ACTIVE", "initial route accepted")
        self.pub_active_route.publish(route)
        self._publish_mission_info()

    # ==============================
    # 出力（/mission_info, /route_state）
    # ==============================

    def _publish_mission_info(self) -> None:
        msg = MissionInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # MissionInfo は start/goal/チェックポイント等。ここでは active_route から推測できる情報があれば付与。
        # 必須でないため、空のままでも動作に問題はない。
        self.pub_mission_info.publish(msg)

    def _publish_route_state(self) -> None:
        msg = RouteState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.route_version = int(self.version.to_int())
        msg.total_waypoints = int(self.total_waypoints)
        msg.current_index = int(self.current_index)
        msg.current_label = str(self.current_label or "")
        msg.status = str(self.state_status or "IDLE")
        msg.message = str(self.state_message or "")
        self.pub_route_state.publish(msg)

    def _set_status(self, status: str, message: str = "") -> None:
        self.state_status = status
        self.state_message = message

    # ==============================
    # /report_stuck: failed 応答ヘルパ
    # ==============================

    def _res_failed(self, res: ReportStuck.Response, note: str) -> ReportStuck.Response:
        self._set_status("HOLDING", note)
        res.accepted = True
        res.decision = "failed"
        res.note = note
        return res


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