
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""route_follower ノード（Phase2 正式版, L字回避必須・deque実装）

本ノードは /active_route に基づき waypoint 追従を行い、滞留（stagnation）を唯一の
トリガとして局所回避（マイクロデトア）を試行する。回避に失敗した場合は
/report_stuck（route_srvs/ReportStuck）を同期呼び出しし、decision に応じて
WAITING_REROUTE で新たな /active_route を待機する。

Phase2の必須要件として、回避は**L字（横→前進）**の2段階サブ目標を**deque**で管理する。

追加仕様（反映済み）:
- 滞留検知（過去位置差 + 速度の複合条件、継続 15s）
- obstacle_monitor の Hint（過去5s キャッシュ）を用いた回避（左右選択・中央値採用）
- 回避は L字：横シフト→前進 の2段階サブ目標を順次消化（deque管理）
- 回避失敗（回避中の再滞留 等）→ report_stuck → WAITING_REROUTE
- WAITING_REROUTE タイムアウト（30s）→ ERROR
- WAITING_STOP の解除条件：
    - line_stop：/manual_start=True のみ
    - signal_stop：/manual_start=True または /sig_recog==1(GO)
- 異常系のWARNログ強化（現在地不明, ルート不整合など）
- /active_target は1Hzで保険再送

出力:
- /active_target : geometry_msgs/PoseStamped
- /follower_state: route_msgs/FollowerState（Phase2拡張）

購読:
- /active_route : route_msgs/Route（TRANSIENT_LOCAL）
- /amcl_pose    : geometry_msgs/PoseStamped
- /obstacle_avoidance_hint : route_msgs/ObstacleAvoidanceHint（BEST_EFFORT）
- /manual_start : std_msgs/Bool（Trueで解除）
- /sig_recog    : std_msgs/Int32（1=GO, 2=NOGO, 他=未定義）

実装方針: Google Python Style / 型ヒント / 日本語コメント。
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from statistics import median
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Header, Int32
from route_msgs.msg import Route, Waypoint, FollowerState, ObstacleAvoidanceHint  # type: ignore
from route_msgs.srv import ReportStuck  # type: ignore


# ===============================
# QoS ヘルパ
# ===============================

def qos_transient_local(depth: int = 1) -> QoSProfile:
    """RELIABLE / TRANSIENT_LOCAL."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_volatile(depth: int = 10) -> QoSProfile:
    """RELIABLE / VOLATILE."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_best_effort(depth: int = 5) -> QoSProfile:
    """BEST_EFFORT / VOLATILE."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ===============================
# 内部ユーティリティ
# ===============================

def yaw_from_quat(q: Quaternion) -> float:
    """Quaternion → yaw[rad]（Z軸回り）."""
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    """yaw[rad] → Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def euclid_xy(a: Pose, b: Pose) -> float:
    """Pose間の平面距離[m]を返す."""
    dx = float(a.position.x) - float(b.position.x)
    dy = float(a.position.y) - float(b.position.y)
    return math.hypot(dx, dy)


def yaw_between_points(x0: float, y0: float, x1: float, y1: float) -> float:
    """(x0,y0)→(x1,y1) の方向の yaw を返す."""
    return math.atan2(y1 - y0, x1 - x0)


@dataclass
class HintSample:
    """Hint の単票（キャッシュ保管用）."""
    t: float
    front_blocked: bool
    left_open: float
    right_open: float


class FollowerStatus(Enum):
    """状態列挙（Phase2）."""
    IDLE = auto()
    RUNNING = auto()
    WAITING_STOP = auto()
    STAGNATION_DETECTED = auto()
    AVOIDING = auto()
    WAITING_REROUTE = auto()
    FINISHED = auto()
    ERROR = auto()


class RouteFollower(Node):
    """route_follower（Phase2 正式版, L字回避必須）."""

    def __init__(self) -> None:
        super().__init__("route_follower")

        # ===== QoS =====
        self.qos_tl = qos_transient_local(1)
        self.qos_vol = qos_volatile(10)
        self.qos_be = qos_best_effort(5)

        # ===== パラメータ（Phase1互換＋Phase2追加） =====
        # 追従
        self.declare_parameter("arrival_threshold", 0.6)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("republish_target_hz", 1.0)  # /active_target 再送
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("start_immediately", True)
        self.declare_parameter("state_debounce_ms", 100)

        # 滞留検知
        self.declare_parameter("window_sec", 2.0)               # 進捗/速度評価窓
        self.declare_parameter("progress_epsilon_m", 0.10)      # 窓内距離改善の最小値
        self.declare_parameter("min_speed_mps", 0.05)           # 窓内平均速度の下限
        self.declare_parameter("stagnation_duration_sec", 15.0) # ← 指定により 15s

        # 回避行動
        self.declare_parameter("avoid_min_offset_m", 0.35)
        self.declare_parameter("avoid_max_offset_m", 5.0)       # ← 指定により 5.0m
        self.declare_parameter("avoid_use_l_shape", True)       # Phase2では必須。False指定時は強制的にL字化。
        self.declare_parameter("avoid_forward_clearance_m", 2.0)
        self.declare_parameter("max_avoidance_attempts_per_wp", 2)

        # 再経路待機
        self.declare_parameter("reroute_timeout_sec", 30.0)

        # Hint キャッシュ
        self.declare_parameter("hint_cache_window_sec", 5.0)
        self.declare_parameter("hint_majority_true_ratio", 0.8)
        self.declare_parameter("hint_min_samples", 5)

        # ===== 内部状態 =====
        self._route: Optional[Route] = None
        self._wp_list: List[Waypoint] = []
        self._index: int = 0
        self._route_version: int = -1

        self._status: FollowerStatus = FollowerStatus.IDLE

        self._pending_route: Optional[Route] = None
        self._route_rejected: bool = False

        self._current_pose_st: Optional[PoseStamped] = None
        self._last_pub_target: Optional[PoseStamped] = None
        self._last_pub_time: float = 0.0

        # signal_stop 解除用
        self._manual_start_true: bool = False
        self._sig_recog_last: int = 0  # 1=GO, 2=NOGO, 他=未定義

        # 滞留検知用（過去位置バッファ）
        self._pose_hist: Deque[Tuple[float, Pose]] = deque()
        self._stagnation_hold_start: Optional[float] = None  # 条件成立継続の開始時刻

        # 回避制御（deque化）
        self._avoid_active: bool = False
        self._avoid_subgoals: Deque[Tuple[str, Pose]] = deque()  # (label, pose)
        self._avoid_attempt_count: int = 0

        # WAITING_REROUTE タイマ
        self._reroute_wait_start: Optional[float] = None

        # Hint キャッシュ
        self._hint_cache: Deque[HintSample] = deque()

        # follower_state デバウンス
        self._last_state_pub_time: float = 0.0
        self._state_debounce_s: float = self.get_parameter("state_debounce_ms").value / 1000.0

        # WARNデバウンス
        self._last_pose_warn_time: float = 0.0
        self._warn_interval_s: float = 2.0

        # ===== Pub/Sub/Service =====
        self.sub_route = self.create_subscription(Route, "/active_route", self._on_route, self.qos_tl)
        self.sub_pose = self.create_subscription(PoseStamped, "/amcl_pose", self._on_pose, self.qos_vol)
        self.sub_hint = self.create_subscription(
            ObstacleAvoidanceHint, "/obstacle_avoidance_hint", self._on_hint, self.qos_be
        )
        self.sub_manual = self.create_subscription(Bool, "/manual_start", self._on_manual_start, self.qos_vol)
        self.sub_sig = self.create_subscription(Int32, "/sig_recog", self._on_sig_recog, self.qos_vol)

        # /active_target は RELIABLE / VOLATILE
        self.pub_target = self.create_publisher(PoseStamped, "/active_target", self.qos_vol)
        self.pub_state = self.create_publisher(FollowerState, "/follower_state", self.qos_vol)

        self.cli_report_stuck = self.create_client(ReportStuck, "/report_stuck")

        # ===== Timer =====
        period = 1.0 / max(self.get_parameter("control_rate_hz").value, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info("route_follower (Phase2 L-shape) started.")

    # ====================== Subscriber callbacks ======================

    def _on_route(self, msg: Route) -> None:
        """新しい /active_route を受信（TRANSIENT_LOCAL）."""
        if not msg.waypoints:
            self.get_logger().warn("/active_route: 空の route を受信。無視します。")
            return

        target_frame: str = self.get_parameter("target_frame").value
        if not msg.header.frame_id:
            msg.header.frame_id = target_frame
        elif msg.header.frame_id != target_frame:
            self.get_logger().error(
                f"/active_route: frame mismatch '{msg.header.frame_id}' != '{target_frame}'"
            )
            self._route_rejected = True
            return
        self._pending_route = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        """現在自己位置（map基準想定）."""
        self._current_pose_st = msg
        # 過去位置ヒストリへ追記
        now = self.get_clock().now().nanoseconds / 1e9
        self._pose_hist.append((now, msg.pose))
        # 古いサンプルを窓長の2〜3倍程度で掃除
        while self._pose_hist and (now - self._pose_hist[0][0]) > max(
            3.0 * float(self.get_parameter("window_sec").value), 10.0
        ):
            self._pose_hist.popleft()

    def _on_hint(self, msg: ObstacleAvoidanceHint) -> None:
        """Hint をキャッシュ（過去 hint_cache_window_sec 秒を保持）."""
        t = self.get_clock().now().nanoseconds / 1e9
        self._hint_cache.append(
            HintSample(
                t=t,
                front_blocked=bool(msg.front_blocked),
                left_open=float(msg.left_is_open),
                right_open=float(msg.right_is_open),
            )
        )
        # 古いサンプルを削除
        window = float(self.get_parameter("hint_cache_window_sec").value)
        while self._hint_cache and (t - self._hint_cache[0].t) > window:
            self._hint_cache.popleft()

    def _on_manual_start(self, msg: Bool) -> None:
        if msg.data:
            self._manual_start_true = True

    def _on_sig_recog(self, msg: Int32) -> None:
        self._sig_recog_last = int(msg.data)

    # ====================== Timer (control loop) ======================

    def _on_timer(self) -> None:
        """周期制御処理（状態遷移含む）."""
        # 0) route frame mismatch エラー
        if self._route_rejected:
            self._route_rejected = False
            self._change_state(FollowerStatus.ERROR)

        # 1) pending route → 適用
        if self._pending_route is not None:
            new = self._pending_route
            self._pending_route = None

            self._route = new
            self._wp_list = list(new.waypoints)
            self._index = 0
            self._avoid_attempt_count = 0
            self._avoid_active = False
            self._avoid_subgoals.clear()
            self._reroute_wait_start = None
            try:
                self._route_version = int(getattr(new, "version", -1))
            except Exception:
                self._route_version = -1

            if self.get_parameter("start_immediately").value:
                if not self._wp_list:
                    self.get_logger().warn("start_immediately=True だが waypoints が空です。")
                    self._change_state(FollowerStatus.ERROR)
                else:
                    self._change_state(FollowerStatus.RUNNING)
                    self._publish_target_pose(self._wp_list[0].pose)
            else:
                self._change_state(FollowerStatus.IDLE)

        # 2) WAITING_STOP の解除条件（signal_stop 時のみ sig_recog を許可）
        if self._status == FollowerStatus.WAITING_STOP and self._wp_list:
            cur_wp = self._wp_list[self._index]
            if getattr(cur_wp, "signal_stop", False):
                can_resume = self._manual_start_true or (self._sig_recog_last == 1)
            else:
                can_resume = self._manual_start_true  # line_stop の場合
            if can_resume:
                self._manual_start_true = False
                if self._index < len(self._wp_list) - 1:
                    self._index += 1
                    self._avoid_attempt_count = 0
                    self._avoid_active = False
                    self._avoid_subgoals.clear()
                    self._publish_target_pose(self._wp_list[self._index].pose)
                    self._change_state(FollowerStatus.RUNNING)
                else:
                    self._change_state(FollowerStatus.FINISHED)

        # 3) WAITING_REROUTE のタイムアウト
        if self._status == FollowerStatus.WAITING_REROUTE and self._reroute_wait_start is not None:
            elapsed = time.time() - self._reroute_wait_start
            if elapsed > float(self.get_parameter("reroute_timeout_sec").value):
                self.get_logger().warn("WAITING_REROUTE timeout. Transition to ERROR.")
                self._change_state(FollowerStatus.ERROR)

        # 4) RUNNING / AVOIDING 動作
        if self._status in (FollowerStatus.RUNNING, FollowerStatus.AVOIDING):
            # ルート整合チェック
            if self._route is None or not self._wp_list:
                self.get_logger().warn("RUNNINGだが /active_route が未設定または空です。")
                self._publish_state()
                return
            if not (0 <= self._index < len(self._wp_list)):
                self.get_logger().warn(f"RUNNINGだが current_index={self._index} が不正です。")
                self._publish_state()
                return

            if self._current_pose_st is None:
                now = time.time()
                if (now - self._last_pose_warn_time) > self._warn_interval_s:
                    self._last_pose_warn_time = now
                    self.get_logger().warn(
                        "Current pose unavailable (/amcl_pose not received). "
                        "Skipping waypoint tracking and only resending target."
                    )
                self._resend_active_target_if_needed()
                self._publish_state()
                return

            cur_wp = self._wp_list[self._index]
            cur_pose = self._current_pose_st.pose
            threshold = float(self.get_parameter("arrival_threshold").value)

            # ===== AVOIDING: deque の先頭サブ目標を消化 =====
            if self._avoid_active and self._avoid_subgoals:
                label, pose = self._avoid_subgoals[0]
                if euclid_xy(cur_pose, pose) < threshold:
                    self.get_logger().info(f"Avoidance sub-goal reached: {label}")
                    self._avoid_subgoals.popleft()
                    if self._avoid_subgoals:
                        next_label, next_pose = self._avoid_subgoals[0]
                        self.get_logger().info(f"Proceed to next avoidance step: {next_label}")
                        self._publish_target_pose(next_pose)
                    else:
                        # 全サブ目標完了 → 本来WPへ復帰
                        self._avoid_active = False
                        self._publish_target_pose(cur_wp.pose)
                        self.get_logger().info("Avoidance sequence completed. Back to main waypoint.")
                else:
                    self._resend_active_target_if_needed()
                    # 回避中も滞留監視（成立→avoidance_failed）
                    if self._check_stagnation_tick(exclude_stop=False):
                        self.get_logger().warn("Re-stagnation during avoidance. Report as 'avoidance_failed'.")
                        self._avoid_subgoals.clear()
                        self._avoid_active = False
                        self._avoid_attempt_count = min(
                            self._avoid_attempt_count + 1,
                            int(self.get_parameter("max_avoidance_attempts_per_wp").value),
                        )
                        self._report_stuck_and_wait(decision_reason="avoidance_failed")
                self._publish_state()
                return

            # ===== RUNNING: 本来 waypoint への到達評価 =====
            if euclid_xy(cur_pose, cur_wp.pose) < threshold:
                # line_stop / signal_stop のハンドリング
                if getattr(cur_wp, "line_stop", False) or getattr(cur_wp, "signal_stop", False):
                    which = "line_stop" if getattr(cur_wp, "line_stop", False) else "signal_stop"
                    self.get_logger().info(f"Reached STOP waypoint ({which}). WAITING_STOP.")
                    self._change_state(FollowerStatus.WAITING_STOP)
                elif self._index < len(self._wp_list) - 1:
                    self._index += 1
                    self._avoid_attempt_count = 0
                    self._avoid_active = False
                    self._avoid_subgoals.clear()
                    next_wp = self._wp_list[self._index]
                    self.get_logger().info(
                        f"Proceed to next waypoint. index={self._index}, label='{next_wp.label}'"
                    )
                    self._publish_target_pose(next_wp.pose)
                else:
                    self.get_logger().info("Final waypoint reached. Follower finished.")
                    self._change_state(FollowerStatus.FINISHED)
            else:
                # 到達していない → 目標再送
                self._resend_active_target_if_needed()

                # STOP 除外を考慮しつつ滞留判定
                exclude_stop = getattr(cur_wp, "line_stop", False) or getattr(cur_wp, "signal_stop", False)
                if self._check_stagnation_tick(exclude_stop=exclude_stop):
                    # 滞留成立 → STAGNATION_DETECTED
                    self._change_state(FollowerStatus.STAGNATION_DETECTED)
                    # 滞留後の状態把握（front_blocked 多数決）
                    fb_major, med_l, med_r, enough = self._evaluate_hint_cache()
                    if not enough:
                        self.get_logger().warn("Insufficient hint samples. Reporting 'unknown_stuck'.")
                        self._report_stuck_and_wait(decision_reason="unknown_stuck")
                    elif not fb_major:
                        self.get_logger().warn("front_blocked majority = False. Reporting 'unknown_stuck'.")
                        self._report_stuck_and_wait(decision_reason="unknown_stuck")
                    else:
                        # 前方閉塞 True → 回避可否を評価（L字必須）
                        success = self._start_avoidance_sequence(cur_wp, cur_pose, med_l, med_r)
                        if not success:
                            self.get_logger().warn("No feasible lateral space. Reporting 'unknown_stuck'.")
                            self._report_stuck_and_wait(decision_reason="unknown_stuck")
                        else:
                            self._change_state(FollowerStatus.AVOIDING)

        self._publish_state()

    # ====================== 追従・滞留・回避ロジック ======================

    def _check_stagnation_tick(self, exclude_stop: bool) -> bool:
        """滞留検知の逐次評価.

        判定条件:
          - 直近 window_sec の移動距離 < progress_epsilon_m
          - 直近 window_sec の平均速度 < min_speed_mps
          - 上記の状態が stagnation_duration_sec 連続で成立
          - exclude_stop=True の場合は判定しない（STOP手前での誤発火防止）
        """
        if exclude_stop:
            self._stagnation_hold_start = None
            return False
        if not self._pose_hist:
            self._stagnation_hold_start = None
            return False

        window = float(self.get_parameter("window_sec").value)
        eps = float(self.get_parameter("progress_epsilon_m").value)
        min_v = float(self.get_parameter("min_speed_mps").value)
        need = float(self.get_parameter("stagnation_duration_sec").value)

        now = self.get_clock().now().nanoseconds / 1e9

        # 窓内の最古と最新を抽出
        recent: List[Tuple[float, Pose]] = []
        for t, p in reversed(self._pose_hist):
            if (now - t) <= window:
                recent.append((t, p))
            else:
                break
        if len(recent) < 2:
            # サンプル不足
            self._stagnation_hold_start = None
            return False

        t_old, p_old = recent[-1]  # 窓内で最古
        t_new, p_new = recent[0]   # 最新
        dt = max(t_new - t_old, 1e-6)
        dist = euclid_xy(p_new, p_old)
        speed = dist / dt

        cond = (dist < eps) and (speed < min_v)
        if cond:
            if self._stagnation_hold_start is None:
                self._stagnation_hold_start = now
            if (now - self._stagnation_hold_start) >= need:
                self._stagnation_hold_start = None
                self.get_logger().info(
                    f"Stagnation detected: dist={dist:.2f}m, speed={speed:.2f}m/s, window={window}s"
                )
                return True
        else:
            self._stagnation_hold_start = None

        return False

    def _evaluate_hint_cache(self) -> Tuple[bool, float, float, bool]:
        """Hint キャッシュを多数決・中央値で評価する.

        Returns:
            (front_blocked_majority, median_left_open, median_right_open, enough_samples)
        """
        window = float(self.get_parameter("hint_cache_window_sec").value)
        ratio_th = float(self.get_parameter("hint_majority_true_ratio").value)
        min_samples = int(self.get_parameter("hint_min_samples").value)

        now = self.get_clock().now().nanoseconds / 1e9
        # 窓内サンプル抽出
        samples = [s for s in self._hint_cache if (now - s.t) <= window]
        if len(samples) < min_samples:
            return False, 0.0, 0.0, False

        true_count = sum(1 for s in samples if s.front_blocked)
        fb_major = (true_count / len(samples)) >= ratio_th

        # front_blocked=Trueのサンプルのみから中央値
        fb_samples = [s for s in samples if s.front_blocked]
        if not fb_samples:
            return fb_major, 0.0, 0.0, True

        med_l = float(median([s.left_open for s in fb_samples]))
        med_r = float(median([s.right_open for s in fb_samples]))
        return fb_major, med_l, med_r, True

    def _start_avoidance_sequence(self, wp: Waypoint, cur_pose: Pose, med_l: float, med_r: float) -> bool:
        """左右の可用幅とWaypoint上限から L字サブ目標列（2点）を生成し開始する."""
        min_off = float(self.get_parameter("avoid_min_offset_m").value)
        max_off_global = float(self.get_parameter("avoid_max_offset_m").value)

        # Waypoint の上限（0 or 負値なら 0 とみなす）
        left_limit = max(float(getattr(wp, "left_open", 0.0)), 0.0)
        right_limit = max(float(getattr(wp, "right_open", 0.0)), 0.0)

        # 実効上限
        left_max = min(left_limit, max_off_global)
        right_max = min(right_limit, max_off_global)

        def pick_offset(hint_open: float, side_max: float) -> float:
            if hint_open <= 0.0 or side_max <= 0.0:
                return 0.0
            allowed = min(hint_open, side_max)
            return max(min_off, min(allowed, side_max))

        off_L = pick_offset(med_l, left_max)
        off_R = pick_offset(med_r, right_max)

        options: List[Tuple[str, float]] = []
        if off_L > 0.0:
            options.append(("L", off_L))
        if off_R > 0.0:
            options.append(("R", off_R))
        if not options:
            return False

        # 「最小オフセット」を優先（同値なら左優先）
        options.sort(key=lambda x: (x[1], 0 if x[0] == "L" else 1))
        side, offset = options[0]

        # Phase2要件：常にL字（二段階）。avoid_use_l_shape=False が来ても強制L字。
        if not self.get_parameter("avoid_use_l_shape").value:
            self.get_logger().warn("avoid_use_l_shape=False はPhase2では無効。L字回避を強制します。")

        forward = float(self.get_parameter("avoid_forward_clearance_m").value)

        # (1) 横シフト点（cur_pose → p1）。向き＝この線分の方向。
        # base_linkの+Yが左, -Yが右。yaw は不要、方向ベクトルから算出する。
        yaw_cur = yaw_from_quat(cur_pose.orientation)
        offset_y = +offset if side == "L" else -offset
        dx1 = -math.sin(yaw_cur) * offset_y
        dy1 =  math.cos(yaw_cur) * offset_y
        p1 = Pose()
        p1.position.x = float(cur_pose.position.x) + dx1
        p1.position.y = float(cur_pose.position.y) + dy1
        p1.position.z = float(cur_pose.position.z)
        yaw1 = yaw_between_points(float(cur_pose.position.x), float(cur_pose.position.y),
                                  float(p1.position.x), float(p1.position.y))
        p1.orientation = quat_from_yaw(yaw1)

        # (2) 前進点（p1 → p2）。向き＝この線分（基本はyaw_cur）。
        dx2 = math.cos(yaw_cur) * forward
        dy2 = math.sin(yaw_cur) * forward
        p2 = Pose()
        p2.position.x = float(p1.position.x) + dx2
        p2.position.y = float(p1.position.y) + dy2
        p2.position.z = float(p1.position.z)
        yaw2 = yaw_between_points(float(p1.position.x), float(p1.position.y),
                                  float(p2.position.x), float(p2.position.y))
        p2.orientation = quat_from_yaw(yaw2)

        # deque へ登録し開始
        self._avoid_subgoals.clear()
        self._avoid_subgoals.append((f"avoid_shift_{side}", p1))
        self._avoid_subgoals.append((f"avoid_forward_{side}", p2))
        self._avoid_active = True
        self._avoid_attempt_count = min(
            self._avoid_attempt_count + 1,
            int(self.get_parameter("max_avoidance_attempts_per_wp").value),
        )
        first_label, first_pose = self._avoid_subgoals[0]
        self.get_logger().info(f"Start avoidance sequence: {first_label} "
                               f"(steps={len(self._avoid_subgoals)}, side={side}, offset={offset:.2f}m)")
        self._publish_target_pose(first_pose)
        return True

    def _report_stuck_and_wait(self, decision_reason: str) -> None:
        """report_stuck を同期呼び出しし、decision に応じて遷移する."""
        if not self.cli_report_stuck.service_is_ready():
            self.get_logger().warn("/report_stuck service not ready.")
            self._change_state(FollowerStatus.WAITING_REROUTE)
            self._reroute_wait_start = time.time()
            return

        req = ReportStuck.Request()
        req.route_version = int(self._route_version)
        req.current_index = int(self._index)
        if self._current_pose_st is not None:
            req.current_pose_map = self._current_pose_st.pose
        else:
            req.current_pose_map = Pose()
        req.reason = str(decision_reason)  # "avoidance_failed" | "unknown_stuck"

        timeout_s = float(self.get_parameter("reroute_timeout_sec").value)
        future = self.cli_report_stuck.call_async(req)
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (time.time() - start) > timeout_s:
                break

        if not future.done():
            self.get_logger().warn("report_stuck timeout. Enter WAITING_REROUTE.")
            self._change_state(FollowerStatus.WAITING_REROUTE)
            self._reroute_wait_start = time.time()
            return

        res: ReportStuck.Response = future.result()
        self.get_logger().info(f"report_stuck decision='{res.decision}', accepted={res.accepted}")

        if res.decision in ("replan", "skip"):
            self._change_state(FollowerStatus.WAITING_REROUTE)
            self._reroute_wait_start = time.time()
        elif res.decision == "failed":
            self._change_state(FollowerStatus.ERROR)
        else:
            self.get_logger().warn(f"Unknown decision: '{res.decision}'. Transition to ERROR.")
            self._change_state(FollowerStatus.ERROR)

    # ====================== Publish helpers ======================

    def _publish_target_pose(self, pose: Pose) -> None:
        """/active_target を発行（保険再送のため内部にキャッシュ）."""
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("target_frame").value)
        msg.pose = pose
        self.pub_target.publish(msg)
        self._last_pub_target = msg
        self._last_pub_time = time.time()

    def _resend_active_target_if_needed(self) -> None:
        """/active_target の保険再送（republish_target_hz）."""
        if self._last_pub_target is None:
            return
        interval = 1.0 / max(self.get_parameter("republish_target_hz").value, 1e-3)
        if (time.time() - self._last_pub_time) >= interval:
            self.pub_target.publish(self._last_pub_target)
            self._last_pub_time = time.time()

    def _change_state(self, new_state: FollowerStatus) -> None:
        if new_state != self._status:
            self.get_logger().info(f"state change: {self._status.name} -> {new_state.name}")
            self._status = new_state

    def _publish_state(self) -> None:
        """/follower_state をデバウンス付きで発行."""
        now = time.time()
        if (now - self._last_state_pub_time) < self._state_debounce_s:
            return
        self._last_state_pub_time = now

        msg = FollowerState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.route_version = int(self._route_version)
        msg.state = self._status.name
        msg.current_index = int(self._index)

        cur_label = ""
        next_label = ""
        if self._wp_list and 0 <= self._index < len(self._wp_list):
            cur_label = self._wp_list[self._index].label
            if self._index + 1 < len(self._wp_list):
                next_label = self._wp_list[self._index + 1].label
        msg.current_waypoint_label = cur_label
        msg.next_waypoint_label = next_label

        if self._current_pose_st:
            msg.current_pose = self._current_pose_st.pose
            if self._wp_list and 0 <= self._index < len(self._wp_list):
                msg.distance_to_target = float(euclid_xy(self._current_pose_st.pose, self._wp_list[self._index].pose))
            else:
                msg.distance_to_target = 0.0
        else:
            msg.current_pose = Pose()
            msg.distance_to_target = 0.0

        # Phase2 追加
        msg.avoidance_attempt_count = int(self._avoid_attempt_count)
        fb_major, med_l, med_r, enough = self._evaluate_hint_cache()
        msg.front_blocked_majority = bool(fb_major) if enough else False
        msg.hint_left_open_m_median = float(med_l if enough else 0.0)
        msg.hint_right_open_m_median = float(med_r if enough else 0.0)
        msg.last_stagnation_reason = ""

        self.pub_state.publish(msg)

    # ====================== main ======================


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()