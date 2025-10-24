#!/usr/bin/env python3
"""robot_console ダッシュボード用モックUI.

このスクリプトはROS2環境を必要とせず、tkinterのみでrobot_consoleの
ダッシュボード案を体験できるようにする。route_stateやfollower_stateの
主要項目、障害物ヒントの画像重畳、制御コマンド、ノード起動操作、
パッケージ別ログタブなど、フェーズ1で整理した要素をすべて一画面に
収めたレイアウトを再現する。
"""

from __future__ import annotations

import math
import random
import tkinter as tk
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from tkinter import ttk
from typing import Dict, List, Optional


def _now() -> datetime:
    """現在時刻を返すユーティリティ."""

    return datetime.now()


@dataclass
class RouteStateSnapshot:
    """route_stateに相当する情報を保持するデータクラス."""

    status: str
    current_index: int
    total_waypoints: int
    current_label: str
    route_version: int
    version_history: List[str] = field(default_factory=list)

    @property
    def progress_ratio(self) -> float:
        """現在地の進捗割合を0.0〜1.0で返す."""

        if self.total_waypoints <= 0:
            return 0.0
        return min(self.current_index / max(self.total_waypoints, 1), 1.0)


@dataclass
class FollowerStateSnapshot:
    """follower_stateに相当する情報を保持するデータクラス."""

    state: str
    current_index: int
    route_version: int
    avoidance_attempt_count: int
    last_stagnation_reason: str
    front_blocked_majority: bool
    left_offset_m_median: float
    right_offset_m_median: float
    current_waypoint_label: str
    next_waypoint_label: str
    signal_stop_active: bool


@dataclass
class ObstacleHintSnapshot:
    """obstacle_avoidance_hintの主要値を保持するデータクラス."""

    front_clearance_m: float
    last_updated: datetime


@dataclass
class ManualSignalSnapshot:
    """手動介入や信号系トピックのラッチ情報."""

    manual_start: bool
    manual_sent_at: Optional[datetime]
    sig_recog: int
    sig_sent_at: Optional[datetime]
    road_blocked: bool
    road_sent_at: Optional[datetime]


@dataclass
class ActiveTargetSnapshot:
    """active_targetとamcl_poseの距離メトリクス."""

    current_distance_m: float
    reference_distance_m: float

    @property
    def ratio(self) -> float:
        """距離をゲージ表示用の割合に変換する."""

        if self.reference_distance_m <= 0.0:
            return 0.0
        return max(min(self.current_distance_m / self.reference_distance_m, 1.0), 0.0)


@dataclass
class CmdVelSnapshot:
    """cmd_velに相当する並進・角速度の疑似値."""

    linear_x: float
    angular_z: float


@dataclass
class ManagerStatusSnapshot:
    """manager_statusと関連イベントの抜粋."""

    state: str
    last_cause: str
    last_transition: datetime


@dataclass
class NodeLaunchStatus:
    """ノード起動UIの状態管理データ."""

    running: bool
    parameter_file: str
    simulator_enabled: bool
    last_action: Optional[datetime]


class MockDataProvider:
    """ダッシュボードに供給する疑似データを生成するクラス."""

    STATUS_CHOICES = [
        "IDLE",
        "WAITING_ROUTE",
        "RUNNING",
        "PAUSED",
        "STUCK_RECOVERY",
    ]

    FOLLOWER_CHOICES = [
        "INITIALIZING",
        "WAITING_START",
        "FOLLOWING",
        "WAITING_STOP",
        "RECOVERING",
    ]

    STAGNATION_REASONS = [
        "front_blocked_majority",
        "waiting_manual_start",
        "no_progress_detected",
        "planner_timeout",
    ]

    ROUTE_LABELS = [f"WP{i:02d}" for i in range(1, 25)]

    PARAMETER_CANDIDATES = [
        "default.yaml",
        "route_a.yaml",
        "dense_city.yaml",
    ]

    SIMULATOR_MAP: Dict[str, str] = {
        "obstacle_monitor": "laser_scan_simulator",
        "robot_navigator": "robot_simulator",
    }

    def __init__(self) -> None:
        now = _now()
        self.route_state = RouteStateSnapshot(
            status="RUNNING",
            current_index=3,
            total_waypoints=20,
            current_label="WP03",
            route_version=1,
            version_history=["v1: 初期ルート"],
        )
        self.follower_state = FollowerStateSnapshot(
            state="FOLLOWING",
            current_index=2,
            route_version=1,
            avoidance_attempt_count=0,
            last_stagnation_reason="front_blocked_majority",
            front_blocked_majority=False,
            left_offset_m_median=0.0,
            right_offset_m_median=0.0,
            current_waypoint_label="WP02",
            next_waypoint_label="WP03",
            signal_stop_active=False,
        )
        self.obstacle_hint = ObstacleHintSnapshot(
            front_clearance_m=5.0,
            last_updated=now,
        )
        self.manager_status = ManagerStatusSnapshot(
            state="OPERATING",
            last_cause="initial_route",
            last_transition=now,
        )
        self.manual_signal = ManualSignalSnapshot(
            manual_start=False,
            manual_sent_at=None,
            sig_recog=0,
            sig_sent_at=None,
            road_blocked=False,
            road_sent_at=None,
        )
        self.obstacle_override_active = False
        self.obstacle_override_settings = {
            "front_blocked": False,
            "clearance_m": 3.0,
            "left_offset_m": 0.0,
            "right_offset_m": 0.0,
        }
        self.active_target = ActiveTargetSnapshot(
            current_distance_m=18.0,
            reference_distance_m=22.0,
        )
        self.cmd_vel = CmdVelSnapshot(linear_x=0.0, angular_z=0.0)
        self.camera_mode = "normal"
        self.image_timestamps: Dict[str, datetime] = {
            "route_map": now,
            "sensor_viewer": now,
            "camera": now,
        }
        self.node_status: Dict[str, NodeLaunchStatus] = {
            "route_manager": NodeLaunchStatus(False, "default.yaml", False, None),
            "route_follower": NodeLaunchStatus(False, "default.yaml", False, None),
            "obstacle_monitor": NodeLaunchStatus(False, "default.yaml", False, None),
            "robot_navigator": NodeLaunchStatus(False, "default.yaml", False, None),
        }
        self.pending_logs: Dict[str, List[str]] = {
            "route_manager": [],
            "route_follower": [],
            "obstacle_monitor": [],
            "robot_navigator": [],
        }
        self._last_tick = now

    # ------------------------------------------------------------------
    # データ生成
    # ------------------------------------------------------------------
    def simulate_tick(self) -> None:
        """疑似的に状態を変化させる."""

        now = _now()
        if (now - self._last_tick) < timedelta(seconds=1):
            return
        self._last_tick = now

        # route_stateの進捗と再計画イベント
        if self.route_state.current_index < self.route_state.total_waypoints:
            self.route_state.current_index += 1
            self.route_state.current_label = self.ROUTE_LABELS[
                min(self.route_state.current_index, len(self.ROUTE_LABELS)) - 1
            ]
        else:
            self.route_state.current_index = 0
            self.route_state.route_version += 1
            self.route_state.version_history.append(
                f"v{self.route_state.route_version}: 再計画 {now.strftime('%H:%M:%S')}"
            )
            self.route_state.version_history = self.route_state.version_history[-3:]

        # manager_statusの状態はランダムに変化
        if random.random() < 0.2:
            self.manager_status.state = random.choice(self.STATUS_CHOICES)
            self.manager_status.last_cause = random.choice(
                ["manual_override", "route_update", "planner_timeout"]
            )
            self.manager_status.last_transition = now

        # follower_stateの更新
        follower_target = max(self.route_state.current_index - random.randint(0, 1), 0)
        self.follower_state.current_index = follower_target
        self.follower_state.state = random.choice(self.FOLLOWER_CHOICES)
        self.follower_state.route_version = self.route_state.route_version
        self.follower_state.avoidance_attempt_count = min(
            self.follower_state.avoidance_attempt_count + random.randint(0, 1), 5
        )
        self.follower_state.last_stagnation_reason = random.choice(
            self.STAGNATION_REASONS
        )
        if self.obstacle_override_active:
            self.follower_state.front_blocked_majority = self.obstacle_override_settings[
                "front_blocked"
            ]
            self.follower_state.left_offset_m_median = self.obstacle_override_settings[
                "left_offset_m"
            ]
            self.follower_state.right_offset_m_median = self.obstacle_override_settings[
                "right_offset_m"
            ]
        else:
            self.follower_state.front_blocked_majority = random.random() < 0.3
            self.follower_state.left_offset_m_median = random.uniform(-0.8, 0.8)
            self.follower_state.right_offset_m_median = random.uniform(-0.8, 0.8)
        self.follower_state.current_waypoint_label = self.route_state.current_label
        next_index = min(
            self.route_state.current_index + 1, self.route_state.total_waypoints
        )
        self.follower_state.next_waypoint_label = self.ROUTE_LABELS[
            min(next_index, len(self.ROUTE_LABELS)) - 1
        ]
        self.follower_state.signal_stop_active = (
            self.follower_state.state == "WAITING_STOP"
        )

        # 障害物ヒントと画像タイムスタンプ
        if self.obstacle_override_active:
            self.obstacle_hint.front_clearance_m = self.obstacle_override_settings[
                "clearance_m"
            ]
        else:
            self.obstacle_hint.front_clearance_m = max(
                round(random.uniform(0.5, 6.0), 2), 0.2
            )
        self.obstacle_hint.last_updated = now
        self.image_timestamps["route_map"] = now
        self.image_timestamps["sensor_viewer"] = now
        self.image_timestamps["camera"] = now

        # active_target距離の更新
        decay = random.uniform(0.3, 1.0)
        self.active_target.current_distance_m = max(
            self.active_target.current_distance_m - decay, 0.0
        )
        if self.active_target.current_distance_m <= 0.5:
            self.active_target.reference_distance_m = random.uniform(10.0, 25.0)
            self.active_target.current_distance_m = self.active_target.reference_distance_m

        # cmd_velの疑似値
        self.cmd_vel.linear_x = round(random.uniform(-0.2, 1.4), 2)
        self.cmd_vel.angular_z = round(random.uniform(-1.2, 1.2), 2)

        # manual_startはワンショットでFalseへ戻す
        if self.manual_signal.manual_start:
            self.manual_signal.manual_start = False

        # カメラモード切替
        self.camera_mode = "signal" if self.follower_state.signal_stop_active else "normal"

        # 起動中ノードの疑似ログ
        for package, status in self.node_status.items():
            if status.running and random.random() < 0.4:
                message = f"[{now.strftime('%H:%M:%S')}] heartbeat OK"
                self.pending_logs[package].append(message)

    # ------------------------------------------------------------------
    # 操作メソッド
    # ------------------------------------------------------------------
    def send_manual_start(self) -> None:
        now = _now()
        self.manual_signal.manual_start = True
        self.manual_signal.manual_sent_at = now
        self.pending_logs["route_follower"].append(
            f"[{now.strftime('%H:%M:%S')}] manual_start=True を送信しました"
        )

    def send_sig_recog(self, value: int) -> None:
        now = _now()
        self.manual_signal.sig_recog = value
        self.manual_signal.sig_sent_at = now
        self.pending_logs["route_follower"].append(
            f"[{now.strftime('%H:%M:%S')}] sig_recog={value} を送信しました"
        )

    def toggle_road_blocked(self) -> None:
        now = _now()
        self.manual_signal.road_blocked = not self.manual_signal.road_blocked
        self.manual_signal.road_sent_at = now
        state = "True" if self.manual_signal.road_blocked else "False"
        self.pending_logs["route_manager"].append(
            f"[{now.strftime('%H:%M:%S')}] road_blocked={state} を送信しました"
        )

    def toggle_obstacle_hint_override(self, enable: bool) -> None:
        now = _now()
        self.obstacle_override_active = enable
        verb = "開始" if enable else "停止"
        self.pending_logs["obstacle_monitor"].append(
            f"[{now.strftime('%H:%M:%S')}] GUIモックがヒント固定値送出を{verb}"
        )
        if enable:
            self._apply_obstacle_override_to_state()

    def update_obstacle_hint_override(
        self,
        *,
        front_blocked: bool,
        clearance_m: float,
        left_offset_m: float,
        right_offset_m: float,
    ) -> None:
        self.obstacle_override_settings.update(
            {
                "front_blocked": front_blocked,
                "clearance_m": clearance_m,
                "left_offset_m": left_offset_m,
                "right_offset_m": right_offset_m,
            }
        )
        if self.obstacle_override_active:
            self._apply_obstacle_override_to_state()
            now = _now()
            self.pending_logs["obstacle_monitor"].append(
                (
                    f"[{now.strftime('%H:%M:%S')}] ヒント固定値を更新: "
                    f"blocked={front_blocked} clearance={clearance_m:.2f}m "
                    f"left={left_offset_m:+.2f}m right={right_offset_m:+.2f}m"
                )
            )

    def _apply_obstacle_override_to_state(self) -> None:
        """固定値送出が有効なときに状態へ即反映する."""

        self.follower_state.front_blocked_majority = self.obstacle_override_settings[
            "front_blocked"
        ]
        self.follower_state.left_offset_m_median = self.obstacle_override_settings[
            "left_offset_m"
        ]
        self.follower_state.right_offset_m_median = self.obstacle_override_settings[
            "right_offset_m"
        ]
        self.obstacle_hint.front_clearance_m = self.obstacle_override_settings[
            "clearance_m"
        ]
        self.obstacle_hint.last_updated = _now()

    def start_node(
        self, package: str, parameter_file: str, simulator_enabled: bool
    ) -> None:
        now = _now()
        status = self.node_status[package]
        status.running = True
        status.parameter_file = parameter_file
        sim_name = self.SIMULATOR_MAP.get(package)
        simulator_flag = simulator_enabled and sim_name is not None
        status.simulator_enabled = simulator_flag
        status.last_action = now
        sim_text = f" with {sim_name}" if simulator_flag and sim_name else ""
        self.pending_logs[package].append(
            f"[{now.strftime('%H:%M:%S')}] 起動 {parameter_file}{sim_text}"
        )

    def stop_node(self, package: str) -> None:
        now = _now()
        status = self.node_status[package]
        status.running = False
        status.simulator_enabled = False
        status.last_action = now
        self.pending_logs[package].append(
            f"[{now.strftime('%H:%M:%S')}] 停止要求を受理しました"
        )

    # ------------------------------------------------------------------
    # 取得メソッド
    # ------------------------------------------------------------------
    def snapshot(self) -> Dict[str, object]:
        """UI更新用のスナップショットを返す."""

        return {
            "route_state": RouteStateSnapshot(
                status=self.route_state.status,
                current_index=self.route_state.current_index,
                total_waypoints=self.route_state.total_waypoints,
                current_label=self.route_state.current_label,
                route_version=self.route_state.route_version,
                version_history=list(self.route_state.version_history),
            ),
            "follower_state": FollowerStateSnapshot(
                state=self.follower_state.state,
                current_index=self.follower_state.current_index,
                route_version=self.follower_state.route_version,
                avoidance_attempt_count=self.follower_state.avoidance_attempt_count,
                last_stagnation_reason=self.follower_state.last_stagnation_reason,
                front_blocked_majority=self.follower_state.front_blocked_majority,
                left_offset_m_median=self.follower_state.left_offset_m_median,
                right_offset_m_median=self.follower_state.right_offset_m_median,
                current_waypoint_label=self.follower_state.current_waypoint_label,
                next_waypoint_label=self.follower_state.next_waypoint_label,
                signal_stop_active=self.follower_state.signal_stop_active,
            ),
            "obstacle_hint": ObstacleHintSnapshot(
                front_clearance_m=self.obstacle_hint.front_clearance_m,
                last_updated=self.obstacle_hint.last_updated,
            ),
            "manager_status": ManagerStatusSnapshot(
                state=self.manager_status.state,
                last_cause=self.manager_status.last_cause,
                last_transition=self.manager_status.last_transition,
            ),
            "manual_signal": ManualSignalSnapshot(
                manual_start=self.manual_signal.manual_start,
                manual_sent_at=self.manual_signal.manual_sent_at,
                sig_recog=self.manual_signal.sig_recog,
                sig_sent_at=self.manual_signal.sig_sent_at,
                road_blocked=self.manual_signal.road_blocked,
                road_sent_at=self.manual_signal.road_sent_at,
            ),
            "active_target": ActiveTargetSnapshot(
                current_distance_m=self.active_target.current_distance_m,
                reference_distance_m=self.active_target.reference_distance_m,
            ),
            "cmd_vel": CmdVelSnapshot(
                linear_x=self.cmd_vel.linear_x,
                angular_z=self.cmd_vel.angular_z,
            ),
            "camera_mode": self.camera_mode,
            "image_timestamps": dict(self.image_timestamps),
            "node_status": {
                key: NodeLaunchStatus(
                    running=value.running,
                    parameter_file=value.parameter_file,
                    simulator_enabled=value.simulator_enabled,
                    last_action=value.last_action,
                )
                for key, value in self.node_status.items()
            },
            "obstacle_override": {
                "active": self.obstacle_override_active,
                "front_blocked": self.obstacle_override_settings["front_blocked"],
                "clearance_m": self.obstacle_override_settings["clearance_m"],
                "left_offset_m": self.obstacle_override_settings["left_offset_m"],
                "right_offset_m": self.obstacle_override_settings["right_offset_m"],
            },
        }

    def consume_logs(self) -> Dict[str, List[str]]:
        """ログバッファを取り出してクリアする."""

        logs = {pkg: entries[:] for pkg, entries in self.pending_logs.items()}
        for pkg in self.pending_logs:
            self.pending_logs[pkg].clear()
        return logs


# ----------------------------------------------------------------------
# UI部品クラス
# ----------------------------------------------------------------------
class ImagePane(ttk.LabelFrame):
    """アスペクト比とレターボックスを考慮した画像プレースホルダー."""

    def __init__(
        self,
        master: tk.Widget,
        title: str,
        *,
        width: int = 320,
        height: int = 220,
        content_ratio: Optional[float] = None,
    ) -> None:
        super().__init__(master, text=title, padding=(8, 6))
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.content_ratio = content_ratio
        self.canvas = tk.Canvas(
            self,
            width=width,
            height=height,
            highlightthickness=0,
            bg="#1f1f1f",
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.background_id = self.canvas.create_rectangle(
            0, 0, width, height, fill="#1f1f1f", outline=""
        )
        self.content_id = self.canvas.create_rectangle(
            0, 0, width, height, fill="#2c3e50", outline=""
        )
        self.letterbox_ids = (
            self.canvas.create_rectangle(0, 0, 0, 0, fill="#000000", outline=""),
            self.canvas.create_rectangle(0, 0, 0, 0, fill="#000000", outline=""),
        )
        self.caption_id = self.canvas.create_text(
            width - 8,
            height - 8,
            text="",
            anchor="se",
            fill="#f0f0f0",
            font=("Helvetica", 10, "bold"),
        )
        self.canvas.bind("<Configure>", self._on_resize)

    def set_content_ratio(self, ratio: Optional[float]) -> None:
        """コンテンツのアスペクト比を更新する."""

        self.content_ratio = ratio
        self._layout_content(
            self.canvas.winfo_width() or int(self.canvas.cget("width")),
            self.canvas.winfo_height() or int(self.canvas.cget("height")),
        )

    def update_content(self, caption: str, color: str) -> None:
        """矩形色とキャプションを更新する."""

        self.canvas.itemconfigure(self.content_id, fill=color)
        self.canvas.itemconfigure(self.caption_id, text=caption)
        self._layout_content(
            self.canvas.winfo_width() or int(self.canvas.cget("width")),
            self.canvas.winfo_height() or int(self.canvas.cget("height")),
        )

    def _on_resize(self, event: tk.Event) -> None:  # type: ignore[override]
        self._layout_content(event.width, event.height)

    def _layout_content(self, width: int, height: int) -> None:
        self.canvas.coords(self.background_id, 0, 0, width, height)
        if self.content_ratio and self.content_ratio > 0:
            target_width = width
            target_height = int(round(target_width / self.content_ratio))
            if target_height > height:
                target_height = height
                target_width = int(round(target_height * self.content_ratio))
            x0 = (width - target_width) / 2
            y0 = (height - target_height) / 2
        else:
            target_width = width
            target_height = height
            x0 = 0
            y0 = 0

        self.canvas.coords(
            self.content_id, x0, y0, x0 + target_width, y0 + target_height
        )
        self.canvas.coords(self.caption_id, width - 8, height - 8)

        if self.content_ratio and self.content_ratio > 0:
            if target_height < height:
                # 上下レターボックス
                self.canvas.coords(self.letterbox_ids[0], 0, 0, width, y0)
                self.canvas.coords(
                    self.letterbox_ids[1], 0, y0 + target_height, width, height
                )
            else:
                # 左右レターボックス
                self.canvas.coords(self.letterbox_ids[0], 0, 0, x0, height)
                self.canvas.coords(
                    self.letterbox_ids[1], x0 + target_width, 0, width, height
                )
            for rect in self.letterbox_ids:
                self.canvas.itemconfigure(rect, state="normal")
        else:
            for rect in self.letterbox_ids:
                self.canvas.itemconfigure(rect, state="hidden")

        self._post_layout(x0, y0, target_width, target_height)

    def _post_layout(
        self, x0: float, y0: float, width: float, height: float
    ) -> None:
        """派生クラスがレイアウト後処理を行うためのフック."""

        del x0, y0, width, height


class SensorImagePane(ImagePane):
    """障害物情報のオーバレイを備えた画像パネル."""

    def __init__(self, master: tk.Widget, title: str) -> None:
        super().__init__(master, title, width=260, height=260, content_ratio=1.0)
        self.overlay_rect = self.canvas.create_rectangle(
            0, 0, 0, 0, fill="#000000", outline="", stipple="gray50"
        )
        self.overlay_text_id = self.canvas.create_text(
            0,
            0,
            anchor="nw",
            text="",
            fill="#ffffff",
            font=("Helvetica", 10, "bold"),
        )

    def update_overlay(self, lines: List[str]) -> None:
        """オーバレイテキストを更新する."""

        text = "\n".join(lines)
        self.canvas.itemconfigure(self.overlay_text_id, text=text)
        self._layout_content(
            self.canvas.winfo_width() or int(self.canvas.cget("width")),
            self.canvas.winfo_height() or int(self.canvas.cget("height")),
        )

    def _post_layout(
        self, x0: float, y0: float, width: float, height: float
    ) -> None:
        padding = 12
        overlay_width = width * 0.85
        overlay_height = 32
        self.canvas.coords(
            self.overlay_rect,
            x0 + padding,
            y0 + padding,
            x0 + padding + overlay_width,
            y0 + padding + overlay_height,
        )
        self.canvas.coords(
            self.overlay_text_id,
            x0 + padding + 6,
            y0 + padding + 4,
        )


class VerticalScrollableFrame(ttk.Frame):
    """縦方向にスクロール可能なフレームを生成するヘルパー."""

    def __init__(self, master: tk.Widget, *, padding: tuple[int, int, int, int] = (0, 0, 0, 0)) -> None:
        super().__init__(master)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(self, highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.inner = ttk.Frame(self.canvas, padding=padding)
        self.inner_id = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")

        self.inner.bind("<Configure>", self._on_inner_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

    def _on_inner_configure(self, event: tk.Event) -> None:  # type: ignore[override]
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        if self.inner.winfo_reqwidth() != self.canvas.winfo_width():
            self.canvas.configure(width=self.inner.winfo_reqwidth())

    def _on_canvas_configure(self, event: tk.Event) -> None:  # type: ignore[override]
        self.canvas.itemconfigure(self.inner_id, width=event.width)


class PackageControlFrame(ttk.LabelFrame):
    """ノード起動設定を模擬するフレーム."""

    def __init__(
        self,
        master: tk.Widget,
        package: str,
        parameter_options: List[str],
        simulator_label: Optional[str],
        on_start,
        on_stop,
    ) -> None:
        super().__init__(master, text=package, padding=(8, 6))
        self.package = package
        self.parameter_var = tk.StringVar(value=parameter_options[0])
        self.simulator_supported = simulator_label is not None
        self.simulator_label = simulator_label or ""
        self.simulator_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="停止中")
        self.on_start = on_start
        self.on_stop = on_stop

        ttk.Label(self, text="パラメータ").grid(row=0, column=0, sticky="w")
        self.param_combo = ttk.Combobox(
            self,
            textvariable=self.parameter_var,
            values=parameter_options,
            state="readonly",
            width=18,
        )
        self.param_combo.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 6))

        next_row = 2
        if self.simulator_supported:
            self.sim_checkbox = ttk.Checkbutton(
                self,
                text=f"{self.simulator_label} 同時起動",
                variable=self.simulator_var,
            )
            self.sim_checkbox.grid(row=next_row, column=0, columnspan=2, sticky="w")
            next_row += 1
        else:
            self.sim_checkbox = None

        start_btn = ttk.Button(
            self,
            text="起動",
            command=self._handle_start,
            width=8,
        )
        stop_btn = ttk.Button(
            self,
            text="停止",
            command=self._handle_stop,
            width=8,
        )
        start_btn.grid(row=next_row, column=0, pady=4, sticky="ew")
        stop_btn.grid(row=next_row, column=1, pady=4, sticky="ew")
        next_row += 1

        ttk.Label(self, textvariable=self.status_var, foreground="#2c3e50").grid(
            row=next_row, column=0, columnspan=2, sticky="w"
        )
        for col in range(2):
            self.columnconfigure(col, weight=1)

    def _handle_start(self) -> None:
        self.on_start(
            self.package, self.parameter_var.get(), self.simulator_var.get()
        )

    def _handle_stop(self) -> None:
        self.on_stop(self.package)

    def update_status(self, status: NodeLaunchStatus) -> None:
        """起動状態表示を更新する."""

        if status.running:
            text = "稼働中"
            if status.simulator_enabled:
                text += "（simulator）"
        else:
            text = "停止中"
        if status.last_action:
            text += f" @ {status.last_action.strftime('%H:%M:%S')}"
        self.status_var.set(text)
        self.parameter_var.set(status.parameter_file)
        if self.simulator_supported:
            self.simulator_var.set(status.simulator_enabled)
        else:
            self.simulator_var.set(False)

    def get_settings(self) -> tuple[str, bool]:
        """現在選択されているパラメータとシミュレータ設定を返す."""

        return self.parameter_var.get(), (
            self.simulator_var.get() if self.simulator_supported else False
        )


# ----------------------------------------------------------------------
# メインアプリケーション
# ----------------------------------------------------------------------
class MockDashboardApp(tk.Tk):
    """robot_consoleダッシュボードのモックアプリ."""

    LOG_PACKAGES = [
        "route_manager",
        "route_follower",
        "obstacle_monitor",
        "robot_navigator",
    ]

    def __init__(self) -> None:
        super().__init__()
        self.title("robot_console dashboard mock")
        self.geometry("1280x720")
        self.minsize(960, 540)
        self._aspect_ratio = 1280 / 720
        self._resizing = False
        self._last_size = (1280, 720)
        self.bind("<Configure>", self._handle_root_configure)
        self.provider = MockDataProvider()
        self.log_widgets: Dict[str, tk.Text] = {}
        self.obstacle_override_active = tk.BooleanVar(value=False)
        self.sidebar_visible = True

        self._configure_style()
        self._build_layout()
        self._schedule_update()

    # ------------------------------------------------------------------
    def _configure_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TLabel", font=("Helvetica", 11))
        style.configure("Card.TLabelframe", background="#f7f9fc")
        style.configure("Card.TLabelframe.Label", font=("Helvetica", 11, "bold"))
        style.configure("Warning.TLabel", foreground="#ffffff", background="#d35400")
        style.configure("Danger.TLabel", foreground="#ffffff", background="#c0392b")
        style.configure("Success.TLabel", foreground="#ffffff", background="#16a085")
        style.configure("TButton", padding=4)

    # ------------------------------------------------------------------
    def _build_layout(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        notebook = ttk.Notebook(self)
        notebook.grid(row=0, column=0, sticky="nsew")
        self.dashboard_tab = ttk.Frame(notebook)
        self.log_tab = ttk.Frame(notebook)
        notebook.add(self.dashboard_tab, text="ダッシュボード")
        notebook.add(self.log_tab, text="コンソールログ")

        self._build_dashboard(self.dashboard_tab)
        self._build_log_tab(self.log_tab)

    def _build_dashboard(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)

        wrapper = ttk.Frame(parent)
        wrapper.grid(row=0, column=0, sticky="nsew")
        wrapper.columnconfigure(0, weight=1)
        wrapper.columnconfigure(1, weight=0)
        wrapper.rowconfigure(0, weight=1)

        main_area = ttk.Frame(wrapper, padding=10)
        main_area.grid(row=0, column=0, sticky="nsew")
        main_area.columnconfigure(0, weight=1)
        main_area.rowconfigure(1, weight=1)

        top_bar = ttk.Frame(main_area)
        top_bar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        top_bar.columnconfigure(0, weight=1)
        self.sidebar_toggle_btn = ttk.Button(
            top_bar,
            text="◀ ノード起動パネル",
            command=self._toggle_sidebar,
            width=20,
        )
        self.sidebar_toggle_btn.grid(row=0, column=1, sticky="e")

        dashboard_body = ttk.Frame(main_area)
        dashboard_body.grid(row=1, column=0, sticky="nsew")
        dashboard_body.columnconfigure(0, weight=1)
        dashboard_body.rowconfigure(2, weight=1)

        self._build_state_summary(dashboard_body)
        self._build_images(dashboard_body)
        self._build_control_panel(dashboard_body)

        self.sidebar_container = VerticalScrollableFrame(
            wrapper, padding=(6, 10, 6, 10)
        )
        self.sidebar_container.grid(row=0, column=1, sticky="ns")
        self.sidebar_frame = self.sidebar_container.inner
        self.sidebar_frame.columnconfigure(0, weight=1)
        ttk.Label(
            self.sidebar_frame,
            text="ノード起動パネル",
            font=("Helvetica", 13, "bold"),
        ).grid(row=0, column=0, sticky="w", pady=(0, 6))

        button_row = ttk.Frame(self.sidebar_frame)
        button_row.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        ttk.Button(button_row, text="全起動", command=self._handle_start_all).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(button_row, text="全停止", command=self._handle_stop_all).grid(
            row=0, column=1, sticky="ew"
        )

        self.package_frames = {}
        for idx, package in enumerate(self.LOG_PACKAGES, start=2):
            frame = PackageControlFrame(
                self.sidebar_frame,
                package,
                MockDataProvider.PARAMETER_CANDIDATES,
                MockDataProvider.SIMULATOR_MAP.get(package),
                self._handle_start_node,
                self._handle_stop_node,
            )
            frame.grid(row=idx, column=0, sticky="ew", pady=4)
            self.package_frames[package] = frame

    def _toggle_sidebar(self) -> None:
        self.sidebar_visible = not self.sidebar_visible
        if self.sidebar_visible:
            self.sidebar_container.grid()
            self.sidebar_toggle_btn.configure(text="◀ ノード起動パネル")
        else:
            self.sidebar_container.grid_remove()
            self.sidebar_toggle_btn.configure(text="▶ ノード起動パネル")

    def _handle_start_all(self) -> None:
        for package, frame in self.package_frames.items():
            parameter, simulator = frame.get_settings()
            self._handle_start_node(package, parameter, simulator)

    def _handle_stop_all(self) -> None:
        for package in self.package_frames:
            self._handle_stop_node(package)

    def _handle_root_configure(self, event: tk.Event) -> None:  # type: ignore[override]
        if event.widget is not self or self._resizing:
            return
        width, height = event.width, event.height
        if width <= 0 or height <= 0:
            return
        current = (width, height)
        if current == self._last_size:
            return

        target_height = int(round(width / self._aspect_ratio))
        target_width = int(round(height * self._aspect_ratio))

        if abs(width - self._last_size[0]) >= abs(height - self._last_size[1]):
            adjusted_width = width
            adjusted_height = target_height
        else:
            adjusted_width = target_width
            adjusted_height = height

        if adjusted_width == width and adjusted_height == height:
            self._last_size = current
            return

        self._resizing = True
        self.geometry(f"{adjusted_width}x{adjusted_height}")
        self._resizing = False
        self._last_size = (adjusted_width, adjusted_height)

    # ------------------------------------------------------------------

    def _build_state_summary(self, parent: ttk.Frame) -> None:
        summary_container = ttk.Frame(parent)
        summary_container.grid(row=0, column=0, sticky="ew")
        summary_container.columnconfigure(0, weight=1, uniform="summary")
        summary_container.columnconfigure(1, weight=1, uniform="summary")
        summary_container.columnconfigure(2, weight=1, uniform="summary")
        summary_container.rowconfigure(0, weight=1)

        # route_stateカード
        route_frame = ttk.LabelFrame(
            summary_container,
            text="ルート進捗 (route_state)",
            style="Card.TLabelframe",
        )
        route_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        route_frame.columnconfigure(1, weight=1)

        ttk.Label(route_frame, text="状態").grid(row=0, column=0, sticky="w")
        self.route_status_var = tk.StringVar()
        ttk.Label(route_frame, textvariable=self.route_status_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(route_frame, text="進捗").grid(row=1, column=0, sticky="w")
        self.route_progress_bar = ttk.Progressbar(
            route_frame, maximum=100, length=180
        )
        self.route_progress_bar.grid(row=1, column=1, sticky="ew", pady=2)
        self.route_progress_text = tk.StringVar()
        ttk.Label(route_frame, textvariable=self.route_progress_text).grid(
            row=2, column=1, sticky="w"
        )
        ttk.Label(route_frame, text="ルート履歴").grid(row=3, column=0, sticky="nw")
        self.route_version_var = tk.StringVar()
        ttk.Label(route_frame, textvariable=self.route_version_var, justify="left").grid(
            row=3, column=1, sticky="w"
        )

        # follower_stateカード
        follower_frame = ttk.LabelFrame(
            summary_container,
            text="フォロワ状態 (follower_state)",
            style="Card.TLabelframe",
        )
        follower_frame.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        follower_frame.columnconfigure(1, weight=1)

        ttk.Label(follower_frame, text="状態").grid(row=0, column=0, sticky="w")
        self.follower_state_var = tk.StringVar()
        ttk.Label(follower_frame, textvariable=self.follower_state_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(follower_frame, text="現在インデックス").grid(row=1, column=0, sticky="w")
        self.follower_index_var = tk.StringVar()
        ttk.Label(follower_frame, textvariable=self.follower_index_var).grid(
            row=1, column=1, sticky="w"
        )
        ttk.Label(follower_frame, text="現在ラベル").grid(row=2, column=0, sticky="w")
        self.follower_label_var = tk.StringVar()
        ttk.Label(follower_frame, textvariable=self.follower_label_var).grid(
            row=2, column=1, sticky="w"
        )
        ttk.Label(follower_frame, text="滞留要因").grid(row=3, column=0, sticky="w")
        self.follower_stagnation_var = tk.StringVar()
        ttk.Label(follower_frame, textvariable=self.follower_stagnation_var).grid(
            row=3, column=1, sticky="w"
        )
        ttk.Label(follower_frame, text="左右中央値").grid(row=4, column=0, sticky="w")
        self.follower_offsets_var = tk.StringVar()
        ttk.Label(follower_frame, textvariable=self.follower_offsets_var).grid(
            row=4, column=1, sticky="w"
        )
        ttk.Label(follower_frame, text="次ウェイポイント").grid(row=5, column=0, sticky="w")
        self.follower_waypoint_var = tk.StringVar()
        ttk.Label(follower_frame, textvariable=self.follower_waypoint_var).grid(
            row=5, column=1, sticky="w"
        )

        # manager_statusカード
        manager_frame = ttk.LabelFrame(
            summary_container,
            text="マネージャ状態 (manager_status)",
            style="Card.TLabelframe",
        )
        manager_frame.grid(row=0, column=2, sticky="nsew")
        manager_frame.columnconfigure(1, weight=1)
        ttk.Label(manager_frame, text="状態").grid(row=0, column=0, sticky="w")
        self.manager_state_var = tk.StringVar()
        ttk.Label(manager_frame, textvariable=self.manager_state_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(manager_frame, text="最終要因").grid(row=1, column=0, sticky="w")
        self.manager_cause_var = tk.StringVar()
        ttk.Label(manager_frame, textvariable=self.manager_cause_var, wraplength=220).grid(
            row=1, column=1, sticky="w"
        )
        ttk.Label(manager_frame, text="遷移時刻").grid(row=2, column=0, sticky="w")
        self.manager_transition_var = tk.StringVar()
        ttk.Label(manager_frame, textvariable=self.manager_transition_var).grid(
            row=2, column=1, sticky="w"
        )

        metrics_container = ttk.Frame(parent)
        metrics_container.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        metrics_container.columnconfigure(0, weight=1, uniform="metrics")
        metrics_container.columnconfigure(1, weight=1, uniform="metrics")

        velocity_frame = ttk.LabelFrame(
            metrics_container, text="ロボット速度 (cmd_vel)", style="Card.TLabelframe"
        )
        velocity_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        velocity_frame.columnconfigure(1, weight=1)
        ttk.Label(velocity_frame, text="並進速度").grid(row=0, column=0, sticky="w")
        self.velocity_linear_var = tk.StringVar(value="0.00 m/s")
        ttk.Label(velocity_frame, textvariable=self.velocity_linear_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(velocity_frame, text="角速度").grid(row=1, column=0, sticky="w")
        self.velocity_angular_var = tk.StringVar(value="0.00 deg/s")
        ttk.Label(velocity_frame, textvariable=self.velocity_angular_var).grid(
            row=1, column=1, sticky="w"
        )

        target_frame = ttk.LabelFrame(
            metrics_container, text="目標までの距離", style="Card.TLabelframe"
        )
        target_frame.grid(row=0, column=1, sticky="nsew")
        target_frame.columnconfigure(0, weight=1)
        self.target_distance_var = tk.StringVar()
        ttk.Label(target_frame, textvariable=self.target_distance_var).grid(
            row=0, column=0, sticky="w"
        )
        self.target_progress = ttk.Progressbar(target_frame, maximum=100)
        self.target_progress.grid(row=1, column=0, sticky="ew", pady=4)
        self.target_hint_var = tk.StringVar()
        ttk.Label(target_frame, textvariable=self.target_hint_var).grid(
            row=2, column=0, sticky="w"
        )
    # ------------------------------------------------------------------
    def _build_images(self, parent: ttk.Frame) -> None:
            image_frame = ttk.Frame(parent)
            image_frame.grid(row=2, column=0, sticky="nsew")
            image_frame.columnconfigure(0, weight=1)
            image_frame.columnconfigure(1, weight=1)
            image_frame.columnconfigure(2, weight=1)
            image_frame.rowconfigure(0, weight=1)

            self.route_image = ImagePane(
                image_frame,
                "ルート地図",
                width=420,
                height=240,
                content_ratio=2100 / 1200,
            )
            self.route_image.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

            self.sensor_image = SensorImagePane(image_frame, "障害物ビュー")
            self.sensor_image.grid(row=0, column=1, sticky="nsew", padx=(0, 8))

            self.camera_image = ImagePane(
                image_frame,
                "外部カメラ",
                width=360,
                height=240,
                content_ratio=4 / 3,
            )
            self.camera_image.grid(row=0, column=2, sticky="nsew")

    # ------------------------------------------------------------------

    def _build_control_panel(self, parent: ttk.Frame) -> None:
        container = ttk.Frame(parent)
        container.grid(row=3, column=0, sticky="ew", pady=(8, 8))
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=3)

        banner_frame = ttk.LabelFrame(
            container, text="信号・手動・封鎖", style="Card.TLabelframe", padding=8
        )
        banner_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        banner_frame.columnconfigure(0, weight=1)
        banner_frame.rowconfigure(0, weight=1)

        self.banner_default_bg = "#f7f9fc"
        self.banner_default_fg = "#2c3e50"
        self.banner_label = tk.Label(
            banner_frame,
            text="",
            font=("Helvetica", 14, "bold"),
            anchor="center",
            bg=self.banner_default_bg,
            fg=self.banner_default_fg,
            padx=8,
            pady=16,
            wraplength=220,
            justify="center",
        )
        self.banner_label.grid(row=0, column=0, sticky="nsew")

        control_frame = ttk.LabelFrame(container, text="制御コマンド", padding=8)
        control_frame.grid(row=0, column=1, sticky="nsew")
        for col in range(2):
            control_frame.columnconfigure(col, weight=1)

        ttk.Button(
            control_frame,
            text="manual_start 送信",
            command=self._handle_manual_start,
        ).grid(row=0, column=0, padx=4, pady=4, sticky="ew")

        ttk.Button(
            control_frame,
            text="road_blocked 切替",
            command=self._handle_road_blocked,
        ).grid(row=0, column=1, padx=4, pady=4, sticky="ew")

        self.sig_value = tk.IntVar(value=1)
        sig_frame = ttk.Frame(control_frame)
        sig_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=(4, 0))
        ttk.Label(sig_frame, text="sig_recog").grid(row=0, column=0, sticky="w")
        sig_combo = ttk.Combobox(
            sig_frame,
            textvariable=self.sig_value,
            values=[0, 1, 2],
            state="readonly",
            width=5,
        )
        sig_combo.grid(row=0, column=1, padx=2)
        ttk.Button(sig_frame, text="送信", command=self._handle_sig_recog).grid(
            row=0, column=2, padx=2
        )

        override_frame = ttk.LabelFrame(
            control_frame, text="障害物ヒント固定値", padding=8
        )
        override_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        override_frame.columnconfigure(0, weight=1)
        override_frame.columnconfigure(1, weight=1)
        override_frame.columnconfigure(2, weight=1)

        ttk.Checkbutton(
            override_frame,
            text="固定値送出を有効化",
            variable=self.obstacle_override_active,
            command=self._handle_obstacle_override,
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        self.obstacle_block_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            override_frame,
            text="front_blocked",
            variable=self.obstacle_block_var,
            command=self._handle_obstacle_params_changed,
        ).grid(row=1, column=0, sticky="w", pady=(4, 0))

        self.obstacle_clearance_var = tk.DoubleVar(value=3.0)
        ttk.Label(override_frame, text="余裕距離[m]").grid(
            row=1, column=1, sticky="e", padx=(8, 2)
        )
        self.obstacle_clearance_spin = ttk.Spinbox(
            override_frame,
            textvariable=self.obstacle_clearance_var,
            from_=0.1,
            to=10.0,
            increment=0.1,
            width=6,
            command=self._handle_obstacle_params_changed,
        )
        self.obstacle_clearance_spin.grid(row=1, column=2, sticky="w")

        self.obstacle_left_offset_var = tk.DoubleVar(value=0.0)
        ttk.Label(override_frame, text="左オフセット[m]").grid(
            row=2, column=1, sticky="e", padx=(8, 2)
        )
        self.obstacle_left_spin = ttk.Spinbox(
            override_frame,
            textvariable=self.obstacle_left_offset_var,
            from_=-2.0,
            to=2.0,
            increment=0.05,
            width=6,
            command=self._handle_obstacle_params_changed,
        )
        self.obstacle_left_spin.grid(row=2, column=2, sticky="w")

        self.obstacle_right_offset_var = tk.DoubleVar(value=0.0)
        ttk.Label(override_frame, text="右オフセット[m]").grid(
            row=3, column=1, sticky="e", padx=(8, 2)
        )
        self.obstacle_right_spin = ttk.Spinbox(
            override_frame,
            textvariable=self.obstacle_right_offset_var,
            from_=-2.0,
            to=2.0,
            increment=0.05,
            width=6,
            command=self._handle_obstacle_params_changed,
        )
        self.obstacle_right_spin.grid(row=3, column=2, sticky="w")

        for widget in (
            self.obstacle_clearance_spin,
            self.obstacle_left_spin,
            self.obstacle_right_spin,
        ):
            widget.bind("<FocusOut>", self._handle_obstacle_params_changed)
            widget.bind("<Return>", self._handle_obstacle_params_changed)
    # ------------------------------------------------------------------

    def _build_log_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        for row in range(len(self.LOG_PACKAGES)):
            parent.rowconfigure(row, weight=1)

        for row, package in enumerate(self.LOG_PACKAGES):
            frame = ttk.LabelFrame(parent, text=package, padding=6)
            pady = (8, 4) if row == 0 else (4, 4)
            frame.grid(row=row, column=0, sticky="nsew", padx=8, pady=pady)
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            text_widget = tk.Text(frame, state="disabled", wrap="none")
            text_widget.grid(row=0, column=0, sticky="nsew")
            scrollbar = ttk.Scrollbar(frame, orient="vertical", command=text_widget.yview)
            scrollbar.grid(row=0, column=1, sticky="ns")
            text_widget.configure(yscrollcommand=scrollbar.set)
            self.log_widgets[package] = text_widget
    # ------------------------------------------------------------------
    def _schedule_update(self) -> None:
            self.provider.simulate_tick()
            snapshot = self.provider.snapshot()
            self._apply_snapshot(snapshot)
            logs = self.provider.consume_logs()
            self._append_logs(logs)
            self.after(1200, self._schedule_update)

    # ------------------------------------------------------------------

    def _apply_snapshot(self, snapshot: Dict[str, object]) -> None:
            route_state: RouteStateSnapshot = snapshot["route_state"]
            follower_state: FollowerStateSnapshot = snapshot["follower_state"]
            obstacle_hint: ObstacleHintSnapshot = snapshot["obstacle_hint"]
            manual_signal: ManualSignalSnapshot = snapshot["manual_signal"]
            active_target: ActiveTargetSnapshot = snapshot["active_target"]
            cmd_vel: CmdVelSnapshot = snapshot["cmd_vel"]
            manager_status: ManagerStatusSnapshot = snapshot["manager_status"]
            image_timestamps: Dict[str, datetime] = snapshot["image_timestamps"]
            node_status: Dict[str, NodeLaunchStatus] = snapshot["node_status"]
            camera_mode: str = snapshot["camera_mode"]
            override_info: Dict[str, object] = snapshot["obstacle_override"]

            # route_state
            self.route_status_var.set(
                f"{route_state.status} / {manager_status.state}"
            )
            progress = int(route_state.progress_ratio * 100)
            self.route_progress_bar.configure(value=progress)
            self.route_progress_text.set(
                f"{route_state.current_index}/{route_state.total_waypoints} ({progress}%)"
            )
            history_text = "\n".join(route_state.version_history)
            self.route_version_var.set(history_text)

            # follower_state
            self.follower_state_var.set(
                f"{follower_state.state} / v{follower_state.route_version}"
            )
            index_text = f"{follower_state.current_index}/{route_state.total_waypoints}"
            self.follower_index_var.set(index_text)
            self.follower_label_var.set(follower_state.current_waypoint_label)
            stagnation = (
                f"{follower_state.last_stagnation_reason} / 試行"
                f" {follower_state.avoidance_attempt_count} 回"
            )
            self.follower_stagnation_var.set(stagnation)
            offsets = (
                f"左 {follower_state.left_offset_m_median:+.2f} m / "
                f"右 {follower_state.right_offset_m_median:+.2f} m"
            )
            self.follower_offsets_var.set(offsets)
            waypoints = (
                f"{follower_state.current_waypoint_label} → "
                f"{follower_state.next_waypoint_label}"
            )
            self.follower_waypoint_var.set(waypoints)

            # manager_status
            self.manager_state_var.set(manager_status.state)
            self.manager_cause_var.set(manager_status.last_cause)
            self.manager_transition_var.set(
                manager_status.last_transition.strftime("%H:%M:%S")
            )

            # manual / signal / road banner
            now = _now()
            banner_text = ""
            banner_bg = self.banner_default_bg
            banner_fg = self.banner_default_fg
            if manual_signal.road_blocked:
                ts = (
                    manual_signal.road_sent_at.strftime("%H:%M:%S")
                    if manual_signal.road_sent_at
                    else "--:--:--"
                )
                banner_text = f"ROAD BLOCKED @{ts}"
                banner_bg = "#c0392b"
                banner_fg = "#ffffff"
            else:
                manual_recent = manual_signal.manual_start or (
                    manual_signal.manual_sent_at
                    and (now - manual_signal.manual_sent_at) < timedelta(seconds=5)
                )
                if manual_recent:
                    ts = (
                        manual_signal.manual_sent_at.strftime("%H:%M:%S")
                        if manual_signal.manual_sent_at
                        else "--:--:--"
                    )
                    banner_text = f"MANUAL START @{ts}"
                    banner_bg = "#16a085"
                    banner_fg = "#ffffff"
                elif manual_signal.sig_recog != 0:
                    ts = (
                        manual_signal.sig_sent_at.strftime("%H:%M:%S")
                        if manual_signal.sig_sent_at
                        else "--:--:--"
                    )
                    status_map = {1: "GO", 2: "STOP"}
                    status_label = status_map.get(manual_signal.sig_recog, str(manual_signal.sig_recog))
                    banner_text = f"SIGNAL {status_label} @{ts}"
                    banner_bg = "#2980b9" if manual_signal.sig_recog == 1 else "#d35400"
                    banner_fg = "#ffffff"
            self.banner_label.configure(text=banner_text, bg=banner_bg, fg=banner_fg)

            # active_target距離
            self.target_distance_var.set(
                f"現在距離: {active_target.current_distance_m:5.2f} m"
            )
            self.target_progress.configure(value=active_target.ratio * 100)
            self.target_hint_var.set(
                f"基準距離 {active_target.reference_distance_m:5.2f} m"
            )

            # cmd_vel
            self.velocity_linear_var.set(f"{cmd_vel.linear_x:+.2f} m/s")
            self.velocity_angular_var.set(
                f"{math.degrees(cmd_vel.angular_z):+.1f} deg/s"
            )

            # 画像更新
            route_caption = f"route_map @ {image_timestamps['route_map'].strftime('%H:%M:%S')}"
            self.route_image.update_content(route_caption, "#2980b9")

            sensor_caption = (
                f"sensor_viewer @ {image_timestamps['sensor_viewer'].strftime('%H:%M:%S')}"
            )
            sensor_color = "#27ae60" if not follower_state.front_blocked_majority else "#c0392b"
            self.sensor_image.update_content(sensor_caption, sensor_color)
            overlay_lines = [
                (
                    f"遮蔽:{'YES' if follower_state.front_blocked_majority else 'NO'}  "
                    f"余裕:{obstacle_hint.front_clearance_m:4.2f}m"
                ),
                (
                    f"左:{follower_state.left_offset_m_median:+.2f}m  "
                    f"右:{follower_state.right_offset_m_median:+.2f}m"
                ),
            ]
            self.sensor_image.update_overlay(overlay_lines)

            camera_caption = (
                f"camera[{camera_mode}] @ {image_timestamps['camera'].strftime('%H:%M:%S')}"
            )
            camera_color = "#8e44ad" if camera_mode == "signal" else "#34495e"
            if camera_mode == "signal":
                self.camera_image.set_content_ratio(4 / 3)
            else:
                self.camera_image.set_content_ratio(16 / 3)
            self.camera_image.update_content(camera_caption, camera_color)

            # ノード起動ステータス
            for package, status in node_status.items():
                frame = self.package_frames.get(package)
                if frame:
                    frame.update_status(status)

            # 障害物ヒント固定値の同期
            if self.obstacle_override_active.get() != override_info["active"]:
                self.obstacle_override_active.set(bool(override_info["active"]))
            self.obstacle_block_var.set(bool(override_info["front_blocked"]))
            self.obstacle_clearance_var.set(float(override_info["clearance_m"]))
            self.obstacle_left_offset_var.set(float(override_info["left_offset_m"]))
            self.obstacle_right_offset_var.set(float(override_info["right_offset_m"]))
    # ------------------------------------------------------------------
    def _append_logs(self, logs: Dict[str, List[str]]) -> None:
        for package, entries in logs.items():
            if not entries:
                continue
            widget = self.log_widgets[package]
            widget.configure(state="normal")
            for line in entries:
                widget.insert("end", line + "\n")
            widget.see("end")
            widget.configure(state="disabled")

    def _handle_manual_start(self) -> None:
        self.provider.send_manual_start()

    def _handle_sig_recog(self) -> None:
        self.provider.send_sig_recog(self.sig_value.get())

    def _handle_obstacle_params_changed(self, *_args) -> None:
        try:
            clearance = float(self.obstacle_clearance_var.get())
            left = float(self.obstacle_left_offset_var.get())
            right = float(self.obstacle_right_offset_var.get())
        except (tk.TclError, ValueError):
            return
        blocked = bool(self.obstacle_block_var.get())
        self.provider.update_obstacle_hint_override(
            front_blocked=blocked,
            clearance_m=clearance,
            left_offset_m=left,
            right_offset_m=right,
        )

    def _handle_obstacle_override(self) -> None:
        self.provider.toggle_obstacle_hint_override(
            self.obstacle_override_active.get()
        )
        self._handle_obstacle_params_changed()

    def _handle_road_blocked(self) -> None:
        self.provider.toggle_road_blocked()

    def _handle_start_node(
        self, package: str, parameter_file: str, simulator_enabled: bool
    ) -> None:
        self.provider.start_node(package, parameter_file, simulator_enabled)

    def _handle_stop_node(self, package: str) -> None:
        self.provider.stop_node(package)




def main() -> None:
    """エントリポイント."""

    app = MockDashboardApp()
    app.mainloop()


if __name__ == "__main__":
    main()
