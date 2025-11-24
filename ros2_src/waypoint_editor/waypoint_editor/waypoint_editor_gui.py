"""Tkinterベースのウェイポイント編集GUI。"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

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


SHIFT_DISTANCE = 0.25
YAW_STEP = math.radians(5.0)


class WaypointEditorGUI(Node):
    """サービス呼び出しを行いながらGUIを制御するクラス。"""

    def __init__(self) -> None:
        super().__init__("waypoint_editor_gui")
        self.root = tk.Tk()
        self.root.title("Waypoint Editor")

        self.selected_label: Optional[int] = None

        self._build_widgets()
        self._create_clients()
        self.create_subscription(Int32, "selected_label", self._on_selected_label, 10)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._pump_ros()

    def _build_widgets(self) -> None:
        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.label_var = tk.StringVar()
        self.x_var = tk.StringVar()
        self.y_var = tk.StringVar()
        self.z_var = tk.StringVar()
        self.yaw_var = tk.StringVar()
        self.right_var = tk.StringVar()
        self.left_var = tk.StringVar()
        self.line_var = tk.BooleanVar()
        self.signal_var = tk.BooleanVar()
        self.skip_var = tk.BooleanVar()
        self.node_var = tk.StringVar()

        search_frame = ttk.LabelFrame(main, text="検索")
        search_frame.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        ttk.Label(search_frame, text="Label").grid(row=0, column=0, padx=2, pady=2)
        ttk.Entry(search_frame, textvariable=self.label_var, width=10).grid(
            row=0, column=1, padx=2, pady=2
        )
        ttk.Button(search_frame, text="検索", command=self._on_search).grid(
            row=0, column=2, padx=2, pady=2
        )
        self.prev_btn = ttk.Button(search_frame, text="Prev", command=self._on_prev)
        self.prev_btn.grid(row=0, column=3, padx=2, pady=2)
        self.next_btn = ttk.Button(search_frame, text="Next", command=self._on_next)
        self.next_btn.grid(row=0, column=4, padx=2, pady=2)

        pose_frame = ttk.LabelFrame(main, text="現在位置・姿勢")
        pose_frame.grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        ttk.Label(pose_frame, text="X").grid(row=0, column=0)
        ttk.Entry(pose_frame, textvariable=self.x_var, width=12).grid(row=0, column=1)
        ttk.Label(pose_frame, text="Y").grid(row=0, column=2)
        ttk.Entry(pose_frame, textvariable=self.y_var, width=12).grid(row=0, column=3)
        ttk.Label(pose_frame, text="Z").grid(row=0, column=4)
        ttk.Entry(pose_frame, textvariable=self.z_var, width=12, state="readonly").grid(
            row=0, column=5
        )
        ttk.Label(pose_frame, text="Yaw(rad)").grid(row=1, column=0)
        ttk.Entry(pose_frame, textvariable=self.yaw_var, width=12).grid(row=1, column=1)
        ttk.Button(pose_frame, text="姿勢更新", command=self._on_update_pose).grid(
            row=1, column=2, padx=4
        )

        shift_frame = ttk.LabelFrame(main, text="シフト操作 (0.25m, yaw基準)")
        shift_frame.grid(row=2, column=0, sticky="ew", padx=4, pady=4)
        buttons = [
            ("↖", -1, 1), ("↑", 0, 1), ("↗", 1, 1),
            ("←", -1, 0), ("→", 1, 0),
            ("↙", -1, -1), ("↓", 0, -1), ("↘", 1, -1),
        ]
        positions = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 2), (2, 0), (2, 1), (2, 2)]
        for (text, dx, dy), (r, c) in zip(buttons, positions):
            ttk.Button(
                shift_frame,
                text=text,
                command=lambda dx=dx, dy=dy: self._on_shift(dx, dy),
                width=4,
            ).grid(row=r, column=c, padx=1, pady=1)
        ttk.Label(shift_frame, text="Yaw調整").grid(row=0, column=3, padx=4)
        ttk.Button(shift_frame, text="-5°", command=lambda: self._on_rotate(-YAW_STEP)).grid(
            row=0, column=4, padx=2
        )
        ttk.Button(shift_frame, text="+5°", command=lambda: self._on_rotate(YAW_STEP)).grid(
            row=0, column=5, padx=2
        )

        flag_frame = ttk.LabelFrame(main, text="フラグ／開放度")
        flag_frame.grid(row=3, column=0, sticky="ew", padx=4, pady=4)
        ttk.Label(flag_frame, text="right_is_open").grid(row=0, column=0)
        ttk.Entry(flag_frame, textvariable=self.right_var, width=8).grid(row=0, column=1)
        ttk.Label(flag_frame, text="left_is_open").grid(row=0, column=2)
        ttk.Entry(flag_frame, textvariable=self.left_var, width=8).grid(row=0, column=3)
        ttk.Checkbutton(
            flag_frame, text="line_is_stop", variable=self.line_var
        ).grid(row=1, column=0)
        ttk.Checkbutton(
            flag_frame, text="signal_is_stop", variable=self.signal_var
        ).grid(row=1, column=1)
        ttk.Checkbutton(flag_frame, text="isnot_skipnum", variable=self.skip_var).grid(
            row=1, column=2
        )
        ttk.Button(flag_frame, text="フラグ更新", command=self._on_update_flags).grid(
            row=1, column=3, padx=4
        )

        node_frame = ttk.LabelFrame(main, text="ノード値")
        node_frame.grid(row=4, column=0, sticky="ew", padx=4, pady=4)
        ttk.Label(node_frame, text="node").grid(row=0, column=0)
        ttk.Entry(node_frame, textvariable=self.node_var, width=12).grid(row=0, column=1)
        ttk.Button(node_frame, text="node更新", command=self._on_update_node).grid(
            row=0, column=2, padx=4
        )

        button_frame = ttk.Frame(main)
        button_frame.grid(row=5, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(button_frame, text="保存", command=self._on_save).grid(row=0, column=0, padx=2)
        ttk.Button(button_frame, text="リセット", command=self._on_reset).grid(row=0, column=1, padx=2)
        ttk.Button(button_frame, text="削除", command=self._on_delete).grid(row=0, column=2, padx=2)

        main.columnconfigure(0, weight=1)
        for idx in range(6):
            main.rowconfigure(idx, weight=0)
        self._update_prev_next_state()

    def _create_clients(self) -> None:
        self.search_client = self.create_client(SearchLabel, "search_label")
        self.update_pose_client = self.create_client(UpdatePose, "update_pose")
        self.update_flags_client = self.create_client(UpdateFlags, "update_flags")
        self.update_node_client = self.create_client(UpdateNode, "update_node")
        self.shift_client = self.create_client(ShiftWaypoint, "shift_waypoint")
        self.rotate_client = self.create_client(RotateWaypoint, "rotate_waypoint")
        self.save_client = self.create_client(SaveWaypoints, "save_waypoints")
        self.reset_client = self.create_client(ResetWaypoints, "reset_waypoints")
        self.delete_client = self.create_client(DeleteWaypoint, "delete_waypoint")
        self.select_client = self.create_client(SelectInRviz, "select_in_rviz")

    def _pump_ros(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(50, self._pump_ros)

    def _on_close(self) -> None:
        self.get_logger().info("GUIを終了します。")
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def _on_selected_label(self, msg: Int32) -> None:
        self.selected_label = msg.data
        self.label_var.set(str(msg.data))
        self._update_prev_next_state()

    def _update_prev_next_state(self) -> None:
        state = tk.NORMAL if self.selected_label is not None else tk.DISABLED
        self.prev_btn.configure(state=state)
        self.next_btn.configure(state=state)

    def _call_service(self, client, request, on_success):
        if not client.wait_for_service(timeout_sec=0.5):
            messagebox.showwarning("Service", "サービスが起動していません。")
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.root.after(0, on_success, f))

    def _on_search(self) -> None:
        try:
            label = int(self.label_var.get())
        except ValueError:
            messagebox.showwarning("入力エラー", "Labelは整数で入力してください。")
            return
        req = SearchLabel.Request()
        req.label = label
        self._call_service(self.search_client, req, self._handle_search_response)

    def _handle_search_response(self, future) -> None:
        if future.cancelled() or future.exception():
            messagebox.showerror("検索", "サービス呼び出しに失敗しました。")
            return
        res = future.result()
        if not res.success:
            messagebox.showwarning("検索", res.message)
            return
        self.selected_label = int(self.label_var.get())
        self.x_var.set(str(res.x))
        self.y_var.set(str(res.y))
        self.z_var.set(str(res.z))
        self.yaw_var.set(str(res.yaw))
        self.right_var.set(str(res.right_is_open))
        self.left_var.set(str(res.left_is_open))
        self.line_var.set(bool(res.line_is_stop))
        self.signal_var.set(bool(res.signal_is_stop))
        self.skip_var.set(bool(res.isnot_skipnum))
        self.node_var.set(str(res.node))
        self._update_prev_next_state()

    def _on_prev(self) -> None:
        if self.selected_label is None:
            return
        req = SelectInRviz.Request()
        req.label = self.selected_label - 1
        self._call_service(self.select_client, req, self._handle_select_response)

    def _on_next(self) -> None:
        if self.selected_label is None:
            return
        req = SelectInRviz.Request()
        req.label = self.selected_label + 1
        self._call_service(self.select_client, req, self._handle_select_response)

    def _handle_select_response(self, future) -> None:
        if future.cancelled() or future.exception():
            messagebox.showerror("選択", "サービス呼び出しに失敗しました。")
            return
        res = future.result()
        if not res.success:
            messagebox.showwarning("選択", res.message)
            return
        self._on_search()

    def _on_shift(self, dx: int, dy: int) -> None:
        if self.selected_label is None:
            return
        yaw = self._parse_float(self.yaw_var.get(), 0.0)
        shift_x = (math.cos(yaw) * dx - math.sin(yaw) * dy) * SHIFT_DISTANCE
        shift_y = (math.sin(yaw) * dx + math.cos(yaw) * dy) * SHIFT_DISTANCE
        req = ShiftWaypoint.Request()
        req.label = self.selected_label
        req.shift_x = shift_x
        req.shift_y = shift_y
        self._call_service(self.shift_client, req, self._handle_simple_response)

    def _on_rotate(self, delta: float) -> None:
        if self.selected_label is None:
            return
        req = RotateWaypoint.Request()
        req.label = self.selected_label
        req.delta_yaw = delta
        self._call_service(self.rotate_client, req, self._handle_simple_response)

    def _on_save(self) -> None:
        req = SaveWaypoints.Request()
        self._call_service(self.save_client, req, self._handle_simple_response)

    def _on_reset(self) -> None:
        req = ResetWaypoints.Request()
        self._call_service(self.reset_client, req, self._handle_simple_response)

    def _on_delete(self) -> None:
        if self.selected_label is None:
            return
        req = DeleteWaypoint.Request()
        req.label = self.selected_label
        self._call_service(self.delete_client, req, self._handle_simple_response)

    def _on_update_pose(self) -> None:
        if self.selected_label is None:
            return
        req = UpdatePose.Request()
        req.label = self.selected_label
        req.x = self._parse_float(self.x_var.get())
        req.y = self._parse_float(self.y_var.get())
        req.yaw = self._parse_float(self.yaw_var.get())
        self._call_service(self.update_pose_client, req, self._handle_simple_response)

    def _on_update_flags(self) -> None:
        if self.selected_label is None:
            return
        req = UpdateFlags.Request()
        req.label = self.selected_label
        req.right_is_open = self._parse_float(self.right_var.get())
        req.left_is_open = self._parse_float(self.left_var.get())
        req.line_is_stop = int(self.line_var.get())
        req.signal_is_stop = int(self.signal_var.get())
        req.isnot_skipnum = int(self.skip_var.get())
        self._call_service(self.update_flags_client, req, self._handle_simple_response)

    def _on_update_node(self) -> None:
        if self.selected_label is None:
            return
        req = UpdateNode.Request()
        req.label = self.selected_label
        req.node = self._parse_float(self.node_var.get())
        self._call_service(self.update_node_client, req, self._handle_simple_response)

    def _handle_simple_response(self, future) -> None:
        if future.cancelled() or future.exception():
            messagebox.showerror("サービス", "呼び出しに失敗しました。")
            return
        res = future.result()
        if not res.success:
            messagebox.showwarning("サービス", res.message)
        self._on_search()

    @staticmethod
    def _parse_float(value: str, default: float = 0.0) -> float:
        try:
            return float(value)
        except ValueError:
            return default


def main(args=None) -> None:
    rclpy.init(args=args)
    gui = WaypointEditorGUI()
    gui.root.mainloop()


if __name__ == "__main__":
    main()
