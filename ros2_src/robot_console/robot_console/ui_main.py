"""tkinter を利用した robot_console GUI 実装。"""

from __future__ import annotations

import base64
import tkinter as tk
from tkinter import ttk
from typing import Dict, Optional, TYPE_CHECKING

try:
    from PIL import Image, ImageTk  # type: ignore
    PIL_IMAGETK_AVAILABLE = True
except ImportError:  # pragma: no cover - 実行環境依存
    try:
        from PIL import Image  # type: ignore
    except ImportError:  # pragma: no cover - Pillow すら無い環境では画像機能を無効化
        Image = None  # type: ignore
    ImageTk = None  # type: ignore
    PIL_IMAGETK_AVAILABLE = False

try:
    import cv2  # type: ignore

    CV2_AVAILABLE = True
except ImportError:  # pragma: no cover - OpenCV が無い環境向けフォールバック
    cv2 = None  # type: ignore
    CV2_AVAILABLE = False

if CV2_AVAILABLE:  # pragma: no branch - OpenCV がある場合のみ NumPy を使用
    import numpy as np
else:  # pragma: no cover - OpenCV 無し環境では未使用
    np = None  # type: ignore

from .gui_core import GuiCore
from .utils import GuiSnapshot, NodeLaunchStatus

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
REFRESH_INTERVAL_MS = 200


class UiMain:
    """GuiCore のスナップショットを可視化する tkinter アプリ。"""

    def __init__(self, gui_core: GuiCore) -> None:
        self._core = gui_core
        self._root = tk.Tk()
        self._root.title("robot_console")
        self._root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self._root.minsize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self._image_refs: Dict[str, Optional[tk.PhotoImage]] = {
            'route': None,
            'obstacle': None,
            'camera': None,
        }

        self._image_render_available = PIL_IMAGETK_AVAILABLE or CV2_AVAILABLE
        self._image_warning_message = (
            'Pillow ImageTk と OpenCV の両方が利用できないため画像パネルを無効化しました。\n'
            'python3-pil.imagetk もしくは python3-opencv をインストールしてください。'
        ) if not self._image_render_available else ''

        self._manual_var = tk.StringVar(value='False')
        self._manual_status = tk.StringVar(value='未送信')
        self._sig_var = tk.StringVar(value='GO')
        self._sig_status = tk.StringVar(value='未送信')
        self._road_var = tk.StringVar(value='False')
        self._road_status = tk.StringVar(value='未送信')
        self._obstacle_clearance = tk.DoubleVar(value=5.0)
        self._obstacle_left = tk.DoubleVar(value=0.0)
        self._obstacle_right = tk.DoubleVar(value=0.0)
        self._obstacle_blocked = tk.BooleanVar(value=False)
        self._obstacle_status = tk.StringVar(value='自動追従')
        self._event_banner = tk.StringVar(value='')

        self._launch_widgets: Dict[str, Dict[str, object]] = {}

        self._build_layout()
        self._schedule_update()

    def _build_layout(self) -> None:
        self._notebook = ttk.Notebook(self._root)
        self._dashboard_tab = ttk.Frame(self._notebook)
        self._logs_tab = ttk.Frame(self._notebook)
        self._notebook.add(self._dashboard_tab, text='Dashboard')
        self._notebook.add(self._logs_tab, text='Console Logs')
        self._notebook.pack(fill=tk.BOTH, expand=True)

        self._build_dashboard(self._dashboard_tab)
        self._build_logs(self._logs_tab)

    def _build_dashboard(self, parent: tk.Widget) -> None:
        container = ttk.Frame(parent)
        container.pack(fill=tk.BOTH, expand=True)
        container.columnconfigure(0, weight=5)
        container.columnconfigure(1, weight=2)
        container.rowconfigure(0, weight=1)

        main_area = ttk.Frame(container)
        main_area.grid(row=0, column=0, sticky='nsew', padx=8, pady=8)
        main_area.rowconfigure(0, weight=1)
        main_area.rowconfigure(1, weight=1)
        main_area.rowconfigure(2, weight=2)
        main_area.columnconfigure(0, weight=1)

        self._build_top_cards(main_area)
        self._build_middle_controls(main_area)
        self._build_images(main_area)

        self._build_launch_sidebar(container)

    def _build_top_cards(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=0, sticky='nsew', pady=(0, 8))
        frame.columnconfigure(0, weight=1, uniform='card')
        frame.columnconfigure(1, weight=1, uniform='card')
        frame.columnconfigure(2, weight=1, uniform='card')

        # Route progress card
        self._route_state_vars = {
            'state': tk.StringVar(value='unknown'),
            'progress': tk.DoubleVar(value=0.0),
            'progress_text': tk.StringVar(value='0 / 0'),
            'message': tk.StringVar(value=''),
        }
        route_card = ttk.LabelFrame(frame, text='ルート進捗')
        route_card.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        route_card.columnconfigure(0, weight=1)
        ttk.Label(route_card, textvariable=self._route_state_vars['state']).grid(
            row=0, column=0, sticky='w'
        )
        progress = ttk.Progressbar(
            route_card,
            maximum=1.0,
            variable=self._route_state_vars['progress'],
        )
        progress.grid(row=1, column=0, sticky='ew', pady=4)
        ttk.Label(route_card, textvariable=self._route_state_vars['progress_text']).grid(
            row=2, column=0, sticky='w'
        )
        ttk.Label(route_card, textvariable=self._route_state_vars['message']).grid(
            row=3, column=0, sticky='w'
        )

        # Follower state card
        self._follower_vars = {
            'state': tk.StringVar(value='unknown'),
            'index': tk.StringVar(value='0'),
            'label': tk.StringVar(value='-'),
            'next_label': tk.StringVar(value='-'),
            'stagnation': tk.StringVar(value=''),
            'offset': tk.StringVar(value='左:0.0m / 右:0.0m'),
        }
        follower_card = ttk.LabelFrame(frame, text='フォロワ状態')
        follower_card.grid(row=0, column=1, sticky='nsew', padx=(0, 8))
        follower_card.columnconfigure(0, weight=1)
        ttk.Label(follower_card, textvariable=self._follower_vars['state']).grid(row=0, column=0, sticky='w')
        ttk.Label(follower_card, textvariable=self._follower_vars['index']).grid(row=1, column=0, sticky='w')
        ttk.Label(follower_card, textvariable=self._follower_vars['label']).grid(row=2, column=0, sticky='w')
        ttk.Label(follower_card, textvariable=self._follower_vars['next_label']).grid(
            row=3, column=0, sticky='w'
        )
        ttk.Label(follower_card, textvariable=self._follower_vars['offset']).grid(row=4, column=0, sticky='w')
        ttk.Label(follower_card, textvariable=self._follower_vars['stagnation']).grid(
            row=5, column=0, sticky='w'
        )

        # Right stacked cards
        right_container = ttk.Frame(frame)
        right_container.grid(row=0, column=2, sticky='nsew')
        right_container.rowconfigure(0, weight=1)
        right_container.rowconfigure(1, weight=1)
        right_container.columnconfigure(0, weight=1)

        self._speed_vars = {
            'linear': tk.StringVar(value='0.00 m/s'),
            'angular': tk.StringVar(value='0.0 deg/s'),
        }
        speed_card = ttk.LabelFrame(right_container, text='ロボット速度')
        speed_card.grid(row=0, column=0, sticky='nsew', padx=(0, 0), pady=(0, 4))
        speed_card.columnconfigure(0, weight=1)
        ttk.Label(speed_card, textvariable=self._speed_vars['linear']).grid(row=0, column=0, sticky='w')
        ttk.Label(speed_card, textvariable=self._speed_vars['angular']).grid(row=1, column=0, sticky='w')

        self._target_vars = {
            'distance': tk.StringVar(value='0.0 m'),
            'baseline': tk.StringVar(value='基準: 0.0 m'),
        }
        target_card = ttk.LabelFrame(right_container, text='目標距離')
        target_card.grid(row=1, column=0, sticky='nsew')
        target_card.columnconfigure(0, weight=1)
        ttk.Label(target_card, textvariable=self._target_vars['distance']).grid(row=0, column=0, sticky='w')
        ttk.Label(target_card, textvariable=self._target_vars['baseline']).grid(row=1, column=0, sticky='w')

    def _build_middle_controls(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=1, column=0, sticky='nsew', pady=(0, 8))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        banner_frame = ttk.LabelFrame(frame, text='手動・信号・封鎖イベント')
        banner_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        banner_label = ttk.Label(banner_frame, textvariable=self._event_banner, anchor='center')
        banner_label.pack(fill=tk.BOTH, expand=True)

        control_frame = ttk.LabelFrame(frame, text='制御コマンド')
        control_frame.grid(row=0, column=1, sticky='nsew')
        control_frame.columnconfigure(0, weight=1)
        self._build_command_tabs(control_frame)

    def _build_command_tabs(self, parent: ttk.Frame) -> None:
        notebook = ttk.Notebook(parent)
        notebook.grid(row=0, column=0, sticky='nsew')

        manual_tab = ttk.Frame(notebook)
        sig_tab = ttk.Frame(notebook)
        obstacle_tab = ttk.Frame(notebook)
        road_tab = ttk.Frame(notebook)
        notebook.add(manual_tab, text='manual_start')
        notebook.add(sig_tab, text='sig_recog')
        notebook.add(obstacle_tab, text='obstacle_hint')
        notebook.add(road_tab, text='road_blocked')

        # manual_start tab
        ttk.Label(manual_tab, text='送信値').grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(manual_tab, text='True', variable=self._manual_var, value='True').grid(
            row=0, column=1, sticky='w'
        )
        ttk.Radiobutton(manual_tab, text='False', variable=self._manual_var, value='False').grid(
            row=0, column=2, sticky='w'
        )
        ttk.Button(manual_tab, text='送信', command=self._on_send_manual).grid(row=0, column=3, padx=4)
        ttk.Label(manual_tab, textvariable=self._manual_status).grid(row=1, column=0, columnspan=4, sticky='w')

        # sig_recog tab
        ttk.Label(sig_tab, text='信号').grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(sig_tab, text='GO', variable=self._sig_var, value='GO').grid(row=0, column=1, sticky='w')
        ttk.Radiobutton(sig_tab, text='STOP', variable=self._sig_var, value='STOP').grid(
            row=0, column=2, sticky='w'
        )
        ttk.Button(sig_tab, text='送信', command=self._on_send_sig).grid(row=0, column=3, padx=4)
        ttk.Label(sig_tab, textvariable=self._sig_status).grid(row=1, column=0, columnspan=4, sticky='w')

        # obstacle tab
        ttk.Label(obstacle_tab, text='余裕距離[m]').grid(row=0, column=0, sticky='w')
        ttk.Spinbox(obstacle_tab, from_=0.0, to=50.0, increment=0.5, textvariable=self._obstacle_clearance).grid(
            row=0, column=1, sticky='ew'
        )
        ttk.Label(obstacle_tab, text='左[m]').grid(row=0, column=2, sticky='w')
        ttk.Spinbox(obstacle_tab, from_=-5.0, to=5.0, increment=0.1, textvariable=self._obstacle_left).grid(
            row=0, column=3, sticky='ew'
        )
        ttk.Label(obstacle_tab, text='右[m]').grid(row=0, column=4, sticky='w')
        ttk.Spinbox(obstacle_tab, from_=-5.0, to=5.0, increment=0.1, textvariable=self._obstacle_right).grid(
            row=0, column=5, sticky='ew'
        )
        ttk.Checkbutton(obstacle_tab, text='front_blocked', variable=self._obstacle_blocked).grid(
            row=1, column=0, sticky='w'
        )
        ttk.Button(obstacle_tab, text='送出開始', command=self._on_start_obstacle).grid(row=1, column=1, padx=4)
        ttk.Button(obstacle_tab, text='送出停止', command=self._on_stop_obstacle).grid(row=1, column=2, padx=4)
        ttk.Label(obstacle_tab, textvariable=self._obstacle_status).grid(
            row=2, column=0, columnspan=6, sticky='w'
        )

        for tab in (manual_tab, sig_tab, obstacle_tab, road_tab):
            for col in range(6):
                tab.columnconfigure(col, weight=1)

        # road_blocked tab
        ttk.Label(road_tab, text='送信値').grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(road_tab, text='True', variable=self._road_var, value='True').grid(
            row=0, column=1, sticky='w'
        )
        ttk.Radiobutton(road_tab, text='False', variable=self._road_var, value='False').grid(
            row=0, column=2, sticky='w'
        )
        ttk.Button(road_tab, text='送信', command=self._on_send_road).grid(row=0, column=3, padx=4)
        ttk.Label(road_tab, textvariable=self._road_status).grid(row=1, column=0, columnspan=4, sticky='w')

    def _build_images(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=2, column=0, sticky='nsew')
        frame.columnconfigure(0, weight=1, uniform='img')
        frame.columnconfigure(1, weight=1, uniform='img')
        frame.columnconfigure(2, weight=1, uniform='img')
        frame.rowconfigure(0, weight=1)

        self._route_image_label = ttk.Label(frame, text='Route Map')
        self._route_image_label.grid(row=0, column=0, sticky='nsew', padx=(0, 8))

        self._obstacle_image_label = ttk.Label(frame, text='Obstacle View')
        self._obstacle_image_label.grid(row=0, column=1, sticky='nsew', padx=4)

        self._camera_image_label = ttk.Label(frame, text='External Camera')
        self._camera_image_label.grid(row=0, column=2, sticky='nsew', padx=(8, 0))

        self._apply_imagetk_warning_if_needed()

    def _build_launch_sidebar(self, parent: ttk.Frame) -> None:
        sidebar = ttk.Frame(parent)
        sidebar.grid(row=0, column=1, sticky='nsew', padx=(8, 8), pady=8)
        sidebar.rowconfigure(1, weight=1)
        sidebar.columnconfigure(0, weight=1)

        header = ttk.Frame(sidebar)
        header.grid(row=0, column=0, sticky='ew')
        ttk.Button(header, text='全起動', command=self._core.request_launch_all).pack(side=tk.LEFT, padx=2)
        ttk.Button(header, text='全停止', command=self._core.request_stop_all).pack(side=tk.LEFT, padx=2)

        canvas = tk.Canvas(sidebar, highlightthickness=0)
        canvas.grid(row=1, column=0, sticky='nsew')
        scrollbar = ttk.Scrollbar(sidebar, orient=tk.VERTICAL, command=canvas.yview)
        scrollbar.grid(row=1, column=1, sticky='ns')
        canvas.configure(yscrollcommand=scrollbar.set)

        inner = ttk.Frame(canvas)
        canvas.create_window((0, 0), window=inner, anchor='nw')

        def _on_configure(event: tk.Event) -> None:
            canvas.configure(scrollregion=canvas.bbox('all'))

        inner.bind('<Configure>', _on_configure)

        snapshot = self._core.snapshot()
        for idx, (profile_id, state) in enumerate(snapshot.launch_states.items()):
            card = ttk.LabelFrame(inner, text=state.display_name)
            card.grid(row=idx, column=0, sticky='ew', padx=2, pady=4)
            card.columnconfigure(0, weight=1)
            status_var = tk.StringVar(value=self._format_launch_status(state.status))
            ttk.Label(card, textvariable=status_var).grid(row=0, column=0, sticky='w')

            param_var = tk.StringVar(value=state.selected_param or '')
            combo = ttk.Combobox(card, textvariable=param_var, values=state.available_params, state='readonly')
            combo.grid(row=1, column=0, sticky='ew', pady=2)
            combo.bind(
                '<<ComboboxSelected>>',
                lambda _e, pid=profile_id, var=param_var: self._core.update_selected_param(pid, var.get()),
            )

            simulator_var = tk.BooleanVar(value=state.simulator_enabled)
            if state.simulator_launch_file:
                chk = ttk.Checkbutton(
                    card,
                    text='Simulator',
                    variable=simulator_var,
                    command=lambda pid=profile_id, var=simulator_var: self._core.update_simulator_enabled(
                        pid, var.get()
                    ),
                )
                chk.grid(row=2, column=0, sticky='w')
            ttk.Button(card, text='起動', command=lambda pid=profile_id: self._core.request_launch(pid)).grid(
                row=3, column=0, sticky='ew', pady=2
            )
            ttk.Button(card, text='停止', command=lambda pid=profile_id: self._core.request_stop(pid)).grid(
                row=4, column=0, sticky='ew', pady=2
            )

            self._launch_widgets[profile_id] = {
                'status': status_var,
                'param': param_var,
                'sim': simulator_var,
            }

        inner.update_idletasks()
        canvas.configure(scrollregion=canvas.bbox('all'))

        self._log_texts: Dict[str, tk.Text] = {}

    def _build_logs(self, parent: ttk.Frame) -> None:
        for col in (0, 1):
            parent.columnconfigure(col, weight=1, uniform='logcol')
        for row in (0, 1, 2):
            parent.rowconfigure(row, weight=1, uniform='logrow')

        snapshot = self._core.snapshot()
        order = ['route_planner', 'route_manager', 'route_follower', 'robot_navigator', 'obstacle_monitor']
        positions = {
            'route_planner': (0, 0),
            'route_manager': (0, 1),
            'route_follower': (1, 0),
            'robot_navigator': (1, 1),
            'obstacle_monitor': (2, 0),
        }
        for profile_id in order:
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            row, col = positions.get(profile_id, (0, 0))
            frame = ttk.LabelFrame(parent, text=state.display_name)
            frame.grid(row=row, column=col, sticky='nsew', padx=4, pady=4)
            frame.rowconfigure(0, weight=1)
            frame.columnconfigure(0, weight=1)
            text = tk.Text(frame, wrap='none', height=10)
            text.grid(row=0, column=0, sticky='nsew')
            scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
            scrollbar.grid(row=0, column=1, sticky='ns')
            text.configure(yscrollcommand=scrollbar.set)
            self._log_texts[profile_id] = text

    # ---------- コマンドハンドラ ----------
    def _on_send_manual(self) -> None:
        value = self._manual_var.get() == 'True'
        self._core.request_manual_start(value)

    def _on_send_sig(self) -> None:
        go = self._sig_var.get() == 'GO'
        self._core.request_sig_recog(go)

    def _on_send_road(self) -> None:
        value = self._road_var.get() == 'True'
        self._core.request_road_blocked(value)

    def _on_start_obstacle(self) -> None:
        self._core.start_obstacle_override(
            self._obstacle_blocked.get(),
            self._obstacle_clearance.get(),
            self._obstacle_left.get(),
            self._obstacle_right.get(),
        )

    def _on_stop_obstacle(self) -> None:
        self._core.stop_obstacle_override()

    # ---------- 更新処理 ----------
    def _schedule_update(self) -> None:
        self._root.after(REFRESH_INTERVAL_MS, self._refresh)

    def _refresh(self) -> None:
        snapshot = self._core.snapshot()
        self._apply_snapshot(snapshot)
        self._schedule_update()

    def _apply_snapshot(self, snapshot: GuiSnapshot) -> None:
        self._route_state_vars['state'].set(snapshot.route_state.state)
        total = snapshot.route_state.total_waypoints
        current = snapshot.route_state.current_index
        progress = snapshot.route_state.progress
        self._route_state_vars['progress'].set(progress)
        self._route_state_vars['progress_text'].set(f"{current} / {total}")
        self._route_state_vars['message'].set(snapshot.route_state.last_replan_reason)

        self._follower_vars['state'].set(snapshot.follower_state.state)
        self._follower_vars['index'].set(f"Index: {snapshot.follower_state.current_index}")
        self._follower_vars['label'].set(f"現在: {snapshot.follower_state.current_label}")
        self._follower_vars['next_label'].set(f"次: {snapshot.follower_state.next_label}")
        offset = (
            f"左:{snapshot.follower_state.left_offset_m:.1f}m / "
            f"右:{snapshot.follower_state.right_offset_m:.1f}m"
        )
        self._follower_vars['offset'].set(offset)
        stagnation = snapshot.follower_state.stagnation_reason or '滞留:なし'
        self._follower_vars['stagnation'].set(stagnation)

        self._speed_vars['linear'].set(f"{snapshot.cmd_vel.linear_mps:.2f} m/s")
        self._speed_vars['angular'].set(f"{snapshot.cmd_vel.angular_dps:.1f} deg/s")
        self._target_vars['distance'].set(f"{snapshot.target_distance.current_distance_m:.1f} m")
        self._target_vars['baseline'].set(f"基準: {snapshot.target_distance.baseline_distance_m:.1f} m")

        banner = self._build_banner(snapshot)
        self._event_banner.set(banner)

        self._manual_status.set(self._format_manual(snapshot))
        self._sig_status.set(self._format_sig(snapshot))
        self._road_status.set(self._format_road(snapshot))

        self._obstacle_status.set(
            f"front_blocked={'ON' if snapshot.obstacle_hint.front_blocked else 'OFF'}"
        )

        self._update_images(snapshot)
        self._update_launch_states(snapshot)
        self._update_logs(snapshot)

    def _build_banner(self, snapshot: GuiSnapshot) -> str:
        if snapshot.manual_signal.road_blocked:
            return '道路封鎖アラート'
        if snapshot.manual_signal.manual_start:
            return 'manual_start: True'
        sig = snapshot.manual_signal.sig_recog
        if sig == 1:
            return '信号: GO'
        if sig == 2:
            return '信号: STOP'
        return ''

    def _format_manual(self, snapshot: GuiSnapshot) -> str:
        ts = snapshot.manual_signal.manual_timestamp
        return f"現在:{snapshot.manual_signal.manual_start} 時刻:{ts}" if ts else '受信なし'

    def _format_sig(self, snapshot: GuiSnapshot) -> str:
        ts = snapshot.manual_signal.sig_timestamp
        value = snapshot.manual_signal.sig_recog
        label = {1: 'GO', 2: 'STOP'}.get(value, '未定義')
        return f"現在:{label} 時刻:{ts}" if ts else '受信なし'

    def _format_road(self, snapshot: GuiSnapshot) -> str:
        ts = snapshot.manual_signal.road_blocked_timestamp
        return f"現在:{snapshot.manual_signal.road_blocked} 時刻:{ts}" if ts else '受信なし'

    def _update_images(self, snapshot: GuiSnapshot) -> None:
        if not self._image_render_available:
            return
        if snapshot.images.route_map is not None:
            photo = self._create_photo_image(snapshot.images.route_map)
            if photo is not None:
                self._image_refs['route'] = photo
                self._route_image_label.configure(image=photo, text='')
        if snapshot.images.obstacle_view is not None:
            overlay = snapshot.images.obstacle_overlay
            image = snapshot.images.obstacle_view.copy()
            if overlay:
                image = self._draw_overlay(image, overlay)
            photo = self._create_photo_image(image)
            if photo is not None:
                self._image_refs['obstacle'] = photo
                self._obstacle_image_label.configure(image=photo, text='')
        if snapshot.images.external_camera is not None:
            photo = self._create_photo_image(snapshot.images.external_camera)
            if photo is not None:
                self._image_refs['camera'] = photo
                self._camera_image_label.configure(image=photo, text='')

    def _apply_imagetk_warning_if_needed(self) -> None:
        if self._image_render_available:
            return
        for label in (
            self._route_image_label,
            self._obstacle_image_label,
            self._camera_image_label,
        ):
            label.configure(
                text=self._image_warning_message,
                anchor='center',
                justify='center',
                wraplength=320,
            )

    def _create_photo_image(self, image: Image.Image) -> Optional[tk.PhotoImage]:
        """Pillow Image から tk.PhotoImage を生成する。"""

        if PIL_IMAGETK_AVAILABLE and ImageTk is not None:
            return ImageTk.PhotoImage(image)
        if CV2_AVAILABLE and cv2 is not None and np is not None and Image is not None:
            return self._create_photo_image_via_cv(image)
        return None

    def _create_photo_image_via_cv(self, image: Image.Image) -> Optional[tk.PhotoImage]:
        """OpenCV を用いて Pillow Image を PNG 化し tk.PhotoImage を生成する。"""

        rgb_image = image.convert('RGB')
        array = np.array(rgb_image)
        bgr_array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        success, buffer = cv2.imencode('.png', bgr_array)
        if not success:
            return None
        data = base64.b64encode(buffer.tobytes()).decode('ascii')
        return tk.PhotoImage(data=data, format='png')

    def _draw_overlay(self, image: Image.Image, text: str) -> Image.Image:
        if Image is None or not text:
            return image
        base = image.convert('RGBA')
        canvas = Image.new('RGBA', base.size, (0, 0, 0, 0))
        from PIL import ImageDraw, ImageFont

        draw = ImageDraw.Draw(canvas)
        padding = 6
        lines = text.split('\n')
        font = ImageFont.load_default()
        widths = [draw.textlength(line, font=font) for line in lines]
        height = sum(font.getbbox(line)[3] for line in lines) + padding
        width = max(widths) if widths else 0
        draw.rectangle((0, 0, width + padding * 2, height + padding), fill=(0, 0, 0, 160))
        y = padding
        for line in lines:
            draw.text((padding, y), line, fill=(255, 255, 255, 255), font=font)
            y += font.getbbox(line)[3]
        composed = Image.alpha_composite(base, canvas)
        return composed.convert(image.mode)

    def _update_launch_states(self, snapshot: GuiSnapshot) -> None:
        for profile_id, widgets in self._launch_widgets.items():
            state = snapshot.launch_states.get(profile_id)
            if not state:
                continue
            status_var = widgets['status']  # type: ignore[assignment]
            if isinstance(status_var, tk.StringVar):
                status_var.set(self._format_launch_status(state.status))
            param_var = widgets['param']  # type: ignore[assignment]
            if isinstance(param_var, tk.StringVar) and state.selected_param != param_var.get():
                param_var.set(state.selected_param or '')
            sim_var = widgets['sim']  # type: ignore[assignment]
            if isinstance(sim_var, tk.BooleanVar):
                sim_var.set(state.simulator_enabled)

    def _update_logs(self, snapshot: GuiSnapshot) -> None:
        for profile_id, text_widget in self._log_texts.items():
            logs = snapshot.console_logs.get(profile_id)
            if logs is None:
                continue
            text_widget.delete('1.0', tk.END)
            text_widget.insert(tk.END, ''.join(logs))
            text_widget.see(tk.END)

    def _format_launch_status(self, status: NodeLaunchStatus) -> str:
        mapping = {
            NodeLaunchStatus.STOPPED: '停止',
            NodeLaunchStatus.STARTING: '起動中',
            NodeLaunchStatus.RUNNING: '稼働中',
            NodeLaunchStatus.STOPPING: '停止中',
            NodeLaunchStatus.ERROR: 'エラー',
        }
        return mapping.get(status, '不明')

    def run(self) -> None:
        """tkinter メインループを開始する。"""

        self._root.mainloop()


__all__ = ['UiMain']
