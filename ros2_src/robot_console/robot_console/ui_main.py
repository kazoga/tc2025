"""tkinter を利用した robot_console GUI 実装。"""

from __future__ import annotations

import logging
import base64
from datetime import datetime, timedelta, timezone
import tkinter as tk
from tkinter import ttk
from typing import Dict, Optional, Tuple

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

from .gui_core import CAMERA_DISPLAY_SIZE, GuiCore
from .utils import GuiSnapshot, NodeLaunchStatus, resize_with_letter_box

LOGGER = logging.getLogger(__name__)

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
REFRESH_INTERVAL_MS = 200
SIDEBAR_WIDTH = 288
JST = timezone(timedelta(hours=9))
EVENT_BANNER_TTL = timedelta(seconds=60)


def _format_time(value: Optional[datetime]) -> str:
    """日時をHH:MM:SS形式の文字列に変換する。"""

    if value is None:
        return "--:--:--"
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(JST).strftime("%H:%M:%S")


class ImagePanel(ttk.LabelFrame):
    """画像表示とキャプション、必要に応じてオーバレイを備えたパネル。"""

    def __init__(
        self,
        master: tk.Widget,
        title: str,
        *,
        size: Tuple[int, int],
        enable_overlay: bool = False,
    ) -> None:
        super().__init__(master, text=title, padding=(6, 6))
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)

        self._target_size = size
        self._canvas = tk.Canvas(
            self,
            background="#1f1f1f",
            highlightthickness=0,
            width=size[0],
            height=size[1],
        )
        self._canvas.grid(row=0, column=0, sticky="nsew")
        self._canvas.bind("<Configure>", lambda _event: self._relayout())

        self._caption_var = tk.StringVar(value="")
        caption = ttk.Label(self, textvariable=self._caption_var, anchor="e")
        caption.grid(row=1, column=0, sticky="ew", pady=(4, 0))

        self._photo: Optional[tk.PhotoImage] = None
        self._image_item: Optional[int] = None
        self._text_item: Optional[int] = None
        self._overlay_label: Optional[tk.Label] = None
        if enable_overlay:
            self._overlay_label = tk.Label(
                self._canvas,
                background="#000000",
                foreground="#ffffff",
                font=("Helvetica", 10, "bold"),
                padx=8,
                pady=4,
                justify="left",
                anchor="nw",
            )

    @property
    def target_size(self) -> Tuple[int, int]:
        """キャンバスに想定する画像サイズを返す。"""

        return self._target_size

    def update_image(self, photo: Optional[tk.PhotoImage], *, alt_text: str = "画像未取得") -> None:
        """画像と代替テキストを設定する。"""

        if self._image_item is not None:
            self._canvas.delete(self._image_item)
            self._image_item = None
        if self._text_item is not None:
            self._canvas.delete(self._text_item)
            self._text_item = None

        self._photo = photo
        if photo is None:
            self._text_item = self._canvas.create_text(
                0,
                0,
                text=alt_text,
                fill="#f0f0f0",
                font=("Helvetica", 12, "bold"),
                justify="center",
                anchor="center",
                width=max(self._target_size[0] - 20, 50),
            )
        else:
            self._image_item = self._canvas.create_image(0, 0, image=photo, anchor="center")
        self._relayout()

    def update_caption(self, text: str) -> None:
        """キャプションテキストを更新する。"""

        self._caption_var.set(text)

    def update_overlay(self, text: str) -> None:
        """オーバレイテキストを更新する。"""

        if self._overlay_label is None:
            return
        if text:
            self._overlay_label.configure(text=text, background="#000000", foreground="#ffffff")
            self._overlay_label.place(x=12, y=12, anchor="nw")
        else:
            self._overlay_label.place_forget()

    def _relayout(self) -> None:
        """キャンバス中心に画像もしくはテキストを配置し直す。"""

        width = max(self._canvas.winfo_width(), self._target_size[0])
        height = max(self._canvas.winfo_height(), self._target_size[1])
        center_x = width / 2
        center_y = height / 2
        if self._image_item is not None:
            self._canvas.coords(self._image_item, center_x, center_y)
        if self._text_item is not None:
            self._canvas.coords(self._text_item, center_x, center_y)
            self._canvas.itemconfigure(self._text_item, width=max(width - 24, 50))


class UiMain:
    """GuiCore のスナップショットを可視化する tkinter アプリ。"""

    def __init__(self, gui_core: GuiCore) -> None:
        self._core = gui_core
        self._root = tk.Tk()
        self._root.title("robot_console")
        self._root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self._root.minsize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self._configure_style()

        self._image_render_available = PIL_IMAGETK_AVAILABLE or CV2_AVAILABLE
        self._image_warning_message = (
            'Pillow ImageTk と OpenCV の両方が利用できないため画像パネルを無効化しました。\n'
            'python3-pil.imagetk もしくは python3-opencv をインストールしてください。'
        ) if not self._image_render_available else ''

        self._sidebar_visible = True

        self._manual_value = tk.BooleanVar(value=False)
        self._manual_status_var = tk.StringVar(value='現在値: False / 最終送信: --:--:--')
        self._sig_value = tk.IntVar(value=1)
        self._sig_status_var = tk.StringVar(value='最終送信: --:--:--')
        self._road_value = tk.BooleanVar(value=False)
        self._road_status_var = tk.StringVar(value='現在値: False / 最終送信: --:--:--')
        self._obstacle_clearance = tk.DoubleVar(value=3.0)
        self._obstacle_left = tk.DoubleVar(value=0.0)
        self._obstacle_right = tk.DoubleVar(value=0.0)
        self._obstacle_blocked = tk.BooleanVar(value=False)
        self._obstacle_override_state = tk.StringVar(value='状態: 自動追従')
        self._obstacle_override_detail = tk.StringVar(value='受信値: 未取得')
        self._latest_obstacle_state_text = '受信値: 未取得'
        self._obstacle_preview_text = ''
        self._event_banner = tk.StringVar(value='')
        self._image_warning_label: Optional[ttk.Label] = None
        self._image_warning_parent: Optional[ttk.Frame] = None

        self._route_state_vars = {
            'manager': tk.StringVar(value='unknown'),
            'route_status': tk.StringVar(value='unknown'),
            'progress': tk.DoubleVar(value=0.0),
            'progress_text': tk.StringVar(value='0 / 0'),
            'version': tk.StringVar(value='バージョン: 0'),
            'detail': tk.StringVar(value=''),
        }
        self._follower_vars = {
            'state': tk.StringVar(value='unknown'),
            'index': tk.StringVar(value='Index: 0'),
            'label': tk.StringVar(value='現在: -'),
            'next': tk.StringVar(value='次: -'),
            'offsets': tk.StringVar(value='左:+0.0m / 右:+0.0m'),
            'stagnation': tk.StringVar(value='滞留なし'),
        }
        self._velocity_vars = {
            'linear': tk.StringVar(value='0.00 m/s'),
            'angular': tk.StringVar(value='0.0 deg/s'),
        }
        self._target_vars = {
            'distance': tk.StringVar(value='現在距離: 0.0 m'),
            'baseline': tk.StringVar(value='基準距離: 0.0 m'),
            'progress': tk.DoubleVar(value=0.0),
        }

        self._launch_widgets: Dict[str, Dict[str, object]] = {}
        self._log_texts: Dict[str, tk.Text] = {}

        self._build_layout()
        self._on_obstacle_params_changed()
        self._schedule_update()

    def _build_layout(self) -> None:
        self._notebook = ttk.Notebook(self._root)
        self._dashboard_tab = ttk.Frame(self._notebook)
        self._logs_tab = ttk.Frame(self._notebook)
        self._notebook.add(self._dashboard_tab, text='ダッシュボード')
        self._notebook.add(self._logs_tab, text='コンソールログ')
        self._notebook.pack(fill=tk.BOTH, expand=True)

        self._build_dashboard(self._dashboard_tab)
        self._build_logs(self._logs_tab)

    def _configure_style(self) -> None:
        """ダッシュボード全体のスタイルを設定する。"""

        style = ttk.Style(self._root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            # 利用可能なテーマが無い場合は既定のまま利用する。
            pass
        style.configure('Card.TLabelframe', background='#f7f9fc')
        style.configure('Card.TLabelframe.Label', font=('Helvetica', 11, 'bold'))
        style.configure('Bold.TLabel', font=('Helvetica', 11, 'bold'))

    def _build_dashboard(self, parent: tk.Widget) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)

        wrapper = ttk.Frame(parent, padding=10)
        wrapper.grid(row=0, column=0, sticky='nsew')
        wrapper.columnconfigure(0, weight=1)
        wrapper.columnconfigure(1, weight=0)
        wrapper.rowconfigure(0, weight=1)

        main_area = ttk.Frame(wrapper)
        main_area.grid(row=0, column=0, sticky='nsew')
        main_area.columnconfigure(0, weight=1)
        main_area.rowconfigure(1, weight=1)

        top_bar = ttk.Frame(main_area)
        top_bar.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        top_bar.columnconfigure(0, weight=1)
        ttk.Label(top_bar, text='robot_console ダッシュボード', style='Bold.TLabel').grid(
            row=0,
            column=0,
            sticky='w',
        )
        self._sidebar_toggle_btn = ttk.Button(
            top_bar,
            text='◀ ノード起動パネル',
            command=self._toggle_sidebar,
            width=18,
        )
        self._sidebar_toggle_btn.grid(row=0, column=1, sticky='e')

        dashboard_body = ttk.Frame(main_area)
        dashboard_body.grid(row=1, column=0, sticky='nsew')
        dashboard_body.columnconfigure(0, weight=1)
        dashboard_body.rowconfigure(1, weight=1)

        self._build_state_summary(dashboard_body)
        self._build_image_row(dashboard_body)
        self._build_control_panel(dashboard_body)

        self._sidebar_container = ttk.Frame(wrapper)
        self._sidebar_container.grid(row=0, column=1, sticky='ns', padx=(12, 0))
        self._sidebar_container.columnconfigure(0, weight=1)
        self._sidebar_container.rowconfigure(0, weight=1)

        self._sidebar_canvas = tk.Canvas(self._sidebar_container, highlightthickness=0)
        self._sidebar_canvas.grid(row=0, column=0, sticky='nsew')
        self._sidebar_canvas.configure(width=SIDEBAR_WIDTH)
        scrollbar = ttk.Scrollbar(
            self._sidebar_container,
            orient=tk.VERTICAL,
            command=self._sidebar_canvas.yview,
        )
        scrollbar.grid(row=0, column=1, sticky='ns')
        self._sidebar_canvas.configure(yscrollcommand=scrollbar.set)

        self._sidebar_inner = ttk.Frame(self._sidebar_canvas, padding=(6, 10, 6, 10))
        self._sidebar_window = self._sidebar_canvas.create_window(
            (0, 0),
            window=self._sidebar_inner,
            anchor='nw',
        )

        def _on_configure(event: tk.Event) -> None:  # type: ignore[override]
            self._sidebar_canvas.configure(scrollregion=self._sidebar_canvas.bbox('all'))

        self._sidebar_inner.bind('<Configure>', _on_configure)
        self._sidebar_canvas.bind(
            '<Configure>',
            lambda event: self._sidebar_canvas.itemconfigure(
                self._sidebar_window,
                width=event.width,
            ),
        )

        self._build_launch_sidebar(self._sidebar_inner)
        self._update_sidebar_visibility()

    def _toggle_sidebar(self) -> None:
        """ノード起動パネルの表示・非表示を切り替える。"""

        self._sidebar_visible = not self._sidebar_visible
        self._update_sidebar_visibility()

    def _update_sidebar_visibility(self) -> None:
        """サイドバーの表示状態とトグルボタンの文言を更新する。"""

        if self._sidebar_visible:
            self._sidebar_container.grid()
            self._sidebar_toggle_btn.configure(text='◀ ノード起動パネル')
        else:
            self._sidebar_container.grid_remove()
            self._sidebar_toggle_btn.configure(text='▶ ノード起動パネル')

    def _build_state_summary(self, parent: ttk.Frame) -> None:
        summary = ttk.Frame(parent)
        summary.grid(row=0, column=0, sticky='nsew')
        summary.columnconfigure(0, weight=1, uniform='summary')
        summary.columnconfigure(1, weight=1, uniform='summary')
        summary.columnconfigure(2, weight=1, uniform='summary')

        route_frame = ttk.LabelFrame(
            summary,
            text='ルート進捗 (route_state)',
            style='Card.TLabelframe',
        )
        route_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        route_frame.columnconfigure(1, weight=1)
        ttk.Label(route_frame, text='マネージャ状態').grid(row=0, column=0, sticky='w')
        ttk.Label(route_frame, textvariable=self._route_state_vars['manager']).grid(
            row=0,
            column=1,
            sticky='w',
        )
        ttk.Label(route_frame, text='ルート状態').grid(row=1, column=0, sticky='w')
        ttk.Label(route_frame, textvariable=self._route_state_vars['route_status']).grid(
            row=1,
            column=1,
            sticky='w',
        )
        ttk.Label(route_frame, text='進捗').grid(row=2, column=0, sticky='w')
        ttk.Progressbar(
            route_frame,
            maximum=100,
            variable=self._route_state_vars['progress'],
        ).grid(row=2, column=1, sticky='ew', pady=2)
        ttk.Label(route_frame, textvariable=self._route_state_vars['progress_text']).grid(
            row=3,
            column=1,
            sticky='w',
        )
        ttk.Label(route_frame, textvariable=self._route_state_vars['version']).grid(
            row=4,
            column=1,
            sticky='w',
        )
        ttk.Label(route_frame, text='最終イベント').grid(row=5, column=0, sticky='nw')
        ttk.Label(
            route_frame,
            textvariable=self._route_state_vars['detail'],
            justify='left',
            wraplength=220,
        ).grid(row=5, column=1, sticky='w')

        follower_frame = ttk.LabelFrame(
            summary,
            text='フォロワ状態 (follower_state)',
            style='Card.TLabelframe',
        )
        follower_frame.grid(row=0, column=1, sticky='nsew', padx=(0, 8))
        follower_frame.columnconfigure(1, weight=1)
        ttk.Label(follower_frame, text='状態').grid(row=0, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['state']).grid(
            row=0,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='現在インデックス').grid(row=1, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['index']).grid(
            row=1,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='現在ラベル').grid(row=2, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['label']).grid(
            row=2,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='滞留要因').grid(row=3, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['stagnation']).grid(
            row=3,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='左右中央値').grid(row=4, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['offsets']).grid(
            row=4,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='次ウェイポイント').grid(row=5, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['next']).grid(
            row=5,
            column=1,
            sticky='w',
        )

        metrics = ttk.Frame(summary)
        metrics.grid(row=0, column=2, sticky='nsew')
        metrics.columnconfigure(0, weight=1)
        metrics.rowconfigure(0, weight=1)
        metrics.rowconfigure(1, weight=1)

        velocity_frame = ttk.LabelFrame(
            metrics,
            text='ロボット速度 (cmd_vel)',
            style='Card.TLabelframe',
        )
        velocity_frame.grid(row=0, column=0, sticky='nsew', pady=(0, 6))
        velocity_frame.columnconfigure(1, weight=1)
        ttk.Label(velocity_frame, text='並進速度').grid(row=0, column=0, sticky='w')
        ttk.Label(velocity_frame, textvariable=self._velocity_vars['linear']).grid(
            row=0,
            column=1,
            sticky='w',
        )
        ttk.Label(velocity_frame, text='角速度').grid(row=1, column=0, sticky='w')
        ttk.Label(velocity_frame, textvariable=self._velocity_vars['angular']).grid(
            row=1,
            column=1,
            sticky='w',
        )

        target_frame = ttk.LabelFrame(metrics, text='目標までの距離', style='Card.TLabelframe')
        target_frame.grid(row=1, column=0, sticky='nsew')
        target_frame.columnconfigure(0, weight=1)
        ttk.Label(target_frame, textvariable=self._target_vars['distance']).grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Progressbar(
            target_frame,
            maximum=100,
            variable=self._target_vars['progress'],
        ).grid(row=1, column=0, sticky='ew', pady=4)
        ttk.Label(target_frame, textvariable=self._target_vars['baseline']).grid(
            row=2,
            column=0,
            sticky='w',
        )

    def _build_image_row(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=1, column=0, sticky='nsew', pady=(8, 8))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.rowconfigure(0, weight=1)

        self._route_panel = ImagePanel(frame, 'ルート地図', size=(640, 360))
        self._route_panel.grid(row=0, column=0, sticky='nsew', padx=(0, 8))

        self._obstacle_panel = ImagePanel(frame, '障害物ビュー', size=(400, 400), enable_overlay=True)
        self._obstacle_panel.grid(row=0, column=1, sticky='nsew', padx=4)

        self._camera_panel = ImagePanel(frame, '外部カメラ', size=CAMERA_DISPLAY_SIZE)
        self._camera_panel.grid(row=0, column=2, sticky='nsew', padx=(8, 0))

        if not self._image_render_available:
            warning = self._image_warning_message
            for panel in (self._route_panel, self._obstacle_panel, self._camera_panel):
                panel.update_image(None, alt_text=warning)

    def _build_control_panel(self, parent: ttk.Frame) -> None:
        container = ttk.Frame(parent)
        container.grid(row=2, column=0, sticky='ew')
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=2)

        banner_frame = ttk.LabelFrame(
            container,
            text='手動・信号・封鎖イベント',
            style='Card.TLabelframe',
            padding=8,
        )
        banner_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        banner_frame.columnconfigure(0, weight=1)
        banner_frame.rowconfigure(0, weight=1)

        self._banner_default_bg = '#f7f9fc'
        self._banner_default_fg = '#2c3e50'
        self._banner_label = tk.Label(
            banner_frame,
            textvariable=self._event_banner,
            font=('Helvetica', 14, 'bold'),
            anchor='center',
            bg=self._banner_default_bg,
            fg=self._banner_default_fg,
            padx=8,
            pady=16,
            wraplength=220,
            justify='center',
        )
        self._banner_label.grid(row=0, column=0, sticky='nsew')

        control_frame = ttk.LabelFrame(container, text='制御コマンド', padding=4)
        control_frame.grid(row=0, column=1, sticky='nsew')
        control_frame.columnconfigure(0, weight=1)

        notebook = ttk.Notebook(control_frame)
        notebook.grid(row=0, column=0, sticky='nsew')

        manual_tab = ttk.Frame(notebook, padding=6)
        manual_tab.columnconfigure(0, weight=1)
        choice = ttk.Frame(manual_tab)
        choice.grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(choice, text='True', value=True, variable=self._manual_value).grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Radiobutton(choice, text='False', value=False, variable=self._manual_value).grid(
            row=0,
            column=1,
            sticky='w',
            padx=(12, 0),
        )
        ttk.Button(
            manual_tab,
            text='manual_start を送信',
            command=self._on_send_manual,
        ).grid(row=1, column=0, sticky='ew', pady=(6, 0))
        ttk.Label(manual_tab, textvariable=self._manual_status_var).grid(
            row=2,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        notebook.add(manual_tab, text='manual_start')

        sig_tab = ttk.Frame(notebook, padding=6)
        sig_tab.columnconfigure(0, weight=1)
        sig_row = ttk.Frame(sig_tab)
        sig_row.grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(sig_row, text='GO', value=1, variable=self._sig_value).grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Radiobutton(sig_row, text='STOP', value=2, variable=self._sig_value).grid(
            row=0,
            column=1,
            sticky='w',
            padx=(12, 0),
        )
        ttk.Button(
            sig_tab,
            text='sig_recog を送信',
            command=self._on_send_sig,
        ).grid(row=1, column=0, sticky='ew', pady=(6, 0))
        ttk.Label(sig_tab, textvariable=self._sig_status_var).grid(
            row=2,
            column=0,
            sticky='w',
            pady=(4, 0),
        )
        notebook.add(sig_tab, text='sig_recog')

        obstacle_tab = ttk.Frame(notebook, padding=6)
        obstacle_tab.columnconfigure(0, weight=1)
        value_row = ttk.Frame(obstacle_tab)
        value_row.grid(row=0, column=0, sticky='ew')
        value_row.columnconfigure(5, weight=1)
        ttk.Label(value_row, text='余裕距離[m]').grid(row=0, column=0, sticky='e')
        clearance_spin = ttk.Spinbox(
            value_row,
            from_=0.1,
            to=50.0,
            increment=0.1,
            textvariable=self._obstacle_clearance,
            width=6,
            command=self._on_obstacle_params_changed,
        )
        clearance_spin.grid(row=0, column=1, sticky='w', padx=(4, 12))
        ttk.Label(value_row, text='左[m]').grid(row=0, column=2, sticky='e')
        left_spin = ttk.Spinbox(
            value_row,
            from_=-5.0,
            to=5.0,
            increment=0.1,
            textvariable=self._obstacle_left,
            width=6,
            command=self._on_obstacle_params_changed,
        )
        left_spin.grid(row=0, column=3, sticky='w', padx=(4, 12))
        ttk.Label(value_row, text='右[m]').grid(row=0, column=4, sticky='e')
        right_spin = ttk.Spinbox(
            value_row,
            from_=-5.0,
            to=5.0,
            increment=0.1,
            textvariable=self._obstacle_right,
            width=6,
            command=self._on_obstacle_params_changed,
        )
        right_spin.grid(row=0, column=5, sticky='w', padx=(4, 0))

        for spin in (clearance_spin, left_spin, right_spin):
            spin.bind('<FocusOut>', self._on_obstacle_params_changed)
            spin.bind('<Return>', self._on_obstacle_params_changed)

        control_row = ttk.Frame(obstacle_tab)
        control_row.grid(row=1, column=0, sticky='w', pady=(6, 0))
        ttk.Checkbutton(
            control_row,
            text='front_blocked',
            variable=self._obstacle_blocked,
            command=self._on_obstacle_params_changed,
        ).grid(row=0, column=0, sticky='w')
        ttk.Button(control_row, text='送出開始', command=self._on_start_obstacle).grid(
            row=0,
            column=1,
            padx=(12, 0),
        )
        ttk.Button(control_row, text='送出停止', command=self._on_stop_obstacle).grid(
            row=0,
            column=2,
            padx=(8, 0),
        )
        ttk.Label(obstacle_tab, textvariable=self._obstacle_override_state).grid(
            row=2,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        ttk.Label(obstacle_tab, textvariable=self._obstacle_override_detail).grid(
            row=3,
            column=0,
            sticky='w',
        )
        notebook.add(obstacle_tab, text='obstacle_hint')

        road_tab = ttk.Frame(notebook, padding=6)
        road_tab.columnconfigure(0, weight=1)
        road_row = ttk.Frame(road_tab)
        road_row.grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(road_row, text='True', value=True, variable=self._road_value).grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Radiobutton(road_row, text='False', value=False, variable=self._road_value).grid(
            row=0,
            column=1,
            sticky='w',
            padx=(12, 0),
        )
        ttk.Button(road_tab, text='road_blocked を送信', command=self._on_send_road).grid(
            row=1,
            column=0,
            sticky='ew',
            pady=(6, 0),
        )
        ttk.Label(road_tab, textvariable=self._road_status_var).grid(
            row=2,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        notebook.add(road_tab, text='road_blocked')

        self._apply_imagetk_warning_if_needed(control_frame)

    def _build_launch_sidebar(self, parent: ttk.Frame) -> None:
        for child in parent.winfo_children():
            child.destroy()
        parent.columnconfigure(0, weight=1)

        ttk.Label(parent, text='ノード起動パネル', style='Bold.TLabel').grid(
            row=0,
            column=0,
            sticky='w',
            pady=(0, 6),
        )

        button_row = ttk.Frame(parent)
        button_row.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        ttk.Button(button_row, text='全起動', command=self._core.request_launch_all).grid(
            row=0,
            column=0,
            sticky='ew',
            padx=(0, 4),
        )
        ttk.Button(button_row, text='全停止', command=self._core.request_stop_all).grid(
            row=0,
            column=1,
            sticky='ew',
        )

        snapshot = self._core.snapshot()
        self._launch_widgets.clear()
        for idx, (profile_id, state) in enumerate(snapshot.launch_states.items(), start=2):
            card = ttk.LabelFrame(parent, text=state.display_name, padding=6)
            card.grid(row=idx, column=0, sticky='ew', pady=4)
            card.columnconfigure(0, weight=1)

            status_var = tk.StringVar(value=self._format_launch_status(state.status))
            ttk.Label(card, textvariable=status_var).grid(row=0, column=0, sticky='w')

            param_var = tk.StringVar(value=state.selected_param_display or '')
            combo = ttk.Combobox(
                card,
                textvariable=param_var,
                values=state.available_params,
                state='readonly',
            )
            combo.grid(row=1, column=0, sticky='ew', pady=2)
            combo.bind(
                '<<ComboboxSelected>>',
                lambda _e, pid=profile_id, var=param_var: self._core.update_selected_param(
                    pid,
                    var.get(),
                ),
            )

            simulator_var = tk.BooleanVar(value=state.simulator_enabled)
            if state.simulator_launch_file:
                chk = ttk.Checkbutton(
                    card,
                    text='Simulator',
                    variable=simulator_var,
                    command=lambda pid=profile_id, var=simulator_var: self._core.update_simulator_enabled(
                        pid,
                        var.get(),
                    ),
                )
                chk.grid(row=2, column=0, sticky='w')
                button_row_index = 3
            else:
                button_row_index = 2

            ttk.Button(
                card,
                text='起動',
                command=lambda pid=profile_id: self._core.request_launch(pid),
            ).grid(row=button_row_index, column=0, sticky='ew', pady=2)
            ttk.Button(
                card,
                text='停止',
                command=lambda pid=profile_id: self._core.request_stop(pid),
            ).grid(row=button_row_index + 1, column=0, sticky='ew', pady=2)

            self._launch_widgets[profile_id] = {
                'status': status_var,
                'param': param_var,
                'sim': simulator_var,
                'combo': combo,
            }

    def _build_logs(self, parent: ttk.Frame) -> None:
        columns = 2
        snapshot = self._core.snapshot()
        ordered = [
            'route_planner',
            'route_manager',
            'route_follower',
            'robot_navigator',
            'obstacle_monitor',
        ]

        count = sum(1 for pid in ordered if pid in snapshot.launch_states)
        rows = max((count + columns - 1) // columns, 1)
        for col in range(columns):
            parent.columnconfigure(col, weight=1)
        for row in range(rows):
            parent.rowconfigure(row, weight=1)

        self._log_texts.clear()
        index = 0
        for profile_id in ordered:
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            row = index // columns
            col = index % columns
            index += 1
            frame = ttk.LabelFrame(parent, text=state.display_name, padding=6)
            frame.grid(row=row, column=col, sticky='nsew', padx=6, pady=6)
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            text = tk.Text(frame, wrap='none', state='disabled')
            text.grid(row=0, column=0, sticky='nsew')
            v_scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
            v_scrollbar.grid(row=0, column=1, sticky='ns')
            h_scrollbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
            h_scrollbar.grid(row=1, column=0, sticky='ew')
            text.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)
            self._log_texts[profile_id] = text

    # ---------- コマンドハンドラ ----------
    def _on_send_manual(self) -> None:
        value = bool(self._manual_value.get())
        self._core.request_manual_start(value)

    def _on_send_sig(self) -> None:
        go = self._sig_value.get() == 1
        self._core.request_sig_recog(go)

    def _on_send_road(self) -> None:
        value = bool(self._road_value.get())
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

    def _on_obstacle_params_changed(self, *_args) -> None:
        self._obstacle_preview_text = (
            '送信予定: '
            f"front_blocked={self._obstacle_blocked.get()} "
            f"clearance={self._obstacle_clearance.get():.2f}m "
            f"左:{self._obstacle_left.get():+.2f}m 右:{self._obstacle_right.get():+.2f}m"
        )
        self._obstacle_override_detail.set(
            f"{self._latest_obstacle_state_text}\n{self._obstacle_preview_text}"
        )

    # ---------- 更新処理 ----------
    def _schedule_update(self) -> None:
        self._root.after(REFRESH_INTERVAL_MS, self._refresh)

    def _refresh(self) -> None:
        try:
            snapshot = self._core.snapshot()
            self._apply_snapshot(snapshot)
        except Exception as exc:  # pylint: disable=broad-except
            LOGGER.exception("スナップショット更新処理で例外が発生しました: %s", exc)
        finally:
            self._schedule_update()

    def _apply_snapshot(self, snapshot: GuiSnapshot) -> None:
        route = snapshot.route_state
        follower = snapshot.follower_state
        self._route_state_vars['manager'].set(route.manager_state)
        self._route_state_vars['route_status'].set(route.route_status)
        total_waypoints = max(route.total_waypoints, 0)
        progress_ratio = 0.0
        if total_waypoints > 0:
            progress_ratio = max(min(route.current_index / total_waypoints, 1.0), 0.0)
        self._route_state_vars['progress'].set(progress_ratio * 100.0)
        display_index = 0
        if total_waypoints > 0:
            follower_index = max(follower.current_index, 0)
            display_index = min(max(follower_index + 1, 1), total_waypoints)
        self._route_state_vars['progress_text'].set(f"{display_index} / {route.total_waypoints}")
        self._route_state_vars['version'].set(f"バージョン: {route.route_version}")
        detail_parts = []
        if route.last_replan_reason:
            detail_parts.append(route.last_replan_reason)
        if route.last_replan_time:
            detail_parts.append(f"@ {_format_time(route.last_replan_time)}")
        self._route_state_vars['detail'].set(' '.join(detail_parts) or '最新イベントなし')
        self._follower_vars['state'].set(follower.state)
        self._follower_vars['index'].set(f"Index: {follower.current_index}")
        current_label = follower.current_label or route.current_label or '-'
        self._follower_vars['label'].set(f"現在: {current_label}")
        self._follower_vars['next'].set(f"次: {follower.next_label or '-'}")
        if follower.stagnation_reason:
            stagnation = follower.stagnation_reason
        else:
            stagnation = '滞留なし'
        self._follower_vars['stagnation'].set(stagnation)
        offsets = (
            f"左:{follower.left_offset_m:+.2f}m / 右:{follower.right_offset_m:+.2f}m"
        )
        self._follower_vars['offsets'].set(offsets)

        self._velocity_vars['linear'].set(f"{snapshot.cmd_vel.linear_mps:.2f} m/s")
        self._velocity_vars['angular'].set(f"{snapshot.cmd_vel.angular_dps:.1f} deg/s")

        target = snapshot.target_distance
        baseline = target.baseline_distance_m
        ratio = 0.0 if baseline <= 0 else max(min(target.current_distance_m / baseline, 1.0), 0.0)
        self._target_vars['distance'].set(f"現在距離: {target.current_distance_m:.1f} m")
        self._target_vars['baseline'].set(f"基準距離: {baseline:.1f} m")
        self._target_vars['progress'].set(ratio * 100.0)

        banner_text, banner_bg, banner_fg = self._resolve_banner(snapshot)
        self._event_banner.set(banner_text)
        self._banner_label.configure(bg=banner_bg, fg=banner_fg)

        manual = snapshot.manual_signal
        self._manual_status_var.set(
            f"現在値: {manual.manual_start} / 最終送信: {_format_time(manual.manual_timestamp)}"
        )
        if manual.sig_recog in (1, 2):
            self._sig_value.set(manual.sig_recog)
        sig_label = {1: 'GO', 2: 'STOP'}.get(manual.sig_recog, '未定義')
        if manual.sig_recog is None:
            self._sig_status_var.set('最終送信: --:--:--')
        else:
            self._sig_status_var.set(
                f"最終送信: {sig_label} @{_format_time(manual.sig_timestamp)}"
            )
        source_label = self._translate_road_source(manual.road_blocked_source)
        self._road_status_var.set(
            (
                "現在値: "
                f"{manual.road_blocked} / 最終送信: "
                f"{_format_time(manual.road_blocked_timestamp)} / 入力元: {source_label}"
            )
        )

        hint = snapshot.obstacle_hint
        state_label = (
            f"状態: front_blocked={'ON' if hint.front_blocked else 'OFF'} / "
            f"余裕距離:{hint.front_clearance_m:.2f}m"
        )
        self._obstacle_override_state.set(state_label)
        self._latest_obstacle_state_text = (
            f"受信値: 左:{hint.left_offset_m:+.2f}m 右:{hint.right_offset_m:+.2f}m "
            f"更新:{_format_time(hint.updated_at)}"
        )
        self._obstacle_override_detail.set(
            f"{self._latest_obstacle_state_text}\n{self._obstacle_preview_text}"
        )

        self._update_images(snapshot)
        self._update_launch_states(snapshot)
        self._update_logs(snapshot)

    def _resolve_banner(self, snapshot: GuiSnapshot) -> Tuple[str, str, str]:
        """ダッシュボードのイベントバナー表示内容と配色を決定する。"""

        base_text = self._build_banner(snapshot)
        if not base_text:
            return base_text, self._banner_default_bg, self._banner_default_fg

        detail = ''
        background = self._banner_default_bg
        foreground = self._banner_default_fg

        if base_text.startswith('道路封鎖'):
            detail = self._format_road(snapshot)
            background = '#c0392b'
            foreground = '#ffffff'
        elif base_text.startswith('信号: GO'):
            detail = self._format_sig(snapshot)
            background = '#2980b9'
            foreground = '#ffffff'
        elif base_text.startswith('信号: STOP'):
            detail = self._format_sig(snapshot)
            background = '#d35400'
            foreground = '#ffffff'
        elif base_text.startswith('停止線: STOP'):
            detail = self._format_sig(snapshot)
            background = '#8e44ad'
            foreground = '#ffffff'
        elif base_text.startswith('manual_start'):
            detail = self._format_manual(snapshot)
            background = '#16a085'
            foreground = '#ffffff'

        text = base_text if not detail else f"{base_text}\n{detail}"
        return text, background, foreground

    def _build_banner(self, snapshot: GuiSnapshot) -> str:
        current_time = datetime.now(timezone.utc)
        if (
            snapshot.manual_signal.road_blocked
            and self._is_recent(snapshot.manual_signal.road_blocked_timestamp, current_time)
        ):
            return '道路封鎖アラート'
        if snapshot.follower_state.state == 'WAITING_STOP':
            if snapshot.follower_state.signal_stop_active:
                return '信号: STOP'
            if snapshot.follower_state.line_stop_active:
                return '停止線: STOP'
        sig_timestamp = snapshot.manual_signal.sig_timestamp
        sig = snapshot.manual_signal.sig_recog
        if (
            sig == 1
            and self._is_recent(sig_timestamp, current_time)
        ):
            return '信号: GO'
        if (
            sig == 2
            and self._is_recent(sig_timestamp, current_time)
        ):
            return '信号: STOP'
        if (
            snapshot.manual_signal.manual_start
            and self._is_recent(snapshot.manual_signal.manual_timestamp, current_time)
        ):
            return 'manual_start: True'
        return ''

    def _format_manual(self, snapshot: GuiSnapshot) -> str:
        ts = snapshot.manual_signal.manual_timestamp
        if ts:
            return f"送信時刻: {_format_time(ts)}"
        return '送信時刻: --:--:--'

    def _format_sig(self, snapshot: GuiSnapshot) -> str:
        if snapshot.follower_state.state == 'WAITING_STOP':
            if snapshot.follower_state.signal_stop_active:
                return '停止要因: 信号 STOP'
            if snapshot.follower_state.line_stop_active:
                return '停止要因: 停止線 STOP'
        ts = snapshot.manual_signal.sig_timestamp
        value = snapshot.manual_signal.sig_recog
        label = {1: 'GO', 2: 'STOP'}.get(value, '未定義')
        if value is None:
            return '受信なし'
        if ts:
            return f"最終送信: {label} @{_format_time(ts)}"
        return f"最終送信: {label}"

    def _format_road(self, snapshot: GuiSnapshot) -> str:
        ts = snapshot.manual_signal.road_blocked_timestamp
        source_label = self._translate_road_source(snapshot.manual_signal.road_blocked_source)
        if ts:
            return (
                f"現在:{snapshot.manual_signal.road_blocked} "
                f"時刻:{_format_time(ts)} 入力元:{source_label}"
            )
        return f"入力元:{source_label} 受信なし"

    @staticmethod
    def _is_recent(timestamp: Optional[datetime], current_time: datetime) -> bool:
        """指定時刻がイベントバナー表示期間内かを判定する。"""

        if timestamp is None:
            return False
        if timestamp.tzinfo is None:
            ts_utc = timestamp.replace(tzinfo=timezone.utc)
        else:
            ts_utc = timestamp.astimezone(timezone.utc)
        delta = current_time - ts_utc
        if delta.total_seconds() < 0:
            return True
        return delta <= EVENT_BANNER_TTL

    @staticmethod
    def _translate_road_source(source: Optional[str]) -> str:
        """road_blocked の入力元を日本語ラベルへ変換する。"""

        mapping = {
            'console': 'GUI',
            'external': '外部',
        }
        if not source:
            return '不明'
        return mapping.get(source, source)

    def _update_images(self, snapshot: GuiSnapshot) -> None:
        if not self._image_render_available:
            return

        def _build_photo(
            source: Optional[Image.Image],
            target_size: Tuple[int, int],
        ) -> Optional[tk.PhotoImage]:
            if source is None:
                return None
            working = source.copy() if hasattr(source, 'copy') else source
            try:
                if hasattr(working, 'size') and working.size != target_size:
                    working = resize_with_letter_box(working, target_size)
            except Exception:  # pragma: no cover - サイズ調整失敗時は元画像を使用
                pass
            return self._create_photo_image(working)

        route_photo = _build_photo(snapshot.images.route_map, self._route_panel.target_size)
        self._route_panel.update_image(route_photo, alt_text='画像未取得')
        self._route_panel.update_caption(
            'ルート地図: 表示中' if route_photo else 'ルート地図: 画像未取得'
        )

        obstacle_source = snapshot.images.obstacle_view
        if obstacle_source is not None and snapshot.images.obstacle_overlay:
            obstacle_source = self._draw_overlay(obstacle_source.copy(), snapshot.images.obstacle_overlay)
        obstacle_photo = _build_photo(obstacle_source, self._obstacle_panel.target_size)
        self._obstacle_panel.update_image(obstacle_photo, alt_text='画像未取得')
        overlay_lines = [
            f"遮蔽:{'YES' if snapshot.obstacle_hint.front_blocked else 'NO'}",
            f"余裕:{snapshot.obstacle_hint.front_clearance_m:.2f}m",
            f"左:{snapshot.obstacle_hint.left_offset_m:+.2f}m 右:{snapshot.obstacle_hint.right_offset_m:+.2f}m",
        ]
        self._obstacle_panel.update_overlay('\n'.join(overlay_lines))
        self._obstacle_panel.update_caption(
            '障害物ビュー: 表示中' if obstacle_photo else '障害物ビュー: 画像未取得'
        )

        camera_photo = _build_photo(snapshot.images.external_camera, self._camera_panel.target_size)
        self._camera_panel.update_image(camera_photo, alt_text='画像未取得')
        camera_mode = snapshot.images.camera_mode or 'unknown'
        caption = f"外部カメラ: {camera_mode}"
        if not camera_photo:
            caption += ' (未取得)'
        self._camera_panel.update_caption(caption)

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
            if isinstance(param_var, tk.StringVar) and state.selected_param_display != param_var.get():
                param_var.set(state.selected_param_display or '')
            sim_var = widgets['sim']  # type: ignore[assignment]
            if isinstance(sim_var, tk.BooleanVar):
                sim_var.set(state.simulator_enabled)
            combo_widget = widgets.get('combo')  # type: ignore[index]
            if isinstance(combo_widget, ttk.Combobox):
                current_values = tuple(combo_widget['values'])
                desired_values = tuple(state.available_params)
                if current_values != desired_values:
                    combo_widget['values'] = state.available_params

    def _update_logs(self, snapshot: GuiSnapshot) -> None:
        for profile_id, text_widget in self._log_texts.items():
            logs = snapshot.console_logs.get(profile_id)
            if logs is None:
                continue
            try:
                x_first, _ = text_widget.xview()
            except tk.TclError:
                x_first = 0.0
            try:
                y_first, y_last = text_widget.yview()
            except tk.TclError:
                y_first, y_last = 0.0, 1.0
            follow_tail = y_last >= 0.999
            text_widget.configure(state='normal')
            text_widget.delete('1.0', tk.END)
            text_widget.insert(tk.END, ''.join(logs))
            if follow_tail:
                text_widget.see(tk.END)
            else:
                text_widget.yview_moveto(max(min(y_first, 1.0), 0.0))
            text_widget.xview_moveto(max(min(x_first, 1.0), 0.0))
            text_widget.configure(state='disabled')

    def _apply_imagetk_warning_if_needed(self, parent: Optional[ttk.Frame] = None) -> None:
        """画像描画に必要な依存が無い場合に警告ラベルを表示する。"""

        if parent is not None:
            self._image_warning_parent = parent

        parent_frame = self._image_warning_parent
        if parent_frame is None:
            return

        if self._image_render_available:
            if self._image_warning_label is not None:
                self._image_warning_label.destroy()
                self._image_warning_label = None
            return

        if self._image_warning_label is None:
            self._image_warning_label = ttk.Label(
                parent_frame,
                text=self._image_warning_message,
                foreground='#c0392b',
                wraplength=360,
                justify='left',
            )
            self._image_warning_label.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        else:
            self._image_warning_label.configure(text=self._image_warning_message)

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
