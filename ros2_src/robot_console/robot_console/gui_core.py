"""GUI スレッドと ROS スレッドを橋渡しする GuiCore 実装。"""

from __future__ import annotations

import json
import queue
import signal
import subprocess
import threading
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image as ImageMsg
try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML が無い環境では JSON のみ対応
    yaml = None  # type: ignore[assignment]

from .utils import (
    CmdVelView,
    ConsoleLogBuffer,
    FollowerStateView,
    GuiCommand,
    GuiCommandType,
    GuiSnapshot,
    ImageBundle,
    ManualSignalView,
    NodeLaunchState,
    NodeLaunchStatus,
    ObstacleHintView,
    RouteStateView,
    TargetDistanceView,
    clone_launch_state,
    convert_image_message,
    create_placeholder_image,
    now,
    resize_with_letter_box,
)


@dataclass
class NodeLaunchProfile:
    """起動対象ノードのメタデータ。"""

    profile_id: str
    display_name: str
    package: str
    launch_file: str
    default_param: Optional[str] = None
    param_argument: Optional[str] = 'param_file'
    simulator_launch_file: Optional[str] = None


class NodeLaunchManager:
    """subprocess ベースでノードを起動しログを収集する。"""

    def __init__(
        self,
        status_callback: Callable[[str, NodeLaunchStatus, Optional[int], Optional[str]], None],
        log_callback: Callable[[str, str], None],
    ) -> None:
        self._status_callback = status_callback
        self._log_callback = log_callback
        self._lock = threading.Lock()
        self._processes: Dict[str, subprocess.Popen[str]] = {}
        self._sim_processes: Dict[str, subprocess.Popen[str]] = {}
        self._threads: List[threading.Thread] = []

    def launch(
        self,
        profile: NodeLaunchProfile,
        param_path: Optional[str],
        simulator_enabled: bool,
    ) -> None:
        """指定ノードを起動する。"""

        with self._lock:
            if profile.profile_id in self._processes:
                raise RuntimeError(f"{profile.profile_id} は既に起動中です")
            self._status_callback(profile.profile_id, NodeLaunchStatus.STARTING, None, None)

            args = [
                "ros2",
                "launch",
                profile.package,
                profile.launch_file,
            ]
            if param_path and profile.param_argument:
                args.append(f"{profile.param_argument}:={param_path}")

            process = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self._processes[profile.profile_id] = process
            self._status_callback(profile.profile_id, NodeLaunchStatus.RUNNING, process.pid, None)
            self._start_reader_threads(profile.profile_id, process)

        if simulator_enabled and profile.simulator_launch_file:
            sim_args = [
                "ros2",
                "launch",
                profile.package,
                profile.simulator_launch_file,
            ]
            sim_process = subprocess.Popen(
                sim_args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            with self._lock:
                self._sim_processes[profile.profile_id] = sim_process
            self._start_reader_threads(f"{profile.profile_id}:sim", sim_process)
            self._status_callback(
                f"{profile.profile_id}:sim",
                NodeLaunchStatus.RUNNING,
                sim_process.pid,
                None,
            )

    def stop(self, profile_id: str) -> None:
        """ノードを停止する。"""

        with self._lock:
            process = self._processes.pop(profile_id, None)
            sim_process = self._sim_processes.pop(profile_id, None)

        if process is not None:
            self._status_callback(profile_id, NodeLaunchStatus.STOPPING, process.pid, None)
            self._terminate_process(process)
            self._status_callback(profile_id, NodeLaunchStatus.STOPPED, None, None)
        else:
            self._status_callback(profile_id, NodeLaunchStatus.STOPPED, None, None)

        if sim_process is not None:
            self._status_callback(
                f"{profile_id}:sim",
                NodeLaunchStatus.STOPPING,
                sim_process.pid,
                None,
            )
            self._terminate_process(sim_process)
            self._status_callback(f"{profile_id}:sim", NodeLaunchStatus.STOPPED, None, None)
        else:
            self._status_callback(f"{profile_id}:sim", NodeLaunchStatus.STOPPED, None, None)

    def _terminate_process(self, process: subprocess.Popen[str]) -> None:
        """プロセスに対し SIGINT → SIGTERM → SIGKILL の順で停止要求を行う。"""

        if process.poll() is not None:
            return
        try:
            process.send_signal(signal.SIGINT)
        except (ProcessLookupError, AttributeError, ValueError):
            process.terminate()
        try:
            process.wait(timeout=5.0)
            return
        except subprocess.TimeoutExpired:
            process.terminate()
        try:
            process.wait(timeout=3.0)
            return
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def _start_reader_threads(self, profile_id: str, process: subprocess.Popen[str]) -> None:
        """stdout/stderr を読み取るスレッドを起動する。"""

        def _reader(stream, prefix: str) -> None:
            try:
                for line in iter(stream.readline, ''):
                    self._log_callback(profile_id, f"[{prefix}] {line.rstrip()}\n")
            finally:
                stream.close()

        if process.stdout is not None:
            thread = threading.Thread(
                target=_reader,
                args=(process.stdout, "INFO"),
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)
        if process.stderr is not None:
            thread = threading.Thread(
                target=_reader,
                args=(process.stderr, "ERR"),
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)


class GuiCore:
    """GUI 更新と ROS I/O を仲介する。"""

    def __init__(self, launch_profiles: Optional[List[NodeLaunchProfile]] = None) -> None:
        self._route_state = RouteStateView()
        self._follower_state = FollowerStateView()
        self._obstacle_hint = ObstacleHintView()
        self._manual_signal = ManualSignalView()
        self._target_distance = TargetDistanceView()
        self._cmd_vel = CmdVelView()
        self._images = ImageBundle()
        self._placeholders = {
            'route': create_placeholder_image((640, 360), 'No Image'),
            'obstacle': create_placeholder_image((400, 400), 'No Image'),
            'camera_drive': create_placeholder_image((480, 270), 'No Image'),
            'camera_signal': create_placeholder_image((480, 360), 'No Image'),
        }
        self._images.route_map = self._placeholders['route']
        self._images.obstacle_view = self._placeholders['obstacle']
        self._images.external_camera = self._placeholders['camera_drive']
        self._launch_states: Dict[str, NodeLaunchState] = {}
        self._log_buffers: Dict[str, ConsoleLogBuffer] = {}
        self._lock = threading.Lock()
        self._command_queue: queue.Queue[GuiCommand] = queue.Queue()
        self._current_target: Optional[PoseStamped] = None
        self._current_pose: Optional[PoseStamped] = None
        self._drive_camera_frame = self._placeholders['camera_drive']
        self._signal_camera_frame = self._placeholders['camera_signal']
        self._launch_manager = NodeLaunchManager(self._on_launch_status, self._on_log_received)
        self._profiles = launch_profiles or default_launch_profiles()
        self._initialize_launch_states()
        self._load_additional_params()

    def _initialize_launch_states(self) -> None:
        for profile in self._profiles:
            state = NodeLaunchState(
                profile_id=profile.profile_id,
                display_name=profile.display_name,
                package=profile.package,
                launch_file=profile.launch_file,
                param_argument=profile.param_argument,
                simulator_launch_file=profile.simulator_launch_file,
                selected_param=profile.default_param,
            )
            state.available_params = self._discover_params(profile)
            if state.selected_param and state.selected_param not in state.available_params:
                state.available_params.insert(0, state.selected_param)
            if state.selected_param is None and state.available_params:
                state.selected_param = state.available_params[0]
            self._launch_states[profile.profile_id] = state
            self._log_buffers[profile.profile_id] = ConsoleLogBuffer()

    def _load_additional_params(self) -> None:
        config_root = Path(__file__).resolve().parent.parent / 'config' / 'node_launch_profiles.yaml'
        if not config_root.exists():
            return
        raw_text = config_root.read_text(encoding='utf-8')
        content = None
        if yaml is not None:
            try:
                content = yaml.safe_load(raw_text)
            except yaml.YAMLError:  # type: ignore[attr-defined]
                content = None
        if content is None:
            try:
                content = json.loads(raw_text)
            except json.JSONDecodeError:
                return
        if not isinstance(content, dict):
            return
        for profile_id, profile_conf in content.items():
            if profile_id not in self._launch_states:
                continue
            additional = profile_conf.get('additional_params', [])
            state = self._launch_states[profile_id]
            for item in additional:
                if item not in state.available_params:
                    state.available_params.append(item)

    def _discover_params(self, profile: NodeLaunchProfile) -> List[str]:
        params: List[str] = []
        search_dirs: List[Path] = []
        try:
            share_dir = Path(get_package_share_directory(profile.package))
            for sub in ('params', 'config'):
                candidate = share_dir / sub
                if candidate.exists():
                    search_dirs.append(candidate)
        except Exception:
            pass
        extra_root = Path(__file__).resolve().parent.parent / 'config' / 'node_params' / profile.package
        if extra_root.exists():
            search_dirs.append(extra_root)
        for directory in search_dirs:
            for path in directory.glob('**/*.yaml'):
                params.append(str(path))
        params = sorted(set(params))
        return params

    # ---------- 状態更新メソッド ----------
    def update_route_state(self, msg) -> None:
        with self._lock:
            self._route_state.route_version = msg.route_version
            self._route_state.total_waypoints = msg.total_waypoints
            self._route_state.current_index = msg.current_index
            self._route_state.progress = (
                msg.current_index / msg.total_waypoints if msg.total_waypoints else 0.0
            )
            self._route_state.state = self._translate_route_status(msg.status)
            self._route_state.last_replan_reason = msg.message
            self._route_state.last_replan_time = now()

    def update_manager_status(self, msg) -> None:
        with self._lock:
            self._route_state.state = msg.state
            self._route_state.last_replan_reason = msg.last_cause or msg.decision
            self._route_state.last_replan_time = now()

    def update_route(self, msg) -> None:
        image = convert_image_message(msg.route_image)
        with self._lock:
            if image is not None:
                self._images.route_map = resize_with_letter_box(image, (640, 360))
            else:
                self._images.route_map = self._placeholders['route']
            self._route_state.route_version = msg.version

    def update_follower_state(self, msg) -> None:
        with self._lock:
            self._follower_state.state = msg.state
            self._follower_state.current_index = msg.current_index
            self._follower_state.current_label = msg.current_waypoint_label
            self._follower_state.next_label = msg.next_waypoint_label
            self._follower_state.front_blocked_majority = msg.front_blocked_majority
            self._follower_state.left_offset_m = msg.left_offset_m_median
            self._follower_state.right_offset_m = msg.right_offset_m_median
            self._follower_state.stagnation_reason = msg.last_stagnation_reason
            self._follower_state.retry_count = msg.avoidance_attempt_count
            self._target_distance.current_distance_m = msg.distance_to_target
            self._target_distance.updated_at = now()

    def update_obstacle_hint(self, msg) -> None:
        with self._lock:
            self._obstacle_hint.front_blocked = msg.front_blocked
            self._obstacle_hint.front_clearance_m = msg.front_clearance_m
            self._obstacle_hint.left_offset_m = msg.left_offset_m
            self._obstacle_hint.right_offset_m = msg.right_offset_m
            self._obstacle_hint.updated_at = now()
            overlay = (
                f"遮蔽:{'あり' if msg.front_blocked else 'なし'} "
                f"余裕:{msg.front_clearance_m:.1f}m\n"
                f"左:{msg.left_offset_m:.1f}m 右:{msg.right_offset_m:.1f}m"
            )
            self._images.obstacle_overlay = overlay

    def update_obstacle_image(self, msg: ImageMsg) -> None:
        image = convert_image_message(msg)
        with self._lock:
            if image is None:
                self._images.obstacle_view = self._placeholders['obstacle']
            else:
                self._images.obstacle_view = resize_with_letter_box(image, (400, 400))

    def update_manual_start(self, msg) -> None:
        with self._lock:
            self._manual_signal.manual_start = bool(msg.data)
            self._manual_signal.manual_timestamp = now()

    def update_sig_recog(self, msg) -> None:
        with self._lock:
            self._manual_signal.sig_recog = msg.data
            self._manual_signal.sig_timestamp = now()
            self._update_camera_frame_locked()

    def update_road_blocked(self, msg, source: str = 'external') -> None:
        with self._lock:
            self._manual_signal.road_blocked = bool(msg.data)
            self._manual_signal.road_blocked_timestamp = now()
            self._manual_signal.road_blocked_source = source

    def update_cmd_vel(self, msg) -> None:
        with self._lock:
            self._cmd_vel.linear_mps = msg.linear.x
            self._cmd_vel.angular_dps = msg.angular.z * 180.0 / 3.141592653589793
            self._cmd_vel.updated_at = now()

    def _update_camera_frame_locked(self) -> None:
        mode = 'signal' if self._manual_signal.sig_recog == 2 else 'drive'
        if mode == 'signal':
            frame = self._signal_camera_frame or self._placeholders['camera_signal']
        else:
            frame = self._drive_camera_frame or self._placeholders['camera_drive']
        self._images.external_camera = frame
        self._images.camera_mode = mode

    def update_active_target(self, msg: PoseStamped) -> None:
        with self._lock:
            if self._current_target is not None and self._current_pose is not None:
                prev_distance = self._compute_distance(self._current_target, self._current_pose)
                self._target_distance.baseline_distance_m = prev_distance
            self._current_target = msg
            if self._current_pose is not None:
                self._target_distance.current_distance_m = self._compute_distance(msg, self._current_pose)
                self._target_distance.updated_at = now()

    def update_amcl_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_pose = msg
            if self._current_target is not None:
                self._target_distance.current_distance_m = self._compute_distance(
                    self._current_target, msg
                )
                if self._target_distance.baseline_distance_m == 0:
                    self._target_distance.baseline_distance_m = self._target_distance.current_distance_m
                self._target_distance.updated_at = now()

    def update_camera_image(self, msg: ImageMsg, mode: str) -> None:
        image = convert_image_message(msg)
        size = (480, 270) if mode == 'drive' else (480, 360)
        resized = (
            resize_with_letter_box(image, size)
            if image is not None
            else self._placeholders['camera_drive' if mode == 'drive' else 'camera_signal']
        )
        with self._lock:
            if mode == 'drive':
                self._drive_camera_frame = resized
            else:
                self._signal_camera_frame = resized
            self._update_camera_frame_locked()

    def update_sensor_viewer(self, msg: ImageMsg) -> None:
        self.update_obstacle_image(msg)

    def append_console_log(self, profile_id: str, line: str) -> None:
        buffer = self._log_buffers.get(profile_id)
        if buffer:
            buffer.append(line)

    # ---------- コマンド処理 ----------
    def request_manual_start(self, value: bool) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.MANUAL_START, {'value': value}))

    def request_sig_recog(self, go: bool) -> None:
        value = 1 if go else 2
        self._command_queue.put(GuiCommand(GuiCommandType.SIG_RECOG, {'value': value}))

    def request_road_blocked(self, value: bool) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.ROAD_BLOCKED, {'value': value}))

    def start_obstacle_override(self, front_blocked: bool, clearance: float, left: float, right: float) -> None:
        payload = {
            'front_blocked': front_blocked,
            'front_clearance_m': clearance,
            'left_offset_m': left,
            'right_offset_m': right,
        }
        self._command_queue.put(GuiCommand(GuiCommandType.OBSTACLE_HINT_OVERRIDE, payload))

    def stop_obstacle_override(self) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.OBSTACLE_HINT_STOP, {}))

    def request_launch(self, profile_id: str) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.LAUNCH_NODE, {'profile_id': profile_id}))

    def request_stop(self, profile_id: str) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.STOP_NODE, {'profile_id': profile_id}))

    def request_launch_all(self) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.LAUNCH_ALL, {}))

    def request_stop_all(self) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.STOP_ALL, {}))

    def update_selected_param(self, profile_id: str, param_path: Optional[str]) -> None:
        self._command_queue.put(
            GuiCommand(GuiCommandType.UPDATE_PARAM, {'profile_id': profile_id, 'param_path': param_path})
        )

    def update_simulator_enabled(self, profile_id: str, enabled: bool) -> None:
        self._command_queue.put(
            GuiCommand(
                GuiCommandType.TOGGLE_SIMULATOR,
                {'profile_id': profile_id, 'enabled': enabled},
            )
        )

    def get_next_command(self, timeout: Optional[float] = None) -> Optional[GuiCommand]:
        try:
            return self._command_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    # ---------- スナップショット ----------
    def snapshot(self) -> GuiSnapshot:
        with self._lock:
            launch_states = {pid: clone_launch_state(state) for pid, state in self._launch_states.items()}
            logs = {pid: buf.snapshot() for pid, buf in self._log_buffers.items()}
            images = ImageBundle(
                route_map=self._images.route_map,
                obstacle_view=self._images.obstacle_view,
                external_camera=self._images.external_camera,
                obstacle_overlay=self._images.obstacle_overlay,
                camera_mode=self._images.camera_mode,
            )
            snapshot = GuiSnapshot(
                route_state=replace(self._route_state),
                follower_state=replace(self._follower_state),
                obstacle_hint=replace(self._obstacle_hint),
                manual_signal=replace(self._manual_signal),
                target_distance=replace(self._target_distance),
                cmd_vel=replace(self._cmd_vel),
                images=images,
                launch_states=launch_states,
                console_logs=logs,
            )
        return snapshot

    # ---------- launch コールバック ----------
    def _on_launch_status(
        self,
        profile_id: str,
        status: NodeLaunchStatus,
        process_id: Optional[int],
        error: Optional[str],
    ) -> None:
        base_id, _, suffix = profile_id.partition(':')
        state = self._launch_states.get(base_id)
        if not state:
            return
        is_simulator = suffix == 'sim'
        with self._lock:
            state.last_action_time = now()
            if is_simulator:
                state.simulator_process_id = process_id
                if status == NodeLaunchStatus.STOPPED:
                    state.simulator_process_id = None
                if status == NodeLaunchStatus.ERROR and error:
                    state.error_message = error
            else:
                state.status = status
                state.process_id = process_id
                if status == NodeLaunchStatus.STOPPED:
                    state.process_id = None
                if error:
                    state.error_message = error
                elif status == NodeLaunchStatus.RUNNING:
                    state.error_message = ''

    def _on_log_received(self, profile_id: str, line: str) -> None:
        base_id, _, suffix = profile_id.partition(':')
        prefix = '[SIM] ' if suffix == 'sim' else ''
        self.append_console_log(base_id, f"{prefix}{line}")

    # ---------- コマンド適用（ROS ノード側で使用） ----------
    def handle_command(self, command: GuiCommand) -> None:
        if command.command_type == GuiCommandType.UPDATE_PARAM:
            self._apply_param_update(command.payload)
            return
        if command.command_type == GuiCommandType.TOGGLE_SIMULATOR:
            self._apply_simulator_toggle(command.payload)
            return
        if command.command_type == GuiCommandType.LAUNCH_NODE:
            self._handle_launch_request(command.payload['profile_id'])
        elif command.command_type == GuiCommandType.STOP_NODE:
            self._handle_stop_request(command.payload['profile_id'])
        elif command.command_type == GuiCommandType.LAUNCH_ALL:
            for profile in self._profiles:
                self._handle_launch_request(profile.profile_id)
        elif command.command_type == GuiCommandType.STOP_ALL:
            for profile in reversed(self._profiles):
                self._handle_stop_request(profile.profile_id)

    def _handle_launch_request(self, profile_id: str) -> None:
        state = self._launch_states.get(profile_id)
        profile = next((p for p in self._profiles if p.profile_id == profile_id), None)
        if not state or not profile:
            return
        try:
            self._launch_manager.launch(profile, state.selected_param, state.simulator_enabled)
        except Exception as exc:  # pylint: disable=broad-except
            self._on_launch_status(profile_id, NodeLaunchStatus.ERROR, None, str(exc))

    def _handle_stop_request(self, profile_id: str) -> None:
        try:
            self._launch_manager.stop(profile_id)
        except Exception as exc:  # pylint: disable=broad-except
            self._on_launch_status(profile_id, NodeLaunchStatus.ERROR, None, str(exc))

    def _apply_param_update(self, payload: Dict[str, object]) -> None:
        profile_id = payload.get('profile_id')
        param_path = payload.get('param_path')
        if not isinstance(profile_id, str):
            return
        state = self._launch_states.get(profile_id)
        if not state:
            return
        state.selected_param = str(param_path) if param_path else None

    def _apply_simulator_toggle(self, payload: Dict[str, object]) -> None:
        profile_id = payload.get('profile_id')
        enabled = payload.get('enabled')
        if not isinstance(profile_id, str) or not isinstance(enabled, bool):
            return
        state = self._launch_states.get(profile_id)
        if not state:
            return
        state.simulator_enabled = enabled

    @staticmethod
    def _compute_distance(target: PoseStamped, pose: PoseStamped) -> float:
        dx = target.pose.position.x - pose.pose.position.x
        dy = target.pose.position.y - pose.pose.position.y
        dz = target.pose.position.z - pose.pose.position.z
        return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    @staticmethod
    def _translate_route_status(status: int) -> str:
        mapping = {
            0: 'unknown',
            1: 'idle',
            2: 'running',
            3: 'updating_route',
            4: 'holding',
            5: 'completed',
            6: 'error',
        }
        return mapping.get(status, 'unknown')


def default_launch_profiles() -> List[NodeLaunchProfile]:
    """既定のノード起動プロファイルを生成する。"""

    return [
        NodeLaunchProfile(
            profile_id='route_planner',
            display_name='Route Planner',
            package='route_planner',
            launch_file='route_planner.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='route_manager',
            display_name='Route Manager',
            package='route_manager',
            launch_file='route_manager.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='route_follower',
            display_name='Route Follower',
            package='route_follower',
            launch_file='route_follower.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='robot_navigator',
            display_name='Robot Navigator',
            package='robot_navigator',
            launch_file='robot_navigator.launch.py',
            simulator_launch_file='robot_simulator.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='obstacle_monitor',
            display_name='Obstacle Monitor',
            package='obstacle_monitor',
            launch_file='obstacle_monitor.launch.py',
            param_argument=None,
            simulator_launch_file='laser_scan_simulator.launch.py',
        ),
    ]
