"""GuiCore の単体テスト。"""

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

if 'ament_index_python' not in sys.modules:
    mock_root = types.ModuleType('ament_index_python')
    packages_mod = types.ModuleType('ament_index_python.packages')

    def _get_package_share_directory(_name: str) -> str:
        raise RuntimeError('ament_index_python is unavailable in tests')

    packages_mod.get_package_share_directory = _get_package_share_directory
    mock_root.packages = packages_mod
    sys.modules['ament_index_python'] = mock_root
    sys.modules['ament_index_python.packages'] = packages_mod


def _install_geometry_stubs() -> None:
    geometry_msgs = types.ModuleType('geometry_msgs')
    msg_module = types.ModuleType('geometry_msgs.msg')

    class PoseStamped:  # pragma: no cover - スタブのみ
        def __init__(self) -> None:
            class _Pose:
                def __init__(self) -> None:
                    class _Point:
                        def __init__(self) -> None:
                            self.x = 0.0
                            self.y = 0.0
                            self.z = 0.0

                    self.position = _Point()

            self.pose = _Pose()

    msg_module.PoseStamped = PoseStamped
    geometry_msgs.msg = msg_module
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = msg_module


def _install_sensor_stubs() -> None:
    sensor_msgs = types.ModuleType('sensor_msgs')
    msg_module = types.ModuleType('sensor_msgs.msg')

    class Image:  # pragma: no cover - スタブのみ
        height = 0
        width = 0
        encoding = 'rgb8'
        data = b''

    msg_module.Image = Image
    sensor_msgs.msg = msg_module
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = msg_module


if 'geometry_msgs' not in sys.modules:
    _install_geometry_stubs()

if 'sensor_msgs' not in sys.modules:
    _install_sensor_stubs()

if 'numpy' not in sys.modules:
    numpy_stub = types.ModuleType('numpy')

    def _frombuffer(data, dtype):  # pragma: no cover - スタブのみ
        return []

    numpy_stub.frombuffer = _frombuffer
    sys.modules['numpy'] = numpy_stub

if 'PIL' not in sys.modules:
    pil_module = types.ModuleType('PIL')
    image_module = types.ModuleType('PIL.Image')
    imagetk_module = types.ModuleType('PIL.ImageTk')

    class _DummyImage:  # pragma: no cover - スタブのみ
        mode = 'RGB'
        width = 1
        height = 1

        def resize(self, size, _filter=None):
            self.width, self.height = size
            return self

        def copy(self):
            return self

        def paste(self, _other, box=None):
            return None

        def convert(self, _mode):
            return self

        def alpha_composite(self, _other):
            return None

    def _fromarray(_array, mode=None):
        return _DummyImage()

    def _new(mode, size, color):
        return _DummyImage()

    image_module.Image = _DummyImage
    image_module.fromarray = _fromarray
    image_module.new = _new
    imagetk_module.PhotoImage = _DummyImage
    pil_module.Image = image_module
    pil_module.ImageTk = imagetk_module
    sys.modules['PIL'] = pil_module
    sys.modules['PIL.Image'] = image_module
    sys.modules['PIL.ImageTk'] = imagetk_module

from robot_console.gui_core import GuiCore
from robot_console.utils import ConsoleLogBuffer, GuiCommandType


def test_console_log_buffer_capacity() -> None:
    buffer = ConsoleLogBuffer(capacity=3)
    buffer.append('line1')
    buffer.append('line2')
    buffer.append('line3')
    buffer.append('line4')
    snapshot = buffer.snapshot()
    assert snapshot == ['line2', 'line3', 'line4']


def test_command_queue_order() -> None:
    core = GuiCore(launch_profiles=[])
    core.request_manual_start(True)
    core.request_sig_recog(True)
    core.start_obstacle_override(True, 2.0, 0.1, -0.1)

    commands = [core.get_next_command() for _ in range(3)]
    types = [cmd.command_type for cmd in commands if cmd]
    assert types == [
        GuiCommandType.MANUAL_START,
        GuiCommandType.SIG_RECOG,
        GuiCommandType.OBSTACLE_HINT_OVERRIDE,
    ]


def test_snapshot_is_copy() -> None:
    core = GuiCore(launch_profiles=[])
    snapshot = core.snapshot()
    snapshot.route_state.state = 'modified'
    new_snapshot = core.snapshot()
    assert new_snapshot.route_state.state != 'modified'
