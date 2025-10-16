from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

# Ensure the ROS2 source packages are importable.
_THIS_DIR = Path(__file__).resolve()
_ROS2_SRC = _THIS_DIR.parents[2]
if str(_ROS2_SRC) not in sys.path:
    sys.path.insert(0, str(_ROS2_SRC))

# Reuse the route_planner pytest stub infrastructure.
_PLANNER_TESTS = _ROS2_SRC / "route_planner" / "route_planner" / "tests"
if str(_PLANNER_TESTS) not in sys.path:
    sys.path.insert(0, str(_PLANNER_TESTS))

# Importing the existing conftest registers the stub path used by route_planner tests.
_planner_conftest_path = _PLANNER_TESTS / "conftest.py"
spec = importlib.util.spec_from_file_location("route_planner_tests_conftest", _planner_conftest_path)
if spec and spec.loader:
    _planner_conftest = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("route_planner_tests_conftest", _planner_conftest)
    spec.loader.exec_module(_planner_conftest)

_STUBS_DIR = _PLANNER_TESTS / "stubs"
if str(_STUBS_DIR) not in sys.path:
    sys.path.insert(0, str(_STUBS_DIR))

# ---------------------------------------------------------------------------
# Additional stub wiring for route_manager specific dependencies.
# ---------------------------------------------------------------------------

# Provide builtin_interfaces.msg.Duration if it does not exist.
if "builtin_interfaces.msg" not in sys.modules:
    builtin_interfaces = types.ModuleType("builtin_interfaces")
    builtin_interfaces_msg = types.ModuleType("builtin_interfaces.msg")

    class Duration:  # noqa: D401 - simple stub
        """Duration message stub."""

        def __init__(self) -> None:
            self.sec = 0
            self.nanosec = 0

    builtin_interfaces_msg.Duration = Duration
    builtin_interfaces.msg = builtin_interfaces_msg  # type: ignore[attr-defined]
    sys.modules["builtin_interfaces"] = builtin_interfaces
    sys.modules["builtin_interfaces.msg"] = builtin_interfaces_msg

# Extend rclpy stub with additional helpers used by route_manager.
import rclpy  # type: ignore  # noqa: E402

if not hasattr(rclpy, "spin_until_future_complete"):
    def _spin_until_future_complete(*_args, **_kwargs) -> None:  # noqa: D401 - stub
        """No-op spin_until_future_complete."""

        return None

    rclpy.spin_until_future_complete = _spin_until_future_complete  # type: ignore[attr-defined]

# rclpy.executors.MultiThreadedExecutor stub
if "rclpy.executors" not in sys.modules:
    executors_mod = types.ModuleType("rclpy.executors")

    class MultiThreadedExecutor:  # noqa: D401 - stub
        """Executor stub used for import compatibility."""

        def __init__(self) -> None:
            return

        def add_node(self, _node) -> None:
            return

        def spin(self) -> None:
            return

    executors_mod.MultiThreadedExecutor = MultiThreadedExecutor
    sys.modules["rclpy.executors"] = executors_mod

# rclpy.qos policies and profile stub
if "rclpy.qos" not in sys.modules:
    qos_mod = types.ModuleType("rclpy.qos")

    class ReliabilityPolicy:  # noqa: D401 - stub
        RELIABLE = "reliable"

    class DurabilityPolicy:  # noqa: D401 - stub
        TRANSIENT_LOCAL = "transient_local"
        VOLATILE = "volatile"

    class HistoryPolicy:  # noqa: D401 - stub
        KEEP_LAST = "keep_last"

    class QoSProfile:  # noqa: D401 - stub
        def __init__(
            self,
            reliability: str | None = None,
            durability: str | None = None,
            history: str | None = None,
            depth: int = 1,
        ) -> None:
            self.reliability = reliability
            self.durability = durability
            self.history = history
            self.depth = depth

    qos_mod.ReliabilityPolicy = ReliabilityPolicy
    qos_mod.DurabilityPolicy = DurabilityPolicy
    qos_mod.HistoryPolicy = HistoryPolicy
    qos_mod.QoSProfile = QoSProfile
    sys.modules["rclpy.qos"] = qos_mod

# Extend route_msgs stubs with messages required by route_manager.
import route_msgs.msg as route_msgs_msg  # type: ignore  # noqa: E402
import route_msgs.srv as route_msgs_srv  # type: ignore  # noqa: E402
from geometry_msgs.msg import Pose  # type: ignore  # noqa: E402
from std_msgs.msg import Header  # type: ignore  # noqa: E402


class FollowerState:  # noqa: D401 - stub
    """Follower state stub containing current index and label."""

    def __init__(self) -> None:
        self.current_index = 0
        self.current_waypoint_label = ""
        self.current_pose = Pose()
        self.state = ""
        self.status = ""


class ManagerStatus:  # noqa: D401 - stub
    """Manager status message stub."""

    def __init__(self) -> None:
        self.header = Header()
        self.state = ""
        self.decision = ""
        self.last_cause = ""
        self.route_version = 0


class MissionInfo:  # noqa: D401 - stub
    """Mission information stub."""

    def __init__(self) -> None:
        self.header = Header()
        self.start_label = ""
        self.goal_label = ""
        self.checkpoint_labels: list[str] = []


class RouteState:  # noqa: D401 - stub
    """Route state message stub."""

    def __init__(self) -> None:
        self.header = Header()
        self.route_version = 0
        self.total_waypoints = 0
        self.current_index = 0
        self.current_label = ""
        self.status = ""
        self.message = ""


route_msgs_msg.FollowerState = FollowerState
route_msgs_msg.ManagerStatus = ManagerStatus
route_msgs_msg.MissionInfo = MissionInfo
route_msgs_msg.RouteState = RouteState

if hasattr(route_msgs_msg, "__all__"):
    for name in ["FollowerState", "ManagerStatus", "MissionInfo", "RouteState"]:
        if name not in route_msgs_msg.__all__:
            route_msgs_msg.__all__.append(name)


class ReportStuck:  # noqa: D401 - stub
    """ReportStuck service stub."""

    class Request:
        def __init__(self) -> None:
            self.current_index = 0
            self.reason = ""
            self.avoid_trial_count = 0
            self.last_hint_blocked = False
            self.last_applied_offset_m = 0.0

    class Response:
        def __init__(self) -> None:
            self.decision = 0
            self.offset_hint = 0.0
            self.waiting_deadline = None
            self.note = ""


route_msgs_srv.ReportStuck = ReportStuck
if hasattr(route_msgs_srv, "__all__") and "ReportStuck" not in route_msgs_srv.__all__:
    route_msgs_srv.__all__.append("ReportStuck")
