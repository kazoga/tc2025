from __future__ import annotations

import importlib.util
import sys
import types
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import pytest

# ---------------------------------------------------------------------------
# Register ROS2 stub packages reused from route_planner and route_manager.
# ---------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve()
_ROS2_SRC = _THIS_DIR.parents[2]
if str(_ROS2_SRC) not in sys.path:
    sys.path.insert(0, str(_ROS2_SRC))

_PLANNER_TESTS = _ROS2_SRC / "route_planner" / "route_planner" / "tests"
if str(_PLANNER_TESTS) not in sys.path:
    sys.path.insert(0, str(_PLANNER_TESTS))

_planner_conftest_path = _PLANNER_TESTS / "conftest.py"
_planner_spec = importlib.util.spec_from_file_location("route_planner_tests_conftest", _planner_conftest_path)
if _planner_spec and _planner_spec.loader:
    _planner_module = importlib.util.module_from_spec(_planner_spec)
    sys.modules.setdefault("route_planner_tests_conftest", _planner_module)
    _planner_spec.loader.exec_module(_planner_module)

_MANAGER_TESTS = _ROS2_SRC / "route_manager" / "tests"
_manager_conftest_path = _MANAGER_TESTS / "conftest.py"
_manager_spec = importlib.util.spec_from_file_location("route_manager_tests_conftest", _manager_conftest_path)
if _manager_spec and _manager_spec.loader:
    _manager_module = importlib.util.module_from_spec(_manager_spec)
    sys.modules.setdefault("route_manager_tests_conftest", _manager_module)
    _manager_spec.loader.exec_module(_manager_module)

# ---------------------------------------------------------------------------
# Extend the shared stubs with route_follower specific behaviour.
# ---------------------------------------------------------------------------
import rclpy  # type: ignore  # noqa: E402
from geometry_msgs.msg import Pose  # type: ignore  # noqa: E402
from std_msgs.msg import Header  # type: ignore  # noqa: E402


# Ensure Parameter exposes .value for compatibility with route_follower.
if not hasattr(rclpy.Parameter, "value"):
    @property
    def value(self: rclpy.Parameter) -> Any:  # type: ignore[misc]
        return self._value  # type: ignore[attr-defined]

    @value.setter  # type: ignore[misc]
    def value(self: rclpy.Parameter, new_value: Any) -> None:  # type: ignore[misc]
        self._value = new_value  # type: ignore[attr-defined]

    rclpy.Parameter.value = value  # type: ignore[attr-defined]


class _TimePoint:
    """Monotonic time snapshot used by ClockStub."""

    def __init__(self, time_source: Callable[[], float]) -> None:
        self._time_source = time_source
        self._captured = float(self._time_source())

    @property
    def nanoseconds(self) -> int:
        return int(self._captured * 1e9)

    def to_msg(self) -> types.SimpleNamespace:
        sec = int(self._captured)
        nanosec = int((self._captured - sec) * 1e9)
        msg = types.SimpleNamespace()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg


class _ClockStub:
    """Clock stub that mirrors rclpy Clock.now behaviour."""

    def __init__(self, time_source: Callable[[], float]) -> None:
        self._time_source = time_source

    def now(self) -> _TimePoint:
        return _TimePoint(self._time_source)


class PublisherStub:
    """Publisher stub capturing published messages."""

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class SubscriptionStub:
    """Subscription stub storing callback."""

    def __init__(self, topic: str, callback: Callable[[Any], None]) -> None:
        self.topic = topic
        self.callback = callback
        self.received: List[Any] = []

    def receive(self, msg: Any) -> None:
        self.received.append(msg)
        self.callback(msg)


class FutureStub:
    """Future stub providing manual completion."""

    def __init__(self, response: Any | None = None, delay: float = 0.0) -> None:
        self._response = response
        self._ready_at: float | None = None
        if response is not None and delay <= 0.0:
            self._done = True
        else:
            self._done = False
            if response is not None and delay > 0.0:
                self._ready_at = time.time() + delay
        self._exception: Exception | None = None

    def done(self) -> bool:
        if not self._done and self._ready_at is not None and time.time() >= self._ready_at:
            self._done = True
        return self._done

    def result(self) -> Any:
        if self._exception is not None:
            raise self._exception
        return self._response

    def set_result(self, response: Any) -> None:
        self._response = response
        self._done = True
        self._ready_at = None

    def set_exception(self, exc: Exception) -> None:
        self._exception = exc
        self._done = True
        self._ready_at = None


class ClientStub:
    """Client stub emulating rclpy.client.Client."""

    def __init__(self, service_name: str) -> None:
        self.service_name = service_name
        self.requests: List[Any] = []
        self._ready = True
        self._queue: deque[Tuple[Any | None, float]] = deque()
        self.last_future: FutureStub | None = None

    def service_is_ready(self) -> bool:
        return self._ready

    def set_service_ready(self, ready: bool) -> None:
        self._ready = ready

    def enqueue_response(self, response: Any | None, delay: float = 0.0) -> None:
        self._queue.append((response, delay))

    def call_async(self, request: Any) -> FutureStub:
        self.requests.append(request)
        if self._queue:
            response, delay = self._queue.popleft()
        else:
            response, delay = None, 0.0
        future = FutureStub(response=response, delay=delay)
        self.last_future = future
        return future


class TimerStub:
    """Timer stub exposing manual trigger."""

    def __init__(self, period: float, callback: Callable[[], None]) -> None:
        self.period = period
        self.callback = callback
        self.call_count = 0
        self.last_trigger_time: float | None = None

    def trigger(self) -> None:
        self.call_count += 1
        self.last_trigger_time = time.time()
        self.callback()


_BaseNode = rclpy.Node


class TestNode(_BaseNode):
    """Extended Node stub covering publisher/subscription/timer APIs."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._clock = _ClockStub(time.time)
        self._publishers: Dict[str, PublisherStub] = {}
        self._subscriptions: Dict[str, SubscriptionStub] = {}
        self._timers: List[TimerStub] = []
        self._clients: Dict[str, ClientStub] = {}

    def get_clock(self) -> _ClockStub:
        return self._clock

    def create_publisher(self, msg_type: Any, topic: str, qos_profile: Any) -> PublisherStub:  # noqa: ARG002
        publisher = PublisherStub(topic)
        self._publishers[topic] = publisher
        return publisher

    def create_subscription(
        self, msg_type: Any, topic: str, callback: Callable[[Any], None], qos_profile: Any
    ) -> SubscriptionStub:  # noqa: ARG002
        subscription = SubscriptionStub(topic, callback)
        self._subscriptions[topic] = subscription
        return subscription

    def create_timer(self, timer_period_sec: float, callback: Callable[[], None]) -> TimerStub:
        timer = TimerStub(timer_period_sec, callback)
        self._timers.append(timer)
        return timer

    def create_client(self, srv_type: Any, srv_name: str) -> ClientStub:  # noqa: ARG002
        client = ClientStub(srv_name)
        self._clients[srv_name] = client
        return client


rclpy.Node = TestNode  # type: ignore[assignment]
if "rclpy.node" in sys.modules:
    sys.modules["rclpy.node"].Node = TestNode  # type: ignore[attr-defined]


_ok_flag = True


def ok() -> bool:
    return _ok_flag


def spin_once(_node: Any, timeout_sec: float = 0.0) -> None:  # noqa: ARG001
    # Nothing to do for the stub executor. The timeout is recorded for completeness.
    return None


rclpy.ok = ok  # type: ignore[attr-defined]
rclpy.spin_once = spin_once  # type: ignore[attr-defined]


qos_mod = sys.modules.get("rclpy.qos")
if qos_mod is not None:
    if hasattr(qos_mod, "ReliabilityPolicy") and not hasattr(qos_mod.ReliabilityPolicy, "BEST_EFFORT"):
        qos_mod.ReliabilityPolicy.BEST_EFFORT = "best_effort"  # type: ignore[attr-defined]
    if not hasattr(qos_mod, "QoSProfile"):
        class QoSProfile:  # type: ignore[no-redef]
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

        qos_mod.QoSProfile = QoSProfile


# std_msgs Bool / Int32 stubs -------------------------------------------------
std_msgs_msg = sys.modules.get("std_msgs.msg")
if std_msgs_msg is None:
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    sys.modules["std_msgs.msg"] = std_msgs_msg


class Bool:
    """std_msgs/Bool stub."""

    def __init__(self, data: bool = False) -> None:
        self.data = data


class Int32:
    """std_msgs/Int32 stub."""

    def __init__(self, data: int = 0) -> None:
        self.data = data


std_msgs_msg.Bool = Bool
std_msgs_msg.Int32 = Int32
if hasattr(std_msgs_msg, "__all__"):
    for _name in ["Bool", "Int32"]:
        if _name not in std_msgs_msg.__all__:
            std_msgs_msg.__all__.append(_name)
else:
    std_msgs_msg.__all__ = ["Header", "Bool", "Int32"]


# route_msgs message/service extensions --------------------------------------
route_msgs_msg_mod = sys.modules.get("route_msgs.msg")
if route_msgs_msg_mod is None:
    route_msgs_msg_mod = types.ModuleType("route_msgs.msg")
    sys.modules["route_msgs.msg"] = route_msgs_msg_mod


class FollowerState:
    """FollowerState stub exposing Phase2 fields used in route_follower."""

    def __init__(self) -> None:
        self.header = Header()
        self.route_version = 0
        self.state = ""
        self.current_index = 0
        self.current_waypoint_label = ""
        self.next_waypoint_label = ""
        self.current_pose = Pose()
        self.distance_to_target = 0.0
        self.avoidance_attempt_count = 0
        self.front_blocked_majority = False
        self.hint_left_open_m_median = 0.0
        self.hint_right_open_m_median = 0.0
        self.last_stagnation_reason = ""
        self.avoidance_active = False
        self.avoidance_subgoal_label = ""


class ObstacleAvoidanceHint:
    """Hint message stub used for cache evaluation."""

    def __init__(self) -> None:
        self.front_blocked = False
        self.left_is_open = 0.0
        self.right_is_open = 0.0


route_msgs_msg_mod.FollowerState = FollowerState
route_msgs_msg_mod.ObstacleAvoidanceHint = ObstacleAvoidanceHint
for _name in ["FollowerState", "ObstacleAvoidanceHint"]:
    if hasattr(route_msgs_msg_mod, "__all__"):
        if _name not in route_msgs_msg_mod.__all__:
            route_msgs_msg_mod.__all__.append(_name)


route_msgs_srv_mod = sys.modules.get("route_msgs.srv")
if route_msgs_srv_mod is None:
    route_msgs_srv_mod = types.ModuleType("route_msgs.srv")
    sys.modules["route_msgs.srv"] = route_msgs_srv_mod


class ReportStuck:
    """ReportStuck service stub including Phase2 fields."""

    class Request:
        def __init__(self) -> None:
            self.route_version = 0
            self.current_index = 0
            self.current_wp_label = ""
            self.current_pose_map = Pose()
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


route_msgs_srv_mod.ReportStuck = ReportStuck
if hasattr(route_msgs_srv_mod, "__all__") and "ReportStuck" not in route_msgs_srv_mod.__all__:
    route_msgs_srv_mod.__all__.append("ReportStuck")


# ---------------------------------------------------------------------------
# Pytest fixtures
# ---------------------------------------------------------------------------


class TimeController:
    """Deterministic time source for unit tests."""

    def __init__(self) -> None:
        self._now = 1000.0

    def time(self) -> float:
        return self._now

    def advance(self, seconds: float) -> None:
        self._now += seconds

    def reset(self, value: float = 1000.0) -> None:
        self._now = value


@pytest.fixture
def time_controller(monkeypatch: pytest.MonkeyPatch) -> TimeController:
    controller = TimeController()
    monkeypatch.setattr(time, "time", controller.time)
    # When route_follower module is imported the time module is already bound.
    # Patching the global time module ensures the node sees the deterministic clock.
    yield controller


@pytest.fixture
def follower_node(time_controller: TimeController) -> Any:
    from route_follower.route_follower.route_follower import RouteFollower

    time_controller.reset()
    node = RouteFollower()
    yield node
    node.destroy_node()
