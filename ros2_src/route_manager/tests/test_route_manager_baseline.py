"""Baseline tests capturing the current route_manager behaviour."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from route_manager.route_manager.route_manager import (  # type: ignore
    DECISION_FAILED,
    DECISION_REPLAN,
    DECISION_SKIP,
    RouteManager,
    VersionMM as Version,
)
from route_msgs.msg import Route, Waypoint
from route_msgs.srv import ReportStuck


class DummyPublisher:
    """Publisher stub that records published messages."""

    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


class DummyLogger:
    """Logger stub storing info/warn messages for assertions."""

    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warns: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warn(self, message: str) -> None:
        self.warns.append(message)

    def error(self, message: str) -> None:  # pragma: no cover - not used but keeps parity
        self.warns.append(f"ERROR:{message}")


class DummyClock:
    """Clock stub providing a predictable timestamp."""

    def now(self) -> SimpleNamespace:
        return SimpleNamespace(to_msg=lambda: SimpleNamespace())


@pytest.fixture
def route_manager_node() -> RouteManager:
    """Create a RouteManager instance without invoking its ROS constructor."""

    node = RouteManager.__new__(RouteManager)
    node.version = Version(major=1, minor=0)
    node.skip_threshold_m = 1.0
    node.offset_step_m = 0.5
    node.avoid_max_retry = 3
    node.waiting_deadline_sec = 8.0
    node.auto_request = False

    node.active_route = None
    node.follow_state = None
    node.manager_state = "idle"
    node.last_decision = "none"
    node.last_cause = ""
    node.current_index = -1
    node.current_label = ""
    node.total_waypoints = 0

    node.pub_active_route = DummyPublisher()
    node.pub_route_state = DummyPublisher()
    node.pub_mission_info = DummyPublisher()
    node.pub_manager_status = DummyPublisher()

    logger = DummyLogger()
    node._logger = logger  # type: ignore[attr-defined]
    node.get_logger = lambda: logger
    node.get_clock = lambda: DummyClock()

    return node


def _make_route(coords: list[tuple[str, float, float]]) -> Route:
    route = Route()
    route.waypoints = []
    for label, x, y in coords:
        wp = Waypoint()
        wp.label = label
        wp.pose.position.x = x
        wp.pose.position.y = y
        route.waypoints.append(wp)
    return route


def test_version_roundtrip() -> None:
    version = Version(major=2, minor=5)
    encoded = version.to_int()
    assert encoded == 2005
    decoded = Version(major=encoded // 1000, minor=encoded % 1000)
    assert decoded == version


def test_apply_skip_updates_route_and_state(route_manager_node: RouteManager) -> None:
    route_manager_node.version = Version(major=3, minor=1)
    route_manager_node.active_route = _make_route([
        ("wp0", 0.0, 0.0),
        ("wp1", 0.4, 0.0),
        ("wp2", 0.7, 0.2),
    ])

    applied = route_manager_node._apply_skip(0)

    assert applied is True
    assert route_manager_node.current_index == 0
    assert [wp.label for wp in route_manager_node.active_route.waypoints] == ["wp1", "wp2"]
    assert route_manager_node.manager_state == "running"
    assert route_manager_node.pub_active_route.messages, "Active route should be published"


def test_compute_offset_hint_alternates_sides(route_manager_node: RouteManager) -> None:
    request = ReportStuck.Request()
    request.reason = "stagnation"
    request.avoid_trial_count = 1
    request.last_hint_blocked = True
    request.last_applied_offset_m = 0.0

    next_wp = Waypoint()
    next_wp.left_open = 1.0
    next_wp.right_open = 0.3

    first_hint = route_manager_node._compute_offset_hint(request, next_wp)
    assert first_hint == -route_manager_node.offset_step_m

    request.last_applied_offset_m = 0.2
    second_hint = route_manager_node._compute_offset_hint(request, next_wp)
    assert second_hint == route_manager_node.offset_step_m


def test_can_skip_requires_short_distance(route_manager_node: RouteManager) -> None:
    wps = _make_route([
        ("prev", 0.0, 0.0),
        ("current", 0.3, 0.0),
        ("next", 1.5, 0.0),
    ]).waypoints

    wps[1].not_skip = False

    allowed = route_manager_node._can_skip("no_hint", wps[1], wps, 1)
    assert allowed is False  # next waypoint is too far away

    wps[2].pose.position.x = 0.9
    allowed_close = route_manager_node._can_skip("no_hint", wps[1], wps, 1)
    assert allowed_close is True


def test_report_stuck_returns_replan_with_offset_hint(route_manager_node: RouteManager) -> None:
    route_manager_node.active_route = _make_route([
        ("wp0", 0.0, 0.0),
        ("wp1", 0.1, 0.0),
        ("wp2", 0.2, 0.0),
    ])
    route_manager_node._compute_offset_hint = lambda _req, _wp: 0.25  # type: ignore
    recorded_reason: list[str] = []
    route_manager_node._try_update_route = lambda reason: recorded_reason.append(reason) or True  # type: ignore

    request = ReportStuck.Request()
    request.reason = "stagnation"
    request.current_index = 1
    response = ReportStuck.Response()

    result = route_manager_node._on_report_stuck(request, response)

    assert result.decision == DECISION_REPLAN
    assert pytest.approx(result.offset_hint) == 0.25
    assert recorded_reason == ["offset_replan:+0.25"]
    assert route_manager_node.last_decision == "replan"


def test_report_stuck_returns_skip_when_skip_available(route_manager_node: RouteManager) -> None:
    route_manager_node.active_route = _make_route([
        ("wp0", 0.0, 0.0),
        ("wp1", 0.1, 0.0),
        ("wp2", 0.15, 0.0),
    ])
    route_manager_node._compute_offset_hint = lambda *_args, **_kwargs: None  # type: ignore
    route_manager_node._can_skip = lambda *_args, **_kwargs: True  # type: ignore
    route_manager_node._apply_skip = lambda _idx: True  # type: ignore
    route_manager_node._try_update_route = lambda *_args, **_kwargs: False  # type: ignore

    request = ReportStuck.Request()
    request.reason = "no_hint"
    request.current_index = 1
    response = ReportStuck.Response()

    result = route_manager_node._on_report_stuck(request, response)

    assert result.decision == DECISION_SKIP
    assert route_manager_node.last_decision == "skip"


def test_report_stuck_returns_failed_when_no_action_possible(route_manager_node: RouteManager) -> None:
    route_manager_node.active_route = _make_route([
        ("wp0", 0.0, 0.0),
        ("wp1", 0.1, 0.0),
    ])
    route_manager_node._compute_offset_hint = lambda *_args, **_kwargs: None  # type: ignore
    route_manager_node._can_skip = lambda *_args, **_kwargs: False  # type: ignore
    route_manager_node._try_update_route = lambda *_args, **_kwargs: False  # type: ignore

    request = ReportStuck.Request()
    request.reason = "unknown"
    request.current_index = 1
    response = ReportStuck.Response()

    result = route_manager_node._on_report_stuck(request, response)

    assert result.decision == DECISION_FAILED
    assert "no valid recovery" in result.note
    assert route_manager_node.last_decision == "failed"


def test_report_stuck_invalid_route_flags_failure(route_manager_node: RouteManager) -> None:
    route_manager_node.active_route = None
    request = ReportStuck.Request()
    request.reason = "stagnation"
    request.current_index = 1
    response = ReportStuck.Response()

    result = route_manager_node._on_report_stuck(request, response)

    assert result.decision == DECISION_FAILED
    assert "active_route is not available" in result.note
    assert route_manager_node.last_decision == "failed"
