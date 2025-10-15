from __future__ import annotations

import types
from typing import List, Tuple

import pytest

from geometry_msgs.msg import PoseStamped  # type: ignore
from route_follower.route_follower.route_follower import (  # type: ignore
    DECISION_FAILED,
    DECISION_REPLAN,
    DECISION_SKIP,
    FollowerStatus,
    RouteFollower,
)
from route_msgs.msg import ObstacleAvoidanceHint, Route, Waypoint  # type: ignore
from route_msgs.srv import ReportStuck  # type: ignore


def _set_parameter(node: RouteFollower, name: str, value: float) -> None:
    """Helper to override a declared parameter value in the stub node."""

    param = node.get_parameter(name)
    param.value = value


def _make_route(labels: List[str]) -> Route:
    route = Route()
    route.header.frame_id = "map"
    for idx, label in enumerate(labels):
        wp = Waypoint()
        wp.label = label
        wp.index = idx
        wp.pose.position.x = float(idx)
        wp.pose.position.y = 0.0
        route.waypoints.append(wp)
    route.version = 7
    return route


def _send_pose(node: RouteFollower, x: float, y: float, *, stamp: bool = True) -> None:
    msg = PoseStamped()
    if stamp:
        msg.header = types.SimpleNamespace()
    msg.pose.position.x = x
    msg.pose.position.y = y
    node._on_pose(msg)


def _emit_hint(node: RouteFollower, blocked: bool, left: float, right: float) -> None:
    hint = ObstacleAvoidanceHint()
    hint.front_blocked = blocked
    hint.left_is_open = left
    hint.right_is_open = right
    node._on_hint(hint)


def _advance_for_state(time_controller, seconds: float) -> None:
    time_controller.advance(seconds)


def _prepare_running_route(
    node: RouteFollower,
    labels: List[str],
    time_controller,
    *,
    initial_pose: Tuple[float, float] | None = None,
) -> None:
    route = _make_route(labels)
    node._on_route(route)
    pose = initial_pose if initial_pose is not None else (-5.0, 0.0)
    _send_pose(node, pose[0], pose[1])
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()
    assert node._status == FollowerStatus.RUNNING


def _clear_pose_history(node: RouteFollower) -> None:
    node._pose_hist.clear()


def test_periodic_loop_runs_once(follower_node: RouteFollower, time_controller) -> None:
    node = follower_node

    assert pytest.approx(1.0 / 20.0) == node.timer.period
    assert node.timer.call_count == 0

    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    assert node.timer.call_count == 1
    assert node._status == FollowerStatus.IDLE
    assert len(node.pub_state.published) == 1
    state_msg = node.pub_state.published[-1]
    assert state_msg.state == FollowerStatus.IDLE.name


def test_stagnation_detection_and_reset(follower_node: RouteFollower, time_controller) -> None:
    node = follower_node
    _set_parameter(node, "arrival_threshold", 0.2)
    _set_parameter(node, "window_sec", 1.0)
    _set_parameter(node, "progress_epsilon_m", 0.01)
    _set_parameter(node, "min_speed_mps", 0.01)
    _set_parameter(node, "stagnation_duration_sec", 0.5)
    _set_parameter(node, "reroute_timeout_sec", 5.0)

    _prepare_running_route(node, ["start", "goal"], time_controller)
    _clear_pose_history(node)

    _send_pose(node, 0.3, 0.0)
    _advance_for_state(time_controller, 0.2)
    _send_pose(node, 0.3, 0.0)

    client = node.cli_report_stuck
    response = ReportStuck.Response()
    response.decision = DECISION_REPLAN
    response.note = "reroute"
    response.waiting_deadline = types.SimpleNamespace(sec=0, nanosec=0)
    client.enqueue_response(response)

    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    _advance_for_state(time_controller, 1.0)
    _send_pose(node, 0.3, 0.0)
    _advance_for_state(time_controller, 0.1)
    _send_pose(node, 0.3, 0.0)
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    assert len(client.requests) == 1
    assert node._last_stagnation_reason == "no_hint"
    assert node._status == FollowerStatus.WAITING_REROUTE

    node._status = FollowerStatus.RUNNING
    _advance_for_state(time_controller, 0.5)
    _send_pose(node, 1.0, 0.0)
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    assert node._stagnation_hold_start is None
    assert node._status == FollowerStatus.RUNNING


def test_waypoint_progression_and_bounds(follower_node: RouteFollower, time_controller) -> None:
    node = follower_node
    _prepare_running_route(node, ["wp0", "wp1"], time_controller)
    _clear_pose_history(node)

    _send_pose(node, 0.0, 0.0)
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    assert node._index == 1
    assert node._status == FollowerStatus.RUNNING
    assert len(node.pub_target.published) == 2
    next_target = node.pub_target.published[-1]
    assert pytest.approx(1.0) == next_target.pose.position.x

    _send_pose(node, 1.0, 0.0)
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    assert node._status == FollowerStatus.FINISHED
    assert node._index == len(node._wp_list) - 1


def test_publisher_invocations(follower_node: RouteFollower, time_controller) -> None:
    node = follower_node
    _prepare_running_route(node, ["solo"], time_controller)
    _clear_pose_history(node)

    _send_pose(node, -5.0, 0.0)
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    first_publish_count = len(node.pub_target.published)
    assert first_publish_count == 1
    assert len(node.pub_state.published) >= 1

    _advance_for_state(time_controller, 1.2)
    node.timer.trigger()

    assert len(node.pub_target.published) == first_publish_count + 1
    assert node.pub_target.published[-1] is node._last_pub_target
    assert len(node.pub_state.published) >= 2


def test_report_stuck_invocation(follower_node: RouteFollower, time_controller) -> None:
    node = follower_node
    _set_parameter(node, "arrival_threshold", 0.2)
    _set_parameter(node, "window_sec", 1.0)
    _set_parameter(node, "progress_epsilon_m", 0.01)
    _set_parameter(node, "min_speed_mps", 0.01)
    _set_parameter(node, "stagnation_duration_sec", 0.5)
    _prepare_running_route(node, ["start"], time_controller)
    _clear_pose_history(node)

    for _ in range(3):
        _send_pose(node, 0.3, 0.0)
        _advance_for_state(time_controller, 0.2)

    client = node.cli_report_stuck
    response = ReportStuck.Response()
    response.decision = DECISION_SKIP
    response.note = "skip"
    response.waiting_deadline = types.SimpleNamespace(sec=1, nanosec=0)
    client.enqueue_response(response)

    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    _advance_for_state(time_controller, 0.6)
    _send_pose(node, 0.3, 0.0)
    _advance_for_state(time_controller, 0.1)
    _send_pose(node, 0.3, 0.0)
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()

    assert len(client.requests) == 1
    request = client.requests[0]
    assert request.reason == "no_hint"
    assert request.current_index == 0
    assert request.route_version == node._route_version
    assert request.last_hint_blocked is False
    assert request.avoid_trial_count == node._avoid_attempt_count
    assert node._last_stagnation_reason == "no_hint"
    assert node._status == FollowerStatus.WAITING_REROUTE
    assert node._reroute_wait_deadline is not None


def test_error_and_recovery_paths(follower_node: RouteFollower, time_controller) -> None:
    node = follower_node

    bad_route = _make_route(["bad"])
    bad_route.header.frame_id = "odom"
    node._on_route(bad_route)
    _advance_for_state(time_controller, 0.2)
    node.timer.trigger()
    assert node._status == FollowerStatus.ERROR

    _prepare_running_route(node, ["start", "end"], time_controller)
    _clear_pose_history(node)

    _set_parameter(node, "arrival_threshold", 0.2)
    _set_parameter(node, "window_sec", 0.5)
    _set_parameter(node, "progress_epsilon_m", 0.01)
    _set_parameter(node, "min_speed_mps", 0.01)
    _set_parameter(node, "stagnation_duration_sec", 0.2)
    _set_parameter(node, "reroute_timeout_sec", 1.0)
    node.cli_report_stuck.set_service_ready(False)

    for _ in range(3):
        _send_pose(node, 0.3, 0.0)
        _advance_for_state(time_controller, 0.1)
    node.timer.trigger()

    _advance_for_state(time_controller, 0.3)
    _send_pose(node, 0.3, 0.0)
    _advance_for_state(time_controller, 0.1)
    _send_pose(node, 0.3, 0.0)
    _advance_for_state(time_controller, 0.1)
    node.timer.trigger()

    assert node._status == FollowerStatus.WAITING_REROUTE
    assert node.cli_report_stuck.requests == []
    assert node._reroute_wait_deadline is not None

    _advance_for_state(time_controller, 2.0)
    node.timer.trigger()

    assert node._status == FollowerStatus.ERROR
