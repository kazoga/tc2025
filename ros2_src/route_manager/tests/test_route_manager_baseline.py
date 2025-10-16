"""Baseline tests validating the RouteManagerNode wrapper."""

from __future__ import annotations

import asyncio
import time
import types

import pytest

from route_manager.route_manager import (
    DECISION_FAILED,
    DECISION_REPLAN,
    RouteManagerNode,
)
from route_msgs.msg import FollowerState
from route_msgs.srv import ReportStuck


@pytest.fixture()
def route_manager_node() -> RouteManagerNode:
    node = RouteManagerNode()
    yield node
    node.destroy_node()


def _bind_async(method, instance):
    return types.MethodType(method, instance)


def test_node_initializes_fsm(route_manager_node: RouteManagerNode) -> None:
    assert route_manager_node.fsm.state == "IDLE"


def test_report_stuck_success_returns_replan(route_manager_node: RouteManagerNode) -> None:
    async def fake_handle(self, event: str, data) -> dict[str, str]:
        await asyncio.sleep(0)
        assert event == "REPORT_STUCK"
        return {"success": True, "message": "Replan requested"}

    route_manager_node.fsm.handle_event = _bind_async(fake_handle, route_manager_node.fsm)

    request = ReportStuck.Request()
    response = ReportStuck.Response()

    result = route_manager_node._on_report_stuck(request, response)

    assert result.decision == DECISION_REPLAN
    assert result.note == "Replan requested"


def test_report_stuck_failure_returns_failed(route_manager_node: RouteManagerNode) -> None:
    async def fake_handle(self, event: str, data) -> dict[str, str]:
        await asyncio.sleep(0)
        return {"success": False, "message": "Unable"}

    route_manager_node.fsm.handle_event = _bind_async(fake_handle, route_manager_node.fsm)

    request = ReportStuck.Request()
    response = ReportStuck.Response()

    result = route_manager_node._on_report_stuck(request, response)

    assert result.decision == DECISION_FAILED
    assert result.note == "Unable"


def test_report_stuck_waits_for_fsm_completion(route_manager_node: RouteManagerNode) -> None:
    async def delayed_handle(self, event: str, data) -> dict[str, str]:
        await asyncio.sleep(0.05)
        return {"success": True, "message": "Done"}

    route_manager_node.fsm.handle_event = _bind_async(delayed_handle, route_manager_node.fsm)

    request = ReportStuck.Request()
    response = ReportStuck.Response()

    start = time.perf_counter()
    result = route_manager_node._on_report_stuck(request, response)
    elapsed = time.perf_counter() - start

    assert elapsed >= 0.05
    assert result.decision == DECISION_REPLAN


def test_follower_state_transitions_to_running(route_manager_node: RouteManagerNode) -> None:
    transitions: list[str] = []

    async def record_transition(self, new_state: str) -> None:
        transitions.append(new_state)

    route_manager_node.fsm.transition = _bind_async(record_transition, route_manager_node.fsm)

    msg = FollowerState()
    msg.state = "RUNNING"
    route_manager_node._on_follower_state(msg)

    time.sleep(0.05)

    assert "RUNNING" in transitions
