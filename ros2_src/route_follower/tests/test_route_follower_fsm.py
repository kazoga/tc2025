from __future__ import annotations

import asyncio
import logging
from typing import Any, List

from route_follower.route_follower.route_follower_fsm import (
    Event,
    ReportStuckRequest,
    RouteFollowerFSM,
    RouteFollowerHooks,
    State,
)


class LogCapture:
    """簡易ロガー."""

    def __init__(self) -> None:
        self.records: List[logging.LogRecord] = []
        self._logger = logging.getLogger("route_follower_fsm_test")
        self._logger.setLevel(logging.DEBUG)
        handler = logging.Handler()
        handler.emit = self.records.append  # type: ignore[assignment]
        self._logger.handlers = [handler]

    @property
    def logger(self) -> logging.Logger:
        return self._logger

    def text(self) -> str:
        return "\n".join(record.getMessage() for record in self.records)


def test_route_received_transition() -> None:
    capture = LogCapture()
    fsm = RouteFollowerFSM(capture.logger)

    asyncio.run(fsm.handle_event(Event.ROUTE_RECEIVED, {"route": 1}))

    assert fsm.state == State.RUNNING
    assert "Transition: IDLE → RUNNING" in capture.text()


def test_stagnation_to_waiting_reroute() -> None:
    capture = LogCapture()
    called: List[ReportStuckRequest] = []

    async def report_hook(request: ReportStuckRequest) -> None:
        await asyncio.sleep(0)
        called.append(request)

    hooks = RouteFollowerHooks(report_stuck=report_hook)
    fsm = RouteFollowerFSM(capture.logger, hooks=hooks)
    asyncio.run(fsm.transition(State.RUNNING))

    asyncio.run(fsm.handle_event(Event.STAGNATION_DETECTED, "blocked"))

    assert fsm.state == State.WAITING_REROUTE
    assert called and called[-1].reason == "blocked"
    assert "Transition: RUNNING → WAITING_REROUTE" in capture.text()


def test_manual_restart() -> None:
    capture = LogCapture()
    fsm = RouteFollowerFSM(capture.logger)
    asyncio.run(fsm.transition(State.WAITING_STOP))

    asyncio.run(fsm.handle_event(Event.MANUAL_START, None))

    assert fsm.state == State.RUNNING
    assert "Transition: WAITING_STOP → RUNNING" in capture.text()


def test_avoidance_cycle() -> None:
    capture = LogCapture()
    fsm = RouteFollowerFSM(capture.logger)
    asyncio.run(fsm.transition(State.AVOIDING))

    asyncio.run(fsm.handle_event(Event.AVOID_FINISHED, None))

    assert fsm.state == State.RUNNING


def test_reroute_completion() -> None:
    capture = LogCapture()
    fsm = RouteFollowerFSM(capture.logger)
    asyncio.run(fsm.transition(State.WAITING_REROUTE))

    asyncio.run(fsm.handle_event(Event.REROUTE_DONE, None))

    assert fsm.state == State.RUNNING


def test_invalid_event_handling() -> None:
    capture = LogCapture()
    fsm = RouteFollowerFSM(capture.logger)

    asyncio.run(fsm.handle_event(Event.AVOID_FINISHED, None))

    assert "No handler for State.IDLE + Event.AVOID_FINISHED" in capture.text()


def test_timer_tick_state_unchanged() -> None:
    capture = LogCapture()
    fsm = RouteFollowerFSM(capture.logger)
    asyncio.run(fsm.transition(State.RUNNING))

    asyncio.run(fsm.handle_event(Event.TIMER_TICK, None))

    assert fsm.state == State.RUNNING
    assert "Timer tick processed in RUNNING" in capture.text()


def test_report_stuck_hook_invoked() -> None:
    capture = LogCapture()
    awaited: List[bool] = []

    async def report_hook(_: ReportStuckRequest) -> None:
        await asyncio.sleep(0)
        awaited.append(True)

    hooks = RouteFollowerHooks(report_stuck=report_hook)
    fsm = RouteFollowerFSM(capture.logger, hooks=hooks)
    asyncio.run(fsm.transition(State.RUNNING))

    asyncio.run(fsm.handle_event(Event.STAGNATION_DETECTED, None))

    assert awaited

