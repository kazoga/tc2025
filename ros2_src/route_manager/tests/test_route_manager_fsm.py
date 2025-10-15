"""Async FSM tests for RouteManagerFSM."""

from __future__ import annotations

import asyncio

from route_manager.route_manager import RouteManagerFSM


def test_report_stuck_transitions_to_waiting_reroute() -> None:
    async def scenario() -> None:
        fsm = RouteManagerFSM(print)
        await fsm.transition("RUNNING")
        result = await fsm.handle_event("REPORT_STUCK", None)
        assert result["success"]
        assert fsm.state == "WAITING_REROUTE"

    asyncio.run(scenario())
