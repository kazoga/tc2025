"""非ROS依存の経路マネージャFSM."""

from __future__ import annotations

import asyncio
from typing import Any, Awaitable, Callable, Dict, Optional

EventHandler = Callable[[Any], Awaitable[Dict[str, Any]]]
ReplanCallback = Callable[[Optional[Any]], Awaitable[bool]]


class RouteManagerFSM:
    """非ROS依存のイベント駆動FSM."""

    def __init__(self, logger: Callable[[str], None]) -> None:
        self.state = "IDLE"
        self.logger = logger
        self.handlers: Dict[tuple[str, str], EventHandler] = {
            ("RUNNING", "REPORT_STUCK"): self._running_report_stuck,
            ("WAITING_REROUTE", "REPLAN_DONE"): self._reroute_done,
            ("WAITING_REROUTE", "REPLAN_FAIL"): self._reroute_fail,
        }
        self._replan_cb: Optional[ReplanCallback] = None
        self._transition_hooks: list[Callable[[str, str], None]] = []

    def set_replan_callback(self, callback: ReplanCallback) -> None:
        """再計画要求処理を差し替える."""

        self._replan_cb = callback

    def add_transition_hook(self, hook: Callable[[str, str], None]) -> None:
        """状態遷移監視用のフックを追加."""

        self._transition_hooks.append(hook)

    async def handle_event(self, event: str, data: Optional[Any] = None) -> Dict[str, Any]:
        handler = self.handlers.get((self.state, event))
        if handler:
            return await handler(data)
        self.logger(f"No handler for {self.state}+{event}")
        return {"success": False, "message": "Unhandled event"}

    async def _running_report_stuck(self, data: Optional[Any]) -> Dict[str, Any]:
        ok = await self._request_replan(data)
        if ok:
            await self.transition("WAITING_REROUTE")
            return {"success": True, "message": "Replan requested"}
        await self.transition("ERROR")
        return {"success": False, "message": "Replan failed"}

    async def _reroute_done(self, _data: Optional[Any]) -> Dict[str, Any]:
        await self.transition("RUNNING")
        return {"success": True, "message": "Replan completed"}

    async def _reroute_fail(self, _data: Optional[Any]) -> Dict[str, Any]:
        await self.transition("ERROR")
        return {"success": False, "message": "Replan failed"}

    async def _request_replan(self, data: Optional[Any]) -> bool:
        if self._replan_cb is not None:
            return await self._replan_cb(data)
        await asyncio.sleep(0.1)
        return True

    async def transition(self, new_state: str) -> None:
        self.logger(f"Transition: {self.state} -> {new_state}")
        old_state = self.state
        self.state = new_state
        for hook in list(self._transition_hooks):
            try:
                hook(old_state, new_state)
            except Exception:  # pragma: no cover - フックは例外を無視
                pass
