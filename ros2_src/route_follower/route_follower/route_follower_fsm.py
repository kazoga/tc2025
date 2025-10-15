"""非ROS依存の route_follower FSM 実装."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Awaitable, Callable, Dict, Optional, Tuple


class State(Enum):
    """FSM状態列挙."""

    IDLE = auto()
    RUNNING = auto()
    WAITING_STOP = auto()
    AVOIDING = auto()
    WAITING_REROUTE = auto()
    FINISHED = auto()
    ERROR = auto()


class Event(Enum):
    """FSMイベント列挙."""

    ROUTE_RECEIVED = auto()
    POSE_UPDATE = auto()
    TARGET_REACHED = auto()
    STAGNATION_DETECTED = auto()
    HINT_RECEIVED = auto()
    AVOID_STEP_DONE = auto()
    AVOID_FINISHED = auto()
    MANUAL_START = auto()
    SIG_RECOG = auto()
    TIMER_TICK = auto()
    REROUTE_DONE = auto()
    REROUTE_FAIL = auto()


@dataclass
class ReportStuckRequest:
    """report_stuck 呼び出し用の簡易データ."""

    reason: str
    metadata: Dict[str, Any]


@dataclass
class RouteFollowerHooks:
    """外部注入コールバック集."""

    report_stuck: Callable[[ReportStuckRequest], Awaitable[None]]


async def _noop_report_stuck(_: ReportStuckRequest) -> None:
    """report_stuck の既定空実装."""

    return None


class RouteFollowerFSM:
    """最小限のイベント駆動FSM."""

    def __init__(
        self,
        logger: Any,
        *,
        hooks: Optional[RouteFollowerHooks] = None,
    ) -> None:
        """初期化."""

        self.logger = logger
        self.state: State = State.IDLE
        self._hooks = hooks or RouteFollowerHooks(report_stuck=_noop_report_stuck)
        self._route_data: Any = None
        self._latest_pose: Any = None
        self._manual_requested: bool = False
        self._sig_recog: Optional[int] = None

        self.handlers: Dict[Tuple[State, Event], Callable[[Any], Awaitable[None]]] = {
            (State.IDLE, Event.ROUTE_RECEIVED): self._on_idle_route_received,
            (State.RUNNING, Event.ROUTE_RECEIVED): self._on_running_route_received,
            (State.RUNNING, Event.STAGNATION_DETECTED): self._on_running_stuck,
            (State.WAITING_STOP, Event.MANUAL_START): self._on_manual_start,
            (State.AVOIDING, Event.AVOID_FINISHED): self._on_avoid_finished,
            (State.WAITING_REROUTE, Event.REROUTE_DONE): self._on_reroute_done,
        }

        async def _timer(_: Any) -> None:
            await self._on_timer_tick()

        for state in State:
            self.handlers[(state, Event.TIMER_TICK)] = _timer

        async def _pose_update(data: Any) -> None:
            self._latest_pose = data

        for state in State:
            self.handlers[(state, Event.POSE_UPDATE)] = _pose_update

        async def _hint(_: Any) -> None:
            return None

        for state in State:
            self.handlers[(state, Event.HINT_RECEIVED)] = _hint

    async def handle_event(self, event: Event, data: Any | None = None) -> None:
        """イベント処理."""

        handler = self.handlers.get((self.state, event))
        if handler is None:
            self.logger.warning(f"No handler for {self.state} + {event}")
            return
        await handler(data)

    async def transition(self, new_state: State) -> None:
        """状態遷移を実行."""

        if new_state == self.state:
            return
        self.logger.info(f"Transition: {self.state.name} → {new_state.name}")
        self.state = new_state

    async def _on_timer_tick(self) -> None:
        """TIMER_TICK共通処理（今回は状態維持のみ）。"""

        self.logger.debug(f"Timer tick processed in {self.state.name}")

    async def _on_idle_route_received(self, route: Any) -> None:
        """IDLEでのROUTE受信処理."""

        self._route_data = route
        await self.transition(State.RUNNING)

    async def _on_running_route_received(self, route: Any) -> None:
        """RUNNING中に新ルートを受信した際の処理."""

        self._route_data = route
        self.logger.info("Route replaced while running.")

    async def _on_running_stuck(self, context: Any) -> None:
        """RUNNINGでの滞留検出処理."""

        reason = "stagnation" if context is None else str(context)
        request = ReportStuckRequest(reason=reason, metadata={"pose": self._latest_pose})
        await self._hooks.report_stuck(request)
        await self.transition(State.WAITING_REROUTE)

    async def _on_manual_start(self, _: Any) -> None:
        """WAITING_STOPでの手動再開."""

        self._manual_requested = True
        await self.transition(State.RUNNING)

    async def _on_avoid_finished(self, _: Any) -> None:
        """回避完了処理."""

        await self.transition(State.RUNNING)

    async def _on_reroute_done(self, _: Any) -> None:
        """再経路完了でRUNNINGへ復帰."""

        await self.transition(State.RUNNING)

