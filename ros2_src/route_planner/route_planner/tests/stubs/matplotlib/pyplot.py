"""Stub matplotlib.pyplot module."""

from __future__ import annotations

from typing import Any, Tuple


class _Figure:
    """Minimal figure placeholder."""

    def __init__(self, figsize: Tuple[float, float] | None = None) -> None:
        self.figsize = figsize
        self.axes: list[Any] = []

    def add_gridspec(self, **_: Any) -> "_GridSpec":
        return _GridSpec()

    def add_subplot(self, *_: Any, **__: Any) -> "_Axes":
        ax = _Axes()
        self.axes.append(ax)
        return ax


class _Axes:
    """Minimal axes placeholder."""

    def imshow(self, *_: Any, **__: Any) -> None:
        return None

    def set_title(self, *_: Any, **__: Any) -> None:
        return None

    def annotate(self, *_: Any, **__: Any) -> None:
        return None

    def set_xlim(self, *_: Any, **__: Any) -> None:
        return None

    def set_ylim(self, *_: Any, **__: Any) -> None:
        return None

    def axis(self, *_: Any, **__: Any) -> None:
        return None


class _GridSpec:
    """Placeholder for GridSpec."""

    def __getitem__(self, *_: Any) -> Tuple[int, int]:
        return (0, 0)


def imread(*_: Any, **__: Any) -> list[list[int]]:
    """Return dummy image array."""
    return [[0]]


def figure(figsize: Tuple[float, float] | None = None) -> _Figure:
    return _Figure(figsize)


def subplots(*_: Any, **__: Any) -> Tuple[_Figure, _Axes]:
    fig = _Figure()
    ax = _Axes()
    fig.axes.append(ax)
    return fig, ax


def tight_layout(*_: Any, **__: Any) -> None:
    return None


def savefig(*_: Any, **__: Any) -> None:
    return None


def close(*_: Any, **__: Any) -> None:
    return None


__all__ = [
    "imread",
    "figure",
    "subplots",
    "tight_layout",
    "savefig",
    "close",
]
