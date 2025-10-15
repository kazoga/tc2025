"""Stub sensor_msgs message definitions."""

from __future__ import annotations


class Image:
    """Simple image container."""

    def __init__(self) -> None:
        self.header = None
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.step = 0
        self.data = b""


__all__ = ["Image"]
