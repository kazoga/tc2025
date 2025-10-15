"""Stub std_msgs definitions."""

from __future__ import annotations


class Header:
    """Minimal header."""

    def __init__(self) -> None:
        self.frame_id = ""
        self.stamp = None


__all__ = ["Header"]
