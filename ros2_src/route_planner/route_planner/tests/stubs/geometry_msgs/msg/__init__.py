"""Stub geometry_msgs message definitions for testing."""

from __future__ import annotations


class _Point:
    """Simple 3D point."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Quaternion:
    """Minimal quaternion representation."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class Pose:
    """Pose with position and orientation."""

    def __init__(self) -> None:
        self.position = _Point()
        self.orientation = Quaternion()


class PoseStamped:
    """Pose with timestamp placeholder."""

    def __init__(self) -> None:
        self.header = None
        self.pose = Pose()


__all__ = ["Quaternion", "Pose", "PoseStamped"]
