"""Stub route_msgs message definitions."""

from __future__ import annotations
from typing import List

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class Waypoint:
    """Waypoint message stub."""

    def __init__(self) -> None:
        self.label = ""
        self.index = 0
        self.pose = Pose()
        self.right_open = 0.0
        self.left_open = 0.0
        self.line_stop = False
        self.signal_stop = False
        self.not_skip = False


class Route:
    """Route message stub."""

    def __init__(self) -> None:
        self.header = Header()
        self.waypoints: List[Waypoint] = []
        self.version = 0
        self.total_distance = 0.0
        self.route_image = Image()


__all__ = ["Route", "Waypoint"]
