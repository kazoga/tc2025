"""Stub route_msgs services."""

from __future__ import annotations
from typing import List

from geometry_msgs.msg import PoseStamped
from route_msgs.msg import Route


class GetRoute:
    """GetRoute service stub."""

    class Request:
        def __init__(self) -> None:
            self.start_label = ""
            self.goal_label = ""
            self.checkpoint_labels: List[str] = []

    class Response:
        def __init__(self) -> None:
            self.success = False
            self.message = ""
            self.route = Route()


class UpdateRoute:
    """UpdateRoute service stub."""

    class Request:
        def __init__(self) -> None:
            self.route_version = 0
            self.prev_index = 0
            self.next_index = 0
            self.prev_wp_label = ""
            self.next_wp_label = ""
            self.block_name = ""
            self.current_pose = PoseStamped()

    class Response:
        def __init__(self) -> None:
            self.success = False
            self.message = ""
            self.route = Route()


__all__ = ["GetRoute", "UpdateRoute"]
