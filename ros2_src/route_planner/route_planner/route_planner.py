"""Compatibility layer exposing route planner utilities and node."""

from __future__ import annotations

import sys

from . import graph_solver
from .route_builder import *  # noqa: F401,F403
from .route_planner_node import RoutePlannerNode, main

__all__ = [
    name
    for name in globals().keys()
    if not name.startswith("_")
]

graph_solver = graph_solver
route_planner = sys.modules[__name__]
__all__.extend(["graph_solver", "route_planner"])
