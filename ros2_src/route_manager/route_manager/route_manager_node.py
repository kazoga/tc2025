"""Compatibility wrapper for legacy imports expecting ``route_manager_node``."""

from __future__ import annotations

from .route_manager import RouteManager, main

__all__ = ["RouteManager", "main"]
