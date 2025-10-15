"""Launch shim for backward compatibility."""

from __future__ import annotations

from .route_manager_node import main

__all__ = ["main"]

if __name__ == "__main__":
    main()
