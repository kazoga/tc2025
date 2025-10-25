"""robot_console パッケージの公開インタフェース。"""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = ['GuiCore', 'RobotConsoleNode', 'UiMain']


def __getattr__(name: str) -> Any:
    """遅延インポートで主要コンポーネントを公開する。"""

    if name == 'GuiCore':
        return import_module('robot_console.gui_core').GuiCore
    if name == 'RobotConsoleNode':
        return import_module('robot_console.robot_console_node').RobotConsoleNode
    if name == 'UiMain':
        return import_module('robot_console.ui_main').UiMain
    raise AttributeError(name)
