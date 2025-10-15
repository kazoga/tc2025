"""Stub rclpy module for testing without ROS2."""

from __future__ import annotations

from typing import Any, Callable, Dict


class ParameterValue:
    """Parameter value wrapper."""

    def __init__(self, string_value: str = "") -> None:
        self.string_value = string_value


class Parameter:
    """Parameter stub with value accessor."""

    def __init__(self, value: str = "") -> None:
        self._value = value

    def get_parameter_value(self) -> ParameterValue:
        return ParameterValue(string_value=str(self._value))


class _Logger:
    """Simple logger writing to stdout."""

    def info(self, message: str) -> None:
        print(f"[INFO] {message}")

    def warn(self, message: str) -> None:
        print(f"[WARN] {message}")

    def error(self, message: str) -> None:
        print(f"[ERROR] {message}")


class MutuallyExclusiveCallbackGroup:
    """Placeholder callback group."""

    def __init__(self) -> None:
        return


class Node:
    """Minimal ROS2 node stub."""

    def __init__(self, name: str) -> None:
        self._name = name
        self._logger = _Logger()
        self._parameters: Dict[str, Parameter] = {}
        self._services: Dict[str, Callable[..., Any]] = {}

    def declare_parameter(self, name: str, default_value: str) -> None:
        self._parameters[name] = Parameter(default_value)

    def get_parameter(self, name: str) -> Parameter:
        return self._parameters.get(name, Parameter(""))

    def create_service(
        self,
        srv_type: Any,
        name: str,
        callback: Callable[..., Any],
        callback_group: Any | None = None,
    ) -> None:
        self._services[name] = callback
        return None

    def get_logger(self) -> _Logger:
        return self._logger

    def destroy_node(self) -> None:
        self._services.clear()


def init(*_: Any, **__: Any) -> None:
    return None


def spin(_node: Node) -> None:
    return None


def shutdown() -> None:
    return None


__all__ = [
    "Node",
    "MutuallyExclusiveCallbackGroup",
    "init",
    "spin",
    "shutdown",
]
