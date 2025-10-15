"""Tests for route builder utilities without ROS2 dependencies."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Generator, List, Tuple

import pytest

from route_planner.route_planner import route_planner as rp


class _StubQuaternion:
    """Simple quaternion stub."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _StubPosition:
    """Position stub for Pose."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _StubPose:
    """Pose stub containing position and orientation."""

    def __init__(self) -> None:
        self.position = _StubPosition()
        self.orientation = _StubQuaternion()


class _StubWaypoint:
    """Waypoint stub mimicking ROS message."""

    def __init__(self) -> None:
        self.label = ""
        self.index = 0
        self.pose = _StubPose()
        self.right_open = 0.0
        self.left_open = 0.0
        self.line_stop = False
        self.signal_stop = False
        self.not_skip = False


class _StubHeader:
    """Header stub."""

    def __init__(self) -> None:
        self.frame_id = ""


class _StubImage:
    """Image stub."""

    def __init__(self) -> None:
        self.header = _StubHeader()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.step = 0
        self.data = b""


class _StubRoute:
    """Route stub for pack_route_msg."""

    def __init__(self) -> None:
        self.header = None
        self.waypoints: List[_StubWaypoint] = []
        self.version = 0
        self.total_distance = 0.0
        self.route_image: _StubImage | None = None


@pytest.fixture(autouse=True)
def _patch_ros_stubs(monkeypatch: pytest.MonkeyPatch) -> Generator[None, None, None]:
    """ROS依存を回避するためのスタブを適用する。"""
    monkeypatch.setattr(rp, "Quaternion", _StubQuaternion)
    monkeypatch.setattr(rp, "Waypoint", _StubWaypoint)
    monkeypatch.setattr(rp, "Header", _StubHeader)
    monkeypatch.setattr(rp, "Image", _StubImage)
    monkeypatch.setattr(rp, "Route", _StubRoute)
    yield


def _quat_to_yaw(quat: _StubQuaternion) -> float:
    """クォータニオンからヨー角を算出する。"""
    return math.atan2(2.0 * quat.w * quat.z, 1.0 - 2.0 * quat.z * quat.z)


def test_yaw_to_quaternion_basic() -> None:
    """45度のヨーで正しいクォータニオンになるか確認する。"""
    yaw = math.pi / 4.0
    quat = rp.yaw_to_quaternion(yaw)
    assert pytest.approx(quat.z, rel=1e-6) == math.sin(yaw / 2.0)
    assert pytest.approx(quat.w, rel=1e-6) == math.cos(yaw / 2.0)


def test_concat_with_dedup_transfers_label() -> None:
    """重複ノード結合時にラベルが維持されるか確認する。"""
    base = [_StubWaypoint(), _StubWaypoint()]
    base[-1].pose.position.x = 1.0
    base[-1].pose.position.y = 1.0
    ext = [_StubWaypoint(), _StubWaypoint()]
    ext[0].pose.position.x = 1.0
    ext[0].pose.position.y = 1.0
    ext[0].label = "A"
    result = rp.concat_with_dedup(base, ext)
    assert len(result) == 3
    assert result[-1].pose.position.x == ext[-1].pose.position.x
    assert result[-2].label == "A"


def test_stamp_edge_end_labels_assigns_correctly() -> None:
    """エッジ端点にラベルが設定されることを検証する。"""
    waypoints = [_StubWaypoint() for _ in range(3)]
    rp.stamp_edge_end_labels(waypoints, "start", "goal")
    assert waypoints[0].label == "start"
    assert waypoints[-1].label == "goal"


def test_resolve_path_handles_relative_and_absolute(tmp_path: Path) -> None:
    """相対パスがベースディレクトリから解決されることを確認する。"""
    base_dir = tmp_path
    rel_path = "segment.csv"
    expected = base_dir / rel_path
    resolved = rp.resolve_path(str(base_dir), rel_path)
    assert Path(resolved) == expected
    absolute = rp.resolve_path(None, str(expected))
    assert Path(absolute) == expected


def test_adjust_orientations_generates_continuous_yaw() -> None:
    """姿勢再計算が隣接点の方向と一致することを確認する。"""
    waypoints = [_StubWaypoint() for _ in range(3)]
    coords: List[Tuple[float, float]] = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    for wp, (x_val, y_val) in zip(waypoints, coords, strict=True):
        wp.pose.position.x = x_val
        wp.pose.position.y = y_val
    rp.adjust_orientations(waypoints, [(0, 2)], [1])
    expected_yaws = [0.0, math.pi / 2.0, math.pi / 2.0]
    for wp, expected in zip(waypoints, expected_yaws, strict=True):
        actual = _quat_to_yaw(wp.pose.orientation)
        assert pytest.approx(actual, abs=1e-6) == expected


def test_pack_route_msg_uses_placeholder_image() -> None:
    """ルート画像が未指定の際にプレースホルダが挿入されることを確認する。"""
    waypoint = _StubWaypoint()
    waypoint.pose.position.x = 1.0
    msg = rp.pack_route_msg([waypoint], version=2, total_distance=1.5, route_image=None)
    assert msg.version == 2
    assert msg.total_distance == 1.5
    assert msg.route_image is not None
    assert msg.route_image.data == b"\x00\x00\x00"
