"""Integration tests for route builder and graph solver."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Generator, List, Tuple

import pytest

from route_planner.route_planner import graph_solver as gs
from route_planner.route_planner import route_planner as rp


class _StubQuaternion:
    """簡易クォータニオン。"""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _StubPosition:
    """位置スタブ。"""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _StubPose:
    """姿勢スタブ。"""

    def __init__(self) -> None:
        self.position = _StubPosition()
        self.orientation = _StubQuaternion()


class _StubWaypoint:
    """Waypointスタブ。"""

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
    """ヘッダスタブ。"""

    def __init__(self) -> None:
        self.frame_id = ""


class _StubImage:
    """画像スタブ。"""

    def __init__(self) -> None:
        self.header = _StubHeader()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.step = 0
        self.data = b""


class _StubRoute:
    """Routeスタブ。"""

    def __init__(self) -> None:
        self.header = None
        self.waypoints: List[_StubWaypoint] = []
        self.version = 0
        self.total_distance = 0.0
        self.route_image: _StubImage | None = None


@pytest.fixture(autouse=True)
def _patch_dependencies(monkeypatch: pytest.MonkeyPatch) -> Generator[None, None, None]:
    """ROS依存と描画機能をスタブ化する。"""

    def _dummy_plot(*_: object, **__: object) -> str:
        return "/tmp/integration.png"

    monkeypatch.setattr(rp, "Quaternion", _StubQuaternion)
    monkeypatch.setattr(rp, "Waypoint", _StubWaypoint)
    monkeypatch.setattr(rp, "Header", _StubHeader)
    monkeypatch.setattr(rp, "Image", _StubImage)
    monkeypatch.setattr(rp, "Route", _StubRoute)
    monkeypatch.setattr(gs, "plot_route_image", _dummy_plot)
    yield


def _create_segment_csv(path: Path, points: List[Tuple[float, float]]) -> None:
    """簡易CSVを作成する。"""
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["x", "y"])
        writer.writeheader()
        for x_val, y_val in points:
            writer.writerow({"x": f"{x_val}", "y": f"{y_val}"})


def _quat_to_yaw(quat: _StubQuaternion) -> float:
    """クォータニオンからヨー角へ変換。"""
    return math.atan2(2.0 * quat.w * quat.z, 1.0 - 2.0 * quat.z * quat.z)


def test_fixed_block_reproduction(tmp_path: Path) -> None:
    """固定ブロックのみの場合にCSVと同じ経路になることを確認する。"""
    csv_path = tmp_path / "fixed.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "label",
                "x",
                "y",
                "z",
                "q1",
                "q2",
                "q3",
                "q4",
                "right_is_open",
                "left_is_open",
                "line_is_stop",
                "signal_is_stop",
                "isnot_skipnum",
            ],
        )
        writer.writeheader()
        writer.writerow({"label": "", "x": "0", "y": "0", "z": "0", "q1": "0", "q2": "0", "q3": "0", "q4": "1", "right_is_open": "0", "left_is_open": "0", "line_is_stop": "0", "signal_is_stop": "0", "isnot_skipnum": "0"})
        writer.writerow({"label": "", "x": "1", "y": "0", "z": "0", "q1": "0", "q2": "0", "q3": "0", "q4": "1", "right_is_open": "0", "left_is_open": "0", "line_is_stop": "0", "signal_is_stop": "0", "isnot_skipnum": "0"})
        writer.writerow({"label": "", "x": "2", "y": "1", "z": "0", "q1": "0", "q2": "0", "q3": "0", "q4": "1", "right_is_open": "0", "left_is_open": "0", "line_is_stop": "0", "signal_is_stop": "0", "isnot_skipnum": "0"})
    waypoints = rp.parse_waypoint_csv(str(csv_path))
    sliced, offset = rp.slice_by_labels(waypoints, "", "")
    assert offset == 0
    coords = [(wp.pose.position.x, wp.pose.position.y) for wp in sliced]
    assert coords == [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]


def test_label_slicing(tmp_path: Path) -> None:
    """ラベル指定で正しく区間抽出できることを確認する。"""
    csv_path = tmp_path / "labels.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "label",
                "x",
                "y",
                "z",
                "q1",
                "q2",
                "q3",
                "q4",
                "right_is_open",
                "left_is_open",
                "line_is_stop",
                "signal_is_stop",
                "isnot_skipnum",
            ],
        )
        writer.writeheader()
        for idx, (label, x_val) in enumerate(zip(["S", "A", "B", "G"], [0, 1, 2, 3], strict=True)):
            writer.writerow({"label": label, "x": str(x_val), "y": "0", "z": "0", "q1": "0", "q2": "0", "q3": "0", "q4": "1", "right_is_open": "0", "left_is_open": "0", "line_is_stop": "0", "signal_is_stop": "0", "isnot_skipnum": "0"})
    waypoints = rp.parse_waypoint_csv(str(csv_path))
    sliced, offset = rp.slice_by_labels(waypoints, "A", "G")
    assert offset == 1
    labels = [wp.label for wp in sliced]
    assert labels == ["A", "B", "G"]


def test_variable_block_route_construction(tmp_path: Path) -> None:
    """可変ブロック経路がスタート・チェックポイント・ゴールを含むことを確認する。"""
    seg_sc = tmp_path / "seg_sc.csv"
    seg_cg = tmp_path / "seg_cg.csv"
    _create_segment_csv(seg_sc, [(0.0, 0.0), (0.0, 1.0)])
    _create_segment_csv(seg_cg, [(0.0, 1.0), (1.0, 1.0)])
    nodes = {"S": (0.0, 0.0), "C": (0.0, 1.0), "G": (1.0, 1.0)}
    edges = [
        {"source": "S", "target": "C", "segment_id": str(seg_sc), "reversible": True},
        {"source": "C", "target": "G", "segment_id": str(seg_cg), "reversible": True},
    ]
    result = gs.solve_variable_route(nodes, edges, "S", "G", ["C"])
    assert result["visit_order"] == ["S", "C", "G"]
    assert set(result["node_sequence"]) >= {"S", "C", "G"}


def test_reversible_edge_handling(tmp_path: Path) -> None:
    """可逆エッジと非可逆エッジでノード集合が一致することを確認する。"""
    seg_sm = tmp_path / "seg_sm.csv"
    seg_mg = tmp_path / "seg_mg.csv"
    _create_segment_csv(seg_sm, [(0.0, 0.0), (1.0, 0.0)])
    _create_segment_csv(seg_mg, [(1.0, 0.0), (1.0, 1.0)])
    nodes = {"S": (0.0, 0.0), "M": (1.0, 0.0), "G": (1.0, 1.0)}
    edges_rev = [
        {"source": "S", "target": "M", "segment_id": str(seg_sm), "reversible": True},
        {"source": "M", "target": "G", "segment_id": str(seg_mg), "reversible": True},
    ]
    edges_nonrev = [
        {"source": "S", "target": "M", "segment_id": str(seg_sm), "reversible": False},
        {"source": "M", "target": "S", "segment_id": str(seg_sm), "reversible": False},
        {"source": "M", "target": "G", "segment_id": str(seg_mg), "reversible": False},
        {"source": "G", "target": "M", "segment_id": str(seg_mg), "reversible": False},
    ]
    result_rev = gs.solve_variable_route(nodes, edges_rev, "S", "G", ["M"])
    result_nonrev = gs.solve_variable_route(nodes, edges_nonrev, "S", "G", ["M"])
    assert set(result_rev["node_sequence"]) == set(result_nonrev["node_sequence"])


def test_pose_orientation_consistency(tmp_path: Path) -> None:
    """経路姿勢が進行方向と連続することを確認する。"""
    seg_sm = tmp_path / "seg_sm.csv"
    seg_mg = tmp_path / "seg_mg.csv"
    _create_segment_csv(seg_sm, [(0.0, 0.0), (1.0, 0.0)])
    _create_segment_csv(seg_mg, [(1.0, 0.0), (1.0, 1.0)])
    nodes = {"S": (0.0, 0.0), "M": (1.0, 0.0), "G": (1.0, 1.0)}
    edges = [
        {"source": "S", "target": "M", "segment_id": str(seg_sm), "reversible": True},
        {"source": "M", "target": "G", "segment_id": str(seg_mg), "reversible": True},
    ]
    result = gs.solve_variable_route(nodes, edges, "S", "G", ["M"])
    coord_map = {"S": (0.0, 0.0), "M": (1.0, 0.0), "G": (1.0, 1.0)}
    waypoints = [_StubWaypoint() for _ in result["node_sequence"]]
    for wp, node_id in zip(waypoints, result["node_sequence"], strict=True):
        x_val, y_val = coord_map[node_id]
        wp.pose.position.x = x_val
        wp.pose.position.y = y_val
    rp.adjust_orientations(waypoints, [(0, len(waypoints) - 1)], [])
    for idx in range(len(waypoints) - 1):
        dx = waypoints[idx + 1].pose.position.x - waypoints[idx].pose.position.x
        dy = waypoints[idx + 1].pose.position.y - waypoints[idx].pose.position.y
        expected_yaw = math.atan2(dy, dx)
        actual_yaw = _quat_to_yaw(waypoints[idx].pose.orientation)
        assert pytest.approx(actual_yaw, abs=1e-6) == expected_yaw
    final_yaw = _quat_to_yaw(waypoints[-1].pose.orientation)
    assert pytest.approx(final_yaw, abs=1e-6) == math.pi / 2.0
