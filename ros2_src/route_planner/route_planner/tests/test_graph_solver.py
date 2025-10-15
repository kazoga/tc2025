"""Tests for graph solver utilities."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List, Tuple

import networkx as nx
import pytest

from route_planner.route_planner import graph_solver as gs


@pytest.fixture(autouse=True)
def _patch_plot(monkeypatch: pytest.MonkeyPatch) -> None:
    """画像生成をダミー化する。"""

    def _dummy_plot(*_: object, **__: object) -> str:
        return "/tmp/dummy.png"

    monkeypatch.setattr(gs, "plot_route_image", _dummy_plot)


def test_parse_bool_various_inputs() -> None:
    """文字列や数値が正しくbool変換されるか確認する。"""
    assert gs._parse_bool("1") is True
    assert gs._parse_bool("Yes") is True
    assert gs._parse_bool("false") is False
    assert gs._parse_bool(0) is False


def test_load_waypoint_csv_length_xy(tmp_path: Path) -> None:
    """x,y列の距離計算がユークリッド長と一致する。"""
    csv_path = tmp_path / "seg_xy.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["x", "y"])
        writer.writeheader()
        writer.writerow({"x": "0", "y": "0"})
        writer.writerow({"x": "3", "y": "4"})
    length, pts = gs._load_waypoint_csv_length(str(csv_path))
    assert pytest.approx(length, abs=1e-6) == 5.0
    assert pts == [(0.0, 0.0), (3.0, 4.0)]


def test_load_waypoint_csv_length_latlon(tmp_path: Path) -> None:
    """緯度経度の距離計算がハヴァーサインに基づく。"""
    csv_path = tmp_path / "seg_latlon.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["lat", "lon"])
        writer.writeheader()
        writer.writerow({"lat": "35.0", "lon": "139.0"})
        writer.writerow({"lat": "35.0009", "lon": "139.0009"})
    length, _ = gs._load_waypoint_csv_length(str(csv_path))
    assert length > 0.0
    expected = gs._haversine_m(35.0, 139.0, 35.0009, 139.0009)
    assert pytest.approx(length, rel=1e-6) == expected


def test_load_nodes_and_edges(tmp_path: Path) -> None:
    """ノードとエッジのCSV読み込みが正しく行われる。"""
    nodes_path = tmp_path / "nodes.csv"
    with nodes_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["id", "lat", "lon"])
        writer.writeheader()
        writer.writerow({"id": "A", "lat": "35.0", "lon": "139.0"})
    edges_path = tmp_path / "edges.csv"
    with edges_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["source", "target", "segment_id", "reversible"])
        writer.writeheader()
        writer.writerow({"source": "A", "target": "B", "segment_id": "seg.csv", "reversible": "1"})
    nodes = gs.load_nodes_csv(str(nodes_path))
    edges = gs.load_edges_csv(str(edges_path))
    assert nodes == {"A": (35.0, 139.0)}
    assert edges[0]["reversible"] is True


def test_dijkstra_and_reconstruct_path() -> None:
    """ダイクストラ探索で最短路が復元される。"""
    graph = nx.Graph()
    graph.add_edge("S", "A", weight=1.0)
    graph.add_edge("A", "G", weight=2.0)
    graph.add_edge("S", "G", weight=5.0)
    dist, prev = gs.dijkstra_single_source(graph, "S")
    path = gs.reconstruct_path(prev, "S", "G")
    assert dist["G"] == 3.0
    assert path == ["S", "A", "G"]


def test_held_karp_path_simple_case() -> None:
    """Held-Karpで端点順序が最短になるか確認する。"""
    k = 3
    dist_matrix: Dict[Tuple[int, int], float] = {
        (0, 0): 0.0,
        (0, 1): 1.0,
        (0, 2): 5.0,
        (1, 0): 1.0,
        (1, 1): 0.0,
        (1, 2): 1.0,
        (2, 0): 5.0,
        (2, 1): 1.0,
        (2, 2): 0.0,
    }
    best_cost, order = gs.held_karp_path(dist_matrix, 0, 2, k)
    assert pytest.approx(best_cost, abs=1e-6) == 2.0
    assert order == [0, 1, 2]
