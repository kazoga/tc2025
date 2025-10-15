"""Minimal networkx stub for offline testing."""

from __future__ import annotations

from typing import Dict, Iterable, Iterator, List, Optional


class _NodeView:
    """簡易的なノードビュー。"""

    def __init__(self, graph: "Graph") -> None:
        self._graph = graph

    def __iter__(self) -> Iterator[str]:
        return iter(self._graph._adj.keys())

    def __len__(self) -> int:
        return len(self._graph._adj)

    def __contains__(self, item: object) -> bool:
        return item in self._graph._adj

    def __call__(self, *args: object, **kwargs: object) -> List[str]:
        return list(self._graph._adj.keys())


class Graph:
    """Undirected graph with weight support."""

    def __init__(self) -> None:
        self._adj: Dict[str, Dict[str, Dict[str, float]]] = {}
        self._node_attrs: Dict[str, Dict[str, float]] = {}
        self.nodes = _NodeView(self)

    def add_node(self, node: str, **attrs: float) -> None:
        """ノードを追加する。"""
        self._adj.setdefault(node, {})
        if attrs:
            self._node_attrs.setdefault(node, {}).update(attrs)

    def add_edge(self, u: str, v: str, **attrs: float) -> None:
        """無向エッジを追加する。"""
        self.add_node(u)
        self.add_node(v)
        meta = dict(attrs)
        self._adj[u][v] = meta
        self._adj[v][u] = dict(meta)

    def neighbors(self, node: str) -> Iterable[str]:
        """隣接ノードを返す。"""
        return self._adj.get(node, {}).keys()

    def number_of_nodes(self) -> int:
        return len(self._adj)

    def number_of_edges(self) -> int:
        return sum(len(v) for v in self._adj.values()) // 2

    def edges(self) -> Iterator[tuple[str, str]]:
        seen = set()
        for u, nbrs in self._adj.items():
            for v in nbrs:
                if (v, u) not in seen:
                    seen.add((u, v))
                    yield (u, v)

    def __getitem__(self, node: str) -> Dict[str, Dict[str, float]]:
        return self._adj[node]


DiGraph = Graph


def draw_networkx_nodes(*_: object, **__: object) -> None:
    """No-op drawing stub."""


def draw_networkx_labels(*_: object, **__: object) -> None:
    """No-op drawing stub."""


def draw_networkx_edges(*_: object, **__: object) -> None:
    """No-op drawing stub."""


__all__ = ["Graph", "DiGraph", "draw_networkx_nodes", "draw_networkx_labels", "draw_networkx_edges"]
