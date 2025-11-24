"""PGWを用いて緯度経度とENU座標を相互変換するモジュール。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Union


@dataclass
class _PgwInfo:
    """PGWのパラメータと投影種別を保持する内部構造。"""

    A: float
    D: float
    B: float
    E: float
    C: float
    F: float
    projected: bool

    def latlon_to_map(self, lat: float, lon: float) -> Tuple[float, float]:
        if self.projected:
            return latlon_to_webmercator(lat, lon)
        return lon, lat

    def map_to_latlon(self, mapx: float, mapy: float) -> Tuple[float, float]:
        if self.projected:
            return webmercator_to_latlon(mapx, mapy)
        return mapy, mapx


_LOADED: Optional[_PgwInfo] = None
RADIUS_EARTH_M = 6378137.0


def _read_pgw(worldfile_path: Union[str, Path]) -> _PgwInfo:
    with open(worldfile_path, "r", encoding="utf-8") as fp:
        values = [float(line.strip()) for line in fp.readlines()]
    if len(values) != 6:
        raise ValueError("PGWファイルの形式が不正です。6行の数値が必要です。")
    A, D, B, E, C, F = values
    projected = _is_projected(C, F, A, E)
    return _PgwInfo(A=A, D=D, B=B, E=E, C=C, F=F, projected=projected)


def _is_projected(C: float, F: float, A: float, E: float) -> bool:
    return (abs(C) > 1e6 or abs(F) > 1e6) and (abs(A) < 1.0 and abs(E) < 1.0)


def load_pgw(worldfile_path: Union[str, Path]) -> None:
    """PGWを読み込み、以降の変換で利用する。"""
    global _LOADED
    _LOADED = _read_pgw(worldfile_path)


def _require_loaded() -> _PgwInfo:
    if _LOADED is None:
        raise RuntimeError("PGWが未ロードです。load_pgw()を先に呼び出してください。")
    return _LOADED


def latlon_to_webmercator(lat: float, lon: float) -> Tuple[float, float]:
    x = math.radians(lon) * RADIUS_EARTH_M
    y = RADIUS_EARTH_M * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
    return x, y


def webmercator_to_latlon(x: float, y: float) -> Tuple[float, float]:
    lon = math.degrees(x / RADIUS_EARTH_M)
    lat = math.degrees(2 * math.atan(math.exp(y / RADIUS_EARTH_M)) - math.pi / 2)
    return lat, lon


def latlon_to_en(lat: float, lon: float) -> Tuple[float, float]:
    """緯度経度(度)を東向きE・北向きN座標に変換する。"""
    pgw = _require_loaded()
    return pgw.latlon_to_map(lat, lon)


def en_to_latlon(E: float, N: float) -> Tuple[float, float]:
    """東向きE・北向きN座標を緯度経度(度)に変換する。"""
    pgw = _require_loaded()
    return pgw.map_to_latlon(E, N)
