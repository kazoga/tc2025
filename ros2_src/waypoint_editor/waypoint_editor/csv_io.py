"""CSV入出力とWayPointモデルを提供するモジュール。"""

from __future__ import annotations

import csv
import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple


@dataclass
class Waypoint:
    """ウェイポイント1件を保持するデータクラス。"""

    label: int
    latitude: float
    longitude: float
    x: float
    y: float
    z: float
    q1: float
    q2: float
    q3: float
    q4: float
    right_is_open: float
    left_is_open: float
    line_is_stop: int
    signal_is_stop: int
    isnot_skipnum: int
    node: float
    extra_fields: Dict[str, Any] = field(default_factory=dict)

    def yaw(self) -> float:
        """平面上のyaw角をq3,q4から算出する。"""
        import math

        return math.atan2(self.q3, self.q4) * 2.0


KNOWN_FIELDS = [
    "label",
    "latitude",
    "longitude",
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
    "node",
]


def _parse_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _parse_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def read_waypoints(csv_path: Path) -> Tuple[List[Waypoint], List[str]]:
    """CSVファイルを読み込み、WayPointリストと列順を返す。"""
    waypoints: List[Waypoint] = []
    header_order: List[str] = []
    if not csv_path.exists():
        return waypoints, KNOWN_FIELDS.copy()

    with csv_path.open("r", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        header_order = reader.fieldnames or []
        for row in reader:
            known_data = {key: row.get(key) for key in KNOWN_FIELDS}
            waypoint = Waypoint(
                label=_parse_int(known_data.get("label")),
                latitude=_parse_float(known_data.get("latitude")),
                longitude=_parse_float(known_data.get("longitude")),
                x=_parse_float(known_data.get("x")),
                y=_parse_float(known_data.get("y")),
                z=_parse_float(known_data.get("z")),
                q1=_parse_float(known_data.get("q1"), 0.0),
                q2=_parse_float(known_data.get("q2"), 0.0),
                q3=_parse_float(known_data.get("q3"), 0.0),
                q4=_parse_float(known_data.get("q4"), 1.0),
                right_is_open=_parse_float(known_data.get("right_is_open")),
                left_is_open=_parse_float(known_data.get("left_is_open")),
                line_is_stop=_parse_int(known_data.get("line_is_stop")),
                signal_is_stop=_parse_int(known_data.get("signal_is_stop")),
                isnot_skipnum=_parse_int(known_data.get("isnot_skipnum")),
                node=_parse_float(known_data.get("node")),
                extra_fields={
                    key: value
                    for key, value in row.items()
                    if key not in KNOWN_FIELDS and key is not None
                },
            )
            waypoints.append(waypoint)
    return waypoints, header_order


def _backup_file(csv_path: Path) -> None:
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = csv_path.with_name(f"{csv_path.stem}.bak.{timestamp}{csv_path.suffix}")
    backup_path.write_bytes(csv_path.read_bytes())


def _waypoint_to_row(waypoint: Waypoint, header: Sequence[str]) -> Dict[str, Any]:
    row: Dict[str, Any] = {}
    for key in header:
        if key in waypoint.extra_fields:
            row[key] = waypoint.extra_fields.get(key, "")
        elif hasattr(waypoint, key):
            row[key] = getattr(waypoint, key)
        else:
            row[key] = ""
    for key, value in waypoint.extra_fields.items():
        if key not in row:
            row[key] = value
    return row


def _collect_all_extra_fields(waypoints: Iterable[Waypoint]) -> List[str]:
    extras: List[str] = []
    for waypoint in waypoints:
        for key in waypoint.extra_fields:
            if key not in extras and key not in KNOWN_FIELDS:
                extras.append(key)
    return extras


def write_waypoints(csv_path: Path, waypoints: Sequence[Waypoint],
                    header_order: Sequence[str] | None = None) -> None:
    """WayPointリストをCSVへ保存する。既存ファイルはバックアップを作成する。"""
    if csv_path.exists():
        _backup_file(csv_path)

    extras = _collect_all_extra_fields(waypoints)
    if header_order:
        header = list(header_order)
        for extra in extras:
            if extra not in header:
                header.append(extra)
    else:
        header = KNOWN_FIELDS + extras

    with csv_path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=header)
        writer.writeheader()
        for waypoint in waypoints:
            row = _waypoint_to_row(waypoint, header)
            writer.writerow(row)


def update_waypoint(original: Waypoint, **kwargs: Any) -> Waypoint:
    """既存WayPointにフィールド更新を適用した新インスタンスを返す。"""
    data = original.__dict__.copy()
    data.update(kwargs)
    return Waypoint(**data)
