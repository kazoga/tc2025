#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
road_generator.py で生成した道路（直線/クランク/S字）に対応する
waypoint.csv を生成するスクリプト。

- 道路中心線は関数で定義（直線/クランク/S字）
- 中心線に沿って 5m ごとに waypoint を生成
- 25度以上折れ曲がる交点では waypoint を追加
- waypoint.csv の形式は既存仕様に合わせる
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import List, Tuple


# -----------------------------------------------------------------------------
# 基本データ型
# -----------------------------------------------------------------------------

@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float  # [rad]


# -----------------------------------------------------------------------------
# 道路中心線の定義（road_generator.py と一致）
# -----------------------------------------------------------------------------

def get_straight_100m() -> List[Tuple[float, float]]:
    return [(0.0, 0.0), (100.0, 0.0)]


def get_crank_50m() -> List[Tuple[float, float]]:
    # 直線 50m → 左折 50m → 右折 50m
    return [
        (0.0, 0.0),
        (50.0, 0.0),
        (50.0, 50.0),
        (100.0, 50.0),
    ]


def get_scurve_100m() -> List[Tuple[float, float]]:
    """S字（曲率B）: road_generator.py と同条件"""
    length_x = 100.0
    amplitude = 20.0
    base_y = 50.0
    num_points = 41  # 約 2.5m ピッチ

    pts = []
    for i in range(num_points):
        x = length_x * i / (num_points - 1)
        y = base_y + amplitude * math.sin(2.0 * math.pi * x / length_x)
        pts.append((x, y))
    return pts


# -----------------------------------------------------------------------------
# サンプリング処理
# -----------------------------------------------------------------------------

def segment_points(p0, p1, step, start_offset=0.0) -> List[Pose2D]:
    """p0→p1 を step[m] 間隔でサンプリング."""
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)

    poses = []
    if length <= 1e-6:
        return poses

    effective_length = length - start_offset
    if effective_length <= 0.0:
        return poses

    n_steps = int(effective_length // step)
    for i in range(n_steps):
        s = start_offset + step * i
        t = s / length
        poses.append(Pose2D(
            x=x0 + dx * t,
            y=y0 + dy * t,
            yaw=yaw
        ))
    return poses


def generate_waypoints_from_polyline(points: List[Tuple[float, float]],
                                     step: float,
                                     angle_threshold_rad: float) -> List[Pose2D]:
    """中心線折れ線に沿って waypoint を生成."""
    waypoints: List[Pose2D] = []

    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        start_offset = step if i == 0 else 0.0

        segposes = segment_points(p0, p1, step, start_offset=start_offset)
        waypoints.extend(segposes)

        # 次のセグメントがあれば曲がり角チェック
        if i < len(points) - 2:
            p2 = points[i + 2]
            yaw_curr = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            yaw_next = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            dyaw = (yaw_next - yaw_curr + math.pi) % (2 * math.pi) - math.pi
            if abs(dyaw) >= angle_threshold_rad:
                waypoints.append(Pose2D(x=p1[0], y=p1[1], yaw=yaw_next))

    # 最後の点を追加
    last = points[-1]
    last_yaw = waypoints[-1].yaw if waypoints else 0.0
    waypoints.append(Pose2D(x=last[0], y=last[1], yaw=last_yaw))

    return waypoints


# -----------------------------------------------------------------------------
# waypoint.csv 出力
# -----------------------------------------------------------------------------

def write_waypoint_csv(waypoints: List[Pose2D], width: float, path: str) -> None:
    header = [
        'label', 'x', 'y', 'z',
        'q1', 'q2', 'q3', 'q4',
        'right_is_open', 'left_is_open',
        'line_is_stop', 'signal_is_stop',
        'isnot_skipnum',
    ]
    half_width = width / 2.0

    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(header)

        for i, p in enumerate(waypoints, start=1):
            z = 0.0
            half = p.yaw / 2.0
            q1, q2 = 0.0, 0.0  # roll,pitch 無し
            q3 = math.sin(half)
            q4 = math.cos(half)

            row = [
                i, p.x, p.y, z,
                q1, q2, q3, q4,
                half_width, half_width,
                0, 0, 1,
            ]
            w.writerow(row)


# -----------------------------------------------------------------------------
# メイン
# -----------------------------------------------------------------------------

def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--road",
        choices=["straight", "crank", "scurve"],
        required=True,
        help="道路種類"
    )
    parser.add_argument(
        "--width",
        type=float,
        required=True,
        help="道幅[m]（2,3,5 など）"
    )
    parser.add_argument(
        "--output",
        default="waypoints.csv",
        help="出力 waypoint.csv のパス"
    )

    args = parser.parse_args()

    # 道路中心線取得
    if args.road == "straight":
        points = get_straight_100m()
    elif args.road == "crank":
        points = get_crank_50m()
    elif args.road == "scurve":
        points = get_scurve_100m()
    else:
        raise ValueError("unknown road type")

    # waypoint 生成
    waypoints = generate_waypoints_from_polyline(
        points,
        step=5.0,
        angle_threshold_rad=math.radians(25.0),
    )

    # CSV 出力
    write_waypoint_csv(waypoints, args.width, args.output)

    print(f"Generated {args.output} with {len(waypoints)} waypoints.")


if __name__ == "__main__":
    main()

