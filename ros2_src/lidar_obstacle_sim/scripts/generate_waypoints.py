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
    num_points = 41  # 約 2.5m ピッチ

    pts = []
    for i in range(num_points):
        x = length_x * i / (num_points - 1)
        y = amplitude * math.sin(2.0 * math.pi * x / length_x)
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
                                     angle_threshold_rad: float,
                                     start_offset: float = 1.0,
                                     end_offset: float = 1.0) -> List[Pose2D]:
    """中心線折れ線に沿って waypoint を生成.

    最初と最後の waypoint は道路端から 1m 内側に配置するため、中心線上の
    start_offset/end_offset を考慮した距離を生成に利用する。
    """
    if len(points) < 2:
        return []

    # セグメント長の累積距離を求め、折れ線上の任意距離を補間するための準備を行う。
    cumulative_distances: List[float] = [0.0]
    segment_yaws: List[float] = []
    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        seg_len = math.hypot(dx, dy)
        cumulative_distances.append(cumulative_distances[-1] + seg_len)
        segment_yaws.append(math.atan2(dy, dx))

    total_length = cumulative_distances[-1]

    def interpolate_pose(distance: float) -> Pose2D:
        """折れ線上の指定距離に対応する Pose2D を返す."""
        for i in range(len(points) - 1):
            if distance <= cumulative_distances[i + 1] + 1e-9:
                seg_len = cumulative_distances[i + 1] - cumulative_distances[i]
                if seg_len <= 0.0:
                    return Pose2D(x=points[i][0], y=points[i][1], yaw=0.0)

                ratio = (distance - cumulative_distances[i]) / seg_len
                x = points[i][0] + (points[i + 1][0] - points[i][0]) * ratio
                y = points[i][1] + (points[i + 1][1] - points[i][1]) * ratio
                return Pose2D(x=x, y=y, yaw=segment_yaws[i])

        # 端数誤差でここに来た場合は終点を返す
        return Pose2D(x=points[-1][0], y=points[-1][1], yaw=segment_yaws[-1])

    valid_start = min(max(start_offset, 0.0), total_length)
    valid_end_candidate = max(total_length - end_offset, 0.0)
    valid_end = max(min(valid_end_candidate, total_length), valid_start)

    # 1m オフセット起点と 5m ピッチのサンプル地点を列挙
    sampled_distances: List[Tuple[float, float | None]] = []
    sampled_distances.append((valid_start, segment_yaws[0]))

    current_distance = valid_start + step
    while current_distance <= valid_end + 1e-6:
        sampled_distances.append((current_distance, None))
        current_distance += step

    # 最終 1m 手前の地点を追加
    sampled_distances.append((valid_end, segment_yaws[-1]))

    # 曲がり角がしきい値以上の場合は必ず waypoint を置く
    for i in range(1, len(points) - 1):
        yaw_prev = segment_yaws[i - 1]
        yaw_next = segment_yaws[i]
        dyaw = (yaw_next - yaw_prev + math.pi) % (2 * math.pi) - math.pi
        if abs(dyaw) >= angle_threshold_rad:
            if valid_start - 1e-6 <= cumulative_distances[i] <= valid_end + 1e-6:
                sampled_distances.append((cumulative_distances[i], yaw_next))

    # 距離の昇順で統合し、重複距離は後勝ち（曲がり角優先）で上書きする。
    sampled_distances.sort(key=lambda item: item[0])
    merged_distances: List[Tuple[float, float | None]] = []
    for dist, yaw_override in sampled_distances:
        if merged_distances and abs(dist - merged_distances[-1][0]) < 1e-6:
            prev_dist, prev_yaw_override = merged_distances[-1]
            merged_distances[-1] = (
                prev_dist,
                yaw_override if yaw_override is not None else prev_yaw_override,
            )
        else:
            merged_distances.append((dist, yaw_override))

    waypoints: List[Pose2D] = []
    for dist, yaw_override in merged_distances:
        pose = interpolate_pose(dist)
        if yaw_override is not None:
            pose.yaw = yaw_override
        waypoints.append(pose)

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

