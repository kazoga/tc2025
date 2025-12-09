#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""道路中心線に沿った waypoint.csv を生成するスクリプト.

- 折れ線: (0,0) → (20,0) → (20,20) → (40,20) を道路中心線とみなす。
- 5m 間隔でサンプリングし、さらに折れ角が 45° 以上の曲がり角に waypoint を追加する。
- 出力する CSV は、元の waypoint.csv 仕様から「緯度経度(latitude,longitude)」「node」列を除いた形式とする。
- right_is_open と left_is_open には「waypoint から右/左の道路端までの距離[m]」を出力する。
  道路幅 5m、waypoint は常に中心線上にある前提なので、両方とも 2.5m 固定とする。
- line_is_stop, signal_is_stop は全て 0。
- isnot_skipnum は全て 1。
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Pose2D:
    """2次元の姿勢表現."""

    x: float
    y: float
    yaw: float  # [rad]


def segment_points(p0: Tuple[float, float],
                   p1: Tuple[float, float],
                   step: float) -> List[Pose2D]:
    """p0→p1 区間を step[m] ごとにサンプリングする.

    Args:
        p0: 始点 (x0, y0)
        p1: 終点 (x1, y1)
        step: サンプリング間隔[m]

    Returns:
        Pose2D のリスト（終点は含まない）
    """
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)

    poses: List[Pose2D] = []
    if length <= 0.0:
        return poses

    n_steps = int(length // step)
    for i in range(n_steps):
        s = step * i
        t = s / length
        x = x0 + dx * t
        y = y0 + dy * t
        poses.append(Pose2D(x=x, y=y, yaw=yaw))

    return poses


def generate_waypoints() -> List[Pose2D]:
    """道路中心線に沿った waypoint の Pose2D リストを生成する."""
    # 折れ線（道路中心線）
    points: List[Tuple[float, float]] = [
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 20.0),
        (40.0, 20.0),
    ]
    step = 5.0
    angle_threshold = math.radians(45.0)

    waypoints: List[Pose2D] = []

    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]

        # セグメント内の5m刻み
        seg_poses = segment_points(p0, p1, step)
        waypoints.extend(seg_poses)

        # 曲がり角の処理（45°以上なら角に waypoint を追加）
        if i < len(points) - 2:
            p_next = points[i + 2]
            yaw_curr = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            yaw_next = math.atan2(p_next[1] - p1[1], p_next[0] - p1[0])
            # -pi〜pi に正規化した差分
            dyaw = (yaw_next - yaw_curr + math.pi) % (2.0 * math.pi) - math.pi
            if abs(dyaw) >= angle_threshold:
                waypoints.append(Pose2D(x=p1[0], y=p1[1], yaw=yaw_next))

    # 終点も追加
    last = points[-1]
    if waypoints:
        yaw_last = waypoints[-1].yaw
    else:
        yaw_last = 0.0
    waypoints.append(Pose2D(x=last[0], y=last[1], yaw=yaw_last))

    return waypoints


def write_waypoints_csv(waypoints: List[Pose2D], output_path: str) -> None:
    """waypoint リストを waypoint.csv として書き出す.

    出力カラムは次の通り:
    label,x,y,z,q1,q2,q3,q4,right_is_open,left_is_open,line_is_stop,signal_is_stop,isnot_skipnum

    - label: 1 から始まる連番
    - x,y,z: map/world 座標系。z は 0.0 固定
    - q1,q2,q3,q4: quaternion (x,y,z,w)。yaw のみを持つ回転を z 軸まわりに付与する。
    - right_is_open,left_is_open: waypoint から右/左の道路端までの距離[m]。道路幅 5m のため両方 2.5。
    - line_is_stop,signal_is_stop: すべて 0
    - isnot_skipnum: すべて 1
    """
    header = [
        'label',
        'x',
        'y',
        'z',
        'q1',
        'q2',
        'q3',
        'q4',
        'right_is_open',
        'left_is_open',
        'line_is_stop',
        'signal_is_stop',
        'isnot_skipnum',
    ]

    road_width = 5.0
    half_width = road_width / 2.0

    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for idx, wp in enumerate(waypoints, start=1):
            z = 0.0
            # yaw -> quaternion (x,y,z,w)
            half = wp.yaw / 2.0
            q1 = 0.0
            q2 = 0.0
            q3 = math.sin(half)
            q4 = math.cos(half)

            # 左右の道路端までの距離[m]
            right_dist = half_width
            left_dist = half_width

            line_is_stop = 0
            signal_is_stop = 0
            isnot_skipnum = 1

            row = [
                idx,
                wp.x,
                wp.y,
                z,
                q1,
                q2,
                q3,
                q4,
                right_dist,
                left_dist,
                line_is_stop,
                signal_is_stop,
                isnot_skipnum,
            ]
            writer.writerow(row)


def main() -> None:
    """エントリーポイント."""
    waypoints = generate_waypoints()
    output_path = 'waypoints.csv'
    write_waypoints_csv(waypoints, output_path)
    print(f'Generated {output_path} with {len(waypoints)} poses.')


if __name__ == '__main__':
    main()
