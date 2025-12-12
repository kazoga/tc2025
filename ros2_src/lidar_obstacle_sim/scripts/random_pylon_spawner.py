#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Gazebo 上に道路形状に応じてパイロンをスポーンする ROS2 ノード.

- road_type: straight / crank / scurve に対応
- road_width: 道幅[m]
- 道路中心線に沿って長手方向にランダムな位置を取り、
  各地点で道幅方向に 1〜3 本のパイロンを配置する。
- 道幅方向に少なくとも 1m 以上の「隙間」が残るように、
  本数・配置を調整する。
"""

from __future__ import annotations

import math
import os
import random
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.logging import get_logger
from rclpy.node import Node


# ==========================
# 中心線定義用のデータ型
# ==========================

@dataclass
class Segment:
    """折れ線の 1 区間."""
    p0: Tuple[float, float]
    p1: Tuple[float, float]

    @property
    def dx(self) -> float:
        return self.p1[0] - self.p0[0]

    @property
    def dy(self) -> float:
        return self.p1[1] - self.p0[1]

    @property
    def length(self) -> float:
        return math.hypot(self.dx, self.dy)

    @property
    def yaw(self) -> float:
        return math.atan2(self.dy, self.dx)


# ==========================
# 道路中心線の定義
# ==========================

def get_polyline_points(road_type: str) -> List[Tuple[float, float]]:
    """道路タイプに応じた中心線の折れ線頂点を返す."""
    if road_type == "straight":
        # 100m 直線
        return [(0.0, 0.0), (100.0, 0.0)]

    if road_type == "crank":
        # 50m → 左折 50m → 右折 50m
        return [
            (0.0, 0.0),
            (50.0, 0.0),
            (50.0, 50.0),
            (100.0, 50.0),
        ]

    if road_type == "scurve":
        # road_generator.py の S 字と同条件（曲率 B）
        length_x = 100.0
        amplitude = 20.0
        num_points = 41  # 約 2.5m ピッチ

        pts: List[Tuple[float, float]] = []
        for i in range(num_points):
            x = length_x * i / (num_points - 1)
            y = amplitude * math.sin(2.0 * math.pi * x / length_x)
            pts.append((x, y))
        return pts

    raise ValueError(f"Unsupported road_type: {road_type}")


def build_segments(points: List[Tuple[float, float]]) -> List[Segment]:
    """折れ線頂点列から Segment リストを生成."""
    segments: List[Segment] = []
    for i in range(len(points) - 1):
        segments.append(Segment(p0=points[i], p1=points[i + 1]))
    return segments


# ==========================
# パイロンスポーナーノード
# ==========================

class RandomPylonSpawner(Node):
    """Gazebo にランダムパイロンをスポーンする ROS2 ノード."""

    def __init__(self) -> None:
        super().__init__('random_pylon_spawner')

        # パラメータ宣言
        self.declare_parameter('model_path', '')
        self.declare_parameter('spawn_service', '/spawn_entity')
        self.declare_parameter('service_wait_timeout', 60.0)
        self.declare_parameter('service_wait_interval', 1.0)
        self.declare_parameter('min_longitudinal_spacing', 5.0)
        self.declare_parameter('longitudinal_margin', 1.0)
        self.declare_parameter('road_type', 'crank')      # straight / crank / scurve
        self.declare_parameter('road_width', 5.0)         # [m]
        self.declare_parameter('min_lateral_gap', 1.0)    # [m] 必ず残す隙間
        self.declare_parameter('pylon_block_half', 0.25)  # [m] パイロンが塞ぐ半幅（概念上）

        # Gazebo 原点周辺を避ける安全距離[m]
        self.origin_safety_radius = 5.0

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
        else:
            self.get_logger().info(f'Using pylon model: {model_path}')

        # 道路パラメータ
        self.road_type = self.get_parameter('road_type').get_parameter_value().string_value
        self.road_width = self.get_parameter('road_width').get_parameter_value().double_value
        self.min_lateral_gap = (
            self.get_parameter('min_lateral_gap').get_parameter_value().double_value
        )
        self.pylon_block_half = (
            self.get_parameter('pylon_block_half').get_parameter_value().double_value
        )

        self.get_logger().info(
            f'Road type: {self.road_type}, width: {self.road_width:.2f} m'
        )

        # 中心線折れ線とセグメント
        points = get_polyline_points(self.road_type)
        self.segments = build_segments(points)
        self.total_length = sum(seg.length for seg in self.segments)

        # サービス待ち
        spawn_service = self.get_parameter('spawn_service').get_parameter_value().string_value
        wait_timeout = self.get_parameter('service_wait_timeout').get_parameter_value().double_value
        wait_interval = (
            self.get_parameter('service_wait_interval').get_parameter_value().double_value
        )

        self.client = self.create_client(SpawnEntity, spawn_service)
        elapsed = 0.0
        while not self.client.wait_for_service(timeout_sec=wait_interval):
            elapsed += wait_interval
            self.get_logger().warn(
                f'{spawn_service} が利用不可のため待機中... '
                f'({elapsed:.1f}s / {wait_timeout:.1f}s)'
            )
            if elapsed >= wait_timeout:
                message = f'{spawn_service} service not available after {elapsed:.1f}s.'
                self.get_logger().error(message)
                raise RuntimeError(message)

        self.get_logger().info('RandomPylonSpawner started. Spawning pylons...')
        self.spawn_pylons_once()

    # ------------------ 中心線上の位置計算 ------------------

    def _sample_positions_on_segment(
        self,
        segment_index: int,
        segment: Segment,
        start_s: float,
        min_spacing: float,
        margin: float,
    ) -> List[float]:
        """各セグメント長に応じて長手方向の配置位置を決める."""
        s_min = start_s + margin
        s_max = start_s + max(segment.length - margin, margin)

        # 50m あたり 2〜4 本を基本とし、セグメント長に比例させる
        segment_factor = max(1, math.ceil(segment.length / 50.0))
        desired_count = random.randint(2, 4) * segment_factor

        positions: List[float] = []
        attempts = 0
        while len(positions) < desired_count and attempts < 200:
            candidate = random.uniform(s_min, s_max)
            if all(abs(candidate - pos) >= min_spacing for pos in positions):
                positions.append(candidate)
            attempts += 1

        positions.sort()
        if not positions:
            self.get_logger().warn(
                f'Segment {segment_index} にパイロンを配置できませんでした。範囲を確認してください。'
            )

        return positions

    def _pose_on_centerline(self, s: float) -> Tuple[float, float, float]:
        """中心線上の距離 s[m] に対応する (x,y,yaw) を返す."""
        remaining = s
        for seg in self.segments:
            if remaining <= seg.length:
                t = remaining / seg.length
                x = seg.p0[0] + seg.dx * t
                y = seg.p0[1] + seg.dy * t
                yaw = seg.yaw
                return x, y, yaw
            remaining -= seg.length

        # 誤差対策：終端を返す
        last_seg = self.segments[-1]
        return last_seg.p1[0], last_seg.p1[1], last_seg.yaw

    # ------------------ 道幅方向と隙間判定 ------------------

    def _choose_arrangement(self, num_pylons: int) -> str:
        """パイロンの並べ方をランダムに選択."""
        if num_pylons == 1:
            return 'center'
        return random.choice(['spread', 'cluster'])

    def _compute_lateral_offsets(
        self,
        num: int,
        road_width: float,
        arrangement: str,
    ) -> List[float]:
        """道路幅方向の相対オフセットを計算."""
        margin = road_width * 0.1
        usable_width = road_width - 2.0 * margin

        if num == 1 or arrangement == 'center':
            return [0.0]

        if arrangement == 'cluster':
            # まとめ配置では 0.3m 間隔で横並び
            start_offset = -0.3 * (num - 1) / 2.0
            offsets = [start_offset + 0.3 * i for i in range(num)]
        else:  # spread
            if num == 2:
                delta = usable_width / 4.0
                offsets = [-delta, delta]
            else:  # num == 3
                delta = usable_width / 6.0
                offsets = [-delta, 0.0, delta]

        # 道路端の内側にクリップ
        clipped_offsets: List[float] = []
        half_width = road_width / 2.0 - margin
        for offset in offsets:
            clipped = max(min(offset, half_width), -half_width)
            clipped_offsets.append(clipped)

        return clipped_offsets

    def _has_enough_lateral_gap(
        self,
        offsets: List[float],
        road_width: float,
    ) -> bool:
        """道幅方向に 1m 以上の隙間が残っているか判定."""
        required_gap = self.min_lateral_gap
        block_half = self.pylon_block_half

        half_w = road_width / 2.0
        intervals: List[Tuple[float, float]] = []

        for o in offsets:
            a = max(-half_w, o - block_half)
            b = min(half_w, o + block_half)
            intervals.append((a, b))

        if not intervals:
            return True

        # 区間をマージ
        intervals.sort()
        merged: List[Tuple[float, float]] = [intervals[0]]
        for a, b in intervals[1:]:
            last_a, last_b = merged[-1]
            if a <= last_b:
                merged[-1] = (last_a, max(last_b, b))
            else:
                merged.append((a, b))

        covered_width = sum(b - a for a, b in merged)
        remaining_width = road_width - covered_width
        return remaining_width >= required_gap

    # ------------------ スポーン処理 ------------------

    def spawn_pylons_once(self) -> None:
        """1度だけランダムなパイロン列をスポーンする."""
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            return

        min_spacing = (
            self.get_parameter('min_longitudinal_spacing').get_parameter_value().double_value
        )
        margin = self.get_parameter('longitudinal_margin').get_parameter_value().double_value

        segment_equiv = max(1, int(math.ceil(self.total_length / 50.0)))
        desired_count_range = (segment_equiv * 2, segment_equiv * 4)

        pylon_index = 0

        # セグメント単位で長手方向の位置 s[m] を決定
        start_s = 0.0
        for segment_index, segment in enumerate(self.segments):
            axis_positions = self._sample_positions_on_segment(
                segment_index=segment_index,
                segment=segment,
                start_s=start_s,
                min_spacing=min_spacing,
                margin=margin,
            )

            start_s += segment.length

            for s in axis_positions:
                center_x, center_y, yaw = self._pose_on_centerline(s)

                # 1〜3 本ランダムに配置しつつ、隙間 1m を残す
                num_pylons = random.randint(1, 3)
                arrangement = self._choose_arrangement(num_pylons)
                lateral_offsets = self._compute_lateral_offsets(
                    num_pylons, self.road_width, arrangement
                )

                if not self._has_enough_lateral_gap(lateral_offsets, self.road_width):
                    # 隙間が足りない場合は本数を減らす（最終的には1本センター）
                    self.get_logger().debug(
                        '隙間 1m を確保できない配置のため、配置パターンを簡素化します。'
                    )
                    lateral_offsets = [0.0]

                # 各オフセットごとにパイロンをスポーン
                for lateral_offset in lateral_offsets:
                    pose = self._build_pose(center_x, center_y, yaw, lateral_offset)
                    if self._is_inside_origin_safety_zone(pose):
                        self.get_logger().debug(
                            '原点から 5m 以内のためパイロン生成をスキップします。'
                        )
                        continue
                    name = f'{self.road_type}_pylon_{pylon_index}'
                    self._spawn_single_pylon(model_path, name, pose)
                    pylon_index += 1

    def _build_pose(self, cx: float, cy: float, yaw: float, lateral_offset: float) -> Pose:
        """中心線上の点 (cx,cy,yaw) と左右オフセットから Pose を構築."""
        pose = Pose()

        # 道路進行方向に対して左が +、右が - になるようなオフセット
        # 進行方向単位ベクトル: (cos yaw, sin yaw)
        # 左向き法線: (-sin yaw, cos yaw)
        nx = -math.sin(yaw)
        ny = math.cos(yaw)

        pose.position.x = cx + lateral_offset * nx
        pose.position.y = cy + lateral_offset * ny
        pose.position.z = 0.35  # パイロン高さ 0.7m の中心付近

        pose.orientation.w = 1.0  # 向きは特に重要でないので単位クォータニオン
        return pose

    def _spawn_single_pylon(self, model_path: str, name: str, pose: Pose) -> None:
        """単一のパイロンモデルを Gazebo にスポーンする."""
        with open(model_path, 'r', encoding='utf-8') as f:
            xml = f.read()

        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.robot_namespace = ''
        request.initial_pose = pose
        request.reference_frame = 'world'

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Spawned pylon: {name}')
        else:
            self.get_logger().error(f'Failed to spawn pylon: {name}')

    def _is_inside_origin_safety_zone(self, pose: Pose) -> bool:
        """Gazebo 原点からの距離が安全半径内か判定する."""
        distance = math.hypot(pose.position.x, pose.position.y)
        return distance < self.origin_safety_radius


def main(args=None) -> None:
    """エントリーポイント."""
    rclpy.init(args=args)
    logger = get_logger('random_pylon_spawner')
    node = None

    try:
        node = RandomPylonSpawner()
    except RuntimeError as exc:
        logger.fatal(f'ノードの起動に失敗しました: {exc}')
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

