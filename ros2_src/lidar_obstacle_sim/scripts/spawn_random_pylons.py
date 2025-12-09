#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Gazebo 上にランダムなパイロン列をスポーンする ROS2 ノード."""

from __future__ import annotations

import os
import random
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.logging import get_logger
from rclpy.node import Node


@dataclass
class RoadSegment:
    """道路セグメントの形状と向きを保持するデータクラス."""

    name: str
    center_x: float
    center_y: float
    length: float
    width: float
    orientation: str  # 'x' or 'y'

    def axis_limits(self, margin: float) -> Tuple[float, float]:
        """長手方向の生成可能範囲を返す."""
        half_length = self.length / 2.0
        start = -half_length + margin
        end = half_length - margin
        center_axis = self.center_x if self.orientation == 'x' else self.center_y
        return center_axis + start, center_axis + end


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

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
        else:
            self.get_logger().info(f'Using pylon model: {model_path}')

        self.road_segments: List[RoadSegment] = [
            RoadSegment(
                name='road_segment_1',
                center_x=10.0,
                center_y=0.0,
                length=20.0,
                width=5.0,
                orientation='x',
            ),
            RoadSegment(
                name='road_segment_2',
                center_x=20.0,
                center_y=10.0,
                length=20.0,
                width=5.0,
                orientation='y',
            ),
            RoadSegment(
                name='road_segment_3',
                center_x=30.0,
                center_y=20.0,
                length=20.0,
                width=5.0,
                orientation='x',
            ),
        ]

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

        pylon_index = 0
        for road_segment in self.road_segments:
            axis_positions = self._generate_axis_positions(road_segment, min_spacing, margin)
            for axis_position in axis_positions:
                num_pylons = random.randint(1, 3)
                arrangement = self._choose_arrangement(num_pylons)
                lateral_offsets = self._compute_lateral_offsets(
                    num_pylons, road_segment.width, arrangement
                )

                for lateral_offset in lateral_offsets:
                    pose = self._build_pose(road_segment, axis_position, lateral_offset)
                    name = f'{road_segment.name}_pylon_{pylon_index}'
                    self._spawn_single_pylon(model_path, name, pose)
                    pylon_index += 1

    def _generate_axis_positions(
        self, road_segment: 'RoadSegment', min_spacing: float, margin: float
    ) -> List[float]:
        """道路長手方向に離間距離を守った生成位置を決定する."""
        axis_min, axis_max = road_segment.axis_limits(margin)
        desired_count = random.randint(2, 4)
        positions: List[float] = []
        attempts = 0

        while len(positions) < desired_count and attempts < 100:
            candidate = random.uniform(axis_min, axis_max)
            if all(abs(candidate - pos) >= min_spacing for pos in positions):
                positions.append(candidate)
            attempts += 1

        positions.sort()
        if not positions:
            self.get_logger().warn(
                f'{road_segment.name} にパイロンを配置できませんでした。範囲を再確認してください。'
            )

        return positions

    def _choose_arrangement(self, num_pylons: int) -> str:
        """パイロンの並べ方をランダムに選択する."""
        if num_pylons == 1:
            return 'center'

        # 均等配置とまとめ配置を同確率で選択する
        return random.choice(['spread', 'cluster'])

    def _compute_lateral_offsets(
        self, num: int, road_width: float, arrangement: str
    ) -> List[float]:
        """道路幅方向の相対オフセットを計算する."""
        margin = road_width * 0.1
        usable_width = road_width - 2.0 * margin

        if num == 1 or arrangement == 'center':
            return [0.0]

        if arrangement == 'cluster':
            # まとめ配置では0.3m間隔で横並びに配置する
            start_offset = -0.3 * (num - 1) / 2.0
            offsets = [start_offset + 0.3 * i for i in range(num)]
        else:
            if num == 2:
                delta = usable_width / 4.0
                offsets = [-delta, delta]
            else:
                delta = usable_width / 6.0
                offsets = [-delta, 0.0, delta]

        clipped_offsets: List[float] = []
        half_width = road_width / 2.0 - margin
        for offset in offsets:
            clipped = max(min(offset, half_width), -half_width)
            clipped_offsets.append(clipped)

        return clipped_offsets

    def _build_pose(
        self, road_segment: 'RoadSegment', axis_position: float, lateral_offset: float
    ) -> Pose:
        """道路の姿勢を考慮したワールド座標のPoseを生成する."""
        pose = Pose()
        if road_segment.orientation == 'x':
            pose.position.x = axis_position
            pose.position.y = road_segment.center_y + lateral_offset
        else:
            pose.position.x = road_segment.center_x + lateral_offset
            pose.position.y = axis_position

        pose.position.z = 0.35  # パイロン高さ 0.7m の中心付近
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
