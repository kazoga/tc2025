#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Gazebo 上にランダムなパイロン列をスポーンする ROS2 ノード."""

from __future__ import annotations

import os
import random
from typing import List, Tuple

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.logging import get_logger
from rclpy.node import Node


class RandomPylonSpawner(Node):
    """Gazebo にランダムパイロンをスポーンする ROS2 ノード."""

    def __init__(self) -> None:
        super().__init__('random_pylon_spawner')

        # パラメータ宣言
        self.declare_parameter('model_path', '')
        self.declare_parameter('road_width', 5.0)
        self.declare_parameter('x_min', 5.0)
        self.declare_parameter('x_max', 35.0)
        self.declare_parameter('y_center', 0.0)
        self.declare_parameter('spawn_service', '/spawn_entity')
        self.declare_parameter('service_wait_timeout', 60.0)
        self.declare_parameter('service_wait_interval', 1.0)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
        else:
            self.get_logger().info(f'Using pylon model: {model_path}')

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

        road_width = self.get_parameter('road_width').get_parameter_value().double_value
        x_min = self.get_parameter('x_min').get_parameter_value().double_value
        x_max = self.get_parameter('x_max').get_parameter_value().double_value
        y_center = self.get_parameter('y_center').get_parameter_value().double_value

        # 1〜3本のパイロン本数をランダムに決定
        num_pylons = random.randint(1, 3)

        # 障害物列の中心 x 座標をランダムに決定
        x_center = random.uniform(x_min, x_max)

        # 道幅の中に横一列で並べる
        positions = self._compute_offsets(num_pylons, road_width, y_center)

        for i, (_, y) in enumerate(positions):
            pose = Pose()
            pose.position.x = x_center
            pose.position.y = y
            pose.position.z = 0.35  # パイロン高さ 0.7m の中心付近

            name = f'pylon_{i}'
            self._spawn_single_pylon(model_path, name, pose)

    def _compute_offsets(self, num: int, road_width: float, y_center: float) -> List[Tuple[float, float]]:
        """横一列に並ぶパイロンの相対オフセットを計算する."""
        margin = road_width * 0.1
        usable_width = road_width - 2.0 * margin

        if num == 1:
            return [(0.0, y_center)]
        elif num == 2:
            dy = usable_width / 4.0
            return [(0.0, y_center - dy), (0.0, y_center + dy)]
        else:
            dy = usable_width / 6.0
            return [
                (0.0, y_center - dy),
                (0.0, y_center),
                (0.0, y_center + dy),
            ]

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
