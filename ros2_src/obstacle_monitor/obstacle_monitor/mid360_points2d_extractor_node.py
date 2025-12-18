#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Livox MID360 の PointCloud2 を擬似2Dの PointCloud2 に変換するノード.

高さ・半径のフィルタを通した点群を Z=0 に正規化した擬似2D点群として出力する。
``/mid360/livox/lidar`` を購読し、``/mid360/points2d`` を Publish する。
"""

import math
from typing import Iterable, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Mid360Points2DExtractorNode(Node):
    """Livox MID360 の点群を Z=0 の PointCloud2 へ正規化するノード."""

    def __init__(self) -> None:
        super().__init__('mid360_points2d_extractor')

        # ---- Parameters ----
        self.declare_parameter('height_min_m', 0.1)
        self.declare_parameter('height_max_m', 1.0)
        self.declare_parameter('max_radius_m', 20.0)
        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('frame_id', 'mid360_frame')
        self.declare_parameter('input_topic', '/mid360/livox/lidar')
        self.declare_parameter('output_topic', '/mid360/points2d')

        self.height_min = float(self.get_parameter('height_min_m').value)
        self.height_max = float(self.get_parameter('height_max_m').value)
        self.max_radius = float(self.get_parameter('max_radius_m').value)
        self.range_min = float(self.get_parameter('range_min_m').value)
        self.frame_id: str = self._normalize_frame_id(
            str(self.get_parameter('frame_id').value)
        )
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._ensure_height_order()

        # ---- Pub/Sub ----
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_point_cloud,
            qos_profile_sensor_data,
        )
        self.pub_cloud = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info('mid360_points2d_extractor を起動しました。')

    def _ensure_height_order(self) -> None:
        """高さ範囲の上下が逆転している場合は修正する."""
        if self.height_max < self.height_min:
            self.get_logger().warn(
                'height_max_m が height_min_m より小さいため交換します。'
            )
            self.height_min, self.height_max = self.height_max, self.height_min

    def _on_point_cloud(self, msg: PointCloud2) -> None:
        """PointCloud2 を受信して擬似2Dの PointCloud2 を生成する."""
        points = []
        for x, y, z in self._iterate_points(msg):
            if z < self.height_min or z > self.height_max:
                continue
            radius = math.hypot(x, y)
            if radius < self.range_min or radius > self.max_radius:
                continue
            points.append((float(x), float(y), 0.0))

        cloud_msg = self._build_cloud_msg(msg, points)
        self.pub_cloud.publish(cloud_msg)

    def _iterate_points(self, msg: PointCloud2) -> Iterable[Tuple[float, float, float]]:
        """PointCloud2 から (x, y, z) を取得するジェネレータ."""
        return point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

    def _build_cloud_msg(
        self, cloud_in: PointCloud2, points: List[Tuple[float, float, float]]
    ) -> PointCloud2:
        """PointCloud2 メッセージを組み立てる."""
        header = Header()
        header.stamp = cloud_in.header.stamp
        header.frame_id = self.frame_id
        return point_cloud2.create_cloud_xyz32(header, points)

    def _normalize_frame_id(self, frame_id: str) -> str:
        """tf2 仕様に合うよう frame_id を正規化する.

        tf2 では先頭にスラッシュを付与したフレーム名は無効となるため、
        誤って指定された場合は除去して警告ログを出す。

        Args:
            frame_id (str): 入力されたフレームID.

        Returns:
            str: 先頭スラッシュを除去したフレームID.
        """

        if frame_id.startswith('/'):
            normalized = frame_id.lstrip('/')
            self.get_logger().warn(
                f'frame_id に先頭スラッシュが含まれていたため除去しました: '
                f"'{frame_id}' -> '{normalized}'"
            )
            return normalized
        return frame_id


def main() -> None:
    rclpy.init()
    node = Mid360Points2DExtractorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
