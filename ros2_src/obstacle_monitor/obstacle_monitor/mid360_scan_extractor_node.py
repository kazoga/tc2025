#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Livox MID360 の PointCloud2 を 2D LaserScan へ変換するノード.

高さ・半径のフィルタを通した点群を擬似的な LiDAR スキャンに変換する。
``/mid360/livox/lidar`` を購読し、``/mid360/scan`` を Publish する。
"""

import math
from typing import Iterable, List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class Mid360ScanExtractorNode(Node):
    """Livox MID360 の点群を LaserScan へ射影するノード."""

    def __init__(self) -> None:
        super().__init__('mid360_scan_extractor')

        # ---- Parameters ----
        self.declare_parameter('height_min_m', 0.1)
        self.declare_parameter('height_max_m', 1.0)
        self.declare_parameter('max_radius_m', 20.0)
        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('angle_min_deg', -180.0)
        self.declare_parameter('angle_max_deg', 180.0)
        self.declare_parameter('angle_increment_deg', 0.5)
        self.declare_parameter('frame_id', 'mid360_frame')
        self.declare_parameter('input_topic', '/mid360/livox/lidar')
        self.declare_parameter('output_topic', '/mid360/scan')

        self.height_min = float(self.get_parameter('height_min_m').value)
        self.height_max = float(self.get_parameter('height_max_m').value)
        self.max_radius = float(self.get_parameter('max_radius_m').value)
        self.range_min = float(self.get_parameter('range_min_m').value)
        angle_min_deg = float(self.get_parameter('angle_min_deg').value)
        angle_max_deg = float(self.get_parameter('angle_max_deg').value)
        angle_increment_deg = float(self.get_parameter('angle_increment_deg').value)
        self.frame_id: str = self._normalize_frame_id(
            str(self.get_parameter('frame_id').value)
        )
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.angle_min = math.radians(angle_min_deg)
        self.angle_max = math.radians(angle_max_deg)
        self.angle_increment = math.radians(angle_increment_deg)
        self._validate_angles()
        self._ensure_height_order()

        self.bin_count = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1

        # ---- Pub/Sub ----
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_point_cloud,
            qos_profile_sensor_data,
        )
        self.pub_scan = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info('mid360_scan_extractor を起動しました。')

    def _validate_angles(self) -> None:
        """角度パラメータの妥当性を検証する."""
        if self.angle_increment <= 0.0:
            raise ValueError('angle_increment_deg は正の値を指定してください。')
        if self.angle_max <= self.angle_min:
            raise ValueError('angle_max_deg は angle_min_deg より大きい値を指定してください。')

    def _ensure_height_order(self) -> None:
        """高さ範囲の上下が逆転している場合は修正する."""
        if self.height_max < self.height_min:
            self.get_logger().warn(
                'height_max_m が height_min_m より小さいため交換します。'
            )
            self.height_min, self.height_max = self.height_max, self.height_min

    def _on_point_cloud(self, msg: PointCloud2) -> None:
        """PointCloud2 を受信して LaserScan を生成する."""
        ranges = [math.inf] * self.bin_count

        for x, y, z in self._iterate_points(msg):
            if z < self.height_min or z > self.height_max:
                continue
            r = math.hypot(x, y)
            if r < self.range_min or r > self.max_radius:
                continue
            theta = math.atan2(y, x)
            if theta < self.angle_min or theta > self.angle_max:
                continue
            index = int((theta - self.angle_min) / self.angle_increment)
            if 0 <= index < self.bin_count and r < ranges[index]:
                ranges[index] = r

        scan_msg = self._build_scan_msg(msg, ranges)
        self.pub_scan.publish(scan_msg)

    def _iterate_points(self, msg: PointCloud2) -> Iterable[List[float]]:
        """PointCloud2 から (x, y, z) を取得するジェネレータ."""
        return point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

    def _build_scan_msg(self, cloud: PointCloud2, ranges: List[float]) -> LaserScan:
        """LaserScan メッセージを組み立てる."""
        scan = LaserScan()
        scan.header.stamp = cloud.header.stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min = float(self.angle_min)
        scan.angle_max = float(self.angle_max)
        scan.angle_increment = float(self.angle_increment)
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.max_radius)
        scan.ranges = [float(r) for r in ranges]
        return scan

    def _normalize_frame_id(self, frame_id: str) -> str:
        """tf2 仕様に従う frame_id へ正規化する."""

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
    node = Mid360ScanExtractorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
