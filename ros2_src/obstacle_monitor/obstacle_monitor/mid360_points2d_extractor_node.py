#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Livox MID360 の PointCloud2 を擬似2Dの PointCloud2 に変換するノード.

高さ・半径のフィルタを通した点群を Z=0 に正規化した擬似2D点群として出力する。
``/mid360/livox/lidar`` を購読し、``/mid360/points2d`` を Publish する。
"""

import math
from typing import Iterable, List, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from cv_bridge import CvBridge


class Mid360Points2DExtractorNode(Node):
    """Livox MID360 の点群を Z=0 の PointCloud2 へ正規化するノード."""

    def __init__(self) -> None:
        super().__init__('mid360_points2d_extractor')

        # ---- Parameters ----
        self.declare_parameter('height_min_m', 0.1)
        self.declare_parameter('height_max_m', 1.0)
        self.declare_parameter('sensor_height_m', 1.005)
        self.declare_parameter('max_radius_m', 20.0)
        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('frame_id', 'mid360_frame')
        self.declare_parameter('input_topic', '/mid360/livox/lidar')
        self.declare_parameter('output_topic', '/mid360/points2d')
        self.declare_parameter('viewer_topic', '/mid360/points2d_viewer')
        self.declare_parameter('viewer_x_range_m', 20.0)
        self.declare_parameter('viewer_y_range_m', 5.0)
        self.declare_parameter('viewer_pixel_pitch', 40)
        self.declare_parameter('viewer_grid_interval_m', 5.0)
        self.declare_parameter('robot_width_m', 0.8)

        self.height_min = float(self.get_parameter('height_min_m').value)
        self.height_max = float(self.get_parameter('height_max_m').value)
        self.sensor_height = float(self.get_parameter('sensor_height_m').value)
        self.max_radius = float(self.get_parameter('max_radius_m').value)
        self.range_min = float(self.get_parameter('range_min_m').value)
        self.frame_id: str = str(self.get_parameter('frame_id').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        viewer_topic = str(self.get_parameter('viewer_topic').value)

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
        qos_rel_volatile_shallow = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.pub_viewer = self.create_publisher(
            Image,
            viewer_topic,
            qos_rel_volatile_shallow,
        )

        self.bridge = CvBridge()

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
            absolute_height = z + self.sensor_height
            if absolute_height < self.height_min or absolute_height > self.height_max:
                continue
            radius = math.hypot(x, y)
            if radius < self.range_min or radius > self.max_radius:
                continue
            points.append((float(x), float(y), 0.0))

        cloud_msg = self._build_cloud_msg(msg, points)
        self.pub_cloud.publish(cloud_msg)
        self._publish_scan_image(points, cloud_msg.header)

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

    def _publish_scan_image(
        self,
        points: List[Tuple[float, float, float]],
        header: Header,
    ) -> None:
        """擬似2D点群を laserScanViewer 相当の画像として publish する."""
        x_range_m = float(self.get_parameter('viewer_x_range_m').value)
        y_range_m = float(self.get_parameter('viewer_y_range_m').value)
        pixel_pitch = int(self.get_parameter('viewer_pixel_pitch').value)
        grid_interval_m = float(self.get_parameter('viewer_grid_interval_m').value)
        robot_width = float(self.get_parameter('robot_width_m').value)

        x_min = 0.0
        x_max = x_range_m
        y_min = -1.0 * y_range_m
        y_max = y_range_m

        width = int((y_max - y_min) * pixel_pitch)
        height = int((x_max - x_min) * pixel_pitch)
        grid = np.full((height, width, 3), 255, dtype=np.uint8)

        # ロボット幅ライン（黒）
        center_px = int(width / 2)
        offset_px = int(robot_width * pixel_pitch)
        for line_x in (center_px - offset_px, center_px + offset_px):
            if 0 <= line_x < width:
                cv2.line(
                    grid,
                    (line_x, height),
                    (line_x, 0),
                    (0, 0, 0),
                    thickness=2,
                    lineType=cv2.LINE_AA,
                )

        # 進行方向の補助線（5mごと）
        if grid_interval_m > 0.0:
            max_step = int(math.floor(x_max / grid_interval_m))
            for idx in range(1, max_step + 1):
                x_pos = grid_interval_m * idx
                py = height - int(round(x_pos * pixel_pitch))
                if 0 <= py < height:
                    cv2.line(
                        grid,
                        (0, py),
                        (width, py),
                        (220, 220, 220),
                        thickness=1,
                        lineType=cv2.LINE_AA,
                    )

        # 点群の描画（赤）
        for x, y, _ in points:
            if x_min < x < x_max and y_min < y < y_max:
                px = int(width / 2) - int(y * pixel_pitch)
                py = height - int(x * pixel_pitch)
                if 0 <= px < width and 0 <= py < height:
                    cv2.circle(grid, (px, py), 3, (0, 0, 255), -1)

        img_msg = self.bridge.cv2_to_imgmsg(grid, encoding='bgr8')
        img_msg.header.stamp = header.stamp
        img_msg.header.frame_id = header.frame_id
        self.pub_viewer.publish(img_msg)


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
