#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LaserScanとPointCloud2をbase_linkへ揃えて可視化するノード.

UTM-30LX (/scan) と Mid-360 (/mid360/points2d) を base_link 座標へ変換し、
laserScanViewer 相当の描画仕様で画像を生成して /grid_viewer に publish する。
"""

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, LookupException, TransformException, TransformListener

from cv_bridge import CvBridge


class LocalGridMapperNode(Node):
    """UTMとMid-360点群を同一画像へ描画するノード."""

    def __init__(self) -> None:
        super().__init__('local_grid_mapper')

        # ---- Parameters ----
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('mid_topic', '/mid360/points2d')
        self.declare_parameter('viewer_topic', '/grid_viewer')
        self.declare_parameter('viewer_x_range_m', 20.0)
        self.declare_parameter('viewer_y_range_m', 5.0)
        self.declare_parameter('viewer_pixel_pitch', 40)
        self.declare_parameter('viewer_grid_interval_m', 5.0)
        self.declare_parameter('robot_width_m', 0.8)

        self.target_frame = str(self.get_parameter('target_frame').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        mid_topic = str(self.get_parameter('mid_topic').value)
        viewer_topic = str(self.get_parameter('viewer_topic').value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Pub/Sub ----
        self.sub_scan = self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.sub_mid = self.create_subscription(
            PointCloud2,
            mid_topic,
            self._on_mid,
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

        self.last_scan_points: List[Tuple[float, float]] = []
        self.last_mid_points: List[Tuple[float, float]] = []
        self.last_header: Optional[Header] = None

        self.get_logger().info('local_grid_mapper を起動しました。')

    def _on_scan(self, msg: LaserScan) -> None:
        """LaserScan を受信し base_link へ変換して描画する."""
        transform = self._lookup_transform(
            msg.header.frame_id, Time.from_msg(msg.header.stamp)
        )
        if transform is None:
            return

        yaw = self._quaternion_to_yaw(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        points: List[Tuple[float, float]] = []
        angle = float(msg.angle_min)
        for raw_distance in msg.ranges:
            distance = float(raw_distance)
            if not math.isfinite(distance):
                angle += msg.angle_increment
                continue
            if distance < msg.range_min or distance > msg.range_max:
                angle += msg.angle_increment
                continue
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            base_x = tx + cos_yaw * local_x - sin_yaw * local_y
            base_y = ty + sin_yaw * local_x + cos_yaw * local_y
            points.append((base_x, base_y))
            angle += msg.angle_increment

        self.last_scan_points = points
        self.last_header = self._make_header(msg.header.stamp)
        self._publish_scan_image(
            self.last_scan_points, self.last_mid_points, self.last_header
        )

    def _on_mid(self, msg: PointCloud2) -> None:
        """Mid-360 の点群を受信し base_link へ変換して描画する."""
        transform = self._lookup_transform(
            msg.header.frame_id, Time.from_msg(msg.header.stamp)
        )
        if transform is None:
            return

        yaw = self._quaternion_to_yaw(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        points: List[Tuple[float, float]] = []
        for x, y, _ in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        ):
            base_x = tx + cos_yaw * x - sin_yaw * y
            base_y = ty + sin_yaw * x + cos_yaw * y
            points.append((float(base_x), float(base_y)))

        self.last_mid_points = points
        self.last_header = self._make_header(msg.header.stamp)
        self._publish_scan_image(
            self.last_scan_points, self.last_mid_points, self.last_header
        )

    def _make_header(self, stamp) -> Header:
        """base_link 基準のヘッダを生成する."""
        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame
        return header

    def _lookup_transform(
        self, source_frame: str, stamp: Time
    ) -> Optional[TransformStamped]:
        """target_frame への変換を取得する."""
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, TransformException) as exc:
            self.get_logger().warn(f'tf 取得に失敗しました: {exc}')
            return None

    def _publish_scan_image(
        self,
        scan_points: List[Tuple[float, float]],
        mid_points: List[Tuple[float, float]],
        header: Header,
    ) -> None:
        """UTMとMidの点群を laserScanViewer 相当の画像として publish する."""
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

        # 点群の描画（UTM: 青, Mid: 赤）
        self._draw_points(
            grid, scan_points, x_min, x_max, y_min, y_max, pixel_pitch, (255, 0, 0)
        )
        self._draw_points(
            grid, mid_points, x_min, x_max, y_min, y_max, pixel_pitch, (0, 0, 255)
        )

        img_msg = self.bridge.cv2_to_imgmsg(grid, encoding='bgr8')
        img_msg.header.stamp = header.stamp
        img_msg.header.frame_id = header.frame_id
        self.pub_viewer.publish(img_msg)

    def _draw_points(
        self,
        grid: np.ndarray,
        points: List[Tuple[float, float]],
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        pixel_pitch: int,
        color: Tuple[int, int, int],
    ) -> None:
        """点群を指定色で描画する."""
        height, width, _ = grid.shape
        for x, y in points:
            if x_min < x < x_max and y_min < y < y_max:
                px = int(width / 2) - int(y * pixel_pitch)
                py = height - int(x * pixel_pitch)
                if 0 <= px < width and 0 <= py < height:
                    cv2.circle(grid, (px, py), 3, color, -1)

    @staticmethod
    def _quaternion_to_yaw(quat) -> float:
        """クォータニオンから yaw [rad] を算出する."""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main() -> None:
    rclpy.init()
    node = LocalGridMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
