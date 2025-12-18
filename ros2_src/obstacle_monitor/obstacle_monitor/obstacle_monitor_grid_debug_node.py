#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LaserScan と PointCloud2 を rolling grid へ蓄積し可視化するデバッグノード."""

import math
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, LookupException, TransformException, TransformListener


class RollingGrid:
    """時間正規化スコアを保持する rolling grid."""

    def __init__(self, size_m: float, resolution_m: float) -> None:
        self.size_m = size_m
        self.resolution = resolution_m
        self.width = int(round(size_m / resolution_m))
        self.height = int(round(size_m / resolution_m))
        self.center_x = 0.0
        self.center_y = 0.0
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0
        self.free_scores = np.zeros((self.height, self.width), dtype=np.float32)
        self.hit_scores = np.zeros((self.height, self.width), dtype=np.float32)
        self.last_hit = np.zeros((self.height, self.width), dtype=np.float64)

    def update_center(self, new_x: float, new_y: float) -> None:
        """中心の平行移動に合わせて配列をシフトする."""
        shift_x = int(round((new_x - self.center_x) / self.resolution))
        shift_y = int(round((new_y - self.center_y) / self.resolution))
        if shift_x == 0 and shift_y == 0:
            return

        if shift_x != 0:
            self.free_scores = np.roll(self.free_scores, -shift_x, axis=1)
            self.hit_scores = np.roll(self.hit_scores, -shift_x, axis=1)
            self.last_hit = np.roll(self.last_hit, -shift_x, axis=1)
            if shift_x > 0:
                self.free_scores[:, -shift_x:] = 0.0
                self.hit_scores[:, -shift_x:] = 0.0
                self.last_hit[:, -shift_x:] = 0.0
            else:
                self.free_scores[:, : -shift_x] = 0.0
                self.hit_scores[:, : -shift_x] = 0.0
                self.last_hit[:, : -shift_x] = 0.0

        if shift_y != 0:
            self.free_scores = np.roll(self.free_scores, -shift_y, axis=0)
            self.hit_scores = np.roll(self.hit_scores, -shift_y, axis=0)
            self.last_hit = np.roll(self.last_hit, -shift_y, axis=0)
            if shift_y > 0:
                self.free_scores[-shift_y:, :] = 0.0
                self.hit_scores[-shift_y:, :] = 0.0
                self.last_hit[-shift_y:, :] = 0.0
            else:
                self.free_scores[: -shift_y, :] = 0.0
                self.hit_scores[: -shift_y, :] = 0.0
                self.last_hit[: -shift_y, :] = 0.0

        self.center_x = new_x
        self.center_y = new_y
        self.origin_x = self.center_x - self.size_m / 2.0
        self.origin_y = self.center_y - self.size_m / 2.0

    def world_to_index(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """世界座標をグリッド添字へ変換する."""
        ix = int(math.floor((x - self.origin_x) / self.resolution))
        iy = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def add_hit(self, x: float, y: float, score: float, timestamp: float) -> None:
        """hit スコアを更新する."""
        index = self.world_to_index(x, y)
        if index is None:
            return
        ix, iy = index
        self.hit_scores[iy, ix] += score
        self.last_hit[iy, ix] = timestamp

    def add_free_line(
        self, start: Tuple[float, float], end: Tuple[float, float], score: float
    ) -> None:
        """Bresenham で線分上の free スコアを更新する（終点は除外）。"""
        start_index = self.world_to_index(*start)
        end_index = self.world_to_index(*end)
        if start_index is None or end_index is None:
            return
        for ix, iy in bresenham(start_index, end_index, include_endpoint=False):
            self.free_scores[iy, ix] += score


def bresenham(
    start: Tuple[int, int],
    end: Tuple[int, int],
    include_endpoint: bool = True,
) -> Iterable[Tuple[int, int]]:
    """Bresenham の整数格子列を生成する."""
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    x, y = x0, y0
    while True:
        if (x, y) != (x1, y1) or include_endpoint:
            yield x, y
        if (x, y) == (x1, y1):
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


class ObstacleMonitorGridDebugNode(Node):
    """LaserScan と Mid-360 点群を可視化する rolling grid ノード."""

    def __init__(self) -> None:
        super().__init__('obstacle_monitor_grid_debug')

        self.declare_parameter('grid_resolution_m', 0.1)
        self.declare_parameter('grid_size_m', 40.0)
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('mid_topic', '/mid360/points2d')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('utm_free_rate', 1.0)
        self.declare_parameter('utm_hit_rate', 1.0)
        self.declare_parameter('mid_hit_rate', 4.0)
        self.declare_parameter('decay_period_s', 0.2)
        self.declare_parameter('visualize_period_s', 0.1)
        self.declare_parameter('default_scan_dt_s', 0.025)
        self.declare_parameter('default_mid_dt_s', 0.1)
        self.declare_parameter('window_name', 'grid_debug')

        self.resolution = float(self.get_parameter('grid_resolution_m').value)
        self.size_m = float(self.get_parameter('grid_size_m').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.mid_topic = str(self.get_parameter('mid_topic').value)
        self.amcl_topic = str(self.get_parameter('amcl_topic').value)
        self.utm_free_rate = float(self.get_parameter('utm_free_rate').value)
        self.utm_hit_rate = float(self.get_parameter('utm_hit_rate').value)
        self.mid_hit_rate = float(self.get_parameter('mid_hit_rate').value)
        self.decay_period = float(self.get_parameter('decay_period_s').value)
        self.visualize_period = float(self.get_parameter('visualize_period_s').value)
        self.default_scan_dt = float(self.get_parameter('default_scan_dt_s').value)
        self.default_mid_dt = float(self.get_parameter('default_mid_dt_s').value)
        self.window_name = str(self.get_parameter('window_name').value)

        self.grid = RollingGrid(self.size_m, self.resolution)
        self.last_scan_time: Optional[Time] = None
        self.last_mid_time: Optional[Time] = None
        self.latest_amcl: Optional[PoseWithCovarianceStamped] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.sub_mid = self.create_subscription(
            PointCloud2, self.mid_topic, self._on_mid, qos_profile_sensor_data
        )
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_topic, self._on_amcl, 10
        )

        self.decay_timer = self.create_timer(self.decay_period, self._on_decay)
        self.visualize_timer = self.create_timer(
            self.visualize_period, self._on_visualize
        )

        self.get_logger().info('obstacle_monitor_grid_debug を起動しました。')

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        """amcl_pose を受信し中心を更新する."""
        self.latest_amcl = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.grid.update_center(x, y)

    def _on_scan(self, msg: LaserScan) -> None:
        """LaserScan を rolling grid へ反映する."""
        stamp = Time.from_msg(msg.header.stamp)
        dt = self._compute_dt(stamp, is_scan=True)

        transform = self._lookup_transform(msg.header.frame_id, stamp)
        if transform is None:
            return

        yaw = self._quaternion_to_yaw(transform.transform.rotation)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        for index, raw_distance in enumerate(msg.ranges):
            distance = raw_distance
            has_valid_hit = math.isfinite(raw_distance)
            if not has_valid_hit:
                distance = msg.range_max
            angle = yaw + msg.angle_min + index * msg.angle_increment
            free_end_x = tx + msg.range_max * math.cos(angle)
            free_end_y = ty + msg.range_max * math.sin(angle)
            self.grid.add_free_line((tx, ty), (free_end_x, free_end_y), self.utm_free_rate * dt)

            if has_valid_hit and msg.range_min <= distance <= msg.range_max:
                hit_x = tx + distance * math.cos(angle)
                hit_y = ty + distance * math.sin(angle)
                self.grid.add_hit(hit_x, hit_y, self.utm_hit_rate * dt, stamp.nanoseconds * 1e-9)

    def _on_mid(self, msg: PointCloud2) -> None:
        """Mid-360 の擬似2D点群を rolling grid へ反映する."""
        stamp = Time.from_msg(msg.header.stamp)
        dt = self._compute_dt(stamp, is_scan=False)

        transform = self._lookup_transform(msg.header.frame_id, stamp)
        if transform is None:
            return

        yaw = self._quaternion_to_yaw(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        for x, y, _ in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        ):
            world_x = tx + cos_yaw * x - sin_yaw * y
            world_y = ty + sin_yaw * x + cos_yaw * y
            self.grid.add_hit(world_x, world_y, self.mid_hit_rate * dt, stamp.nanoseconds * 1e-9)

    def _compute_dt(self, stamp: Time, is_scan: bool) -> float:
        """前回受信時刻との差分を計算する."""
        if is_scan:
            if self.last_scan_time is None:
                dt = self.default_scan_dt
            else:
                dt = max((stamp - self.last_scan_time).nanoseconds * 1e-9, 0.0)
            self.last_scan_time = stamp
            return dt

        if self.last_mid_time is None:
            dt = self.default_mid_dt
        else:
            dt = max((stamp - self.last_mid_time).nanoseconds * 1e-9, 0.0)
        self.last_mid_time = stamp
        return dt

    def _lookup_transform(self, source_frame: str, stamp: Time) -> Optional[TransformStamped]:
        """tf から target_frame への変換を取得する."""
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

    def _on_decay(self) -> None:
        """free/hit スコアを減衰させる."""
        self.grid.free_scores *= 0.8
        self.grid.hit_scores *= 0.95

    def _on_visualize(self) -> None:
        """OpenCV でグリッドを色分け表示する."""
        obstacle_mask = self.grid.hit_scores > self.grid.free_scores
        free_mask = self.grid.free_scores > self.grid.hit_scores

        image = np.zeros((self.grid.height, self.grid.width, 3), dtype=np.uint8)
        image[obstacle_mask] = (0, 0, 255)
        image[free_mask] = (255, 0, 0)

        flipped = np.ascontiguousarray(np.flipud(image))

        yaw = self._get_latest_yaw()
        if yaw is not None:
            self._draw_robot_orientation(flipped, yaw)

        cv2.imshow(self.window_name, flipped)
        cv2.waitKey(1)

    @staticmethod
    def _quaternion_to_yaw(quat) -> float:
        """クォータニオンから yaw [rad] を算出する."""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _get_latest_yaw(self) -> Optional[float]:
        """最新の amcl_pose から yaw を取得する."""
        if self.latest_amcl is None:
            return None
        return self._quaternion_to_yaw(
            self.latest_amcl.pose.pose.orientation
        )

    def _draw_robot_orientation(self, image: np.ndarray, yaw: float) -> None:
        """ロボット方位を示す三角形を画像中心に描画する."""
        center_x = self.grid.width // 2
        center_y = self.grid.height // 2
        tip_length = 12
        base_length = 7

        tip = (
            int(round(center_x + tip_length * math.cos(yaw))),
            int(round(center_y - tip_length * math.sin(yaw))),
        )
        left_base = (
            int(
                round(
                    center_x
                    + base_length * math.cos(yaw + 3.0 * math.pi / 4.0)
                )
            ),
            int(
                round(
                    center_y
                    - base_length * math.sin(yaw + 3.0 * math.pi / 4.0)
                )
            ),
        )
        right_base = (
            int(
                round(
                    center_x
                    + base_length * math.cos(yaw - 3.0 * math.pi / 4.0)
                )
            ),
            int(
                round(
                    center_y
                    - base_length * math.sin(yaw - 3.0 * math.pi / 4.0)
                )
            ),
        )

        points = np.array([tip, left_base, right_base], dtype=np.int32)
        cv2.fillPoly(image, [points], color=(0, 255, 0))


def main() -> None:
    rclpy.init()
    node = ObstacleMonitorGridDebugNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
