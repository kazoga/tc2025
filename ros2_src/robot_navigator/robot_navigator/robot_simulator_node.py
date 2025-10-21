#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot_simulator_node.py
ROS 2 ノード: robot_simulator
- /cmd_vel を購読し、差動二輪モデルで運動学を積分
- 最初に受信した /active_target で初期姿勢と map→odom 原点を確定し、以降は /amcl_pose で原点更新に対応
- /odom, /amcl_pose, /tf を配信
"""

from __future__ import annotations

import math
import time
import random
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = sy
    q.w = cy
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """z軸回転のヨー角をクォータニオンから取得。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


@dataclass
class SimulatorState:
    x: float
    y: float
    yaw: float
    v: float
    w: float
    last_cmd_time: float


class RobotSimulatorNode(Node):
    """robot_simulator ノード本体。"""

    def __init__(self) -> None:
        super().__init__('robot_simulator')

        # --- パラメータ ---
        self.declare_parameter('cycle_hz', 10.0)
        self.declare_parameter('max_linear_mps', 2.0)
        self.declare_parameter('max_angular_rps', 3.0)
        self.declare_parameter('enable_cmd_limit', True)
        self.declare_parameter('pose_noise_std_m', 0.0)
        self.declare_parameter('yaw_noise_std_deg', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('frame_map', 'map')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('yaw0_deg', 0.0)
        self.declare_parameter('timer_publish_odom_ms', 100)
        self.declare_parameter('cmd_timeout_sec', 2.0)

        self._cycle_hz: float = float(self.get_parameter('cycle_hz').value)
        self._dt: float = 1.0 / self._cycle_hz if self._cycle_hz > 0.0 else 0.1
        self._max_v: float = float(self.get_parameter('max_linear_mps').value)
        self._max_w: float = float(self.get_parameter('max_angular_rps').value)
        self._enable_limit: bool = bool(self.get_parameter('enable_cmd_limit').value)
        self._pose_noise_std_m: float = float(self.get_parameter('pose_noise_std_m').value)
        self._yaw_noise_std_deg: float = float(self.get_parameter('yaw_noise_std_deg').value)
        self._enable_tf_pub: bool = bool(self.get_parameter('publish_tf').value)
        self._frame_map: str = str(self.get_parameter('frame_map').value)
        self._frame_odom: str = str(self.get_parameter('frame_odom').value)
        self._frame_base: str = str(self.get_parameter('frame_base').value)
        x0: float = float(self.get_parameter('x0').value)
        y0: float = float(self.get_parameter('y0').value)
        yaw0: float = math.radians(float(self.get_parameter('yaw0_deg').value))
        self._odom_period_ms: int = int(self.get_parameter('timer_publish_odom_ms').value)
        self._cmd_timeout_sec: float = float(self.get_parameter('cmd_timeout_sec').value)

        # --- 状態 ---
        now = time.time()
        self._state = SimulatorState(x0, y0, yaw0, 0.0, 0.0, now)
        self._last_timeout_warn = 0.0
        self._last_limit_warn = 0.0
        self._odom_accum_ms = 0.0
        self._odom_period_s = max(0.01, float(self._odom_period_ms) / 1000.0)
        self._last_wait_log = 0.0

        # --- 初期AMCL姿勢保持（map→odom変換） ---
        self._amcl_origin_received = False
        self._initial_pose_set = False
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_origin_yaw = 0.0
        self._last_published_amcl_stamp: Optional[tuple[int, int]] = None

        qos = QoSProfile(depth=10)
        self._pub_odom = self.create_publisher(Odometry, '/odom', qos)
        self._pub_amcl = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', qos)
        self._tf_broadcaster: Optional[TransformBroadcaster] = TransformBroadcaster(self) if self._enable_tf_pub else None

        # --- 購読 ---
        self._sub_cmd = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, qos)
        self._sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_origin, qos)
        self._sub_active_target = self.create_subscription(PoseStamped, '/active_target', self._on_active_target, qos)

        # --- タイマ ---
        self._timer_update = self.create_timer(self._dt, self._on_timer_update)

        self.get_logger().info(f'robot_simulator_node started (cycle_hz={self._cycle_hz:.1f})')

    # ------------------------------
    # コールバック
    # ------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        now = time.time()
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        if self._enable_limit:
            v_clamped = clamp(v, -self._max_v, self._max_v)
            w_clamped = clamp(w, -self._max_w, self._max_w)
            if (abs(v) > self._max_v or abs(w) > self._max_w) and (now - self._last_limit_warn > 1.0):
                self._last_limit_warn = now
                self.get_logger().warn(f'/cmd_vel exceeds limit (v={v:.2f}, w={w:.2f}) -> clamped')
            v, w = v_clamped, w_clamped

        if not math.isfinite(v) or not math.isfinite(w):
            self.get_logger().error('Invalid /cmd_vel (NaN/Inf) ignored')
            return

        if not self._initial_pose_set:
            return

        self._state.v = v
        self._state.w = w
        self._state.last_cmd_time = now

    def _on_amcl_origin(self, msg: PoseWithCovarianceStamped) -> None:
        """初期化完了後の /amcl_pose で map→odom 原点を更新。"""
        if not self._initial_pose_set:
            return

        stamp_tuple = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self._last_published_amcl_stamp is not None and stamp_tuple == self._last_published_amcl_stamp:
            return

        self._map_origin_x = msg.pose.pose.position.x
        self._map_origin_y = msg.pose.pose.position.y
        self._map_origin_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self._amcl_origin_received = True

        self.get_logger().info(
            f"AMCL origin updated from /amcl_pose: "
            f"({self._map_origin_x:.2f}, {self._map_origin_y:.2f}, yaw={math.degrees(self._map_origin_yaw):.1f}°)"
        )

    
    def _on_active_target(self, msg: PoseStamped) -> None:
        """最初の /active_target を初期姿勢として採用（1回のみ）。"""
        if self._initial_pose_set:
            return
        # 位置・姿勢
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = quaternion_to_yaw(msg.pose.orientation)
        # 内部状態に反映
        self._state.x = x
        self._state.y = y
        self._state.yaw = yaw
        # map→odom 初期変換としても採用（/amcl_pose を map 座標で自然に見せる）
        self._map_origin_x = x
        self._map_origin_y = y
        self._map_origin_yaw = yaw
        self._amcl_origin_received = True
        self._initial_pose_set = True
        self._state.last_cmd_time = time.time()
        self._last_wait_log = 0.0
        self.get_logger().info(
            f"Initial pose set from first /active_target: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°"
        )

    def _on_timer_update(self) -> None:
        now = time.time()
        if not self._initial_pose_set:
            if now - self._last_wait_log > 1.0:
                self._last_wait_log = now
                self.get_logger().info('Waiting for /active_target to initialize initial pose. Simulator outputs are on hold.')
            return

        if (now - self._state.last_cmd_time) > self._cmd_timeout_sec:
            self._state.v = 0.0
            self._state.w = 0.0
            if now - self._last_timeout_warn > 1.0:
                self._last_timeout_warn = now
                self.get_logger().warn('No /cmd_vel within timeout window → stop.')

        # 運動学更新
        self._state.x += self._state.v * math.cos(self._state.yaw) * self._dt
        self._state.y += self._state.v * math.sin(self._state.yaw) * self._dt
        self._state.yaw = normalize_angle(self._state.yaw + self._state.w * self._dt)

        self._odom_accum_ms += self._dt
        if self._odom_accum_ms + 1e-9 >= self._odom_period_s:
            self._odom_accum_ms = 0.0
            self._publish_odom()
            self._publish_amcl_pose()
            if self._enable_tf_pub and self._tf_broadcaster is not None:
                self._publish_tf()

    # ------------------------------
    # Publish helpers
    # ------------------------------
    def _publish_odom(self) -> None:
        msg = Odometry()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame_odom)
        msg.child_frame_id = self._frame_base

        msg.pose.pose.position.x = self._state.x
        msg.pose.pose.position.y = self._state.y
        msg.pose.pose.orientation = yaw_to_quaternion(self._state.yaw)

        cov = [0.0] * 36
        cov[0] = cov[7] = 1e-4
        cov[35] = 1e-3
        msg.pose.covariance = cov

        msg.twist.twist.linear.x = self._state.v
        msg.twist.twist.angular.z = self._state.w
        self._pub_odom.publish(msg)

    def _publish_amcl_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        stamp = self.get_clock().now().to_msg()
        msg.header = Header(stamp=stamp, frame_id=self._frame_map)

        # odom->map 変換
        x_odom, y_odom, yaw_odom = self._state.x, self._state.y, self._state.yaw
        if self._amcl_origin_received:
            c = math.cos(self._map_origin_yaw)
            s = math.sin(self._map_origin_yaw)
            x = self._map_origin_x + c * x_odom - s * y_odom
            y = self._map_origin_y + s * x_odom + c * y_odom
            yaw = normalize_angle(self._map_origin_yaw + yaw_odom)
        else:
            x, y, yaw = x_odom, y_odom, yaw_odom

        # ノイズ付与
        nx = random.gauss(0.0, self._pose_noise_std_m) if self._pose_noise_std_m > 0 else 0.0
        ny = random.gauss(0.0, self._pose_noise_std_m) if self._pose_noise_std_m > 0 else 0.0
        nyaw_deg = random.gauss(0.0, self._yaw_noise_std_deg) if self._yaw_noise_std_deg > 0 else 0.0
        nyaw = math.radians(nyaw_deg)
        x += nx
        y += ny
        yaw = normalize_angle(yaw + nyaw)

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        cov = [0.0] * 36
        cov[0] = cov[7] = max(1e-4, self._pose_noise_std_m ** 2)
        cov[35] = max(1e-3, math.radians(max(1e-3, self._yaw_noise_std_deg)) ** 2)
        msg.pose.covariance = cov

        self._last_published_amcl_stamp = (stamp.sec, stamp.nanosec)
        self._pub_amcl.publish(msg)

    def _publish_tf(self) -> None:
        assert self._tf_broadcaster is not None
        stamp = self.get_clock().now().to_msg()

        # map->odom
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = stamp
        t_map_odom.header.frame_id = self._frame_map
        t_map_odom.child_frame_id = self._frame_odom
        if self._amcl_origin_received:
            t_map_odom.transform.translation.x = self._map_origin_x
            t_map_odom.transform.translation.y = self._map_origin_y
            t_map_odom.transform.rotation = yaw_to_quaternion(self._map_origin_yaw)
        else:
            t_map_odom.transform.translation.x = 0.0
            t_map_odom.transform.translation.y = 0.0
            t_map_odom.transform.rotation = yaw_to_quaternion(0.0)

        # odom->base_link
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = self._frame_odom
        t_odom_base.child_frame_id = self._frame_base
        t_odom_base.transform.translation.x = self._state.x
        t_odom_base.transform.translation.y = self._state.y
        t_odom_base.transform.rotation = yaw_to_quaternion(self._state.yaw)

        self._tf_broadcaster.sendTransform([t_map_odom, t_odom_base])


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
