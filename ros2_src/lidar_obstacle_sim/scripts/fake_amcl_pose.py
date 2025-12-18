#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Gazebo の /odom から /amcl_pose と odom→base_link TF を生成する簡易ノード."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeAmclPose(Node):
    """Gazebo の /ypspur_ros/odom を /amcl_pose と TF に変換して publish するノード."""

    def __init__(self) -> None:
        super().__init__('fake_amcl_pose')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('odom_frame_id', 'odom')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        amcl_topic = self.get_parameter('amcl_topic').get_parameter_value().string_value
        self.odom_frame_id = (
            self.get_parameter('odom_frame_id').get_parameter_value().string_value
        )

        self.sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            amcl_topic,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            f'FakeAmclPose started. Subscribing {odom_topic}, publishing {amcl_topic}.'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """/odom を受信して /amcl_pose と odom→base_link の TF を publish."""
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        # ここでは map = odom とみなす簡易実装
        out.header.frame_id = 'map'
        out.pose.pose = msg.pose.pose
        # 共分散は簡略化して 0 のまま
        out.pose.covariance = [0.0] * 36
        self.pub.publish(out)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """エントリーポイント."""
    rclpy.init(args=args)
    node = FakeAmclPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
