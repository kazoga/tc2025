#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class ImagePublisherFromFile:
    def __init__(self):
        rospy.init_node('image_publisher_from_file', anonymous=True)

        # パラメータ取得
        self.image_path = rospy.get_param('~image_path', '/home/nkb/catkin_ws/src/tc2025/config/road-closed_5m.jpg')
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # Hz
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.frame_id = rospy.get_param('~frame_id', 'usb_cam')

        # CvBridge初期化
        self.bridge = CvBridge()

        # パブリッシャー
        self.image_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)

        # 画像読み込み
        if not os.path.exists(self.image_path):
            rospy.logerr(f"Image file not found: {self.image_path}")
            rospy.signal_shutdown("Image file not found")
            return

        self.image = cv2.imread(self.image_path)
        if self.image is None:
            rospy.logerr(f"Failed to load image: {self.image_path}")
            rospy.signal_shutdown("Failed to load image")
            return

        # リサイズ
        self.image = cv2.resize(self.image, (self.width, self.height))
        rospy.loginfo(f"Loaded and resized image: {self.image_path} to {self.width}x{self.height}")

        # タイマー設定
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_image)

        rospy.loginfo(f"Image Publisher Node Started - Publishing at {self.publish_rate} Hz")

    def publish_image(self, event):
        """画像をROSトピックとして配信"""
        try:
            # OpenCV画像をROS Imageメッセージに変換
            ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = self.frame_id

            # 配信
            self.image_pub.publish(ros_image)

        except Exception as e:
            rospy.logerr(f"Failed to publish image: {e}")

    def run(self):
        """ノード実行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = ImagePublisherFromFile()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
