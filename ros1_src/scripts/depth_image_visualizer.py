#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import threading

class DepthImageVisualizer:
    def __init__(self):
        rospy.init_node('depth_image_visualizer', anonymous=True)

        # パラメータ設定
        self.x_min = 0.0    # 前方向の最小距離 [m]
        self.x_max = 5.0    # 前方向の最大距離 [m]
        self.y_min = -5.0   # 横方向の最小距離 [m] (左)
        self.y_max = 5.0    # 横方向の最大距離 [m] (右)
        self.z_min = -1.0   # 下方向 [m]
        self.z_max = 3.0    # 上方向 [m]

        # 画像解像度
        self.image_width = 500   # y軸方向 (横) のピクセル数
        self.image_height = 500  # x軸方向 (前方) のピクセル数

        # データ保持用
        self.latest_pointcloud = None
        self.lock = threading.Lock()

        # サブスクライバ
        self.pc_sub = rospy.Subscriber('/mid360/livox/lidar', PointCloud2, self.pointcloud_callback)

        # 表示周期 1Hz
        self.timer = rospy.Timer(rospy.Duration(1.0), self.visualize_callback)

        rospy.loginfo("Depth Image Visualizer Node Started")
        rospy.loginfo(f"ROI: x[{self.x_min}, {self.x_max}], y[{self.y_min}, {self.y_max}], z[{self.z_min}, {self.z_max}]")

    def pointcloud_callback(self, msg):
        """PointCloud2メッセージを受信"""
        with self.lock:
            self.latest_pointcloud = msg

    def visualize_callback(self, event):
        """1Hzでデプス画像を生成・表示"""
        with self.lock:
            if self.latest_pointcloud is None:
                rospy.logwarn("No pointcloud data received yet")
                return
            pc_msg = self.latest_pointcloud

        # PointCloudをnumpy配列に変換
        points = []
        for p in pc2.read_points(pc_msg, skip_nans=True, field_names=("x", "y", "z")):
            x, y, z = p
            # ROI内の点のみ抽出
            if (self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max and
                self.z_min <= z <= self.z_max):
                points.append([x, y, z])

        if len(points) == 0:
            rospy.logwarn("No points in ROI")
            return

        points = np.array(points)
        rospy.loginfo(f"Points in ROI: {len(points)}")

        # デプス画像を生成 (x-y平面に投影、距離は3D距離)
        depth_image = self.create_depth_image(points)

        # 可視化
        self.display_image(depth_image)

    def create_depth_image(self, points):
        """
        ポイントクラウドからカメラ視点のデプス画像を生成
        y: 横方向 (-5 to 5m) -> 画像の横軸 (左が左、右が右)
        z: 上下方向 (-1 to 3m) -> 画像の縦軸 (上が上、下が下)
        深度: x方向の距離（前方距離）をグレースケールで表現
        """
        # 初期化 (無限大で初期化)
        depth_map = np.full((self.image_height, self.image_width), np.inf, dtype=np.float32)

        for point in points:
            x, y, z = point

            # ピクセル座標に変換（カメラビュー）
            # y軸: -5m(左)が画像左、5m(右)が画像右
            py = int((y - self.y_min) / (self.y_max - self.y_min) * (self.image_width - 1))
            # z軸: 3m(上)が画像上、-1m(下)が画像下
            px = int((self.z_max - z) / (self.z_max - self.z_min) * (self.image_height - 1))

            # 範囲外チェック
            if 0 <= px < self.image_height and 0 <= py < self.image_width:
                # 前方距離xを深度として記録（最も近い点を採用）
                if x < depth_map[px, py]:
                    depth_map[px, py] = x

        # 無限大を最大距離に置き換え
        depth_map[depth_map == np.inf] = self.x_max

        # グレースケール画像に変換 (0-255)
        # 近い点を白く、遠い点を黒く
        depth_normalized = (self.x_max - depth_map) / self.x_max
        depth_image = (depth_normalized * 255).astype(np.uint8)

        return depth_image

    def display_image(self, depth_image):
        """デプス画像を表示"""
        # カラーマップを適用して見やすくする
        depth_colored = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)

        # テキスト情報を追加（カメラビュー）
        cv2.putText(depth_colored, "Camera View (Depth 0-5m)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(depth_colored, f"Left {self.y_min}m", (10, self.image_height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(depth_colored, f"Right {self.y_max}m", (self.image_width - 100, self.image_height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(depth_colored, f"Top {self.z_max}m", (self.image_width // 2 - 50, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(depth_colored, f"Bottom {self.z_min}m", (self.image_width // 2 - 70, self.image_height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 表示
        cv2.imshow('Depth Image (Camera View)', depth_colored)
        cv2.waitKey(1)

    def run(self):
        """ノード実行"""
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        visualizer = DepthImageVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
