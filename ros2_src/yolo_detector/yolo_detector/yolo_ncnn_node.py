#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory


class YoloNCNNDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_ncnn_detector_node')

        # パラメータ宣言
        self.declare_parameter('model_path', '')  # NCNNモデルのディレクトリパス
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('detection_interval', 1.0)  # 1秒に1回
        self.declare_parameter('confidence_threshold', 0.5)  # 信頼度の閾値
        self.declare_parameter('class_names', ['item'])  # クラス名のリスト

        # パラメータ取得
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.detection_interval = self.get_parameter('detection_interval').get_parameter_value().double_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.class_names = self.get_parameter('class_names').get_parameter_value().string_array_value

        # モデルパスの設定
        if model_path == '':
            self.get_logger().error('model_path parameter is required for NCNN model!')
            raise ValueError('model_path not specified')

        self.get_logger().info(f'Loading NCNN model from: {model_path}')

        # NCNN推論のセットアップ
        try:
            from ultralytics import YOLO
            # UltralyticsのNCNNサポートを使用
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info('NCNN model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load NCNN model: {e}')
            raise

        # CvBridgeの初期化
        self.bridge = CvBridge()

        # 最新の画像を保持
        self.latest_image = None
        self.image_lock = False

        # 画像サブスクライバー
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # 最後に処理した時刻
        self.last_detection_time = time.time()

        # タイマー (100ms毎にチェック)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'YoloNCNNDetectorNode started. Subscribing to {image_topic}')
        self.get_logger().info(f'Detection interval: {self.detection_interval} seconds')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Class names: {self.class_names}')

    def image_callback(self, msg):
        """画像トピックのコールバック - 最新画像を保持"""
        if not self.image_lock:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Failed to convert image: {e}')

    def timer_callback(self):
        """タイマーコールバック - 一定間隔で検出処理を実行"""
        current_time = time.time()

        # 指定間隔が経過したかチェック
        if current_time - self.last_detection_time >= self.detection_interval:
            if self.latest_image is not None:
                self.run_detection()
                self.last_detection_time = current_time

    def run_detection(self):
        """NCNN検出処理を実行"""
        self.image_lock = True

        try:
            # 処理時間を計測
            start_time = time.time()

            # NCNN推論実行
            results = self.model(
                self.latest_image,
                verbose=False,
                conf=self.confidence_threshold
            )

            # 処理時間を計算
            inference_time = time.time() - start_time

            # 結果を表示
            self.print_detection_results(results, inference_time)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
        finally:
            self.image_lock = False

    def print_detection_results(self, results, inference_time):
        """検出結果をターミナルに表示"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Detection Results (Inference time: {inference_time:.3f}s = {1/inference_time:.1f}fps):')

        for result in results:
            boxes = result.boxes

            if len(boxes) == 0:
                self.get_logger().info('No objects detected')
            else:
                self.get_logger().info(f'Detected {len(boxes)} object(s):')

                for i, box in enumerate(boxes):
                    # クラスID、クラス名、信頼度を取得
                    cls_id = int(box.cls[0])

                    # クラス名を取得（result.namesが利用可能な場合はそれを使用）
                    if hasattr(result, 'names') and result.names:
                        class_name = result.names[cls_id]
                    elif cls_id < len(self.class_names):
                        class_name = self.class_names[cls_id]
                    else:
                        class_name = f"class_{cls_id}"

                    confidence = float(box.conf[0])

                    # バウンディングボックスの座標
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                    self.get_logger().info(
                        f'  [{i+1}] {class_name} (ID:{cls_id}): {confidence:.2f} '
                        f'(x1:{int(x1)}, y1:{int(y1)}, x2:{int(x2)}, y2:{int(y2)})'
                    )

        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YoloNCNNDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
