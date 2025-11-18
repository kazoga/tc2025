#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # パラメータ宣言
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('detection_interval', 1.0)  # 1秒に1回

        # パラメータ取得
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.detection_interval = self.get_parameter('detection_interval').get_parameter_value().double_value

        # モデルパスの設定
        if model_path == '':
            # デフォルトでmodelsディレクトリのyolo11n.ptを使用
            try:
                package_share_directory = get_package_share_directory('yolo_detector')
                model_path = os.path.join(package_share_directory, 'models', 'yolo11n.pt')
            except:
                # パッケージが見つからない場合は相対パスで探す
                current_dir = os.path.dirname(os.path.abspath(__file__))
                model_path = os.path.join(os.path.dirname(current_dir), 'models', 'yolo11n.pt')

        self.get_logger().info(f'Loading YOLO model from: {model_path}')

        # YOLOモデルの読み込み (CPU使用)
        try:
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
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

        self.get_logger().info(f'YoloDetectorNode started. Subscribing to {image_topic}')
        self.get_logger().info(f'Detection interval: {self.detection_interval} seconds')

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
        """YOLO検出処理を実行"""
        self.image_lock = True

        try:
            # CPUで推論実行
            results = self.model(self.latest_image, device='cpu', verbose=False)

            # 結果を表示
            self.print_detection_results(results)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
        finally:
            self.image_lock = False

    def print_detection_results(self, results):
        """検出結果をターミナルに表示"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Detection Results:')

        for result in results:
            boxes = result.boxes

            if len(boxes) == 0:
                self.get_logger().info('No objects detected')
            else:
                self.get_logger().info(f'Detected {len(boxes)} object(s):')

                for i, box in enumerate(boxes):
                    # クラスID、クラス名、信頼度を取得
                    cls_id = int(box.cls[0])
                    class_name = result.names[cls_id]
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
        node = YoloDetectorNode()
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
