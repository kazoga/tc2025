#!/usr/bin/env python3

import os
import time
from typing import Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovariance
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


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

        # 最新の画像とヘッダーを保持
        self.latest_image = None
        self.latest_header = None
        self.image_lock = False

        # 画像サブスクライバー
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # 検出結果のパブリッシャー
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10,
        )

        # 最後に処理した時刻
        self.last_detection_time = time.time()

        # タイマー (100ms毎にチェック)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'YoloNCNNDetectorNode started. Subscribing to {image_topic}')
        self.get_logger().info(f'Detection interval: {self.detection_interval} seconds')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Class names: {self.class_names}')

    def image_callback(self, msg: Image) -> None:
        """画像トピックのコールバックで最新画像とヘッダーを保持する."""

        if not self.image_lock:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8'
                )
                self.latest_header = msg.header
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f'Failed to convert image: {exc}')

    def timer_callback(self) -> None:
        """タイマーコールバックで一定間隔に検出処理を行う."""

        current_time = time.time()

        if current_time - self.last_detection_time >= self.detection_interval:
            if self.latest_image is not None and self.latest_header is not None:
                self.run_detection()
                self.last_detection_time = current_time

    def run_detection(self) -> None:
        """NCNN推論を実行し、検出結果を生成する."""

        self.image_lock = True

        try:
            start_time = time.time()

            results = self.model(
                self.latest_image,
                verbose=False,
                conf=self.confidence_threshold,
            )

            inference_time = time.time() - start_time
            annotated_image, detection_array = self._create_outputs(
                results, self.latest_header
            )
            self._publish_outputs(annotated_image, detection_array)
            self.print_detection_results(results, inference_time)

        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Detection error: {exc}')
        finally:
            self.image_lock = False

    def _create_outputs(
        self,
        results,
        header,
    ) -> Tuple[np.ndarray, Detection2DArray]:
        """検出結果から描画画像とDetection2DArrayを生成する."""

        annotated_image = self.latest_image.copy()
        detection_array = Detection2DArray()
        detection_array.header = header

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self._resolve_class_name(result, cls_id)
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                detection = self._build_detection(
                    header, cls_id, class_name, confidence, x1, y1, x2, y2
                )
                detection_array.detections.append(detection)
                self._draw_detection(
                    annotated_image, class_name, confidence, int(x1), int(y1), int(x2), int(y2)
                )

        return annotated_image, detection_array

    def _build_detection(
        self,
        header,
        class_id: int,
        class_name: str,
        confidence: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> Detection2D:
        """Detection2Dメッセージを組み立てる."""

        detection = Detection2D()
        detection.header = header

        bbox = BoundingBox2D()
        bbox.center.position.x = float((x1 + x2) / 2)
        bbox.center.position.y = float((y1 + y2) / 2)
        bbox.size_x = float(x2 - x1)
        bbox.size_y = float(y2 - y1)
        detection.bbox = bbox

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = str(class_id)
        hypothesis.score = float(confidence)
        hypothesis.pose = PoseWithCovariance()
        detection.results.append(hypothesis)

        return detection

    def _draw_detection(
        self,
        annotated_image: np.ndarray,
        class_name: str,
        confidence: float,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> None:
        """画像上にバウンディングボックスとラベルを描画する."""

        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f'{class_name}: {confidence:.2f}'
        cv2.putText(
            annotated_image,
            label,
            (x1, max(y1 - 10, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def _resolve_class_name(self, result, cls_id: int) -> str:
        """Ultralyticsの結果からクラス名を決定する."""

        if hasattr(result, 'names') and result.names:
            return result.names[cls_id]
        if cls_id < len(self.class_names):
            return self.class_names[cls_id]
        return f'class_{cls_id}'

    def _publish_outputs(
        self, annotated_image: np.ndarray, detection_array: Detection2DArray
    ) -> None:
        """検出結果と描画済み画像をパブリッシュする."""

        image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        image_msg.header = self.latest_header

        self.detections_pub.publish(detection_array)

        if not hasattr(self, 'image_pub'):
            self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
        self.image_pub.publish(image_msg)

    def print_detection_results(self, results, inference_time: float) -> None:
        """検出結果をターミナルに表示する."""

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            'Detection Results '
            f'(Inference time: {inference_time:.3f}s = {1 / inference_time:.1f}fps):'
        )

        for result in results:
            boxes = result.boxes

            if len(boxes) == 0:
                self.get_logger().info('No objects detected')
                continue

            self.get_logger().info(f'Detected {len(boxes)} object(s):')

            for index, box in enumerate(boxes):
                cls_id = int(box.cls[0])
                class_name = self._resolve_class_name(result, cls_id)
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                self.get_logger().info(
                    f'  [{index + 1}] {class_name} (ID:{cls_id}): {confidence:.2f} '
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
