# YOLO Detector for ROS2

ROS2でUSBカメラの画像をYOLOモデルで物体検出するパッケージです。

## 機能

- `/usb_cam/image_raw` トピックから画像をサブスクライブ
- YOLOモデルを使用してCPUで物体検出
- 1秒に1回の間隔で検出処理を実行
- 検出結果をターミナルに表示

## 必要な依存関係

### ROS2パッケージ
- rclpy
- sensor_msgs
- cv_bridge

### Pythonパッケージ
```bash
pip install ultralytics opencv-python
```

## インストール

1. ワークスペースのsrcディレクトリに配置されていることを確認

2. 必要なPythonパッケージをインストール:
```bash
pip install ultralytics opencv-python
```

3. YOLOモデルファイルを`models/`ディレクトリに配置:
```bash
# モデルファイル（例: best.pt, yolo11n.pt など）を配置
cp /path/to/your/model.pt /home/nkb/ros2_ws/src/yolo_detector/models/
```

4. ワークスペースをビルド:
```bash
cd /home/nkb/ros2_ws
colcon build --packages-select yolo_detector
source install/setup.bash
```

## 使用方法

### カスタムモデルで起動（推奨）

モデルファイルのパスを指定して起動:
```bash
ros2 run yolo_detector yolo_node --ros-args \
  -p model_path:=/home/nkb/ros2_ws/src/yolo_detector/models/best.pt
```

### デフォルトモデルで起動

`models/yolo11n.pt`が存在する場合はパラメータなしで起動可能:
```bash
ros2 run yolo_detector yolo_node
```

### パラメータ付きで起動

```bash
ros2 run yolo_detector yolo_node --ros-args \
  -p image_topic:=/usb_cam/image_raw \
  -p detection_interval:=1.0 \
  -p model_path:=/home/nkb/ros2_ws/src/yolo_detector/models/best.pt
```

### パラメータ

- `image_topic` (string, default: "/usb_cam/image_raw")
  - サブスクライブする画像トピック名

- `detection_interval` (double, default: 1.0)
  - 検出処理の実行間隔（秒）

- `model_path` (string, default: "")
  - YOLOモデルファイルのパス（空の場合はデフォルトモデルを使用）

## 出力例

```
[INFO] [yolo_detector_node]: ============================================================
[INFO] [yolo_detector_node]: Detection Results:
[INFO] [yolo_detector_node]: Detected 3 object(s):
[INFO] [yolo_detector_node]:   [1] person: 0.89 (x1:120, y1:80, x2:340, y2:480)
[INFO] [yolo_detector_node]:   [2] chair: 0.76 (x1:450, y1:200, x2:600, y2:450)
[INFO] [yolo_detector_node]:   [3] bottle: 0.65 (x1:200, y1:150, x2:250, y2:280)
[INFO] [yolo_detector_node]: ============================================================
```

## ディレクトリ構造

```
yolo_detector/
├── yolo_detector/
│   ├── __init__.py
│   └── yolo_node.py          # メインノード
├── models/
│   ├── README.md
│   └── best.pt              # YOLOモデルファイル（配置済み）
├── resource/
│   └── yolo_detector
├── test/
├── package.xml
├── setup.py
└── README.md
```

## トラブルシューティング

### モデルが見つからないエラー
モデルファイルのパスを確認し、正しいパスを`model_path`パラメータで指定してください:
```bash
ls /home/nkb/ros2_ws/src/yolo_detector/models/
```

### ultralyticsがインストールされていない
```bash
pip install ultralytics
```

### cv_bridgeのエラー
```bash
sudo apt install ros-<your-distro>-cv-bridge
```

## ライセンス

TODO: ライセンスを指定してください
