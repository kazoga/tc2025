# YOLO Detector for ROS2

ROS2でUSBカメラの画像をYOLOモデルで物体検出するパッケージです。

## 機能

- `/usb_cam/image_raw` トピックから画像をサブスクライブ
- YOLOモデル（PyTorch / NCNN）を使用してCPUで物体検出
- **NCNN版は通常のPyTorch版より高速（推奨）**
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

### 方法1: NCNN版を使用（高速・推奨）

#### 1. モデルをNCNN形式に変換

```bash
cd /home/nkb/ros2_ws/src/yolo_detector
python3 scripts/convert_to_ncnn.py models/best.pt
```

これにより `models/best_ncnn_model/` ディレクトリが作成されます。

#### 2. NCNN版ノードを起動

```bash
ros2 run yolo_detector yolo_ncnn_node --ros-args \
  -p model_path:=/home/nkb/ros2_ws/src/yolo_detector/models/best_ncnn_model
```

### 方法2: PyTorch版を使用（従来版）

#### カスタムモデルで起動

```bash
ros2 run yolo_detector yolo_node --ros-args \
  -p model_path:=/home/nkb/ros2_ws/src/yolo_detector/models/best.pt \
  -p image_size:=320
```

#### パラメータ付きで起動

```bash
ros2 run yolo_detector yolo_node --ros-args \
  -p image_topic:=/usb_cam/image_raw \
  -p detection_interval:=1.0 \
  -p model_path:=/home/nkb/ros2_ws/src/yolo_detector/models/best.pt \
  -p image_size:=256 \
  -p confidence_threshold:=0.5
```

### パラメータ

#### 共通パラメータ（両方のノードで使用可能）

- `image_topic` (string, default: "/usb_cam/image_raw")
  - サブスクライブする画像トピック名

- `detection_interval` (double, default: 1.0)
  - 検出処理の実行間隔（秒）

- `confidence_threshold` (double, default: 0.5)
  - 検出の信頼度閾値

- `model_path` (string, default: "")
  - モデルファイル/ディレクトリのパス
  - NCNN版: モデルディレクトリ（例: `/path/to/best_ncnn_model`）
  - PyTorch版: .ptファイルのパス（例: `/path/to/best.pt`）

#### PyTorch版のみのパラメータ

- `image_size` (int, default: 320)
  - 推論時の画像サイズ（小さいほど高速、大きいほど精度向上）
  - 推奨値: 256（高速）、320（バランス）、416（高精度）

#### NCNN版のみのパラメータ

- `class_names` (string array, default: ["item"])
  - クラス名のリスト

### 方法3: camera_simulatorノードで静止画を配信

検出用の入力が無い環境向けに、任意の静止画を`/usb_cam/image_raw`として配信する
`camera_simulator_node`を追加しました。

```bash
ros2 run yolo_detector camera_simulator_node --ros-args \
  -p frame_image_path:=/path/to/image.jpg \
  -p frame_width:=640 \
  -p frame_height:=480 \
  -p frame_ratio:=10.0
```

#### camera_simulatorノードのパラメータ

- `frame_image_path` (string, default: "")
  - 配信する静止画のパス。`cv2.imread()`で読み込める画像を指定。
- `frame_width` (int, default: -1)
  - リサイズ後の横幅。負値の場合はリサイズ無し。片方のみ指定時はアスペクト比維持で拡縮。
- `frame_height` (int, default: -1)
  - リサイズ後の高さ。負値の場合はリサイズ無し。
- `frame_ratio` (double, default: 10.0)
  - 画像をpublishするレート(Hz)。

## 出力例

```
[INFO] [yolo_ncnn_detector_node]: ============================================================
[INFO] [yolo_ncnn_detector_node]: Detection Results (Inference time: 0.085s = 11.8fps):
[INFO] [yolo_ncnn_detector_node]: Detected 2 object(s):
[INFO] [yolo_ncnn_detector_node]:   [1] item (ID:0): 0.81 (x1:535, y1:222, x2:579, y2:316)
[INFO] [yolo_ncnn_detector_node]:   [2] item (ID:0): 0.77 (x1:71, y1:220, x2:116, y2:316)
[INFO] [yolo_ncnn_detector_node]: ============================================================
```

**NCNN版は通常のPyTorch版と比較して3〜5倍高速です！**

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
