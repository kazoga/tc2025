# YOLO Models Directory

このディレクトリにYOLO11nモデルファイルを配置してください。

## モデルのダウンロード方法

### 方法1: Ultralyticsのコマンドでダウンロード

```bash
cd /home/nkb/ros2_ws/src/yolo_detector/models
pip install ultralytics
yolo export model=yolo11n.pt format=torchscript
```

または、Pythonで自動ダウンロード:

```python
from ultralytics import YOLO
import os

# このディレクトリに移動
os.chdir('/home/nkb/ros2_ws/src/yolo_detector/models')

# モデルを読み込むと自動的にダウンロードされる
model = YOLO('yolo11n.pt')
print("モデルがダウンロードされました")
```

### 方法2: 直接ダウンロード

公式リポジトリからダウンロード:
```bash
cd /home/nkb/ros2_ws/src/yolo_detector/models
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
```

## 必要なモデルファイル

- `yolo11n.pt` - YOLO11 Nanoモデル（デフォルト）

## カスタムモデルの使用

カスタムモデルを使用する場合は、起動時にパラメータで指定してください:

```bash
ros2 run yolo_detector yolo_node --ros-args -p model_path:=/path/to/your/model.pt
```

## 注意事項

- 初回実行時、モデルファイルが存在しない場合は自動的にダウンロードされる場合があります
- モデルファイルのサイズは約6MB（yolo11n.pt）です
- このディレクトリにはGitで追跡されないように.gitignoreに追加することを推奨します
