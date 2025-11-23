#!/usr/bin/env python3
"""
YOLOモデルをNCNN形式に変換するスクリプト

使用方法:
    python3 convert_to_ncnn.py /path/to/best.pt

これにより、同じディレクトリに best_ncnn_model/ フォルダが作成されます。
"""

import sys
import os
from ultralytics import YOLO


def convert_to_ncnn(model_path):
    """YOLOモデルをNCNN形式に変換"""

    if not os.path.exists(model_path):
        print(f"Error: Model file not found: {model_path}")
        return False

    print(f"Loading YOLO model from: {model_path}")
    model = YOLO(model_path)

    # NCNN形式にエクスポート
    print("Converting to NCNN format...")
    print("This may take a few minutes...")

    try:
        model.export(format='ncnn', imgsz=320)  # 320x320サイズで変換
        print("\nConversion successful!")

        # 出力先を表示
        model_dir = os.path.dirname(model_path)
        model_name = os.path.splitext(os.path.basename(model_path))[0]
        ncnn_dir = os.path.join(model_dir, f"{model_name}_ncnn_model")

        print(f"\nNCNN model saved to: {ncnn_dir}")
        print(f"Files created:")
        print(f"  - {model_name}.param")
        print(f"  - {model_name}.bin")

        return True

    except Exception as e:
        print(f"\nError during conversion: {e}")
        print("\nMake sure you have the required dependencies:")
        print("  pip install ultralytics ncnn")
        return False


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 convert_to_ncnn.py <model_path>")
        print("Example: python3 convert_to_ncnn.py ../models/best.pt")
        sys.exit(1)

    model_path = sys.argv[1]
    success = convert_to_ncnn(model_path)

    sys.exit(0 if success else 1)
