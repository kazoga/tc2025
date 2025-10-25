"""画像変換とリサイズのユーティリティ。"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from PIL import Image
from sensor_msgs.msg import Image as ImageMsg


def convert_image_message(msg: ImageMsg) -> Optional[Image.Image]:
    """sensor_msgs/Image を PIL.Image に変換する。"""

    if msg.height == 0 or msg.width == 0:
        return None

    encoding = msg.encoding.lower()
    expected_size = msg.height * msg.width
    if encoding in {"rgb8", "bgr8"}:
        channels = 3
    elif encoding in {"rgba8", "bgra8"}:
        channels = 4
    elif encoding == "mono8":
        channels = 1
    else:
        return None

    expected_size *= channels
    if len(msg.data) < expected_size:
        return None

    array = np.frombuffer(msg.data, dtype=np.uint8)
    array = array[:expected_size]
    array = array.reshape((msg.height, msg.width, channels))

    if encoding == "bgr8":
        array = array[:, :, ::-1]
        mode = "RGB"
    elif encoding == "bgra8":
        array = array[:, :, [2, 1, 0, 3]]
        mode = "RGBA"
    elif encoding == "rgb8":
        mode = "RGB"
    elif encoding == "rgba8":
        mode = "RGBA"
    elif encoding == "mono8":
        mode = "L"
    else:
        return None

    return Image.fromarray(array, mode=mode)


def resize_with_letter_box(
    image: Image.Image,
    target_size: Tuple[int, int],
    background_color: Tuple[int, int, int] = (24, 24, 24),
) -> Image.Image:
    """アスペクト比を保ちながら余白を背景色で塗った画像を返す。"""

    target_width, target_height = target_size
    if target_width <= 0 or target_height <= 0:
        raise ValueError("target_size must be positive")

    scale = min(target_width / image.width, target_height / image.height)
    new_width = max(1, int(image.width * scale))
    new_height = max(1, int(image.height * scale))
    resized = image.resize((new_width, new_height), Image.BILINEAR)

    canvas = Image.new(image.mode, (target_width, target_height), color=background_color)
    offset_x = (target_width - new_width) // 2
    offset_y = (target_height - new_height) // 2
    canvas.paste(resized, (offset_x, offset_y))
    return canvas


def create_placeholder_image(
    size: Tuple[int, int],
    text: str,
    background_color: Tuple[int, int, int] = (24, 24, 24),
    foreground_color: Tuple[int, int, int] = (255, 255, 255),
) -> Image.Image:
    """テキストを中央に描画したプレースホルダ画像を生成する。"""

    image = Image.new('RGB', size, color=background_color)
    try:
        from PIL import ImageDraw, ImageFont
    except ImportError:  # pragma: no cover - Pillow 以外の環境では文字なしで返却
        return image

    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.load_default()
    except Exception:  # pragma: no cover - フォント取得失敗時は描画なし
        return image

    if hasattr(draw, 'textbbox'):
        left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
        text_width = right - left
        text_height = bottom - top
    else:  # pragma: no cover - 古い Pillow のみ
        text_width, text_height = draw.textsize(text, font=font)

    pos_x = max(0, (size[0] - text_width) // 2)
    pos_y = max(0, (size[1] - text_height) // 2)
    draw.text((pos_x, pos_y), text, fill=foreground_color, font=font)
    return image
