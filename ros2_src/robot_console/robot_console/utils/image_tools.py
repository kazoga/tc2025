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
    background_color: Tuple[int, int, int] = (31, 31, 31),
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
    placeholder_text = getattr(image, "info", {}).get("placeholder_tag")
    if placeholder_text:
        canvas.info["placeholder_tag"] = placeholder_text
    return canvas


def create_placeholder_image(
    size: Tuple[int, int],
    text: str,
    background_color: Tuple[int, int, int] = (31, 31, 31),
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
        base_font = ImageFont.load_default()
    except Exception:  # pragma: no cover - フォント取得失敗時は描画なし
        return image

    # 既定ビットマップフォントを拡大してプレースホルダ文字を見やすくする。
    if hasattr(draw, 'textbbox'):
        try:
            left, top, right, bottom = draw.textbbox((0, 0), text, font=base_font)
        except ValueError:
            width, height = draw.textsize(text, font=base_font)
            left, top, right, bottom = 0, 0, width, height
    else:  # pragma: no cover - 古い Pillow のみ
        width, height = draw.textsize(text, font=base_font)
        left, top, right, bottom = 0, 0, width, height

    text_width = max(right - left, 1)
    text_height = max(bottom - top, 1)
    mask = Image.new('L', (text_width, text_height), 0)
    mask_draw = ImageDraw.Draw(mask)
    mask_draw.text((-left, -top), text, fill=255, font=base_font)
    scale = 5
    scaled_mask = mask.resize((text_width * scale, text_height * scale), Image.NEAREST)

    pos_x = max(0, (size[0] - scaled_mask.width) // 2)
    pos_y = max(0, (size[1] - scaled_mask.height) // 2)
    image.paste(foreground_color, (pos_x, pos_y, pos_x + scaled_mask.width, pos_y + scaled_mask.height), scaled_mask)
    image.info['placeholder_tag'] = text
    return image
