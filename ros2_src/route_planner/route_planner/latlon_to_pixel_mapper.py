#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地理座標（緯度・経度, WGS84）をPGWに基づく画像座標へマッピングするスクリプト。
PGWの座標単位を自動的に判定し、必要に応じて投影変換（Web Mercator）を行う。

使用例:
    python latlon_to_pixel_mapper.py map.png map.pgw 36.081263 140.079143 36.081500 140.080000

依存:
    Pillow (pip install pillow)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

from PIL import Image, ImageDraw


# ----------------------------------------------------------------------
# 内部ユーティリティ
# ----------------------------------------------------------------------

def _read_pgw(worldfile_path: str) -> Tuple[float, float, float, float, float, float]:
    """PGWファイルを読み込み、6つのパラメータ(A,D,B,E,C,F)を返す。"""
    try:
        with open(worldfile_path, "r", encoding="utf-8") as f:
            vals = [float(line.strip()) for line in f.readlines()]
        if len(vals) != 6:
            raise ValueError("PGWファイルの行数が不正です（6行必要）。")
        return tuple(vals)
    except Exception as e:
        raise RuntimeError(f"PGWファイル読み込みエラー: {e}") from e


def _is_projected_pgw(C: float, F: float, A: float, E: float) -> bool:
    """PGWの値から座標単位を推定する。
    True: 投影座標系（メートル単位） / False: 地理座標系（度単位）
    """
    # 絶対値が1e6オーダーならWebMercator/EPSG:3857相当とみなす
    return (abs(C) > 1e6 or abs(F) > 1e6) and (abs(A) < 1.0 and abs(E) < 1.0)


# WGS84⇄WebMercator 相互変換
RADIUS_EARTH_M = 6378137.0


def latlon_to_webmercator(lat: float, lon: float) -> Tuple[float, float]:
    """WGS84(度)→Web Mercator(メートル)"""
    x = math.radians(lon) * RADIUS_EARTH_M
    y = RADIUS_EARTH_M * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
    return x, y


def webmercator_to_latlon(x: float, y: float) -> Tuple[float, float]:
    """Web Mercator(メートル)→WGS84(度)"""
    lon = math.degrees(x / RADIUS_EARTH_M)
    lat = math.degrees(2 * math.atan(math.exp(y / RADIUS_EARTH_M)) - math.pi / 2)
    return lat, lon


# ----------------------------------------------------------------------
# 主要関数
# ----------------------------------------------------------------------

def latlon_to_pixel(
    image_path: str,
    worldfile_path: str,
    latlon_list: Sequence[Tuple[float, float]],
    *,
    verbose: bool = False,
) -> List[Tuple[int | None, int | None]]:
    """
    緯度経度を画像ピクセル座標に変換する。

    Args:
        image_path: PNG画像のパス
        worldfile_path: 対応するPGWファイルのパス
        latlon_list: (lat, lon)のタプルリスト

    Returns:
        [(x, y), ...]（画像外は(None, None)）
    """
    A, D, B, E, C, F = _read_pgw(worldfile_path)

    projected = _is_projected_pgw(C, F, A, E)

    with Image.open(image_path) as img:
        width, height = img.size

    if verbose:
        print("=== PGW parameters ===")
        print(f"A={A}, D={D}, B={B}, E={E}, C={C}, F={F}")
        print(f"Image size: width={width}, height={height}")
        print(f"PGW interpreted as {'Projected (m)' if projected else 'Geographic (deg)'}\n")

    det = A * E - B * D
    if abs(det) < 1e-12:
        raise ValueError("PGWアフィン行列が特異です。")
    invA = E / det
    invB = -B / det
    invD = -D / det
    invE = A / det

    results: List[Tuple[int | None, int | None]] = []
    for lat, lon in latlon_list:
        # --- 変換手順 ---
        if projected:
            mapx, mapy = latlon_to_webmercator(lat, lon)
        else:
            mapx, mapy = lon, lat

        x_img = invA * (mapx - C) + invB * (mapy - F)
        y_img = invD * (mapx - C) + invE * (mapy - F)

        x_pix = round(x_img)
        y_pix = round(y_img)

        inside = 0 <= x_pix < width and 0 <= y_pix < height

        if verbose:
            print(
                f"lat={lat:.6f}, lon={lon:.6f} -> "
                f"mapX={mapx:.3f}, mapY={mapy:.3f}, "
                f"x_img={x_img:.3f}, y_img={y_img:.3f}, "
                f"x_pix={x_pix}, y_pix={y_pix}, inside={inside}"
            )

        if inside:
            results.append((x_pix, y_pix))
        else:
            results.append((None, None))

    if verbose:
        print("")  # 改行を入れて見やすく
    return results

def _draw_points(image_path: str, points: List[Tuple[int | None, int | None]], output_path: str) -> None:
    """変換結果を画像上に描画して保存する。"""
    with Image.open(image_path).convert("RGB") as img:
        draw = ImageDraw.Draw(img)
        for pt in points:
            if pt == (None, None):
                continue
            x, y = pt
            r = 5
            draw.ellipse((x - r, y - r, x + r, y + r), outline="red", fill="red")
        img.save(output_path)
        print(f"出力画像を保存しました: {output_path}")


def main() -> None:
    """コマンドライン実行エントリポイント。"""
    if len(sys.argv) < 5 or (len(sys.argv) - 3) % 2 != 0:
        print(
            "使用方法: python latlon_to_pixel_mapper.py <image.png> <worldfile.pgw> <lat1> <lon1> [<lat2> <lon2> ...]"
        )
        sys.exit(1)

    image_path = sys.argv[1]
    worldfile_path = sys.argv[2]

    # (lat, lon) のペアを収集
    args = sys.argv[3:]
    latlon_pairs: List[Tuple[float, float]] = []
    for i in range(0, len(args), 2):
        try:
            lat = float(args[i])
            lon = float(args[i + 1])
            latlon_pairs.append((lat, lon))
        except ValueError:
            print(f"緯度経度の形式が不正です: {args[i]}, {args[i+1]}")
            sys.exit(1)

    # 変換実行
    try:
        pixels = latlon_to_pixel(image_path, worldfile_path, latlon_pairs, verbose=True)
    except Exception as e:
        print(f"エラー: {e}")
        sys.exit(1)

    # 結果出力
    for (lat, lon), (x, y) in zip(latlon_pairs, pixels):
        print(f"({lat:.6f}, {lon:.6f}) -> ({x}, {y})")

    # 出力画像の保存
    out_path = Path(image_path).with_name(Path(image_path).stem + "_mapped.png")
    _draw_points(image_path, pixels, str(out_path))


if __name__ == "__main__":
    main()

