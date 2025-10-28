#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PGW（World File）を、与えられた画像サイズと地理範囲（WGS84）に合わせて補正するツール。

- 前提：PGW の単位は投影座標（地理院タイル想定：EPSG:3857, Web Mercator, m）
- やること：X/Y それぞれのスケール（A, E）を再計算し、左上ピクセル中心基準で C/F を再設定
- 回転項（B, D）が 0 の正射影 PGW を想定（0 でないときは警告し、その値は保持）

使い方例：
    python auto_correct_pgw.py image.png image.pgw \
        --lat-min 35.649077 --lat-max 35.650193 \
        --lon-min 139.502084 --lon-max 139.503457 \
        --out image_corr.pgw
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

from PIL import Image


RADIUS_EARTH_M = 6378137.0  # Web Mercator 半径 [m]


@dataclass
class Pgw:
    """PGW 6 パラメータの入れ物。"""
    A: float  # pixel size in the x-direction in map units/pixel
    D: float  # rotation term
    B: float  # rotation term
    E: float  # pixel size in the y-direction in map units, typically negative
    C: float  # x-coordinate of the center of the upper left pixel
    F: float  # y-coordinate of the center of the upper left pixel


def read_pgw(path: str | Path) -> Pgw:
    """PGW を読み込む。"""
    with open(path, "r", encoding="utf-8") as f:
        vals = [float(line.strip()) for line in f]
    if len(vals) != 6:
        raise ValueError("PGW の行数が 6 行ではありません。")
    return Pgw(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])


def write_pgw(path: str | Path, pgw: Pgw) -> None:
    """PGW を 6 行で書き出す。"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"{pgw.A:.15f}\n")
        f.write(f"{pgw.D:.0f}\n" if abs(pgw.D) < 1e-12 else f"{pgw.D:.15f}\n")
        f.write(f"{pgw.B:.0f}\n" if abs(pgw.B) < 1e-12 else f"{pgw.B:.15f}\n")
        f.write(f"{pgw.E:.15f}\n")
        f.write(f"{pgw.C:.15f}\n")
        f.write(f"{pgw.F:.15f}\n")


def latlon_to_webmercator(lat: float, lon: float) -> Tuple[float, float]:
    """WGS84（度）→ Web Mercator（m）へ変換。"""
    x = math.radians(lon) * RADIUS_EARTH_M
    y = RADIUS_EARTH_M * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
    return x, y


def inverse_affine_to_pixel(pgw: Pgw, mapx: float, mapy: float) -> Tuple[float, float]:
    """アフィン逆変換（ピクセル中心基準）。x_img,y_img は連続座標。"""
    det = pgw.A * pgw.E - pgw.B * pgw.D
    if abs(det) < 1e-18:
        raise ValueError("PGW のアフィン行列が特異です。")
    invA = pgw.E / det
    invB = -pgw.B / det
    invD = -pgw.D / det
    invE = pgw.A / det
    # 左上ピクセル中心が (C,F) なので 0.5 補正を差し引く
    x_img = invA * (mapx - pgw.C) + invB * (mapy - pgw.F) - 0.5
    y_img = invD * (mapx - pgw.C) + invE * (mapy - pgw.F) - 0.5
    return x_img, y_img


def correct_pgw(
    image_path: str | Path,
    pgw_in: Pgw,
    lat_min: float,
    lat_max: float,
    lon_min: float,
    lon_max: float,
) -> Tuple[Pgw, dict]:
    """範囲指定から補正済み PGW を計算する。

    回転 (B,D) は 0 を前提。0 でない場合は警告し、値はそのまま保持する。
    """
    width, height = Image.open(image_path).size

    # 四隅を Web Mercator(m) に変換
    x_left, _ = latlon_to_webmercator(lat=(lat_min + lat_max) / 2, lon=lon_min)
    x_right, _ = latlon_to_webmercator(lat=(lat_min + lat_max) / 2, lon=lon_max)
    _, y_top = latlon_to_webmercator(lat=lat_max, lon=(lon_min + lon_max) / 2)
    _, y_bottom = latlon_to_webmercator(lat=lat_min, lon=(lon_min + lon_max) / 2)

    # XY 独立スケール（ピクセル当たりの距離[m/px]）
    A_corr = (x_right - x_left) / float(width)
    E_corr = - (y_top - y_bottom) / float(height)  # 下向きが +y になるためマイナス

    # 左上ピクセル中心の世界座標（C, F）を再設定（0.5 px 内側）
    C_corr = x_left + A_corr * 0.5
    F_corr = y_top + E_corr * 0.5

    # 回転は元の値を保持（0 推奨）
    D_corr, B_corr = pgw_in.D, pgw_in.B
    if abs(D_corr) > 1e-12 or abs(B_corr) > 1e-12:
        print(
            "[WARN] このPGWは回転項(B,D)が 0 ではありません。スケール補正は実施しますが、"
            "厳密な合わせ込みには回転を含めた境界フィッティングが必要です。"
        )

    pgw_out = Pgw(A_corr, D_corr, B_corr, E_corr, C_corr, F_corr)

    # 検証用メタ
    meta = {
        "image_size": (width, height),
        "x_left": x_left,
        "x_right": x_right,
        "y_top": y_top,
        "y_bottom": y_bottom,
        "A_corr": A_corr,
        "E_corr": E_corr,
        "C_corr": C_corr,
        "F_corr": F_corr,
    }
    return pgw_out, meta


def main() -> None:
    parser = argparse.ArgumentParser(
        description="画像と指定緯度経度範囲に合わせて PGW を補正するツール（Web Mercator想定）。"
    )
    parser.add_argument("image", help="PNG 等の画像ファイルパス")
    parser.add_argument("pgw", help="補正前 PGW ファイルパス")
    parser.add_argument("--lat-min", type=float, required=True, help="範囲の南端緯度（度）")
    parser.add_argument("--lat-max", type=float, required=True, help="範囲の北端緯度（度）")
    parser.add_argument("--lon-min", type=float, required=True, help="範囲の西端経度（度）")
    parser.add_argument("--lon-max", type=float, required=True, help="範囲の東端経度（度）")
    parser.add_argument("--out", type=str, required=True, help="補正後 PGW の出力先パス")
    parser.add_argument("--dry-run", action="store_true", help="ファイルを書き出さず、計算結果のみ表示")
    args = parser.parse_args()

    pgw_in = read_pgw(args.pgw)
    pgw_out, meta = correct_pgw(
        args.image, pgw_in, args.lat_min, args.lat_max, args.lon_min, args.lon_max
    )

    # ログ出力（詳細）
    print("=== Input PGW ===")
    print(f"A={pgw_in.A}, D={pgw_in.D}, B={pgw_in.B}, E={pgw_in.E}, C={pgw_in.C}, F={pgw_in.F}")
    print(f"Image size: {meta['image_size'][0]} x {meta['image_size'][1]}")
    print("\n=== Desired geographic extent (WGS84) ===")
    print(f"lat: {args.lat_min} .. {args.lat_max}")
    print(f"lon: {args.lon_min} .. {args.lon_max}")
    print("\n=== Converted to Web Mercator (m) ===")
    print(
        f"x_left={meta['x_left']:.3f}, x_right={meta['x_right']:.3f}, "
        f"y_top={meta['y_top']:.3f}, y_bottom={meta['y_bottom']:.3f}"
    )
    print("\n=== Corrected PGW ===")
    print(
        f"A={meta['A_corr']:.12f}, D={pgw_out.D}, B={pgw_out.B}, "
        f"E={meta['E_corr']:.12f}, C={meta['C_corr']:.6f}, F={meta['F_corr']:.6f}"
    )

    # 検証：四隅（外縁）の内側 0.5px を含む 5 点を逆変換でチェック
    w, h = meta["image_size"]
    test_points = [
        ("UL(center)", 0.5, 0.5),
        ("UR(center)", w - 0.5, 0.5),
        ("LL(center)", 0.5, h - 0.5),
        ("LR(center)", w - 0.5, h - 0.5),
        ("Center", w / 2, h / 2),
    ]
    print("\n=== Back-check (pixel -> should match given extent) ===")
    for name, px, py in test_points:
        # ピクセル中心（px,py）の世界座標を正方向に生成（順変換）
        Xw = pgw_out.A * (px - 0.5) + pgw_out.B * (py - 0.5) + pgw_out.C
        Yw = pgw_out.D * (px - 0.5) + pgw_out.E * (py - 0.5) + pgw_out.F
        # 与えた地理範囲のどこに相当するか（相対位置）を確認
        rel_x = (Xw - meta["x_left"]) / (meta["x_right"] - meta["x_left"])
        rel_y = (meta["y_top"] - Yw) / (meta["y_top"] - meta["y_bottom"])
        print(
            f"{name:12s}: Xw={Xw:.3f}, Yw={Yw:.3f}, "
            f"rel_x={rel_x:.6f}, rel_y={rel_y:.6f}"
        )

    # 書き出し
    if args.dry_run:
        print("\n[dry-run] 補正 PGW は書き出していません。")
    else:
        out_path = Path(args.out)
        write_pgw(out_path, pgw_out)
        print(f"\nWrote corrected PGW: {out_path}")
        print("（四隅のピクセル中心は、与えられた地理範囲の外縁 ±0.5px に一致します）")


if __name__ == "__main__":
    main()

