#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo 用の道路モデル自動生成スクリプト.

以下の 6 種類の道路モデルを SDF で生成する:
- 直線路 100m （幅 2m, 3m, 5m）
- クランク路：直線 50m → 左折 50m → 右折 50m （幅 3m, 5m）
- S 字路：100m 四方に収まる中程度カーブ （幅 3m, 5m）

実行すると、このスクリプトと同じディレクトリ配下に models/ ディレクトリを作り、
各モデル用の model.sdf と model.config を出力する。

使い方:
    $ python3 road_generator.py

生成されるパス例:
    models/road_straight_100m_w2/
        model.sdf
        model.config
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


# 出力先のベースディレクトリ（このスクリプトのディレクトリ直下に models/ を作成）
BASE_DIR = Path(__file__).resolve().parent
MODELS_DIR = BASE_DIR / "models"


@dataclass
class RoadSegment:
    """道路セグメント1つ分の情報.

    Attributes:
        start: セグメント始点 (x, y).
        end: セグメント終点 (x, y).
    """

    start: Tuple[float, float]
    end: Tuple[float, float]

    @property
    def length(self) -> float:
        """セグメントの長さ."""
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        return math.hypot(dx, dy)

    @property
    def yaw(self) -> float:
        """セグメント向き（yaw ラジアン）."""
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        return math.atan2(dy, dx)

    @property
    def center(self) -> Tuple[float, float]:
        """セグメント中心座標 (x, y)."""
        return (0.5 * (self.start[0] + self.end[0]),
                0.5 * (self.start[1] + self.end[1]))


def ensure_dir(path: Path) -> None:
    """指定ディレクトリを作成（既にあれば何もしない）."""
    path.mkdir(parents=True, exist_ok=True)


def make_model_config(model_name: str) -> str:
    """model.config の文字列を生成."""
    return f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>road_generator</name>
    <email>example@example.com</email>
  </author>
  <description>Auto-generated road model: {model_name}</description>
</model>
""".strip() + "\n"


def make_box_link_sdf(
    link_name: str,
    center_x: float,
    center_y: float,
    length: float,
    width: float,
    height: float,
    yaw: float,
) -> str:
    """1つの箱セグメントの <link> SDF を生成.

    Args:
        link_name: link 要素の name.
        center_x: セグメント中心 X 座標.
        center_y: セグメント中心 Y 座標.
        length: 箱の長さ（X 方向）.
        width: 箱の幅（Y 方向）.
        height: 箱の高さ（Z 方向）.
        yaw: yaw 角 [rad].

    Returns:
        link 要素の XML 文字列.
    """
    # 箱の中心を z = height/2 に置く（上面が z = height になる）
    pose = f"{center_x:.3f} {center_y:.3f} {height / 2:.3f} 0 0 {yaw:.6f}"
    size = f"{length:.3f} {width:.3f} {height:.3f}"

    link_xml = f"""
    <link name="{link_name}">
      <pose>{pose}</pose>
      <collision name="{link_name}_collision">
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="{link_name}_visual">
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>
"""
    return link_xml


def segments_along_polyline(
    points: List[Tuple[float, float]],
) -> List[RoadSegment]:
    """折れ線（ポリライン）から RoadSegment を生成.

    Args:
        points: 折れ線の頂点列 [(x0, y0), (x1, y1), ...].

    Returns:
        RoadSegment のリスト.
    """
    segments: List[RoadSegment] = []
    for i in range(len(points) - 1):
        segments.append(RoadSegment(start=points[i], end=points[i + 1]))
    return segments


def subdivide_segment(
    segment: RoadSegment,
    max_panel_length: float,
) -> List[RoadSegment]:
    """1つのセグメントを、指定長さ以下の小セグメントに分割する.

    長い直線を 5m ごとのパネルに分割するのに使用。

    Args:
        segment: 元のセグメント.
        max_panel_length: 1 パネルあたりの最大長さ.

    Returns:
        分割されたセグメント群.
    """
    n_panels = max(1, int(math.ceil(segment.length / max_panel_length)))
    dx = (segment.end[0] - segment.start[0]) / n_panels
    dy = (segment.end[1] - segment.start[1]) / n_panels

    sub_segments: List[RoadSegment] = []
    for i in range(n_panels):
        x0 = segment.start[0] + dx * i
        y0 = segment.start[1] + dy * i
        x1 = segment.start[0] + dx * (i + 1)
        y1 = segment.start[1] + dy * (i + 1)
        sub_segments.append(RoadSegment(start=(x0, y0), end=(x1, y1)))

    return sub_segments


def build_road_model_sdf(
    model_name: str,
    segments: List[RoadSegment],
    width: float,
    panel_max_length: float = 5.0,
    height: float = 0.1,
) -> str:
    """複数セグメントから 1 つの道路モデル SDF を生成.

    Args:
        model_name: <model> の name.
        segments: 道路中心線のセグメントリスト.
        width: 道幅 [m].
        panel_max_length: パネル1枚の最大長さ.
        height: 道路の厚み [m].

    Returns:
        model.sdf 全体の XML 文字列.
    """
    # すべてのセグメントを 5m 以下程度に細分化する
    panel_segments: List[RoadSegment] = []
    for seg in segments:
        panel_segments.extend(subdivide_segment(seg, panel_max_length))

    link_xml_list: List[str] = []
    for idx, panel in enumerate(panel_segments):
        center_x, center_y = panel.center
        link_name = f"panel_{idx:03d}"
        link_xml_list.append(
            make_box_link_sdf(
                link_name=link_name,
                center_x=center_x,
                center_y=center_y,
                length=panel.length,
                width=width,
                height=height,
                yaw=panel.yaw,
            )
        )

    links_xml = "\n".join(link_xml_list)

    model_xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
{links_xml}
  </model>
</sdf>
""".strip() + "\n"

    return model_xml


# ========== 各道路形状の定義 ==========


def build_straight_100m_segments() -> List[RoadSegment]:
    """100m 直線路のセグメント（中心線）を生成.

    X 方向に 0 → 100m の直線。
    """
    p0 = (0.0, 0.0)
    p1 = (100.0, 0.0)
    return [RoadSegment(start=p0, end=p1)]


def build_crank_50m_segments(width: float) -> List[RoadSegment]:
    """1辺 50m のクランク路のセグメント（中心線）を生成.

    ユーザ指定どおり:
        直線 50m → 左折 50m → 右折 50m

    図示すると概ね以下のような形（上から見た図）:
        (0,0) → (50,0) → (50,50) → (100,50)
    """
    w2 = width / 2.0
    p0 = (0.0, 0.0)
    p1 = (50.0 + w2, 0.0)          # ← 左折前の直線を延伸
    p2 = (50.0, 50.0 + w2)    # ← 左折後も延伸
    p3 = (100.0, 50.0)   # ← 右折後も延伸

    return segments_along_polyline([p0, p1, p2, p3])


def build_s_curve_segments() -> List[RoadSegment]:
    """100m 四方に収まる中程度の S 字路のセグメントを生成.

    - X: 0 ～ 100m
    - Y: 30 ～ 70m 付近に収まるような S 字（中程度の曲率: B）

    ここでは Y = 50 + A * sin(2π * x / L) を利用。
    A=20, L=100 として、Y 範囲は 30～70 付近。
    """
    length_x = 100.0
    amplitude = 20.0  # 振幅（道の“振れ幅”）
    num_points = 41  # 0, 2.5, 5.0, ..., 100.0 （約2.5m刻み）

    points: List[Tuple[float, float]] = []
    for i in range(num_points):
        x = length_x * i / (num_points - 1)
        y = amplitude * math.sin(2.0 * math.pi * x / length_x)
        points.append((x, y))

    segments = segments_along_polyline(points)
    return segments


# ========== モデル生成処理 ==========


def write_model(model_name: str, model_sdf: str) -> None:
    """models/ 以下に model.config と model.sdf を書き出す."""
    model_dir = MODELS_DIR / model_name
    ensure_dir(model_dir)

    config_path = model_dir / "model.config"
    sdf_path = model_dir / "model.sdf"

    config_path.write_text(make_model_config(model_name), encoding="utf-8")
    sdf_path.write_text(model_sdf, encoding="utf-8")
    print(f"Generated model: {model_dir}")


def main() -> None:
    """メイン処理."""
    ensure_dir(MODELS_DIR)

    # 1) 100m 直線路（幅 2m, 3m, 5m）
    straight_segments = build_straight_100m_segments()
    for width in (2.0, 3.0, 5.0):
        model_name = f"road_straight_100m_w{int(width)}"
        sdf = build_road_model_sdf(
            model_name=model_name,
            segments=straight_segments,
            width=width,
            panel_max_length=5.0,
            height=0.1,
        )
        write_model(model_name, sdf)

    # 2) クランク路（1辺 50m） 幅 3m, 5m
    for width in (3.0, 5.0):
        crank_segments = build_crank_50m_segments(width)
        model_name = f"road_crank_50m_w{int(width)}"
        sdf = build_road_model_sdf(
            model_name=model_name,
            segments=crank_segments,
            width=width,
            panel_max_length=5.0,
            height=0.1,
        )
        write_model(model_name, sdf)

    # 3) S 字路（100m 四方、曲率 B） 幅 3m, 5m
    s_segments = build_s_curve_segments()
    for width in (3.0, 5.0):
        model_name = f"road_scurve_100m_w{int(width)}"
        sdf = build_road_model_sdf(
            model_name=model_name,
            segments=s_segments,
            width=width,
            panel_max_length=5.0,
            height=0.1,
        )
        write_model(model_name, sdf)

    print("All road models generated under:", MODELS_DIR)


if __name__ == "__main__":
    main()

