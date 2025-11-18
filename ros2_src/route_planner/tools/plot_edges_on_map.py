#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
edges.csv に記載された全てのセグメントCSVを読み取り、各waypointを地図画像上に可視化するツール。
- 個別(セグメントごと)のマップ: 先頭/末尾/中間でマーカーを変え、先頭と末尾のみラベル描画。<segment>_mapped.png を同ディレクトリへ出力。
- まとめ(all_mapped): すべてのセグメントを1枚の地図に重ね描き。ファイルごとに色/マーカーを変える。端点特別扱い・ラベルなし。all_mapped.png を edges.csv と同ディレクトリに出力。

要件メモ:
- スクリプト設置: route_planner/tools/
- 引数: <edges.csv> <map.png> <map.pgw> [--verbose]
- edges.csv に記載のパスは「edges.csv の 1つ上のディレクトリ」からの相対パス。
- セグメントCSV 形式: 1行=1 waypoint。必須列: label, latitude, longitude（小文字固定）。
- マッピングは route_planner/route_planner/latlon_to_pixel_mapper.py の latlon_to_pixel() を利用。
- 画像外(None)になった点はスキップ。スキップ件数はログ出力。
- セグメントCSVが壊れている等はスキップして警告、全体は続行。PGW不正などは即時終了。

Coding Rules:
- Python 3
- Google Python Style Guide 準拠を意識
- 型ヒントあり
- 日本語コメント（簡潔・初見者向け）
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Pillow は画像描画用
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# latlon_to_pixel の import 準備（実行ディレクトリに依らず動くようにする）
# route_planner/tools/ からの実行を前提に、パッケージ or 近傍相対での import を試行。
# 失敗時はスクリプト位置からの相対パス、および最後に /mnt/data も試す。
# ---------------------------------------------------------------------------
def _import_mapper() -> "callable":
    """latlon_to_pixel() を import して返す。見つからなければ SystemExit。"""
    import importlib
    import sys as _sys

    candidates = [
        # パッケージ経由
        "route_planner.route_planner.latlon_to_pixel_mapper",
        # スクリプト相対（tools/ の隣の route_planner/ を想定）
        str((Path(__file__).resolve().parents[1] / "route_planner").as_posix()),
        # 開発環境（本チャット実行時の補助）
        "/mnt/data",
    ]

    # 1) パッケージ import
    try:
        mod = importlib.import_module("route_planner.route_planner.latlon_to_pixel_mapper")
        return getattr(mod, "latlon_to_pixel")
    except Exception:
        pass

    # 2) スクリプト位置からの相対パスを sys.path へ追加して import
    for p in candidates[1:]:
        if p not in _sys.path:
            _sys.path.insert(0, p)
        try:
            mod = importlib.import_module("latlon_to_pixel_mapper")
            return getattr(mod, "latlon_to_pixel")
        except Exception:
            continue

    print("ERROR: could not import latlon_to_pixel() from latlon_to_pixel_mapper.py", file=sys.stderr)
    sys.exit(1)


latlon_to_pixel = _import_mapper()


# ---------------------------------------------------------------------------
# データ構造
# ---------------------------------------------------------------------------
@dataclass
class Waypoint:
    """セグメントCSVの1レコードを表す。"""
    label: str
    latitude: float
    longitude: float


@dataclass
class SegmentPlotData:
    """1セグメント分の描画データ（ピクセル座標）"""
    segment_csv: Path
    pixels: List[Tuple[Optional[int], Optional[int]]]
    labels: List[str]  # Waypoint.label（先頭/末尾で利用）


# ---------------------------------------------------------------------------
# CSV 読み取り
# ---------------------------------------------------------------------------
def read_edges(edges_csv: Path, verbose: bool = False) -> List[Path]:
    """edges.csv を読み取り、セグメントCSVへのパス一覧を返す。

    仕様: edges.csv の置かれているディレクトリの「1つ上」を基準に、
    相対パスが記録されている。
    """
    # edges.csv の 1つ上を基準ディレクトリとする
    base_dir = edges_csv.parent.parent

    segment_paths: List[Path] = []
    with edges_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        columns = reader.fieldnames or []
        # 既知の列名を優先
        preferred_cols = ["waypoint_list", "segment", "file", "path"]
        col: Optional[str] = None
        for c in preferred_cols:
            if c in columns:
                col = c
                break
        # 見つからない場合は「値が .csv で終わる列」を探索
        if col is None:
            # 先頭行を覗くため読み直し
            f.seek(0)
            reader2 = csv.DictReader(f)
            sample_row = next(reader2, None)
            if sample_row:
                for c in (reader2.fieldnames or []):
                    v = (sample_row.get(c) or "").strip()
                    if v.lower().endswith(".csv"):
                        col = c
                        break
        if col is None:
            print("ERROR: edges.csv にセグメントCSVへの列（例: waypoint_list）が見つかりません。", file=sys.stderr)
            sys.exit(1)

    # 本体の再読込（1行ずつ相対→絶対へ）
    with edges_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rel = (row.get(col) or "").strip()
            if not rel:
                continue
            seg_path = (base_dir / rel).resolve()
            if not seg_path.exists():
                print(f"WARNING: セグメントCSVが見つかりません: {seg_path}", file=sys.stderr)
                continue
            segment_paths.append(seg_path)
            if verbose:
                print(f"[edges] {rel} -> {seg_path}")
    return segment_paths


def read_segment_csv(path: Path) -> List[Waypoint]:
    """セグメントCSV（label, latitude, longitude）を読み取り、Waypoint配列を返す。"""
    waypoints: List[Waypoint] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = ["label", "latitude", "longitude"]
        if reader.fieldnames is None or any(c not in reader.fieldnames for c in required):
            raise ValueError(f"必須列が足りません（必要: {required}）: {path}")
        for row in reader:
            try:
                label = (row["label"] or "").strip()
                lat = float(row["latitude"])
                lon = float(row["longitude"])
            except Exception as e:
                raise ValueError(f"CSV値の解釈に失敗しました: {path} row={row}") from e
            waypoints.append(Waypoint(label=label, latitude=lat, longitude=lon))
    return waypoints


# ---------------------------------------------------------------------------
# 描画ユーティリティ（PIL）
# ---------------------------------------------------------------------------
def _draw_marker(draw: ImageDraw.ImageDraw, x: int, y: int, kind: str, color: Tuple[int, int, int], size: int = 5) -> None:
    """マーカーを描画する。kind: 'circle' | 'square' | 'triangle'"""
    if kind == "circle":
        r = size
        draw.ellipse((x - r, y - r, x + r, y + r), outline=color, width=2)
    elif kind == "square":
        r = size
        draw.rectangle((x - r, y - r, x + r, y + r), outline=color, width=2)
    elif kind == "triangle":
        r = size + 1
        pts = [(x, y - r), (x - r, y + r), (x + r, y + r)]
        draw.polygon(pts, outline=color)
    else:
        # デフォルトは小さめの点
        draw.point((x, y), fill=color)


def _draw_polyline(draw: ImageDraw.ImageDraw, points: List[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
    """折れ線を描画する（少なくとも2点ある場合のみ）。"""
    if len(points) >= 2:
        draw.line(points, fill=color, width=2)


def _load_font() -> ImageFont.ImageFont:
    """ラベル描画用のフォントを返す。未インストールでも動くようにデフォルトを利用。"""
    try:
        # よくある日本語フォントがあれば利用（環境依存）
        return ImageFont.truetype("DejaVuSans.ttf", 14)
    except Exception:
        return ImageFont.load_default()


def _offset(pos: Tuple[int, int], dx: int = 6, dy: int = -6) -> Tuple[int, int]:
    """ラベルが潰れにくいよう、基準点から少しオフセット。"""
    return (pos[0] + dx, pos[1] + dy)


# カラーパレット（12色程度をローテーション）
PALETTE: List[Tuple[int, int, int]] = [
    (230, 25, 75),   # red
    (60, 180, 75),   # green
    (0, 130, 200),   # blue
    (245, 130, 48),  # orange
    (145, 30, 180),  # purple
    (70, 240, 240),  # cyan
    (240, 50, 230),  # magenta
    (210, 245, 60),  # lime
    (250, 190, 190), # pink
    (0, 128, 128),   # teal
    (230, 190, 255), # lavender
    (170, 110, 40),  # brown
]


# ---------------------------------------------------------------------------
# 個別セグメントの描画
# ---------------------------------------------------------------------------
def map_and_draw_single(
    image_path: Path,
    worldfile_path: Path,
    segment_csv: Path,
    verbose: bool = False,
) -> SegmentPlotData:
    """1セグメントCSVを読み、マッピングして _mapped.png を保存。描画データも返す。"""
    waypoints = read_segment_csv(segment_csv)

    # (lat, lon) の配列へ
    latlon_list: List[Tuple[float, float]] = [(wp.latitude, wp.longitude) for wp in waypoints]
    labels: List[str] = [wp.label for wp in waypoints]

    # ピクセル座標へ変換（None は画像外）
    pixels = latlon_to_pixel(str(image_path), str(worldfile_path), latlon_list, verbose=verbose)

    # 出力パス: <segment_dir>/<segment_stem>_mapped.png
    out_path = segment_csv.with_name(segment_csv.stem + "_mapped.png")

    # 画像へ描画
    img = Image.open(image_path).convert("RGBA")
    draw = ImageDraw.Draw(img)

    # マーカー規則: 先頭=緑三角 / 中間=青丸 / 末尾=赤四角
    color_start = (0, 200, 0)
    color_mid = (0, 120, 255)
    color_end = (220, 0, 0)
    font = _load_font()

    # 画像内に収まる点だけ集めて、線も描画
    valid_points: List[Tuple[int, int]] = []
    skipped = 0
    for idx, p in enumerate(pixels):
        if p[0] is None or p[1] is None:
            skipped += 1
            continue
        valid_points.append((int(p[0]), int(p[1]))

)

    # 折れ線（視認性向上のため）
    _draw_polyline(draw, valid_points, color=(50, 50, 50))

    # マーカーとラベル
    for i, pos in enumerate(valid_points):
        if i == 0:
            _draw_marker(draw, pos[0], pos[1], kind="triangle", color=color_start, size=6)
            # ラベル（先頭）
            label = labels[0] if labels else ""
            if label:
                draw.text(_offset(pos), label, fill=color_start, font=font)
        elif i == len(valid_points) - 1:
            _draw_marker(draw, pos[0], pos[1], kind="square", color=color_end, size=6)
            # ラベル（末尾）
            label = labels[-1] if labels else ""
            if label:
                draw.text(_offset(pos), label, fill=color_end, font=font)
        else:
            _draw_marker(draw, pos[0], pos[1], kind="circle", color=color_mid, size=4)

    # 保存
    out_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_path)
    if verbose:
        print(f"[single] saved: {out_path} (skipped={skipped})")

    return SegmentPlotData(segment_csv=segment_csv, pixels=pixels, labels=labels)


# ---------------------------------------------------------------------------
# 全体(all_mapped)の描画
# ---------------------------------------------------------------------------
def draw_all_mapped(
    image_path: Path,
    worldfile_path: Path,
    segments: List[SegmentPlotData],
    edges_csv: Path,
    verbose: bool = False,
) -> Path:
    """全セグメントを1枚へ重ね描きし、all_mapped.png を edges.csv と同じ場所に保存。"""
    out_path = edges_csv.parent / "all_mapped.png"
    img = Image.open(image_path).convert("RGBA")
    draw = ImageDraw.Draw(img)

    # マーカー種をローテーション
    marker_kinds = ["circle", "square", "triangle"]

    for i, seg in enumerate(segments):
        color = PALETTE[i % len(PALETTE)]
        kind = marker_kinds[i % len(marker_kinds)]

        # 画像内の点だけ使用
        valid_points: List[Tuple[int, int]] = []
        skipped = 0
        for p in seg.pixels:
            if p[0] is None or p[1] is None:
                skipped += 1
                continue
            valid_points.append((int(p[0]), int(p[1])))

        # 折れ線とマーカー（端点の特別扱い・ラベルなし）
        _draw_polyline(draw, valid_points, color=color)
        for (x, y) in valid_points:
            _draw_marker(draw, x, y, kind=kind, color=color, size=4)

        if verbose:
            print(f"[all] {seg.segment_csv.name}: plotted={len(valid_points)} skipped={skipped}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_path)
    if verbose:
        print(f"[all] saved: {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# メイン処理
# ---------------------------------------------------------------------------
def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """コマンドライン引数を解析。"""
    p = argparse.ArgumentParser(description="edges.csv のセグメントCSVを地図に可視化します。")
    p.add_argument("edges_csv", type=str, help="エッジリスト CSV のパス")
    p.add_argument("map_png", type=str, help="地図画像 PNG のパス")
    p.add_argument("map_pgw", type=str, help="PGW ワールドファイルのパス")
    p.add_argument("--verbose", "-v", action="store_true", help="詳細ログを出力")
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> None:
    """エントリポイント。"""
    args = parse_args(argv)
    edges_csv = Path(args.edges_csv).resolve()
    map_png = Path(args.map_png).resolve()
    map_pgw = Path(args.map_pgw).resolve()
    verbose = args.verbose

    if not edges_csv.exists():
        print(f"ERROR: edges.csv が見つかりません: {edges_csv}", file=sys.stderr)
        sys.exit(1)
    if not map_png.exists():
        print(f"ERROR: PNG が見つかりません: {map_png}", file=sys.stderr)
        sys.exit(1)
    if not map_pgw.exists():
        print(f"ERROR: PGW が見つかりません: {map_pgw}", file=sys.stderr)
        sys.exit(1)

    # edges.csv -> セグメントCSV一覧
    seg_paths = read_edges(edges_csv, verbose=verbose)
    if not seg_paths:
        print("WARNING: セグメントCSV が1件も見つかりませんでした。", file=sys.stderr)

    # 各セグメントを個別に描画しつつ、描画データを保持
    plotted_segments: List[SegmentPlotData] = []
    for seg_csv in seg_paths:
        try:
            seg_data = map_and_draw_single(map_png, map_pgw, seg_csv, verbose=verbose)
            plotted_segments.append(seg_data)
        except KeyboardInterrupt:
            raise
        except ValueError as e:
            # 必須列欠落など
            print(f"WARNING: セグメントCSVをスキップします: {seg_csv} reason={e}", file=sys.stderr)
        except Exception as e:
            print(f"WARNING: セグメントCSV処理中にエラー: {seg_csv} reason={e}", file=sys.stderr)

    # まとめ(all_mapped)の描画
    try:
        out_all = draw_all_mapped(map_png, map_pgw, plotted_segments, edges_csv, verbose=verbose)
    except Exception as e:
        print(f"ERROR: all_mapped の生成に失敗しました: {e}", file=sys.stderr)
        sys.exit(1)

    if verbose:
        print("Done.")
        print(f"all_mapped: {out_all}")


if __name__ == "__main__":
    main()
