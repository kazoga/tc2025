#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
route_planner ノード（GetRoute 完了 + UpdateRoute 完了：仮想エッジ/履歴/経度キー修正対応）

主な機能と仕様:
-----------------
本ノードは、複数の固定/可変ブロックから構成されるルートを生成・配布（GetRoute）し、
走行中の経路封鎖イベントに応じて可変ブロックを再探索した新ルートを配布（UpdateRoute）する。

"""

from __future__ import annotations

import math
import os
import sys
import traceback
from pathlib import Path
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple, Union

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from sensor_msgs.msg import Image

from route_msgs.msg import Route, Waypoint
from route_msgs.srv import GetRoute, UpdateRoute

# 可変ルート探索（外部モジュール）
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

from .route_builder import (
    RouteBuilder,
    RouteBuildResult,
    WaypointOrigin,
    WaypointRecord,
    render_variable_route_overlay,
    normalize_nodes,
    resolve_path,
)

def _copy_pose(src: Pose) -> Pose:
    p = Pose()
    p.position.x = src.position.x
    p.position.y = src.position.y
    p.position.z = src.position.z
    p.orientation.x = src.orientation.x
    p.orientation.y = src.orientation.y
    p.orientation.z = src.orientation.z
    p.orientation.w = src.orientation.w
    return p


def convert_rgb_array_to_image(rgb_array: np.ndarray) -> Image:
    """RGB配列を sensor_msgs/Image に変換する。"""

    if rgb_array.ndim != 3 or rgb_array.shape[2] != 3:
        raise ValueError("RGB配列は (height, width, 3) の形状である必要があります。")

    rgb_uint8 = np.clip(rgb_array, 0, 255).astype(np.uint8)
    img = Image()
    img.header = Header()
    img.header.frame_id = "map"
    height, width, _ = rgb_uint8.shape
    img.height = int(height)
    img.width = int(width)
    img.encoding = "rgb8"
    img.step = int(width) * 3
    img.data = rgb_uint8.tobytes()
    return img


# ===== データクラス ==================================================================

@dataclass
class SegmentCacheEntry:
    """CSVから読み込んだ waypoint 配列のキャッシュ。

    Attributes:
        segment_id: CSVファイル（相対/絶対パス）を識別するID。
        waypoints: CSVから復元済みのWaypoint配列。ここでは端点ラベルの刻印等は行わない。
    """
    segment_id: str
    waypoints: List[Waypoint]


# ===== ユーティリティ関数 =============================================================

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """yaw[rad] をクォータニオンに変換する（roll,pitch=0固定）。"""
    q = Quaternion()
    half = yaw * 0.5
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def make_text_png_image(text: str = "No variable route in this plan") -> Image:
    """最小1x1の有効Imageを返す。"""
    img = Image()
    img.header = Header()
    img.header.frame_id = "map"
    img.height = 1
    img.width = 1
    img.encoding = "bgr8"
    img.step = 3
    img.data = b"\x00\x00\x00"
    return img


def almost_same_xy(ax: float, ay: float, bx: float, by: float, eps: float = 1e-6) -> bool:
    """2点のXY距離が閾値以下か判定。"""
    return math.hypot(ax - bx, ay - by) <= eps


def concat_with_dedup(base: List[Waypoint], ext: List[Waypoint], eps: float = 1e-6) -> List[Waypoint]:
    """2つの waypoint 配列を連結。境界が同一点なら ext 先頭を除外（重複排除）して結合する。"""
    if not base:
        return list(ext)
    if not ext:
        return base
    bx = base[-1].pose.position.x
    by = base[-1].pose.position.y
    ex = ext[0].pose.position.x
    ey = ext[0].pose.position.y
    if almost_same_xy(bx, by, ex, ey, eps):
        # ext[0]にラベルがありbase[-1]に無ければ移す（端点ラベルを保護）
        if ext[0].label and not base[-1].label:
            base[-1].label = ext[0].label
        return base + ext[1:]
    return base + ext


def apply_segment_fixed_flags(
    wps: List[Waypoint], origins: List[WaypointOrigin], blocks: List[Dict[str, Any]]
) -> None:
    """Waypointごとに「直前区間が固定かどうか」のフラグを設定する。"""
    if not wps:
        return
    block_type_map: Dict[Tuple[str, int], str] = {}
    for block in blocks:
        name = block.get("name")
        index = block.get("index")
        btype = block.get("type")
        if name is None or index is None:
            continue
        block_type_map[(str(name), int(index))] = str(btype or "")

    for wp in wps:
        wp.segment_is_fixed = False

    count = min(len(wps), len(origins))
    for i in range(count):
        origin = origins[i]
        wp = wps[i]
        block_name = getattr(origin, "block_name", None)
        block_index = getattr(origin, "block_index", None)
        if block_name is None or block_index is None:
            wp.segment_is_fixed = False
            continue
        try:
            block_key = (str(block_name), int(block_index))
        except (TypeError, ValueError):
            wp.segment_is_fixed = False
            continue
        block_type = block_type_map.get(block_key, "")
        is_fixed = block_type == "fixed"
        if getattr(origin, "segment_id", None) == "__virtual__":
            is_fixed = False
        wp.segment_is_fixed = bool(is_fixed)


def stamp_edge_end_labels(wps: List[Waypoint], src_label: str, dst_label: str) -> None:
    """エッジ両端の waypoint にノードラベルを刻印（境界識別・スライス用）。"""
    if not wps:
        return
    wps[0].label = src_label
    wps[-1].label = dst_label


def indexing(wps: List[Waypoint]) -> None:
    """Waypoint.index を 0..N-1 に再採番する。"""
    for i, wp in enumerate(wps):
        wp.index = i


def slice_by_labels(
    wps: List[Waypoint],
    start_label: str,
    goal_label: str,
) -> Tuple[List[Waypoint], int]:
    """start_label を前方から、goal_label を後方から検索し、その範囲でスライスする。
    start_label または goal_label が空文字/None の場合は、
    それぞれ先頭または末尾ノードを採用する。

    Returns:
        (sliced_list, start_offset) を返す。
        start_offset は元配列に対する切り出し開始位置。
    """
    start_idx: Optional[int] = None
    goal_idx: Optional[int] = None

    # --- start_label の補完または検索 ---
    if not start_label:  # 空文字や None の場合は先頭ノードから
        start_idx = 0
        print(f"[WARN][slice_by_labels] start_label is empty; using first waypoint '{wps[0].label}'")
    else:
        for i, wp in enumerate(wps):
            if wp.label == start_label:
                start_idx = i
                break

    # --- goal_label の補完または検索 ---
    if not goal_label:  # 空文字や None の場合は末尾ノードまで
        goal_idx = len(wps) - 1
        print(f"[WARN][slice_by_labels] goal_label is empty; using last waypoint '{wps[-1].label}'")
    else:
        for j in range(len(wps) - 1, -1, -1):
            if wps[j].label == goal_label:
                goal_idx = j
                break

    # --- バリデーション ---
    if start_idx is None:
        raise ValueError(f"Start label '{start_label}' not found.")
    if goal_idx is None:
        raise ValueError(f"Goal label '{goal_label}' not found.")
    if start_idx > goal_idx:
        raise ValueError("Invalid slice order: start > goal.")

    # --- スライス処理 ---
    sliced = wps[start_idx: goal_idx + 1]
    if len(sliced) <= 1:
        # start == goal で1点のみはエラー扱い（要件）
        raise ValueError("Single-point route not allowed.")

    return sliced, start_idx


def calc_total_distance(wps: List[Waypoint]) -> float:
    """総距離[m]を算出。"""
    dist = 0.0
    for i in range(1, len(wps)):
        x0, y0 = wps[i - 1].pose.position.x, wps[i - 1].pose.position.y
        x1, y1 = wps[i].pose.position.x, wps[i].pose.position.y
        dist += math.hypot(x1 - x0, y1 - y0)
    return dist


def adjust_orientations(
    wps: List[Waypoint],
    recalc_ranges: List[Tuple[int, int]],
    block_boundaries: List[int],
) -> None:
    """姿勢を再計算する範囲を限定して実施する。

    Args:
        wps: ルート全体のwaypoints（スライス後）。
        recalc_ranges: 完全再計算すべき連続区間のインデックス範囲（start_idx, end_idx）。逆走区間や仮想区間を入れる。
        block_boundaries: ブロック末尾インデックス（端点姿勢のみ補正）。
    """
    # 完全再計算区間（仮想/逆走を含む）
    for start, end in recalc_ranges:
        if start < 0 or end >= len(wps) or start >= end:
            continue
        for i in range(start, end):
            dx = wps[i+1].pose.position.x - wps[i].pose.position.x
            dy = wps[i+1].pose.position.y - wps[i].pose.position.y
            yaw = math.atan2(dy, dx)
            wps[i].pose.orientation = yaw_to_quaternion(yaw)
        dx = wps[end].pose.position.x - wps[end-1].pose.position.x
        dy = wps[end].pose.position.y - wps[end-1].pose.position.y
        yaw = math.atan2(dy, dx)
        wps[end].pose.orientation = yaw_to_quaternion(yaw)

    # ブロック末尾waypoint（端点のみ）
    for idx in block_boundaries:
        if 0 < idx < len(wps):
            dx = wps[idx].pose.position.x - wps[idx-1].pose.position.x
            dy = wps[idx].pose.position.y - wps[idx-1].pose.position.y
            yaw = math.atan2(dy, dx)
            wps[idx].pose.orientation = yaw_to_quaternion(yaw)

    # ルート全体の末尾（端点のみ）
    if len(wps) > 1:
        dx = wps[-1].pose.position.x - wps[-2].pose.position.x
        dy = wps[-1].pose.position.y - wps[-2].pose.position.y
        yaw = math.atan2(dy, dx)
        wps[-1].pose.orientation = yaw_to_quaternion(yaw)


def pack_route_msg(
    wps: List[Waypoint],
    version: int,
    total_distance: float,
    route_image: Optional[Image],
) -> Route:
    """Route.msg を組み立てる。必要に応じて route_image はテキストPNGプレースホルダを入れる。"""
    msg = Route()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.waypoints = wps
    msg.version = int(version)
    msg.total_distance = float(total_distance)
    msg.route_image = route_image if route_image is not None else make_text_png_image()
    return msg


# ===== RoutePlanner ノード ============================================================

class RoutePlannerNode(Node):
    """ルート配布サーバ（GetRoute 完了 + UpdateRoute 完了：仮想エッジ/履歴対応）。"""

    def __init__(self) -> None:
        super().__init__("route_planner")

        # --- 起動時パラメータ ---
        self.declare_parameter("config_yaml_path", "routes/config.yaml")
        self.declare_parameter("csv_base_dir", "routes")
        self.declare_parameter("map_image_path", None)
        self.declare_parameter("map_worldfile_path", None)

        try:
            pkg_share = get_package_share_directory("route_planner")
        except PackageNotFoundError:
            pkg_share = None
            self.get_logger().warn("パッケージ共有ディレクトリが取得できませんでした。相対パスはそのまま扱います。")

        config_yaml_raw = str(self.get_parameter("config_yaml_path").value)
        csv_base_dir_raw = str(self.get_parameter("csv_base_dir").value)
        map_image_raw = self.get_parameter("map_image_path").value
        map_worldfile_raw = self.get_parameter("map_worldfile_path").value
        get_service_name = 'get_route'
        update_service_name = 'update_route'

        self.config_yaml_path: str = resolve_path(pkg_share, config_yaml_raw) if config_yaml_raw else ""
        self.csv_base_dir: str = resolve_path(pkg_share, csv_base_dir_raw) if csv_base_dir_raw else ""
        self.map_image_path: Optional[str] = self._resolve_map_resource(map_image_raw, pkg_share, "map_image_path")
        self.map_worldfile_path: Optional[str] = self._resolve_map_resource(map_worldfile_raw, pkg_share, "map_worldfile_path")

        if not self.config_yaml_path:
            self.get_logger().error("config_yaml_path is required.")
        else:
            self.get_logger().info(f"Using config_yaml_path: {self.config_yaml_path}")
        if self.csv_base_dir:
            self.get_logger().info(f"Using csv_base_dir: {self.csv_base_dir}")

        if self.map_image_path and self.map_worldfile_path:
            self.get_logger().info(f"Using map_image_path: {self.map_image_path}")
            self.get_logger().info(f"Using map_worldfile_path: {self.map_worldfile_path}")
        elif map_image_raw or map_worldfile_raw:
            self.get_logger().warn(
                "地図画像またはワールドファイルの解決に失敗したため、地図描画を無効化します。"
            )

        # --- メンバ（状態） ---
        self.blocks: List[Dict[str, Any]] = []                  # YAMLのブロック原義（固定/可変）
        self.segments: Dict[str, SegmentCacheEntry] = {}        # segment_id -> キャッシュ済みwaypoints
        self.current_route: Optional[Route] = None
        self.current_route_origins: List[WaypointOrigin] = []   # current_route.waypoints と同長
        self.route_version: int = 0                             # Route.msgのversion(int32)に対応
        self.current_block_name: Optional[str] = None           # 累積封鎖の対象ブロック名
        self.closed_edges: Set[frozenset] = set()               # {frozenset({u,v}), }
        self.last_request_checkpoints: Set[str] = set()         # GetRoute時に追加されたチェックポイント
        self.visited_checkpoints_hist: Dict[str, Set[str]] = {} # ブロック名 -> 訪問済みチェックポイント（永続）

        # --- ルート定義のロード ---
        self.route_builder = RouteBuilder(
            config_yaml_path=self.config_yaml_path,
            csv_base_dir=self.csv_base_dir,
            logger=self.get_logger(),
        )
        try:
            self.route_builder.load()
            self.csv_base_dir = self.route_builder.csv_base_dir
            self.blocks = list(self.route_builder.blocks)
            self._refresh_segment_cache()
        except Exception as exc:
            self.get_logger().error(f"ルート設定の読み込みに失敗しました: {exc}")
            self.blocks = []
            self.segments = {}

        # --- サービス登録（逐次処理: MutuallyExclusive） ---
        cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_get = self.create_service(
            GetRoute, get_service_name, self.handle_get_route, callback_group=cb_group
        )
        self._srv_update = self.create_service(
            UpdateRoute, update_service_name, self.handle_update_route, callback_group=cb_group
        )

        self.get_service_name = self._resolve_service_name(get_service_name)
        self.update_service_name = self._resolve_service_name(update_service_name)

        self.get_logger().info("route_planner is ready.")

    def _resolve_service_name(self, name: str) -> str:
        """リマップ後のサービス名を取得する。"""
        try:
            return self.resolve_service_name(name)
        except AttributeError:
            return name

    def _resolve_map_resource(
        self,
        raw_value: Optional[Any],
        pkg_share: Optional[str],
        param_name: str,
    ) -> Optional[str]:
        """地図関連パラメータをパッケージ相対パスから絶対パスへ解決する。"""

        if raw_value is None:
            return None
        value = str(raw_value).strip()
        if not value:
            return None

        candidates = []
        if os.path.isabs(value):
            candidates.append(os.path.normpath(value))
        if pkg_share:
            candidates.append(resolve_path(pkg_share, value))
        package_root = str(_THIS_DIR.parent)
        candidates.append(resolve_path(package_root, value))

        for candidate in candidates:
            if candidate and os.path.exists(candidate):
                return candidate

        self.get_logger().warn(f"{param_name} で指定されたファイルが見つかりません: {value}")
        return None

    def _render_variable_route_image(
        self,
        nodes: Dict[str, Tuple[float, float]],
        solver_result: Dict[str, Any],
    ) -> Optional[Image]:
        """solve_variable_routeの結果を用いて地図画像を生成する。"""

        if not (self.map_image_path and self.map_worldfile_path):
            return None

        rgb_array = render_variable_route_overlay(
            nodes,
            solver_result,
            self.map_image_path,
            self.map_worldfile_path,
            logger=self.get_logger(),
        )
        if rgb_array is None:
            return None

        try:
            return convert_rgb_array_to_image(rgb_array)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"地図描画のメッセージ変換に失敗しました: {exc}")
            return None

    def _record_to_waypoint(self, record: WaypointRecord) -> Waypoint:
        """WaypointRecordをRoute用のWaypointメッセージへ変換する。"""

        wp = Waypoint()
        wp.label = record.label
        wp.index = int(record.index)
        wp.pose.position.x = float(record.pose.position.x)
        wp.pose.position.y = float(record.pose.position.y)
        wp.pose.position.z = float(record.pose.position.z)
        wp.pose.orientation.x = float(record.pose.orientation.x)
        wp.pose.orientation.y = float(record.pose.orientation.y)
        wp.pose.orientation.z = float(record.pose.orientation.z)
        wp.pose.orientation.w = float(record.pose.orientation.w)
        wp.right_open = float(record.right_open)
        wp.left_open = float(record.left_open)
        wp.line_stop = bool(record.line_stop)
        wp.signal_stop = bool(record.signal_stop)
        wp.not_skip = bool(record.not_skip)
        if hasattr(wp, "segment_is_fixed"):
            wp.segment_is_fixed = bool(record.segment_is_fixed)
        return wp

    def _refresh_segment_cache(self) -> None:
        """RouteBuilderのキャッシュからROSメッセージ形式のセグメントを再構築する。"""

        self.segments = {}
        for seg_id, entry in self.route_builder.segments.items():
            waypoints = [self._record_to_waypoint(wp) for wp in entry.waypoints]
            self.segments[seg_id] = SegmentCacheEntry(seg_id, waypoints)

    def _run_variable_solver(
        self,
        nodes: Dict[str, Tuple[float, float]],
        edges: List[Dict[str, Any]],
        start: str,
        goal: str,
        checkpoints: List[str],
    ) -> Dict[str, Any]:
        """RouteBuilder経由で可変ルート探索を実行する。"""

        return self.route_builder.run_variable_solver(
            nodes=nodes,
            edges=edges,
            start=start,
            goal=goal,
            checkpoints=checkpoints,
        )


    # ===== GetRoute / UpdateRoute =====================================================

    def handle_get_route(self, request: GetRoute.Request, response: GetRoute.Response) -> GetRoute.Response:
        """初期ルートの生成と配布。Update 用の由来メタも同時に構築して保存する。"""
        try:
            # --- サービスリクエストのフィールド名を設計書仕様に統一 ---
            start_label = getattr(request, "start_label", None)
            goal_label = getattr(request, "goal_label", None)
            checkpoint_labels = getattr(request, "checkpoint_labels", [])
            self.get_logger().info(
                f"Get route: start_label='{start_label}', goal_label='{goal_label}', checkpoints='{checkpoint_labels}'"
            )
            if not self.route_builder.blocks:
                raise RuntimeError("No blocks configuration loaded.")

            result: RouteBuildResult = self.route_builder.build_route(
                start_label=str(start_label or ""),
                goal_label=str(goal_label or ""),
                checkpoint_labels=list(checkpoint_labels),
            )
            # builder側でブロック情報が更新されている可能性があるため反映しておく。
            self.blocks = list(self.route_builder.blocks)

            route_wps: List[Waypoint] = [
                self._record_to_waypoint(record) for record in result.waypoints
            ]
            origins: List[WaypointOrigin] = list(result.origins)
            total_distance = result.total_distance

            route_image: Optional[Image] = None
            if result.has_variable_block:
                solver_info = result.solver_info
                if solver_info is not None:
                    img = self._render_variable_route_image(
                        solver_info.nodes, solver_info.solver_result
                    )
                    if img is not None:
                        route_image = img
                if route_image is None:
                    route_image = make_text_png_image("variable part image (placeholder)")
            else:
                route_image = make_text_png_image("No variable route in this plan")

            self.route_version = 1
            route = pack_route_msg(route_wps, self.route_version, total_distance, route_image)

            self.current_route = route
            self.current_route_origins = origins
            self.last_request_checkpoints = set(checkpoint_labels)
            self.visited_checkpoints_hist.clear()  # 初期ルート生成時に訪問履歴をリセット

            response.success = True
            response.message = ""
            response.route = route
            return response

        except Exception as e:
            self.get_logger().error(f"[GetRoute] {e}\n{traceback.format_exc()}")
            response.success = False
            response.message = str(e)
            response.route = Route()
            return response

    def handle_update_route(self, request: UpdateRoute.Request, response: UpdateRoute.Response) -> UpdateRoute.Response:
        """経路封鎖リルートの実行。仮想エッジ（current→prev→U）を先頭に挿入し、その後に新可変ルートと後続ブロックを連結する。"""
        try:
            # 0) 前提検証
            if self.current_route is None or not self.current_route.waypoints:
                response.success = False
                response.message = "No current route in server."
                response.route = Route()
                return response
            if request.route_version != self.current_route.version:
                self.get_logger().error("[UpdateRoute] Route version mismatch.")
                response.success = False
                response.message = "Route version mismatch."
                response.route = self.current_route or Route()
                return response

            wps = self.current_route.waypoints
            origins = self.current_route_origins
            n = len(wps)

            # 1) prev/next の正当性検証（現ルート上で隣接）
            if not (0 <= request.prev_index < n - 1):
                raise RuntimeError("Invalid prev_index.")
            if request.next_index != request.prev_index + 1:
                raise RuntimeError("prev/next must be adjacent (next_index = prev_index + 1).")
            if wps[request.prev_index].label != request.prev_wp_label:
                raise RuntimeError("prev_wp_label mismatch current route.")
            if wps[request.next_index].label != request.next_wp_label:
                raise RuntimeError("next_wp_label mismatch current route.")

            prev_origin = origins[request.prev_index]
            next_origin = origins[request.next_index]

            # 2) 対象ブロックを特定（同一ブロックである必要あり）
            if prev_origin.block_name != next_origin.block_name:
                raise RuntimeError("prev/next belong to different blocks.")
            block_name = prev_origin.block_name
            block_idx = prev_origin.block_index

            # 対象ブロックを取得
            block = None
            for b in self.blocks:
                if b["name"] == block_name and b["index"] == block_idx:
                    block = b
                    break
            if block is None:
                raise RuntimeError(f"Block not found: {block_name}")
            if block["type"] == "fixed":
                # 固定ブロックはリルート不可（封鎖検知時はエラー返却。ただしノードは継続）
                self.get_logger().warn("Closure in fixed block (not reroutable).")
                response.success = False
                response.message = "Closure in fixed block (not reroutable)."
                response.route = self.current_route or Route()
                return response

            # 3) 現在走行中エッジの (U,V) と、prev のエッジ内位置を特定
            u_node = prev_origin.edge_u
            v_node = prev_origin.edge_v
            seg_id_running = prev_origin.segment_id
            u_first_flag = prev_origin.u_first
            local_idx_prev = prev_origin.index_in_edge
            if not (u_node and v_node and seg_id_running is not None and u_first_flag is not None and local_idx_prev is not None):
                # 可変ブロックなのに必要メタが欠けるのは異常
                raise RuntimeError("Failed to identify running edge metadata for closure.")

            # 4) 封鎖累積の管理（同ブロック内は追加／別ブロックならリセット）
            if self.current_block_name != block_name:
                self.closed_edges = set()
                self.current_block_name = block_name
            # 片方向通行可能は考えないため {U,V} を丸ごと封鎖
            self.closed_edges.add(frozenset({u_node, v_node}))

            # 5) グラフ = 原義 - 累積封鎖
            nodes, edges = self._build_graph_with_closures(block_name)

            # 6) 未通過チェックポイント集合（永続履歴を考慮）
            yaml_cps = set(block.get("checkpoints", []))
            req_cps = set(self.last_request_checkpoints)  # GetRoute で追加された分
            # このブロックの nodes に存在するものだけを有効チェックポイントとする
            node_ids = {nd["id"] for nd in (block.get("nodes") or [])}
            required = {c for c in (yaml_cps | req_cps) if c in node_ids}

            # 永続履歴（これまでの封鎖を跨いでも消えない）
            hist = self.visited_checkpoints_hist.setdefault(block_name, set())
            # 今回の current_route で prev までに通過済みを追加
            visited_now: Set[str] = set()
            for i in range(0, request.prev_index + 1):
                lab = wps[i].label
                if lab in required:
                    visited_now.add(lab)
            hist |= visited_now
            remaining_cps = list(required - hist)

            # 7) 仮想ノード/エッジは solver に __virtual__ を流さず、
            #    代わりに「current→prev→U」の waypoint 群を先頭に自前で連結する。
            #    solver の start は U、goal は block.goal。
            goal = block.get("goal")
            if not goal:
                raise RuntimeError("Block goal not set.")

            # 7-1) 先頭: current（今回の現在位置）
            new_wps: List[Waypoint] = []
            new_origins: List[WaypointOrigin] = []
            new_wps.append(self._make_waypoint_from_pose(request.current_pose, label="current"))
            new_origins.append(WaypointOrigin(
                block_name=block_name,
                block_index=block_idx,
                segment_id=None,
                edge_u=None,
                edge_v=None,
                index_in_edge=None,
                u_first=None,
            ))

            # 7-2) 仮想エッジ: current→prev→U の waypoint 群を生成して連結
            virtual_wps = self._make_virtual_edge_waypoints(
                seg_id=seg_id_running,
                u_label=u_node,
                local_idx_prev=local_idx_prev,
                u_first_on_route=u_first_flag,
                prev_wp=wps[request.prev_index],
                current_pose=request.current_pose,
            )
            # 端点ラベル（Uラベル）を刻印（virtual_wps の末尾は U になる）
            if virtual_wps:
                stamp_edge_end_labels(virtual_wps, src_label="current", dst_label=u_node)

            # 追加（重複境界は concat 内で解消される）
            start_idx_virtual = len(new_wps)
            new_wps = concat_with_dedup(new_wps, virtual_wps)
            end_idx_virtual = len(new_wps) - 1
            # origin は "仮想エッジ" として記録（segment_id="__virtual__"相当）
            for i_local in range(end_idx_virtual - start_idx_virtual + 1):
                new_origins.append(WaypointOrigin(
                    block_name=block_name,
                    block_index=block_idx,
                    segment_id="__virtual__",
                    edge_u=u_node,
                    edge_v=v_node,
                    index_in_edge=i_local,
                    u_first=True,  # virtual は current→U の向き（U側へ向かう）としてUファースト扱いで良い
                ))

            # 仮想区間は「姿勢完全再計算区間」に追加
            recalc_ranges: List[Tuple[int, int]] = []
            if end_idx_virtual > start_idx_virtual:
                recalc_ranges.append((start_idx_virtual, end_idx_virtual))

            block_tail_indices: List[int] = []  # 末尾補正用

            # 8) solver 実行（start=U, goal=block.goal, checkpoints=remaining_cps）
            nodes_dict = normalize_nodes(nodes)
            result = self._run_variable_solver(
                nodes=nodes_dict,
                edges=edges,
                start=u_node,
                goal=goal,
                checkpoints=remaining_cps,
            )
            edge_seq: List[Dict[str, Any]] = result.get("edge_sequence", [])
            if not edge_seq:
                raise RuntimeError("No route found after applying closures.")
            # 画像取得
            img_solver = self._render_variable_route_image(nodes_dict, result)

            # 9) 可変結果を連結（__virtual__ は solver から基本返さない前提。返ってきても無視）
            for e in edge_seq:
                if e.get("segment_id") == "__virtual__":
                    # 念のためスキップ（通常は発生しない想定）
                    continue
                seg_id2 = e["segment_id"]
                direction2 = e["direction"]  # "forward" | "reverse"
                src2 = e["source"]
                dst2 = e["target"]
                entry2 = self.segments.get(seg_id2)
                if entry2 is None:
                    raise RuntimeError(f"Segment not loaded: {seg_id2}")

                u_first2 = True if direction2 == "forward" else False
                seg_wps2 = entry2.waypoints if u_first2 else list(reversed(entry2.waypoints))
                stamp_edge_end_labels(seg_wps2, src_label=src2, dst_label=dst2)

                start_idx2 = len(new_wps)
                new_wps = concat_with_dedup(new_wps, seg_wps2)
                end_idx2 = len(new_wps) - 1

                # origin 付与
                for i_local in range(end_idx2 - start_idx2 + 1):
                    new_origins.append(WaypointOrigin(
                        block_name=block_name,
                        block_index=block_idx,
                        segment_id=seg_id2,
                        edge_u=src2,
                        edge_v=dst2,
                        index_in_edge=i_local,
                        u_first=u_first2,
                    ))

                # 逆走区間は姿勢完全再計算に追加
                if not u_first2 and end_idx2 > start_idx2:
                    recalc_ranges.append((start_idx2, end_idx2))

            # ブロック末尾を記録（可変ブロックの末尾）
            block_tail_indices.append(len(new_wps) - 1)

            # 10) 後続の未走行ブロックを定義どおり連結（固定はCSV、可変は再探索）
            for b in self.blocks:
                if b["index"] <= block_idx:
                    continue
                bname2 = b["name"]
                bidx2 = b["index"]
                if b["type"] == "fixed":
                    seg_id3 = b["segment_id"]
                    entry3 = self.segments.get(seg_id3)
                    if entry3 is None:
                        raise RuntimeError(f"[fixed:{bname2}] segment not loaded: {seg_id3}")
                    before = len(new_wps)
                    new_wps = concat_with_dedup(new_wps, entry3.waypoints)
                    for _ in range(len(new_wps) - before):
                        new_origins.append(WaypointOrigin(
                            block_name=bname2,
                            block_index=bidx2,
                            segment_id=seg_id3,
                            edge_u=None,
                            edge_v=None,
                            index_in_edge=None,
                            u_first=None,
                        ))
                    block_tail_indices.append(len(new_wps) - 1)
                elif b["type"] == "variable":
                    nodes2 = b.get("nodes") or []
                    edges2 = b.get("edges") or []
                    start2 = b.get("start")
                    goal2 = b.get("goal")
                    if not nodes2 or not edges2 or not start2 or not goal2:
                        raise RuntimeError(f"[variable:{bname2}] definition incomplete.")

                    merged_cps2 = list(dict.fromkeys((b.get("checkpoints", []) or []) + list(self.last_request_checkpoints)))
                    node_ids2 = {nd["id"] for nd in nodes2}
                    cps2 = [c for c in merged_cps2 if c in node_ids2]

                    nodes_dict2 = normalize_nodes(nodes2)
                    res2 = self._run_variable_solver(
                        nodes=nodes_dict2,
                        edges=edges2,
                        start=start2,
                        goal=goal2,
                        checkpoints=cps2,
                    )
                    img_extra = self._render_variable_route_image(nodes_dict2, res2)
                    if img_extra is not None and img_solver is None:
                        img_solver = img_extra
                    es2: List[Dict[str, Any]] = res2.get("edge_sequence", [])
                    if not es2:
                        raise RuntimeError(f"[variable:{bname2}] solver returned empty edge_sequence.")
                    for e2 in es2:
                        seg_id4 = e2["segment_id"]
                        direction4 = e2["direction"]
                        src4 = e2["source"]
                        dst4 = e2["target"]
                        entry4 = self.segments.get(seg_id4)
                        if entry4 is None:
                            raise RuntimeError(f"[variable:{bname2}] segment not loaded: {seg_id4}")
                        u_first4 = True if direction4 == "forward" else False
                        seg_wps4 = entry4.waypoints if u_first4 else list(reversed(entry4.waypoints))
                        stamp_edge_end_labels(seg_wps4, src_label=src4, dst_label=dst4)

                        start_idx4 = len(new_wps)
                        new_wps = concat_with_dedup(new_wps, seg_wps4)
                        end_idx4 = len(new_wps) - 1

                        for i_local in range(end_idx4 - start_idx4 + 1):
                            new_origins.append(WaypointOrigin(
                                block_name=bname2,
                                block_index=bidx2,
                                segment_id=seg_id4,
                                edge_u=src4,
                                edge_v=dst4,
                                index_in_edge=i_local,
                                u_first=u_first4,
                            ))

                        if not u_first4 and end_idx4 > start_idx4:
                            recalc_ranges.append((start_idx4, end_idx4))

                    block_tail_indices.append(len(new_wps) - 1)

            # 11) finalize（姿勢補正・採番・距離・画像・version++）
            adjust_orientations(new_wps, recalc_ranges, block_tail_indices)
            apply_segment_fixed_flags(new_wps, new_origins, self.blocks)
            indexing(new_wps)
            total_distance = calc_total_distance(new_wps)
            route_image = img_solver if img_solver is not None else make_text_png_image("variable part image (placeholder)")
            self.route_version += 1
            new_route = pack_route_msg(new_wps, self.route_version, total_distance, route_image)

            # 状態更新
            self.current_route = new_route
            self.current_route_origins = new_origins
            # visited 履歴は維持（本ブロックのキーに対して既に更新済み）

            response.success = True
            response.message = ""
            response.route = new_route
            return response

        except Exception as e:
            self.get_logger().error(f"[UpdateRoute] {e}\n{traceback.format_exc()}")
            response.success = False
            response.message = str(e)
            response.route = Route()
            return response

    # ===== 内部補助 =====================================================

    def _build_graph_with_closures(self, block_name: str) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """可変ブロックの原義から累積封鎖を反映した nodes/edges を構築する。"""
        # 対象ブロックを取得
        block = None
        for b in self.blocks:
            if b["name"] == block_name:
                block = b
                break
        if block is None or block["type"] != "variable":
            raise RuntimeError("Block not found or not variable.")

        nodes = list(block.get("nodes") or [])
        raw_edges = list(block.get("edges") or [])
        # 累積封鎖の適用：{u,v} が self.closed_edges にあれば、その行は除外
        filt_edges: List[Dict[str, Any]] = []
        for e in raw_edges:
            u = str(e["source"])
            v = str(e["target"])
            if frozenset({u, v}) in self.closed_edges:
                continue
            filt_edges.append(dict(e))  # シャローコピー

        return nodes, filt_edges

    def _make_waypoint_from_pose(self, pose_stamped: PoseStamped, label: str) -> Waypoint:
        """PoseStamped から Waypoint を作る簡易ヘルパ。index は後段で採番し直す。"""
        wp = Waypoint()
        wp.label = label
        wp.index = 0
        wp.pose = _copy_pose(pose_stamped.pose)
        if hasattr(wp, "segment_is_fixed"):
            wp.segment_is_fixed = False
        return wp

    def _make_virtual_edge_waypoints(
        self,
        seg_id: str,
        u_label: str,
        local_idx_prev: int,
        u_first_on_route: bool,
        prev_wp: Waypoint,
        current_pose: PoseStamped,
    ) -> List[Waypoint]:
        """仮想エッジ current→prev→U の waypoint 群を生成する。

        ロジック:
          - 走行中エッジ（seg_id）の CSV を取得し、"Uが先頭" となる向き（u_first）で配列を準備する。
            * GetRoute時に記録した u_first_on_route を使って、prev のローカルindexを u_first 基準のindexに変換する。
          - current（Pose）を先頭に、prev（既存Waypoint）を挟み、prev→U 方向に向かう配列を切り出す。
            * prev が既に U（= index 0）なら、[current, U] の最短列になる（処理は同一）。
          - 生成した配列は、後段で stamp_edge_end_labels("current", U) で端点ラベルを刻む。

        Args:
            seg_id: 走行中エッジの segment_id（CSVを特定）。
            u_label: 手前側ノードのラベル（U）。
            local_idx_prev: prev のエッジ内ローカルindex（GetRoute時の向き基準）。
            u_first_on_route: GetRoute時にそのエッジを U→V 向きで使ったかどうかのフラグ。
            prev_wp: 現ルート上の prev Waypoint（座標/姿勢を持つ）。
            current_pose: 現在位置の PoseStamped。

        Returns:
            仮想エッジ（current→prev→U）に相当する Waypoint 配列。
        """
        entry = self.segments.get(seg_id)
        if entry is None:
            raise RuntimeError(f"Virtual edge source segment not loaded: {seg_id}")

        base = entry.waypoints  # CSV由来の素の配列（端点ラベルは未刻印）

        # "Uが先頭" となる向きの配列を作る（u_first=True の配列）
        # GetRoute時の向きが U→V (u_first_on_route=True) なら、そのまま。
        # 逆なら反転して U→V 向きに合わせる。
        if u_first_on_route:
            seg_u_first = list(base)
            # prev のindexはそのまま
            prev_idx_u_first = int(local_idx_prev)
        else:
            seg_u_first = list(reversed(base))
            # 反転したため prev の index は変換（len-1 - idx）
            prev_idx_u_first = (len(seg_u_first) - 1) - int(local_idx_prev)

        if not (0 <= prev_idx_u_first < len(seg_u_first)):
            raise RuntimeError("Invalid prev index in virtual edge generation.")

        # Uは seg_u_first[0] に対応する（末尾は V）
        # prev→U 方向に向かう配列は seg_u_first[0:prev_idx_u_first+1] を逆順にせず、
        # current→prev→  →U となるように、current, prev, seg_u_first[prev_idx_u_first-1  0] を連結する。
        virtual: List[Waypoint] = []
        # current を先頭に追加（Waypoint化）
        current_wp = self._make_waypoint_from_pose(current_pose, label="current")
        virtual.append(current_wp)

        # prev を続けて追加（既存prevをコピーして先頭重複回避。位置はそのまま）
        prev_copy = Waypoint()
        prev_copy.label = prev_wp.label  # prev は中間なのでlabelは通常空/任意だが、そのまま踏襲
        prev_copy.index = 0
        prev_copy.pose = _copy_pose(prev_wp.pose)
        if hasattr(prev_copy, "right_open"):
            prev_copy.right_open = float(
                getattr(prev_wp, "right_open", getattr(prev_wp, "right_is_open", 0.0))
            )
        if hasattr(prev_copy, "left_open"):
            prev_copy.left_open = float(
                getattr(prev_wp, "left_open", getattr(prev_wp, "left_is_open", 0.0))
            )
        if hasattr(prev_copy, "line_stop"):
            prev_copy.line_stop = bool(
                getattr(prev_wp, "line_stop", getattr(prev_wp, "line_is_stop", False))
            )
        if hasattr(prev_copy, "signal_stop"):
            prev_copy.signal_stop = bool(
                getattr(prev_wp, "signal_stop", getattr(prev_wp, "signal_is_stop", False))
            )
        if hasattr(prev_copy, "not_skip"):
            prev_copy.not_skip = bool(
                getattr(prev_wp, "not_skip", getattr(prev_wp, "isnot_skipnum", False))
            )
        virtual.append(prev_copy)

        # prev_idx_u_first から 1 ずつデクリメントして U=0 まで辿る
        # prev が U と同一（prev_idx_u_first==0）のときは追加なし（[current, prev(=U)]のみ）
        for idx in range(prev_idx_u_first - 1, -1, -1):
            src = seg_u_first[idx]
            # コピーして追加（オリジナルを汚さない）
            wp = Waypoint()
            wp.label = src.label
            wp.index = 0
            wp.pose = _copy_pose(src.pose)
            if hasattr(wp, "right_open"):
                wp.right_open = float(
                    getattr(src, "right_open", getattr(src, "right_is_open", 0.0))
                )
            if hasattr(wp, "left_open"):
                wp.left_open = float(
                    getattr(src, "left_open", getattr(src, "left_is_open", 0.0))
                )
            if hasattr(wp, "line_stop"):
                wp.line_stop = bool(
                    getattr(src, "line_stop", getattr(src, "line_is_stop", False))
                )
            if hasattr(wp, "signal_stop"):
                wp.signal_stop = bool(
                    getattr(src, "signal_stop", getattr(src, "signal_is_stop", False))
                )
            if hasattr(wp, "not_skip"):
                wp.not_skip = bool(
                    getattr(src, "not_skip", getattr(src, "isnot_skipnum", False))
                )
            virtual.append(wp)

        return virtual


# ===== エントリポイント ===============================================================

def main(argv: Optional[List[str]] = None) -> None:
    """ノードのエントリポイント。"""
    rclpy.init(args=argv)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
