"""Core route building utilities independent from ROS node lifecycle."""

from __future__ import annotations

import csv
import importlib
import importlib.util
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Protocol, Sequence, Set, Tuple
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from route_msgs.msg import Route, Waypoint

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

from .graph_solver import solve_variable_route

_YAML_SPEC = importlib.util.find_spec("yaml")
yaml = importlib.import_module("yaml") if _YAML_SPEC is not None else None


class LoggerProtocol(Protocol):
    """ロガーが満たすべき最小インターフェース。"""

    def info(self, message: str) -> None:
        """情報ログを出力する。"""

    def warn(self, message: str) -> None:
        """警告ログを出力する。"""

    def error(self, message: str) -> None:
        """エラーログを出力する。"""


class _StdoutLogger:
    """stdoutへ出力する簡易ロガー。"""

    def info(self, message: str) -> None:
        print(f"[INFO] {message}")

    def warn(self, message: str) -> None:
        print(f"[WARN] {message}")

    def error(self, message: str) -> None:
        print(f"[ERROR] {message}")


@dataclass
class SegmentCacheEntry:
    """CSVから読み込んだ waypoint 配列のキャッシュ。"""

    segment_id: str
    waypoints: List[Waypoint]


@dataclass
class WaypointOrigin:
    """waypoint の由来情報。"""

    block_name: str
    block_index: int
    segment_id: Optional[str]
    edge_u: Optional[str]
    edge_v: Optional[str]
    index_in_edge: Optional[int]
    u_first: Optional[bool]


def _strip_comment(line: str) -> str:
    """行末コメントを除去する。"""
    result_chars: List[str] = []
    in_quote = False
    quote_char = ""
    for ch in line:
        if ch in {'"', "'"}:
            if not in_quote:
                in_quote = True
                quote_char = ch
            elif quote_char == ch:
                in_quote = False
        if ch == "#" and not in_quote:
            break
        result_chars.append(ch)
    return "".join(result_chars)


def _parse_scalar(token: str) -> Any:
    """スカラ値をPython型へ変換する。"""
    if token == "":
        return ""
    lowered = token.lower()
    if lowered in {"null", "none", "~"}:
        return None
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if (token.startswith('"') and token.endswith('"')) or (
        token.startswith("'") and token.endswith("'")
    ):
        return token[1:-1]
    try:
        if any(ch in token for ch in (".", "e", "E")):
            return float(token)
        return int(token)
    except ValueError:
        return token


def _split_by_comma(expr: str) -> List[str]:
    """トップレベルのカンマで分割する。"""
    parts: List[str] = []
    depth_brace = 0
    depth_bracket = 0
    in_quote = False
    quote_char = ""
    current: List[str] = []
    for ch in expr:
        if ch in {'"', "'"}:
            if not in_quote:
                in_quote = True
                quote_char = ch
            elif quote_char == ch:
                in_quote = False
        elif not in_quote:
            if ch == "{":
                depth_brace += 1
            elif ch == "}":
                depth_brace -= 1
            elif ch == "[":
                depth_bracket += 1
            elif ch == "]":
                depth_bracket -= 1
            elif ch == "," and depth_brace == 0 and depth_bracket == 0:
                parts.append("".join(current).strip())
                current = []
                continue
        current.append(ch)
    if current:
        parts.append("".join(current).strip())
    return [p for p in parts if p != ""]


def _parse_flow_mapping(expr: str) -> Dict[str, Any]:
    """{a:1,b:2} 形式のマッピングを解析する。"""
    inner = expr.strip()[1:-1].strip()
    if not inner:
        return {}
    result: Dict[str, Any] = {}
    for part in _split_by_comma(inner):
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        key = key.strip()
        value = value.strip()
        result[key] = _parse_flow_value(value)
    return result


def _parse_flow_sequence(expr: str) -> List[Any]:
    """[a,b,c] 形式のシーケンスを解析する。"""
    inner = expr.strip()[1:-1].strip()
    if not inner:
        return []
    return [_parse_flow_value(part) for part in _split_by_comma(inner)]


def _parse_flow_value(value: str) -> Any:
    """フロースタイル値を解析する。"""
    if value.startswith("{") and value.endswith("}"):
        return _parse_flow_mapping(value)
    if value.startswith("[") and value.endswith("]"):
        return _parse_flow_sequence(value)
    return _parse_scalar(value)


def _parse_block(
    prepared_lines: List[Tuple[int, str]],
    start_index: int,
    indent: int,
) -> Tuple[Any, int]:
    """インデント情報からブロックを解析する。"""
    mapping: Dict[str, Any] = {}
    items: List[Any] = []
    container: Optional[str] = None
    index = start_index
    while index < len(prepared_lines):
        current_indent, content = prepared_lines[index]
        if current_indent < indent:
            break
        if current_indent > indent:
            raise ValueError("Invalid indentation structure.")
        if content.startswith("- "):
            if container == "dict":
                raise ValueError("Mixed mapping and sequence.")
            container = "list"
            item_content = content[2:].strip()
            index += 1
            item, index = _parse_list_item(
                item_content, prepared_lines, index, indent
            )
            items.append(item)
        else:
            if container == "list":
                raise ValueError("Mixed mapping and sequence.")
            container = "dict"
            key, value, index = _parse_dict_entry(
                content, prepared_lines, index, indent
            )
            mapping[key] = value
    if container == "list":
        return items, index
    return mapping, index


def _parse_list_item(
    item_content: str,
    prepared_lines: List[Tuple[int, str]],
    index: int,
    indent: int,
) -> Tuple[Any, int]:
    """リスト要素を解析する。"""
    if not item_content:
        return _parse_block(prepared_lines, index, indent + 2)
    if item_content.startswith("{") and item_content.endswith("}"):
        item = _parse_flow_mapping(item_content)
        if index < len(prepared_lines) and prepared_lines[index][0] > indent:
            nested, index = _parse_block(prepared_lines, index, indent + 2)
            if isinstance(nested, dict):
                item.update(nested)
        return item, index
    if item_content.startswith("[") and item_content.endswith("]"):
        item = _parse_flow_sequence(item_content)
        if index < len(prepared_lines) and prepared_lines[index][0] > indent:
            nested, index = _parse_block(prepared_lines, index, indent + 2)
            if isinstance(nested, list):
                item.extend(nested)
        return item, index
    if ":" in item_content:
        key, value_str = item_content.split(":", 1)
        item = {key.strip(): _parse_scalar(value_str.strip())}
        if index < len(prepared_lines) and prepared_lines[index][0] > indent:
            nested, index = _parse_block(prepared_lines, index, indent + 2)
            if isinstance(nested, dict):
                item.update(nested)
        return item, index
    value = _parse_scalar(item_content)
    if index < len(prepared_lines) and prepared_lines[index][0] > indent:
        nested, index = _parse_block(prepared_lines, index, indent + 2)
        if isinstance(nested, list):
            return [value] + nested, index
    return value, index


def _parse_dict_entry(
    content: str,
    prepared_lines: List[Tuple[int, str]],
    index: int,
    indent: int,
) -> Tuple[str, Any, int]:
    """マッピング要素を解析する。"""
    if ":" not in content:
        raise ValueError("Invalid mapping entry")
    key, value_str = content.split(":", 1)
    key = key.strip()
    value_str = value_str.strip()
    index += 1
    if not value_str:
        value, index = _parse_block(prepared_lines, index, indent + 2)
        return key, value, index
    if value_str.startswith("{") and value_str.endswith("}"):
        value = _parse_flow_mapping(value_str)
        if index < len(prepared_lines) and prepared_lines[index][0] > indent:
            nested, index = _parse_block(prepared_lines, index, indent + 2)
            if isinstance(nested, dict):
                value.update(nested)
        return key, value, index
    if value_str.startswith("[") and value_str.endswith("]"):
        value = _parse_flow_sequence(value_str)
        if index < len(prepared_lines) and prepared_lines[index][0] > indent:
            nested, index = _parse_block(prepared_lines, index, indent + 2)
            if isinstance(nested, list):
                value.extend(nested)
        return key, value, index
    return key, _parse_scalar(value_str), index


def _fallback_yaml_safe_load(text: str) -> Any:
    """PyYAMLが無い環境向けの簡易YAMLパーサ。"""
    prepared: List[Tuple[int, str]] = []
    for raw in text.splitlines():
        stripped = _strip_comment(raw.rstrip("\n"))
        if not stripped.strip():
            continue
        indent = len(stripped) - len(stripped.lstrip(" "))
        prepared.append((indent, stripped.strip()))
    if not prepared:
        return {}
    parsed, _ = _parse_block(prepared, 0, prepared[0][0])
    return parsed


def _copy_pose(src: Pose) -> Pose:
    """Poseをディープコピーするヘルパ。"""
    pose = Pose()
    pose.position.x = src.position.x
    pose.position.y = src.position.y
    pose.position.z = src.position.z
    pose.orientation.x = src.orientation.x
    pose.orientation.y = src.orientation.y
    pose.orientation.z = src.orientation.z
    pose.orientation.w = src.orientation.w
    return pose


def _load_png_as_image(path: str) -> Image:
    """PNG画像を読み込みImageを生成する。"""
    import cv2

    image = Image()
    image.header = Header()
    image.header.frame_id = "map"
    cv_img = cv2.imread(path, cv2.IMREAD_COLOR)
    if cv_img is None:
        raise FileNotFoundError(f"Cannot read PNG file: {path}")
    image.height, image.width, _ = cv_img.shape
    image.encoding = "bgr8"
    image.step = cv_img.shape[1] * 3
    image.data = cv_img.tobytes()
    return image


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """yaw[rad] をクォータニオンに変換する。"""
    quat = Quaternion()
    half = yaw * 0.5
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half)
    quat.w = math.cos(half)
    return quat


def load_route_image_from_solver(result: Dict[str, Any]) -> Optional[Image]:
    """ソルバー結果からPNGを読み込む。"""
    image_path = result.get("route_image_path")
    if not image_path or not os.path.exists(image_path):
        print(f"[WARN] route_image_path not found or missing: {image_path}")
        return None
    return _load_png_as_image(image_path)


def make_text_png_image(text: str = "No variable route in this plan") -> Image:
    """最小サイズのダミー画像を返す。"""
    image = Image()
    image.header = Header()
    image.header.frame_id = "map"
    image.height = 1
    image.width = 1
    image.encoding = "bgr8"
    image.step = 3
    image.data = b"\x00\x00\x00"
    return image


def almost_same_xy(ax: float, ay: float, bx: float, by: float, eps: float = 1e-6) -> bool:
    """2点のXY距離を比較する。"""
    return math.hypot(ax - bx, ay - by) <= eps


def concat_with_dedup(base: List[Waypoint], ext: List[Waypoint], eps: float = 1e-6) -> List[Waypoint]:
    """境界が同一点なら重複を除去して結合する。"""
    if not base:
        return list(ext)
    if not ext:
        return base
    bx = base[-1].pose.position.x
    by = base[-1].pose.position.y
    ex = ext[0].pose.position.x
    ey = ext[0].pose.position.y
    if almost_same_xy(bx, by, ex, ey, eps):
        if ext[0].label and not base[-1].label:
            base[-1].label = ext[0].label
        return base + ext[1:]
    return base + ext


def stamp_edge_end_labels(wps: List[Waypoint], src_label: str, dst_label: str) -> None:
    """エッジ端点のラベルを刻印する。"""
    if not wps:
        return
    wps[0].label = src_label
    wps[-1].label = dst_label


def resolve_path(base_dir: Optional[str], path_str: str) -> str:
    """相対パスをベースディレクトリから解決する。"""
    if not base_dir:
        return path_str
    if os.path.isabs(path_str):
        return path_str
    return os.path.normpath(os.path.join(base_dir, path_str))


def parse_waypoint_csv(csv_path: str) -> List[Waypoint]:
    """waypoint CSV を読み込み Waypoint 配列を生成する。"""
    waypoints: List[Waypoint] = []
    with open(csv_path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for i, row in enumerate(reader):
            wp = Waypoint()
            label_val = row.get("label")
            if label_val is None or label_val == "":
                label_val = row.get("id") or row.get("num") or ""
            wp.label = str(label_val)
            wp.index = i
            try:
                wp.pose.position.x = float(row.get("x", 0.0))
                wp.pose.position.y = float(row.get("y", 0.0))
                wp.pose.position.z = float(row.get("z", 0.0))
            except ValueError as exc:
                raise ValueError(f"Invalid XYZ in {csv_path} at row {i+1}") from exc
            try:
                wp.pose.orientation.x = float(row.get("q1", 0.0))
                wp.pose.orientation.y = float(row.get("q2", 0.0))
                wp.pose.orientation.z = float(row.get("q3", 0.0))
                wp.pose.orientation.w = float(row.get("q4", 1.0))
            except ValueError as exc:
                raise ValueError(f"Invalid quaternion in {csv_path} at row {i+1}") from exc

            def _bool_from_int(value: Any) -> bool:
                try:
                    return bool(int(str(value)))
                except Exception:
                    return False

            def _float_from_any(value: Any) -> float:
                try:
                    return float(value)
                except Exception:
                    return 0.0

            wp.right_open = _float_from_any(row.get("right_open", row.get("right_is_open", 0.0)))
            wp.left_open = _float_from_any(row.get("left_open", row.get("left_is_open", 0.0)))
            wp.line_stop = _bool_from_int(row.get("line_stop", row.get("line_is_stop", 0)))
            wp.signal_stop = _bool_from_int(row.get("signal_stop", row.get("signal_is_stop", 0)))
            wp.not_skip = _bool_from_int(row.get("not_skip", row.get("isnot_skipnum", 0)))
            waypoints.append(wp)
    return waypoints


def indexing(wps: List[Waypoint]) -> None:
    """Waypoint.index を再採番する。"""
    for i, wp in enumerate(wps):
        wp.index = i


def slice_by_labels(wps: List[Waypoint], start_label: str, goal_label: str) -> Tuple[List[Waypoint], int]:
    """ラベル区間でwaypointをスライスする。"""
    start_idx: Optional[int] = None
    goal_idx: Optional[int] = None
    if not start_label:
        start_idx = 0
        print(
            f"[WARN][slice_by_labels] start_label is empty; using first waypoint '{wps[0].label}'"
        )
    else:
        for i, wp in enumerate(wps):
            if wp.label == start_label:
                start_idx = i
                break
    if not goal_label:
        goal_idx = len(wps) - 1
        print(
            f"[WARN][slice_by_labels] goal_label is empty; using last waypoint '{wps[-1].label}'"
        )
    else:
        for j in range(len(wps) - 1, -1, -1):
            if wps[j].label == goal_label:
                goal_idx = j
                break
    if start_idx is None:
        raise ValueError(f"Start label '{start_label}' not found.")
    if goal_idx is None:
        raise ValueError(f"Goal label '{goal_label}' not found.")
    if start_idx > goal_idx:
        raise ValueError("Invalid slice order: start > goal.")
    sliced = wps[start_idx : goal_idx + 1]
    if len(sliced) <= 1:
        raise ValueError("Single-point route not allowed.")
    return sliced, start_idx


def calc_total_distance(wps: List[Waypoint]) -> float:
    """総距離[m]を算出する。"""
    dist = 0.0
    for i in range(1, len(wps)):
        x0 = wps[i - 1].pose.position.x
        y0 = wps[i - 1].pose.position.y
        x1 = wps[i].pose.position.x
        y1 = wps[i].pose.position.y
        dist += math.hypot(x1 - x0, y1 - y0)
    return dist


def adjust_orientations(
    wps: List[Waypoint],
    recalc_ranges: List[Tuple[int, int]],
    block_boundaries: List[int],
) -> None:
    """姿勢を区間指定で再計算する。"""
    for start, end in recalc_ranges:
        if start < 0 or end >= len(wps) or start >= end:
            continue
        for i in range(start, end):
            dx = wps[i + 1].pose.position.x - wps[i].pose.position.x
            dy = wps[i + 1].pose.position.y - wps[i].pose.position.y
            yaw = math.atan2(dy, dx)
            wps[i].pose.orientation = yaw_to_quaternion(yaw)
        dx = wps[end].pose.position.x - wps[end - 1].pose.position.x
        dy = wps[end].pose.position.y - wps[end - 1].pose.position.y
        yaw = math.atan2(dy, dx)
        wps[end].pose.orientation = yaw_to_quaternion(yaw)
    for idx in block_boundaries:
        if not (0 <= idx < len(wps)):
            continue
        if idx < len(wps) - 1:
            dx = wps[idx + 1].pose.position.x - wps[idx].pose.position.x
            dy = wps[idx + 1].pose.position.y - wps[idx].pose.position.y
        elif idx > 0:
            dx = wps[idx].pose.position.x - wps[idx - 1].pose.position.x
            dy = wps[idx].pose.position.y - wps[idx - 1].pose.position.y
        else:
            dx = dy = 0.0
        if dx != 0.0 or dy != 0.0:
            yaw = math.atan2(dy, dx)
            wps[idx].pose.orientation = yaw_to_quaternion(yaw)
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
    """Route.msg を組み立てる。"""
    msg = Route()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.waypoints = wps
    msg.version = int(version)
    msg.total_distance = float(total_distance)
    msg.route_image = route_image if route_image is not None else make_text_png_image()
    return msg


class RouteBuilder:
    """YAML/CSVとソルバーを組み合わせてルートを生成するコア。"""

    def __init__(
        self,
        config_yaml_path: str,
        csv_base_dir: str,
        logger: Optional[LoggerProtocol] = None,
    ) -> None:
        """設定パスとロガーを受け取って初期化する。"""
        self.config_yaml_path = config_yaml_path
        self.csv_base_dir = csv_base_dir
        self._logger: LoggerProtocol = logger or _StdoutLogger()
        self.blocks: List[Dict[str, Any]] = []
        self.segments: Dict[str, SegmentCacheEntry] = {}
        self.current_route: Optional[Route] = None
        self.current_route_origins: List[WaypointOrigin] = []
        self.route_version: int = 0
        self.current_block_name: Optional[str] = None
        self.closed_edges: Set[frozenset[str]] = set()
        self.last_request_checkpoints: Set[str] = set()
        self.visited_checkpoints_hist: Dict[str, Set[str]] = {}
        if self.config_yaml_path:
            self.reload()

    def reload(self) -> None:
        """YAMLとCSVを再読込する。"""
        self._load_blocks_from_yaml()
        self._load_csv_segments()

    def _load_blocks_from_yaml(self) -> None:
        """YAMLからブロック定義を読み込む。"""
        if not self.config_yaml_path:
            self._logger.warn("No config_yaml_path specified.")
            self.blocks = []
            return
        try:
            with open(self.config_yaml_path, "r", encoding="utf-8") as handle:
                text = handle.read()
        except Exception as exc:
            self._logger.error(f"Failed to load YAML: {exc}")
            self.blocks = []
            return
        try:
            data = (
                yaml.safe_load(text) if yaml is not None else _fallback_yaml_safe_load(text)
            )
        except Exception as exc:
            self._logger.error(f"Failed to parse YAML: {exc}")
            self.blocks = []
            return
        if not data:
            self._logger.error("YAML is empty or invalid.")
            self.blocks = []
            return
        blocks = data.get("blocks")
        if not isinstance(blocks, list):
            self._logger.error("YAML must contain list under 'blocks'.")
            self.blocks = []
            return
        norm_blocks: List[Dict[str, Any]] = []
        for idx, block in enumerate(blocks):
            btype = block.get("type")
            bname = block.get("name", f"block_{idx}")
            if btype not in {"fixed", "variable"}:
                self._logger.warn(f"Unknown block type: {btype}")
                continue
            norm = dict(block)
            norm["name"] = str(bname)
            norm["index"] = idx
            norm_blocks.append(norm)
        self.blocks = norm_blocks

    def _load_csv_segments(self) -> None:
        """CSVファイル群を読み込んでキャッシュする。"""
        self.segments = {}
        if not self.blocks:
            return
        for block in self.blocks:
            btype = block.get("type")
            bname = block.get("name")
            if btype == "fixed":
                seg_id = block.get("segment_id")
                if not seg_id:
                    self._logger.error(f"[fixed:{bname}] segment_id missing.")
                    continue
                seg_path = resolve_path(self.csv_base_dir, seg_id)
                try:
                    wps = parse_waypoint_csv(seg_path)
                    self.segments[seg_id] = SegmentCacheEntry(seg_id, wps)
                except Exception as exc:
                    self._logger.error(
                        f"[fixed:{bname}] failed to load segment '{seg_path}': {exc}"
                    )
            elif btype == "variable":
                self._load_variable_block(block)

    def _load_variable_block(self, block: Dict[str, Any]) -> None:
        """可変ブロックのCSVを読み込む。"""
        bname = block.get("name")
        nodes_file = resolve_path(self.csv_base_dir, block.get("nodes_file", ""))
        try:
            nodes: List[Dict[str, Any]] = []
            with open(nodes_file, newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                for row in reader:
                    nid = row.get("id")
                    if nid is None or nid == "":
                        raise ValueError("nodes.csv: 'id' is required.")
                    try:
                        lat = float(row.get("lat"))
                        lon = float(row.get("lon"))
                    except Exception as exc:
                        raise ValueError("nodes.csv: 'lat'/'lon' must be float.") from exc
                    nodes.append({"id": str(nid), "lat": lat, "lon": lon})
            block["nodes"] = nodes
        except Exception as exc:
            self._logger.error(
                f"[variable:{bname}] failed to load nodes '{nodes_file}': {exc}"
            )
            block["nodes"] = []
            return
        edges_file = resolve_path(self.csv_base_dir, block.get("edges_file", ""))
        try:
            edges: List[Dict[str, Any]] = []
            with open(edges_file, newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                for i, row in enumerate(reader):
                    source = row.get("source")
                    target = row.get("target")
                    seg_rel = row.get("waypoint_list") or row.get("segment_id")
                    rev_raw = row.get("reversible", "1")
                    if not source or not target or not seg_rel:
                        raise ValueError(
                            f"edges.csv row {i+1}: source/target/waypoint_list are required."
                        )
                    try:
                        reversible = int(str(rev_raw))
                    except Exception as exc:
                        raise ValueError(
                            f"edges.csv row {i+1}: reversible must be 0 or 1."
                        ) from exc
                    if reversible not in (0, 1):
                        raise ValueError(
                            f"edges.csv row {i+1}: reversible must be 0 or 1."
                        )
                    edge = {
                        "source": str(source),
                        "target": str(target),
                        "waypoint_list": seg_rel,
                        "reversible": reversible,
                    }
                    edges.append(edge)
                    seg_path = resolve_path(self.csv_base_dir, seg_rel)
                    try:
                        if seg_rel not in self.segments:
                            wps = parse_waypoint_csv(seg_path)
                            self.segments[seg_rel] = SegmentCacheEntry(seg_rel, wps)
                    except Exception as exc:
                        raise RuntimeError(
                            f"failed to load segment '{seg_path}': {exc}"
                        ) from exc
            block["edges"] = edges
        except Exception as exc:
            self._logger.error(
                f"[variable:{bname}] failed to load edges '{edges_file}': {exc}"
            )
            block["edges"] = []

    def build_initial_route(
        self,
        start_label: Optional[str],
        goal_label: Optional[str],
        checkpoint_labels: Sequence[str],
    ) -> Route:
        """GetRoute相当のルートを生成する。"""
        if not self.blocks:
            raise RuntimeError("No blocks configuration loaded.")
        self._logger.info(
            f"Get route: start_label='{start_label}', goal_label='{goal_label}', checkpoints='{list(checkpoint_labels)}'"
        )
        route_wps: List[Waypoint] = []
        origins: List[WaypointOrigin] = []
        reversed_ranges: List[Tuple[int, int]] = []
        block_tail_indices: List[int] = []
        has_variable = False
        last_route_image: Optional[Image] = None
        for block in self.blocks:
            btype = block.get("type")
            bname = block.get("name")
            bindex = block.get("index", 0)
            if btype == "fixed":
                seg_id = block.get("segment_id")
                entry = self.segments.get(seg_id)
                if entry is None:
                    raise RuntimeError(f"[fixed:{bname}] segment not loaded: {seg_id}")
                before_len = len(route_wps)
                route_wps = concat_with_dedup(route_wps, entry.waypoints)
                added = len(route_wps) - before_len
                for _ in range(added):
                    origins.append(
                        WaypointOrigin(
                            block_name=str(bname),
                            block_index=int(bindex),
                            segment_id=seg_id,
                            edge_u=None,
                            edge_v=None,
                            index_in_edge=None,
                            u_first=None,
                        )
                    )
                block_tail_indices.append(len(route_wps) - 1)
            elif btype == "variable":
                has_variable = True
                merged_cps = list(
                    dict.fromkeys(
                        (block.get("checkpoints", []) or [])
                        + list(checkpoint_labels)
                    )
                )
                nodes = block.get("nodes") or []
                edges = block.get("edges") or []
                start = block.get("start")
                goal = block.get("goal")
                if not nodes or not edges:
                    raise RuntimeError(f"[variable:{bname}] nodes/edges not loaded.")
                if not start or not goal:
                    raise RuntimeError(f"[variable:{bname}] start/goal not set.")
                solve_result = solve_variable_route(
                    nodes=nodes,
                    edges=edges,
                    start=start,
                    goal=goal,
                    checkpoints=merged_cps,
                )
                edge_seq: List[Dict[str, Any]] = solve_result.get("edge_sequence", [])
                if not edge_seq:
                    raise RuntimeError(
                        f"[variable:{bname}] solver returned empty edge_sequence."
                    )
                image = load_route_image_from_solver(solve_result)
                if image is not None:
                    last_route_image = image
                for edge in edge_seq:
                    seg_id = edge["segment_id"]
                    direction = edge["direction"]
                    src = edge["source"]
                    dst = edge["target"]
                    entry = self.segments.get(seg_id)
                    if entry is None:
                        raise RuntimeError(
                            f"[variable:{bname}] segment not loaded: {seg_id}"
                        )
                    u_first = direction == "forward"
                    seg_wps = (
                        entry.waypoints if u_first else list(reversed(entry.waypoints))
                    )
                    stamp_edge_end_labels(seg_wps, src_label=src, dst_label=dst)
                    start_idx = len(route_wps)
                    route_wps = concat_with_dedup(route_wps, seg_wps)
                    end_idx = len(route_wps) - 1
                    for i_local in range(end_idx - start_idx + 1):
                        origins.append(
                            WaypointOrigin(
                                block_name=str(bname),
                                block_index=int(bindex),
                                segment_id=seg_id,
                                edge_u=src,
                                edge_v=dst,
                                index_in_edge=i_local,
                                u_first=u_first,
                            )
                        )
                    if not u_first:
                        reversed_ranges.append((start_idx, end_idx))
                block_tail_indices.append(len(route_wps) - 1)
            else:
                self._logger.warn(f"Unknown block type: {btype}")
        if not route_wps:
            raise RuntimeError("No waypoints generated from blocks configuration.")
        sliced, start_offset = slice_by_labels(route_wps, start_label or "", goal_label or "")
        origins = origins[start_offset : start_offset + len(sliced)]
        indexing(sliced)
        total_distance = calc_total_distance(sliced)

        def _shift_inside(r: Tuple[int, int], offset: int, length: int) -> Optional[Tuple[int, int]]:
            s = r[0] - offset
            e = r[1] - offset
            if e < 0 or s >= length:
                return None
            s = max(s, 0)
            e = min(e, length - 1)
            if s >= e:
                return None
            return (s, e)

        recalc_ranges: List[Tuple[int, int]] = []
        for rng in reversed_ranges:
            shifted = _shift_inside(rng, start_offset, len(sliced))
            if shifted is not None:
                recalc_ranges.append(shifted)
        shifted_block_tails: List[int] = []
        for idx in block_tail_indices:
            candidate = idx - start_offset
            if 0 <= candidate < len(sliced):
                shifted_block_tails.append(candidate)
        adjust_orientations(sliced, recalc_ranges, shifted_block_tails)
        if has_variable and last_route_image is not None:
            route_image = last_route_image
        elif has_variable:
            route_image = make_text_png_image("variable part image (placeholder)")
        else:
            route_image = make_text_png_image("No variable route in this plan")
        self.route_version = 1
        route = pack_route_msg(sliced, self.route_version, total_distance, route_image)
        self.current_route = route
        self.current_route_origins = origins
        self.last_request_checkpoints = set(checkpoint_labels)
        self.visited_checkpoints_hist.clear()
        self.closed_edges = set()
        self.current_block_name = None
        return route

    def update_route(
        self,
        route_version: int,
        prev_index: int,
        next_index: int,
        prev_wp_label: str,
        next_wp_label: str,
        block_name_hint: str,
        current_pose: PoseStamped,
    ) -> Route:
        """UpdateRoute相当の経路更新を実行する。"""
        if self.current_route is None or not self.current_route.waypoints:
            raise RuntimeError("No current route in server.")
        if route_version != self.current_route.version:
            raise RuntimeError("Route version mismatch.")
        wps = self.current_route.waypoints
        origins = self.current_route_origins
        n = len(wps)
        if not (0 <= prev_index < n - 1):
            raise RuntimeError("Invalid prev_index.")
        if next_index != prev_index + 1:
            raise RuntimeError("prev/next must be adjacent (next_index = prev_index + 1).")
        if wps[prev_index].label != prev_wp_label:
            raise RuntimeError("prev_wp_label mismatch current route.")
        if wps[next_index].label != next_wp_label:
            raise RuntimeError("next_wp_label mismatch current route.")
        prev_origin = origins[prev_index]
        next_origin = origins[next_index]
        if prev_origin.block_name != next_origin.block_name:
            raise RuntimeError("prev/next belong to different blocks.")
        block_name = prev_origin.block_name
        if block_name_hint and block_name_hint != block_name:
            self._logger.warn(
                f"Block name hint '{block_name_hint}' mismatches detected block '{block_name}'."
            )
        block_idx = prev_origin.block_index
        block = None
        for candidate in self.blocks:
            if candidate.get("name") == block_name and candidate.get("index") == block_idx:
                block = candidate
                break
        if block is None:
            raise RuntimeError(f"Block not found: {block_name}")
        if block.get("type") != "variable":
            self._logger.warn("Closure in fixed block (not reroutable).")
            raise RuntimeError("Closure in fixed block (not reroutable).")
        u_node = prev_origin.edge_u
        v_node = prev_origin.edge_v
        seg_id_running = prev_origin.segment_id
        u_first_flag = prev_origin.u_first
        local_idx_prev = prev_origin.index_in_edge
        if not (
            u_node
            and v_node
            and seg_id_running is not None
            and u_first_flag is not None
            and local_idx_prev is not None
        ):
            raise RuntimeError("Failed to identify running edge metadata for closure.")
        if self.current_block_name != block_name:
            self.closed_edges = set()
            self.current_block_name = block_name
        self.closed_edges.add(frozenset({u_node, v_node}))
        nodes, edges = self._build_graph_with_closures(block_name)
        yaml_cps = set(block.get("checkpoints", []))
        req_cps = set(self.last_request_checkpoints)
        node_ids = {nd["id"] for nd in (block.get("nodes") or [])}
        required = {c for c in (yaml_cps | req_cps) if c in node_ids}
        hist = self.visited_checkpoints_hist.setdefault(block_name, set())
        visited_now: Set[str] = set()
        for idx in range(0, prev_index + 1):
            label = wps[idx].label
            if label in required:
                visited_now.add(label)
        hist |= visited_now
        remaining_cps = list(required - hist)
        goal = block.get("goal")
        if not goal:
            raise RuntimeError("Block goal not set.")
        new_wps: List[Waypoint] = []
        new_origins: List[WaypointOrigin] = []
        new_wps.append(self._make_waypoint_from_pose(current_pose, label="current"))
        new_origins.append(
            WaypointOrigin(
                block_name=block_name,
                block_index=block_idx,
                segment_id=None,
                edge_u=None,
                edge_v=None,
                index_in_edge=None,
                u_first=None,
            )
        )
        virtual_wps = self._make_virtual_edge_waypoints(
            seg_id=str(seg_id_running),
            u_label=str(u_node),
            local_idx_prev=int(local_idx_prev),
            u_first_on_route=bool(u_first_flag),
            prev_wp=wps[prev_index],
            current_pose=current_pose,
        )
        if virtual_wps:
            stamp_edge_end_labels(virtual_wps, src_label="current", dst_label=str(u_node))
        start_idx_virtual = len(new_wps)
        new_wps = concat_with_dedup(new_wps, virtual_wps)
        end_idx_virtual = len(new_wps) - 1
        for i_local in range(end_idx_virtual - start_idx_virtual + 1):
            new_origins.append(
                WaypointOrigin(
                    block_name=block_name,
                    block_index=block_idx,
                    segment_id="__virtual__",
                    edge_u=u_node,
                    edge_v=v_node,
                    index_in_edge=i_local,
                    u_first=True,
                )
            )
        recalc_ranges: List[Tuple[int, int]] = []
        if end_idx_virtual > start_idx_virtual:
            recalc_ranges.append((start_idx_virtual, end_idx_virtual))
        block_tail_indices: List[int] = []
        result = solve_variable_route(
            nodes=nodes,
            edges=edges,
            start=u_node,
            goal=goal,
            checkpoints=remaining_cps,
        )
        edge_seq: List[Dict[str, Any]] = result.get("edge_sequence", [])
        if not edge_seq:
            raise RuntimeError("No route found after applying closures.")
        img_solver = load_route_image_from_solver(result)
        for edge in edge_seq:
            if edge.get("segment_id") == "__virtual__":
                continue
            seg_id2 = edge["segment_id"]
            direction2 = edge["direction"]
            src2 = edge["source"]
            dst2 = edge["target"]
            entry2 = self.segments.get(seg_id2)
            if entry2 is None:
                raise RuntimeError(f"Segment not loaded: {seg_id2}")
            u_first2 = direction2 == "forward"
            seg_wps2 = (
                entry2.waypoints if u_first2 else list(reversed(entry2.waypoints))
            )
            stamp_edge_end_labels(seg_wps2, src_label=src2, dst_label=dst2)
            start_idx2 = len(new_wps)
            new_wps = concat_with_dedup(new_wps, seg_wps2)
            end_idx2 = len(new_wps) - 1
            for i_local in range(end_idx2 - start_idx2 + 1):
                new_origins.append(
                    WaypointOrigin(
                        block_name=block_name,
                        block_index=block_idx,
                        segment_id=seg_id2,
                        edge_u=src2,
                        edge_v=dst2,
                        index_in_edge=i_local,
                        u_first=u_first2,
                    )
                )
            if not u_first2 and end_idx2 > start_idx2:
                recalc_ranges.append((start_idx2, end_idx2))
        block_tail_indices.append(len(new_wps) - 1)
        for block_next in self.blocks:
            if block_next.get("index") <= block_idx:
                continue
            bname2 = block_next.get("name")
            bidx2 = block_next.get("index")
            if block_next.get("type") == "fixed":
                seg_id3 = block_next.get("segment_id")
                entry3 = self.segments.get(seg_id3)
                if entry3 is None:
                    raise RuntimeError(
                        f"[fixed:{bname2}] segment not loaded: {seg_id3}"
                    )
                before = len(new_wps)
                new_wps = concat_with_dedup(new_wps, entry3.waypoints)
                for _ in range(len(new_wps) - before):
                    new_origins.append(
                        WaypointOrigin(
                            block_name=str(bname2),
                            block_index=int(bidx2),
                            segment_id=seg_id3,
                            edge_u=None,
                            edge_v=None,
                            index_in_edge=None,
                            u_first=None,
                        )
                    )
                block_tail_indices.append(len(new_wps) - 1)
            elif block_next.get("type") == "variable":
                nodes2 = block_next.get("nodes") or []
                edges2 = block_next.get("edges") or []
                start2 = block_next.get("start")
                goal2 = block_next.get("goal")
                if not nodes2 or not edges2 or not start2 or not goal2:
                    raise RuntimeError(
                        f"[variable:{bname2}] definition incomplete."
                    )
                merged_cps2 = list(
                    dict.fromkeys(
                        (block_next.get("checkpoints", []) or [])
                        + list(self.last_request_checkpoints)
                    )
                )
                node_ids2 = {nd["id"] for nd in nodes2}
                cps2 = [c for c in merged_cps2 if c in node_ids2]
                res2 = solve_variable_route(
                    nodes=nodes2,
                    edges=edges2,
                    start=start2,
                    goal=goal2,
                    checkpoints=cps2,
                )
                img_extra = load_route_image_from_solver(res2)
                if img_extra is not None and img_solver is None:
                    img_solver = img_extra
                es2: List[Dict[str, Any]] = res2.get("edge_sequence", [])
                if not es2:
                    raise RuntimeError(
                        f"[variable:{bname2}] solver returned empty edge_sequence."
                    )
                for edge2 in es2:
                    seg_id4 = edge2["segment_id"]
                    direction4 = edge2["direction"]
                    src4 = edge2["source"]
                    dst4 = edge2["target"]
                    entry4 = self.segments.get(seg_id4)
                    if entry4 is None:
                        raise RuntimeError(
                            f"[variable:{bname2}] segment not loaded: {seg_id4}"
                        )
                    u_first4 = direction4 == "forward"
                    seg_wps4 = (
                        entry4.waypoints if u_first4 else list(reversed(entry4.waypoints))
                    )
                    stamp_edge_end_labels(seg_wps4, src_label=src4, dst_label=dst4)
                    start_idx4 = len(new_wps)
                    new_wps = concat_with_dedup(new_wps, seg_wps4)
                    end_idx4 = len(new_wps) - 1
                    for i_local in range(end_idx4 - start_idx4 + 1):
                        new_origins.append(
                            WaypointOrigin(
                                block_name=str(bname2),
                                block_index=int(bidx2),
                                segment_id=seg_id4,
                                edge_u=src4,
                                edge_v=dst4,
                                index_in_edge=i_local,
                                u_first=u_first4,
                            )
                        )
                    if not u_first4 and end_idx4 > start_idx4:
                        recalc_ranges.append((start_idx4, end_idx4))
                block_tail_indices.append(len(new_wps) - 1)
        adjust_orientations(new_wps, recalc_ranges, block_tail_indices)
        indexing(new_wps)
        total_distance = calc_total_distance(new_wps)
        route_image = (
            img_solver
            if img_solver is not None
            else make_text_png_image("variable part image (placeholder)")
        )
        self.route_version += 1
        new_route = pack_route_msg(
            new_wps,
            self.route_version,
            total_distance,
            route_image,
        )
        self.current_route = new_route
        self.current_route_origins = new_origins
        return new_route

    def _build_graph_with_closures(
        self, block_name: str
    ) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """累積封鎖を適用したnodes/edgesを返す。"""
        block = None
        for candidate in self.blocks:
            if candidate.get("name") == block_name:
                block = candidate
                break
        if block is None or block.get("type") != "variable":
            raise RuntimeError("Block not found or not variable.")
        nodes = list(block.get("nodes") or [])
        raw_edges = list(block.get("edges") or [])
        filtered: List[Dict[str, Any]] = []
        for edge in raw_edges:
            u = str(edge["source"])
            v = str(edge["target"])
            if frozenset({u, v}) in self.closed_edges:
                continue
            filtered.append(dict(edge))
        return nodes, filtered

    def _make_waypoint_from_pose(self, pose_stamped: PoseStamped, label: str) -> Waypoint:
        """PoseStampedからWaypointを生成する。"""
        wp = Waypoint()
        wp.label = label
        wp.index = 0
        wp.pose = _copy_pose(pose_stamped.pose)
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
        """仮想エッジ current→prev→U の waypoint 群を生成する。"""
        entry = self.segments.get(seg_id)
        if entry is None:
            raise RuntimeError(f"Virtual edge source segment not loaded: {seg_id}")
        base = entry.waypoints
        if u_first_on_route:
            seg_u_first = list(base)
            prev_idx_u_first = int(local_idx_prev)
        else:
            seg_u_first = list(reversed(base))
            prev_idx_u_first = (len(seg_u_first) - 1) - int(local_idx_prev)
        if not (0 <= prev_idx_u_first < len(seg_u_first)):
            raise RuntimeError("Invalid prev index in virtual edge generation.")
        virtual: List[Waypoint] = []
        current_wp = self._make_waypoint_from_pose(current_pose, label="current")
        virtual.append(current_wp)
        prev_copy = Waypoint()
        prev_copy.label = prev_wp.label
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
        for idx in range(prev_idx_u_first - 1, -1, -1):
            src = seg_u_first[idx]
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
