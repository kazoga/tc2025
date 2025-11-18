"""route_builderのスライス処理に関するテスト."""

import sys
from pathlib import Path
import types

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if "yaml" not in sys.modules:
    yaml_stub = types.ModuleType("yaml")
    yaml_stub.safe_load = lambda _: {}
    sys.modules["yaml"] = yaml_stub

if "matplotlib" not in sys.modules:
    matplotlib_stub = types.ModuleType("matplotlib")
    pyplot_stub = types.ModuleType("matplotlib.pyplot")
    matplotlib_stub.pyplot = pyplot_stub
    sys.modules["matplotlib"] = matplotlib_stub
    sys.modules["matplotlib.pyplot"] = pyplot_stub

for optional_module in ("japanize_matplotlib", "networkx", "numpy"):
    if optional_module not in sys.modules:
        sys.modules[optional_module] = types.ModuleType(optional_module)

if "PIL" not in sys.modules:
    pil_stub = types.ModuleType("PIL")
    pil_image_stub = types.ModuleType("PIL.Image")
    pil_image_draw_stub = types.ModuleType("PIL.ImageDraw")
    pil_stub.Image = pil_image_stub
    pil_stub.ImageDraw = pil_image_draw_stub
    sys.modules["PIL"] = pil_stub
    sys.modules["PIL.Image"] = pil_image_stub
    sys.modules["PIL.ImageDraw"] = pil_image_draw_stub

from route_planner.route_builder import (
    PositionRecord,
    PoseRecord,
    SegmentRecord,
    WaypointRecord,
    RouteBuilder,
    slice_by_labels_records,
)


def _make_wp(label: str, x: float) -> WaypointRecord:
    wp = WaypointRecord(label=label, index=0)
    wp.pose = PoseRecord(position=PositionRecord(x=x, y=0.0, z=0.0))
    return wp


def test_slice_by_labels_records_uses_hint() -> None:
    """start_index_hintが指定された場合に優先されることを確認する."""

    waypoints = [
        _make_wp("A", 0.0),
        _make_wp("B", 1.0),
        _make_wp("A", 2.0),
        _make_wp("C", 3.0),
    ]

    sliced_default, offset_default = slice_by_labels_records(waypoints, "A", "C")
    assert offset_default == 0
    assert [wp.label for wp in sliced_default] == ["A", "B", "A", "C"]

    sliced_hint, offset_hint = slice_by_labels_records(
        waypoints,
        "A",
        "C",
        start_index_hint=2,
    )
    assert offset_hint == 2
    assert [wp.label for wp in sliced_hint] == ["A", "C"]


def test_build_route_prefers_block_local_start_label() -> None:
    """RouteBuilder.build_routeがブロック情報を用いて適切なスライス開始位置を選択する."""

    builder = RouteBuilder(config_yaml_path="")
    builder.blocks = [
        {
            "type": "fixed",
            "name": "entry",
            "segment_id": "seg_entry",
            "index": 0,
        },
        {
            "type": "fixed",
            "name": "reroute",
            "segment_id": "seg_reroute",
            "index": 1,
        },
    ]

    entry_segment = [
        _make_wp("C1", 0.0),
        _make_wp("S", 1.0),
    ]
    reroute_segment = [
        _make_wp("C1", 2.0),
        _make_wp("X", 3.0),
        _make_wp("Goal", 4.0),
    ]

    builder.segments = {
        "seg_entry": SegmentRecord(segment_id="seg_entry", waypoints=entry_segment),
        "seg_reroute": SegmentRecord(segment_id="seg_reroute", waypoints=reroute_segment),
    }

    result = builder.build_route(
        start_label="C1",
        goal_label="Goal",
        checkpoint_labels=[],
        start_label_origin=("reroute", 1),
    )

    labels = [wp.label for wp in result.waypoints]
    assert labels == ["C1", "X", "Goal"]
