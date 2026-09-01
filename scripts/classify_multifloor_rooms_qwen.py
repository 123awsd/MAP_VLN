#!/usr/bin/env python3
"""Build and semantically label a transition-aware multi-floor scene graph.

Room geometry stays owned by OccuSG.  Reconstructed transitions deterministically
define stair spaces.  Qwen sees only ordinary rooms and may only assign a label.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors as mpl_colors

from classify_room_semantics_qwen import QwenRoomSemanticClassifier, atomic_json


ROOT = Path(__file__).resolve().parents[1]
SEMANTIC_COLORS = {
    "bedroom": "#4e79a7", "living_room": "#59a14f", "kitchen": "#f28e2b",
    "dining_room": "#edc948", "bathroom": "#76b7b2", "office": "#b07aa1",
    "laundry_utility": "#9c755f", "entry_hall": "#bab0ac",
    "corridor": "#86bcb6", "unknown": "#c7c7c7", "stairwell": "#e15759",
}

# Presentation-only styling. These values affect exported figures only; the
# occupancy grid and room polygons written to disk remain unchanged.
WALL_RGB = (28, 28, 28)
FREE_RGB = (224, 224, 224)
UNKNOWN_RGB = (255, 255, 255)


def polygon_gap(first: list[list[float]], second: list[list[float]]) -> float:
    """Return zero for touching/overlapping polygons, otherwise boundary gap."""
    a = np.asarray(first, dtype=np.float32).reshape(-1, 1, 2)
    b = np.asarray(second, dtype=np.float32).reshape(-1, 1, 2)
    distances = []
    for point in a[:, 0, :]:
        signed = cv2.pointPolygonTest(b, (float(point[0]), float(point[1])), True)
        if signed >= 0:
            return 0.0
        distances.append(-float(signed))
    for point in b[:, 0, :]:
        signed = cv2.pointPolygonTest(a, (float(point[0]), float(point[1])), True)
        if signed >= 0:
            return 0.0
        distances.append(-float(signed))
    return min(distances, default=math.inf)


def overlay_transition_roles(graph: dict, transition_data: dict, floor: int) -> list[dict]:
    """Lock reconstructed transition regions before semantic classification."""
    links = []
    source_to_transition = {}
    for transition in transition_data.get("transition_spaces", []):
        for region_id in transition.get("source_region_ids", []):
            source_to_transition.setdefault(int(region_id), []).append(transition)
    for room in graph.get("rooms", []):
        source_ids = {int(value) for value in room.get("merged_from_region_ids", [room["id"]])}
        matches = []
        for source_id in source_ids:
            matches.extend(source_to_transition.get(source_id, []))
        unique = {item["id"]: item for item in matches}
        room["space_role"] = "room"
        if not unique:
            continue
        room["space_role"] = "transition_space"
        room["semantic_type"] = "stairwell"
        room["semantic_score"] = max(float(item.get("confidence", 0.0)) for item in unique.values())
        room["transition_ids"] = sorted(unique)
        for transition in unique.values():
            links.append({
                "transition_id": transition["id"], "floor": floor,
                "room_id": int(room["id"]), "polygon_xy_m": room["polygon_xy_m"],
                "connected_floors": transition.get("connected_floors", []),
                "confidence": float(transition.get("confidence", 0.0)),
            })
    return links


def group_stairwells(links: list[dict], maximum_shared_floor_gap_m: float) -> list[dict]:
    """Join consecutive transitions only when their spaces meet on a shared floor."""
    transition_ids = sorted({item["transition_id"] for item in links})
    parent = {item: item for item in transition_ids}

    def find(item):
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    def union(first, second):
        first, second = find(first), find(second)
        if first != second:
            parent[second] = first

    for index, first in enumerate(links):
        for second in links[index + 1:]:
            if first["transition_id"] == second["transition_id"] or first["floor"] != second["floor"]:
                continue
            if polygon_gap(first["polygon_xy_m"], second["polygon_xy_m"]) <= maximum_shared_floor_gap_m:
                union(first["transition_id"], second["transition_id"])

    groups = {}
    for transition_id in transition_ids:
        groups.setdefault(find(transition_id), []).append(transition_id)
    stairwells = []
    for index, members in enumerate(sorted(groups.values()), start=1):
        member_set = set(members)
        local_links = [item for item in links if item["transition_id"] in member_set]
        floor_regions = {}
        for item in local_links:
            floor_regions.setdefault(str(item["floor"]), []).append(item["room_id"])
        stairwells.append({
            "id": f"stairwell_{index}", "semantic_type": "stairwell",
            "transition_ids": sorted(members),
            "floors": sorted({floor for item in local_links for floor in item["connected_floors"]}),
            "floor_region_ids": {key: sorted(set(value)) for key, value in sorted(floor_regions.items())},
            "confidence": min((item["confidence"] for item in local_links), default=0.0),
            "geometry_note": "One logical stairwell; per-floor polygons remain separate observed components.",
        })
    return stairwells


def apply_qwen(graph: dict, classifier: QwenRoomSemanticClassifier) -> tuple[list[dict], dict]:
    ordinary = [room for room in graph.get("rooms", []) if room.get("space_role") != "transition_space"]
    if not ordinary:
        return [], {"classifier": "qwen_llm", "skipped": True, "reason": "no ordinary rooms"}
    decisions, provenance = classifier.classify(ordinary)
    by_id = {int(item["room_id"]): item for item in decisions}
    for room in ordinary:
        decision = by_id[int(room["id"])]
        room["rule_semantic_type"] = room.get("semantic_type", "unknown")
        room["qwen_semantic"] = decision
        room["qwen_semantic_type"] = decision["room_type"]
        room["qwen_semantic_score"] = decision["confidence"]
        room["semantic_type"] = decision["room_type"] if decision["accepted"] else "unknown"
        room["semantic_score"] = decision["confidence"]
    return decisions, provenance


def grid_rgb(grid: np.ndarray) -> np.ndarray:
    image = np.zeros((*grid.shape, 3), dtype=np.uint8)
    image[grid < 0] = UNKNOWN_RGB
    image[grid == 0] = FREE_RGB
    image[grid == 100] = WALL_RGB
    return image


def grid_extent(grid: np.ndarray, metadata: dict) -> list[float]:
    origin = np.asarray(metadata["origin_xy_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    return [
        float(origin[0]), float(origin[0] + grid.shape[1] * resolution),
        float(origin[1]), float(origin[1] + grid.shape[0] * resolution),
    ]


def draw_semantic_floor(
    axis, floor: int, graph: dict, grid: np.ndarray, metadata: dict,
    show_labels: bool = True,
) -> None:
    axis.imshow(
        grid_rgb(grid), origin="lower", extent=grid_extent(grid, metadata),
        interpolation="nearest", zorder=0,
    )
    room_overlay = np.zeros((*grid.shape, 4), dtype=float)
    origin = np.asarray(metadata["origin_xy_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    free_visual_mask = cv2.dilate(
        (grid == 0).astype(np.uint8), np.ones((5, 5), dtype=np.uint8), iterations=1
    ).astype(bool)
    for room in graph.get("rooms", []):
        polygon = np.asarray(room.get("polygon_xy_m", []), dtype=float)
        if len(polygon) < 3:
            continue
        label = room.get("semantic_type", "unknown")
        room_color = SEMANTIC_COLORS.get(label, "#c7c7c7")
        pixel_polygon = np.round((polygon - origin) / resolution).astype(np.int32)
        room_mask = np.zeros(grid.shape, dtype=np.uint8)
        cv2.fillPoly(room_mask, [pixel_polygon], 1)
        # Fill the small rasterization gap between the OccuSG contour and the
        # wall grid. The wall overlay below remains authoritative visually.
        room_mask = cv2.dilate(room_mask, np.ones((7, 7), dtype=np.uint8), iterations=1)
        # Clip only the rendered color to observed FREE space. This removes
        # polygons that spill into exterior UNKNOWN cells without changing the
        # source polygon used by room assignment or planning.
        room_mask = room_mask.astype(bool) & free_visual_mask
        rgb = mpl_colors.to_rgb(room_color)
        room_overlay[room_mask > 0, :3] = rgb
        room_overlay[room_mask > 0, 3] = .72
    axis.imshow(
        room_overlay, origin="lower", extent=grid_extent(grid, metadata),
        interpolation="nearest", zorder=2.2,
    )
    # Keep presentation overlays inside the observed wall structure. Some
    # OccuSG polygons slightly cross an occupied raster cell; redraw those
    # wall cells above the fill so a semantic color cannot paint over a wall.
    # This affects only the figure, never the stored polygons.
    structure_overlay = np.zeros((*grid.shape, 4), dtype=float)
    unknown_mask = (grid < 0).astype(np.uint8)
    component_count, component_ids = cv2.connectedComponents(unknown_mask, connectivity=4)
    outside_ids = set(np.unique(np.concatenate((
        component_ids[0, :], component_ids[-1, :],
        component_ids[:, 0], component_ids[:, -1],
    ))).tolist())
    outside_unknown = np.isin(component_ids, list(outside_ids)) & (grid < 0)
    structure_overlay[outside_unknown, :3] = 1.0
    structure_overlay[outside_unknown, 3] = 1.0
    structure_overlay[grid == 100, :3] = np.asarray(WALL_RGB, dtype=float) / 255.0
    structure_overlay[grid == 100, 3] = 1.0
    axis.imshow(
        structure_overlay, origin="lower", extent=grid_extent(grid, metadata),
        interpolation="nearest", zorder=2.5,
    )
    if show_labels:
        for room in graph.get("rooms", []):
            polygon = np.asarray(room.get("polygon_xy_m", []), dtype=float)
            if len(polygon) < 3:
                continue
            label = room.get("semantic_type", "unknown")
            center = room.get("centroid_xy_m", np.mean(polygon, axis=0))
            prefix = room.get("stairwell_id", f'R{room["id"]}')
            axis.text(
                center[0], center[1], f"{prefix}\n{label}", ha="center", va="center",
                fontsize=8, color="#101010", zorder=3,
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": .58, "pad": 1.0},
            )
    if not show_labels:
        axis.axis("off")
        return
    axis.set_aspect("equal"); axis.grid(False)
    axis.set_title(f"L{floor} | wall grid + Qwen room semantics")
    axis.set_xlabel("FALCON X (m)"); axis.set_ylabel("FALCON Y (m)")


def render_semantics(
    path: Path, floor_graphs: dict[int, dict], floor_grids: dict[int, tuple[np.ndarray, dict]],
) -> None:
    figure, axes = plt.subplots(1, len(floor_graphs), figsize=(8 * len(floor_graphs), 8), squeeze=False)
    for axis, (floor, graph) in zip(axes[0], sorted(floor_graphs.items())):
        grid, metadata = floor_grids[floor]
        draw_semantic_floor(axis, floor, graph, grid, metadata)
    figure.tight_layout()
    figure.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("transition_room_dir", type=Path)
    parser.add_argument("floor_boxes_dir", type=Path)
    parser.add_argument("wall_grid_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-floor", type=int, default=3)
    parser.add_argument("--budget-cny", type=float, default=20.0)
    parser.add_argument("--confidence-threshold", type=float, default=0.65)
    parser.add_argument("--stairwell-join-gap", type=float, default=0.30)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    classifier = QwenRoomSemanticClassifier(args.budget_cny, args.confidence_threshold)
    graphs, floor_grids, all_links, floor_semantics = {}, {}, [], []
    for floor in range(1, args.max_floor + 1):
        raw = args.transition_room_dir / f"L{floor}_regions_raw.json"
        aware = args.transition_room_dir / f"L{floor}_regions_transition_aware.json"
        boxes = args.floor_boxes_dir / f"L{floor}_boxes.csv"
        grid_meta = args.wall_grid_dir / f"L{floor}_wall_grid.json"
        grid_path = args.wall_grid_dir / f"L{floor}_wall_grid.npy"
        for required in (raw, aware, boxes, grid_meta, grid_path):
            if not required.is_file():
                raise FileNotFoundError(required)
        floor_grids[floor] = (
            np.load(grid_path), json.loads(grid_meta.read_text(encoding="utf-8")),
        )
        graph_path = args.output / f"L{floor}_scene_graph_geometry.json"
        subprocess.run([
            sys.executable, str(ROOT / "scripts/fuse_rooms_boxes.py"),
            str(raw), str(boxes), str(graph_path), "--grid-meta", str(grid_meta),
        ], check=True)
        graph = json.loads(graph_path.read_text(encoding="utf-8"))
        transition_data = json.loads(aware.read_text(encoding="utf-8"))
        all_links.extend(overlay_transition_roles(graph, transition_data, floor))
        decisions, provenance = apply_qwen(graph, classifier)
        graph["floor_id"] = floor
        graph["room_semantics"] = {
            "format": "pre_map_vln.room_semantics_qwen.v2",
            "provenance": provenance, "decisions": decisions,
            "note": "Qwen labels ordinary OccuSG rooms only; transition geometry is locked before inference.",
        }
        graphs[floor] = graph
        floor_semantics.append({"floor": floor, "decisions": decisions, "provenance": provenance})

    stairwells = group_stairwells(all_links, args.stairwell_join_gap)
    stairwell_by_transition = {
        transition_id: stairwell["id"] for stairwell in stairwells
        for transition_id in stairwell["transition_ids"]
    }
    for graph in graphs.values():
        for room in graph["rooms"]:
            ids = room.get("transition_ids", [])
            if ids:
                room["stairwell_id"] = stairwell_by_transition[ids[0]]
        graph["stairwells"] = stairwells

    for floor, graph in graphs.items():
        atomic_json(args.output / f"L{floor}_scene_graph_qwen.json", graph)
        figure, axis = plt.subplots(1, 1, figsize=(9, 9))
        draw_semantic_floor(axis, floor, graph, *floor_grids[floor])
        figure.tight_layout()
        figure.savefig(args.output / f"L{floor}_wall_room_semantics.png", dpi=180, bbox_inches="tight")
        plt.close(figure)
    atomic_json(args.output / "stairwells.json", {
        "format": "pre_map_vln.stairwells.v1", "stairwells": stairwells,
    })
    atomic_json(args.output / "multifloor_scene_graph_qwen.json", {
        "format": "pre_map_vln.multifloor_scene_graph.v1",
        "floors": [graphs[floor] for floor in sorted(graphs)],
        "stairwells": stairwells, "qwen_floor_semantics": floor_semantics,
    })
    render_semantics(args.output / "all_floors_qwen_semantics.png", graphs, floor_grids)
    print(json.dumps({
        "output": str(args.output), "floors": len(graphs), "stairwells": stairwells,
        "semantic_figure": str(args.output / "all_floors_qwen_semantics.png"),
        "labels": {
            f"L{floor}": {"R{}".format(room["id"]): room["semantic_type"] for room in graph["rooms"]}
            for floor, graph in graphs.items()
        },
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
