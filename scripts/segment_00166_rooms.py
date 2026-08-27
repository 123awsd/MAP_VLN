#!/usr/bin/env python3
"""Infer room regions for the single 00166 stage-1 episode.

The HM3D semantic asset contains object-to-region membership but does not
contain usable room polygons.  This diagnostic therefore projects pixels whose
semantic instance is a labelled floor object from the recorded RGB-D frames
into the FALCON XY frame.  The resulting regions are useful for visualization
and downstream analysis, but are explicitly not fed back into the explorer and
are not claimed to be human-authored room ground truth.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch


ROOT = Path(__file__).resolve().parents[1]
SCENE_TOKEN = "00166-RaYrxWt5pR1"
DEFAULT_EPISODE = ROOT / "outputs/stage1_00166/ground_v5/episode"
DEFAULT_LABELS = Path(
    "/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train/"
    "00166-RaYrxWt5pR1/RaYrxWt5pR1.semantic.txt"
)
DEFAULT_TRUTH = ROOT / "outputs/stage1_00166/ground_v5/truth_metrics.npz"
DEFAULT_OUTPUT = ROOT / "outputs/stage1_00166/ground_v5/room_segmentation"


REGION_NAMES = {
    0: ("入口/过道", "entry_hall"),
    1: ("厨房/餐区", "kitchen_dining"),
    2: ("洗衣/杂物间", "laundry_utility"),
    3: ("卫生间 A", "bathroom_a"),
    4: ("卧室 A", "bedroom_a"),
    5: ("客厅", "living_room"),
    6: ("卧室 B", "bedroom_b"),
    7: ("卫生间 B", "bathroom_b"),
}

ROOM_CUES = {
    "kitchen_dining": {
        "refrigerator", "stove", "microwave", "kitchen top", "kitchen island",
        "sink", "coffee machine", "kettle", "plate",
    },
    "laundry_utility": {"washing machine", "clothes dryer"},
    "bathroom": {"toilet", "bathtub", "shower cabin", "shower", "sink"},
    "bedroom": {"bed", "wardrobe", "dresser", "bed table", "pillow"},
    "living_room": {"tv", "couch", "armchair", "coffee table", "fireplace"},
}

PALETTE = [
    "#f2a65a", "#e76f51", "#8ecae6", "#457b9d",
    "#8e6bbf", "#2a9d8f", "#6a994e", "#d1495b",
]
PLOT_NAMES = {
    "entry_hall": "entry / hall",
    "kitchen_dining": "kitchen / dining",
    "laundry_utility": "laundry / utility",
    "bathroom_a": "bathroom A",
    "bathroom_b": "bathroom B",
    "bathroom": "bathroom",
    "bedroom_a": "bedroom A",
    "bedroom_b": "bedroom B",
    "bedroom": "bedroom",
    "living_room": "living room",
    "structural_unknown": "structural region",
}


def load_labels(path: Path) -> tuple[dict[int, tuple[str, int]], dict[int, list[int]], dict[int, Counter]]:
    labels: dict[int, tuple[str, int]] = {}
    floor_ids: dict[int, list[int]] = defaultdict(list)
    object_counts: dict[int, Counter] = defaultdict(Counter)
    with path.open(newline="", encoding="utf-8") as handle:
        next(handle)
        for row in csv.reader(handle):
            if len(row) < 4:
                continue
            try:
                object_id = int(row[0])
                region_id = int(row[3])
            except ValueError:
                continue
            label = row[2].strip().lower()
            labels[object_id] = (label, region_id)
            object_counts[region_id][label] += 1
            if label == "floor":
                floor_ids[region_id].append(object_id)
    return labels, dict(floor_ids), dict(object_counts)


def quat_to_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = [float(value) for value in quaternion]
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-8:
        raise ValueError("zero-norm camera quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def classify_region(labels: Counter) -> tuple[str, str, list[str]]:
    scores = {}
    for room_type, cues in ROOM_CUES.items():
        scores[room_type] = sum(float(labels[label]) for label in cues)
    if not scores or max(scores.values()) <= 0:
        return "结构区域（待确认）", "structural_unknown", []
    room_type = max(scores, key=scores.get)
    cue_labels = sorted(label for label in ROOM_CUES[room_type] if labels[label])
    display = room_type
    if room_type == "bathroom":
        display = "卫生间（语义推断）"
    elif room_type == "bedroom":
        display = "卧室（语义推断）"
    elif room_type == "living_room":
        display = "客厅（语义推断）"
    elif room_type == "kitchen_dining":
        display = "厨房/餐区（语义推断）"
    elif room_type == "laundry_utility":
        display = "洗衣/杂物间（语义推断）"
    return display, room_type, cue_labels


def frame_files(episode_dir: Path) -> list[Path]:
    files = sorted(episode_dir.glob("frame_*.npz"))
    if not files:
        raise FileNotFoundError(f"no recorded frames in {episode_dir}")
    return files


def make_plot(
    output: Path,
    region_map: np.ndarray,
    observed: np.ndarray,
    meta: dict,
    regions: list[dict],
    trajectory: np.ndarray,
    truth: np.ndarray | None,
) -> None:
    resolution = float(meta["resolution_m"])
    origin = np.asarray(meta["origin_xy_m"], dtype=float)
    height, width = region_map.shape
    extent = [
        float(origin[0]), float(origin[0] + width * resolution),
        float(origin[1]), float(origin[1] + height * resolution),
    ]

    base = np.full(observed.shape, 0.0, dtype=float)
    base[observed == 0] = 1.0
    base[observed == 100] = 2.0
    base_cmap = ListedColormap(["#d2d2d2", "#ffffff", "#202020"])

    room_ids = sorted(region["region_id"] for region in regions)
    room_index = {region_id: index for index, region_id in enumerate(room_ids)}
    display = np.full(region_map.shape, -1, dtype=np.int16)
    for region_id, index in room_index.items():
        display[region_map == region_id] = index
    room_cmap = ListedColormap([PALETTE[region_id % len(PALETTE)] for region_id in room_ids])
    masked_display = np.ma.masked_where(display < 0, display)

    fig, ax = plt.subplots(figsize=(13, 10))
    ax.imshow(base, origin="lower", extent=extent, interpolation="nearest", cmap=base_cmap)
    ax.imshow(
        masked_display, origin="lower", extent=extent, interpolation="nearest",
        cmap=room_cmap, alpha=0.62, vmin=-0.5, vmax=max(0.5, len(room_ids) - 0.5),
    )
    if truth is not None:
        ax.contour(
            truth.astype(float), levels=[0.5], origin="lower", extent=extent,
            colors="#b2182b", linewidths=0.8, linestyles="--",
        )
    if len(trajectory):
        ax.plot(trajectory[:, 0], trajectory[:, 1], color="#1d3557", linewidth=0.65, alpha=0.8)

    handles = []
    for region in regions:
        centroid = region["centroid_xy_m"]
        region_id = int(region["region_id"])
        color = PALETTE[region_id % len(PALETTE)]
        plot_name = PLOT_NAMES.get(region["semantic_type"], region["semantic_type"])
        ax.text(
            centroid[0], centroid[1], f"R{region_id} {plot_name}",
            fontsize=8, ha="center", va="center", color="#111111",
            bbox={"boxstyle": "round,pad=0.22", "fc": "white", "ec": color, "alpha": 0.88},
        )
        handles.append(Patch(facecolor=color, alpha=0.62, label=f"R{region_id} {plot_name}"))
    handles.append(Patch(facecolor="#b2182b", alpha=0.0, edgecolor="#b2182b", label="red dashed: Habitat main-floor navmesh"))
    handles.append(Patch(facecolor="#1d3557", alpha=0.75, label="blue: executed trajectory"))
    ax.legend(handles=handles, loc="upper left", fontsize=8, framealpha=0.9)
    ax.set_aspect("equal")
    ax.set_xlabel("FALCON X (m)")
    ax.set_ylabel("FALCON Y (m)")
    ax.set_title(
        "00166 room segmentation (semantic floor projection)\n"
        "Inferred semantic regions; not official room polygons and not used for planning",
        fontsize=13,
    )
    ax.grid(alpha=0.12)
    fig.tight_layout()
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-dir", type=Path, default=DEFAULT_EPISODE)
    parser.add_argument("--semantic-labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--observed-grid", type=Path, default=None)
    parser.add_argument("--truth", type=Path, default=DEFAULT_TRUTH)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--pixel-stride", type=int, default=1)
    parser.add_argument("--floor-z-band", type=float, default=0.25)
    parser.add_argument("--max-depth", type=float, default=10.0)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.pixel_stride < 1:
        raise ValueError("--pixel-stride must be >= 1")
    if SCENE_TOKEN not in str(args.episode_dir) or SCENE_TOKEN not in str(args.semantic_labels):
        # The episode directory is named stage1_00166, so also accept the
        # checked-in 00166 directory while rejecting all other scene assets.
        if "stage1_00166" not in str(args.episode_dir):
            raise ValueError("this diagnostic is restricted to HM3D scene 00166-RaYrxWt5pR1")
    manifest_path = args.episode_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if SCENE_TOKEN not in json.dumps(manifest, ensure_ascii=False):
        raise ValueError("episode manifest is not scene 00166-RaYrxWt5pR1")
    labels, floor_ids_by_region, object_counts = load_labels(args.semantic_labels)
    region_ids = sorted(region_id for region_id in floor_ids_by_region if region_id in REGION_NAMES)
    if not region_ids:
        raise RuntimeError("semantic labels contain no usable floor regions")
    region_index = {region_id: index for index, region_id in enumerate(region_ids)}
    floor_to_region = {
        object_id: region_id
        for region_id, object_ids in floor_ids_by_region.items()
        for object_id in object_ids
    }

    observed_path = args.observed_grid or args.episode_dir.parent / "observed_ground.npy"
    observed = np.load(observed_path).astype(np.int8, copy=False)
    meta = json.loads(observed_path.with_suffix(".json").read_text(encoding="utf-8"))
    height, width = observed.shape
    votes = np.zeros((len(region_ids), height, width), dtype=np.uint32)
    point_counts = Counter()
    frame_count = 0
    floor_point_count = 0
    z_values = []
    fx = float(manifest.get("fx", 320.0))
    fy = float(manifest.get("fy", 320.0))
    cx = float(manifest.get("cx", 320.0))
    cy = float(manifest.get("cy", 240.0))
    origin = np.asarray(meta["origin_xy_m"], dtype=float)
    resolution = float(meta["resolution_m"])

    for path in frame_files(args.episode_dir):
        data = np.load(path)
        semantic = np.asarray(data["semantic"], dtype=np.int32)
        depth = np.asarray(data["depth_m"], dtype=np.float32)
        mask = np.isin(semantic, list(floor_to_region))
        mask &= np.isfinite(depth) & (depth > 0.1) & (depth < args.max_depth)
        yy, xx = np.nonzero(mask)
        if args.pixel_stride > 1:
            yy, xx = yy[::args.pixel_stride], xx[::args.pixel_stride]
        if not len(yy):
            continue
        distance = depth[yy, xx]
        camera_points = np.column_stack([
            (xx - cx) * distance / fx,
            (yy - cy) * distance / fy,
            distance,
        ])
        rotation = quat_to_matrix_xyzw(np.asarray(data["orientation_xyzw"], dtype=float))
        world_points = camera_points @ rotation.T + np.asarray(data["position"], dtype=float)
        z_values.extend(world_points[:, 2].tolist()[::max(1, len(world_points) // 200)])
        ground = np.abs(world_points[:, 2]) <= args.floor_z_band
        world_points = world_points[ground]
        object_ids = semantic[yy, xx][ground]
        floor_point_count += len(world_points)
        if not len(world_points):
            continue
        grid_x = np.floor((world_points[:, 0] - origin[0]) / resolution).astype(np.int32)
        grid_y = np.floor((world_points[:, 1] - origin[1]) / resolution).astype(np.int32)
        in_bounds = (
            (grid_x >= 0) & (grid_x < width) &
            (grid_y >= 0) & (grid_y < height)
        )
        grid_x, grid_y, object_ids = grid_x[in_bounds], grid_y[in_bounds], object_ids[in_bounds]
        for object_id in np.unique(object_ids):
            region_id = floor_to_region[int(object_id)]
            selected = object_ids == object_id
            np.add.at(votes[region_index[region_id]], (grid_y[selected], grid_x[selected]), 1)
            point_counts[region_id] += int(selected.sum())
        frame_count += 1

    raw_region_map = np.full((height, width), -1, dtype=np.int16)
    max_votes = votes.max(axis=0)
    winner = votes.argmax(axis=0)
    has_votes = max_votes > 0
    for region_id, index in region_index.items():
        raw_region_map[has_votes & (winner == index)] = region_id

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    output_npz = output_dir / "room_segmentation.npz"
    output_json = output_dir / "room_regions.json"
    output_png = output_dir / "room_segmentation.png"
    if not args.force and any(path.exists() for path in (output_npz, output_json, output_png)):
        raise FileExistsError(f"refusing to overwrite existing room segmentation in {output_dir}")

    trajectory_path = output_dir.parent / "bag_export" / "trajectory.csv"
    trajectory = []
    if trajectory_path.exists():
        with trajectory_path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                try:
                    trajectory.append([float(row["x"]), float(row["y"])])
                except (KeyError, ValueError):
                    continue
    trajectory_array = np.asarray(trajectory, dtype=float).reshape(-1, 2)

    truth = None
    if args.truth.exists():
        truth_data = np.load(args.truth)
        truth = np.asarray(truth_data["main_floor_target"], dtype=bool)

    regions = []
    for region_id in region_ids:
        region_mask = raw_region_map == region_id
        yy, xx = np.nonzero(region_mask)
        if len(xx):
            xy = np.column_stack([
                origin[0] + (xx + 0.5) * resolution,
                origin[1] + (yy + 0.5) * resolution,
            ])
            centroid = xy.mean(axis=0)
            bbox = [xy.min(axis=0).tolist(), xy.max(axis=0).tolist()]
        else:
            centroid = np.asarray([float("nan"), float("nan")])
            bbox = []
        display_name, semantic_type = REGION_NAMES[region_id]
        inferred_name, inferred_type, cue_labels = classify_region(object_counts[region_id])
        if semantic_type == "entry_hall":
            inferred_name, inferred_type = display_name, semantic_type
        regions.append({
            "region_id": int(region_id),
            "display_name": inferred_name,
            "semantic_type": inferred_type,
            "semantic_object_count": int(sum(object_counts[region_id].values())),
            "semantic_object_labels": dict(object_counts[region_id].most_common()),
            "room_cue_labels": cue_labels,
            "floor_instance_ids": floor_ids_by_region[region_id],
            "projected_floor_points": int(point_counts[region_id]),
            "projected_cells": int(len(xx)),
            "projected_area_m2": float(len(xx) * resolution * resolution),
            "centroid_xy_m": [float(value) for value in centroid],
            "bbox_xy_m": bbox,
            "boundary_status": "inferred_from_semantic_floor_pixels",
        })

    summary = {
        "format": "pre_map_vln.room_segmentation_00166.v1",
        "scene": SCENE_TOKEN,
        "method": "semantic_floor_pixel_projection",
        "frames_used": int(frame_count),
        "floor_points_used": int(floor_point_count),
        "floor_z_band_m": float(args.floor_z_band),
        "resolution_m": resolution,
        "region_count": len(regions),
        "labelled_cells": int(np.count_nonzero(raw_region_map >= 0)),
        "labelled_area_m2": float(np.count_nonzero(raw_region_map >= 0) * resolution * resolution),
        "main_floor_truth_cells_for_overlay_only": None if truth is None else int(truth.sum()),
        "trajectory_points": int(len(trajectory_array)),
        "notes": [
            "Semantic region IDs and object labels come from the 00166 semantic asset.",
            "Floor projections were generated from recorded RGB-D/semantic frames.",
            "Region polygons are not provided by HM3D; these are inferred display regions.",
            "Habitat navmesh truth is used only as a red overlay and metric reference.",
            "This output was not fed to FALCON and does not change the stage-1 Bag.",
        ],
    }
    result = {**summary, "regions": regions}
    np.savez_compressed(output_npz, region_map=raw_region_map, votes=votes)
    output_json.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    make_plot(output_png, raw_region_map, observed, meta, regions, trajectory_array, truth)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"png={output_png}")
    print(f"json={output_json}")
    print(f"npz={output_npz}")


if __name__ == "__main__":
    main()
