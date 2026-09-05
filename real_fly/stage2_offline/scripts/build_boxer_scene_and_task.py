#!/usr/bin/env python3
"""Convert Boxer detections into a real-flight scene graph and feasible demo task."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

PREFERRED_DEMO_LABELS = (
    "chair", "table", "desk", "cabinet", "door", "sofa", "bench",
    "monitor", "backpack", "suitcase", "box", "trash can",
)

from stage2.astar_3d import AstarFailure, CoarseAstar3D  # noqa: E402
from stage2.candidate_poses import generate_candidates  # noqa: E402
from stage2.motion_cost_oracle import MotionCostOracle  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def finite_floats(row: dict[str, str], keys: list[str]) -> list[float] | None:
    try:
        values = [float(row[key]) for key in keys]
    except (KeyError, TypeError, ValueError):
        return None
    return values if all(math.isfinite(value) for value in values) else None


def read_boxes(path: Path, minimum_probability: float) -> list[dict[str, Any]]:
    boxes = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            center = finite_floats(row, ["tx_world_object", "ty_world_object", "tz_world_object"])
            size = finite_floats(row, ["scale_x", "scale_y", "scale_z"])
            quaternion = finite_floats(
                row, ["qw_world_object", "qx_world_object", "qy_world_object", "qz_world_object"]
            )
            try:
                probability = float(row.get("prob", "nan"))
            except ValueError:
                continue
            label = str(row.get("name", "")).strip().lower()
            if not center or not size or not quaternion or not label or not math.isfinite(probability):
                continue
            size = [abs(value) for value in size]
            if probability < minimum_probability or min(size) < 0.03 or max(size) > 12.0:
                continue
            norm = float(np.linalg.norm(quaternion))
            if norm < 1e-6:
                quaternion = [1.0, 0.0, 0.0, 0.0]
            else:
                quaternion = [value / norm for value in quaternion]
            boxes.append({
                "label": label,
                "center_xyz_m": center,
                "size_xyz_m": size,
                "orientation_wxyz": quaternion,
                "probability": probability,
                "instance": str(row.get("instance", "")),
            })
    return boxes


def cluster_raw_boxes(boxes: list[dict[str, Any]], radius_m: float = 0.9) -> list[dict[str, Any]]:
    """Fallback for unfused Boxer CSV: greedily merge repeated frame detections."""
    clusters: list[list[dict[str, Any]]] = []
    for box in sorted(boxes, key=lambda item: -item["probability"]):
        match = next((
            cluster for cluster in clusters
            if cluster[0]["label"] == box["label"]
            and math.dist(cluster[0]["center_xyz_m"], box["center_xyz_m"]) <= radius_m
        ), None)
        if match is None:
            clusters.append([box])
        else:
            match.append(box)
    result = []
    for cluster in clusters:
        weights = np.asarray([max(0.01, item["probability"]) for item in cluster], dtype=float)
        best = dict(max(cluster, key=lambda item: item["probability"]))
        best["center_xyz_m"] = np.average(
            np.asarray([item["center_xyz_m"] for item in cluster]), axis=0, weights=weights
        ).tolist()
        best["observation_count"] = len(cluster)
        result.append(best)
    return result


def valid_task(label: str, room_id: str) -> dict[str, Any]:
    return {
        "format": "pre_map_vln.task_graph.v1",
        "instruction": f"离线检查已建图场景中的{label}。",
        "tasks": [{
            "id": "inspect_detected_object",
            "action": "inspect",
            "target": {
                "label": label, "room": "flight_room", "room_id": room_id,
                "floor_id": 1, "reference": None, "reference_secondary": None,
                "references": [],
            },
            "verification_label": label,
            "spatial_constraints": {
                "relation": None, "distance_m": None, "region_type": "auto",
                "observation_detail": "normal", "vertical_fov_deg": 70,
                "horizontal_fov_deg": 90, "yaw_tolerance_deg": 55,
                "face_target": True, "visibility_required": True,
            },
            "prerequisites": [], "active_initially": True,
            "success_outcome": "found",
            "search_policy": {"mode": "fixed", "maximum_location_hypotheses": 1},
        }],
        "conditional_rules": [],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--boxer-dir", type=Path, required=True)
    parser.add_argument("--voxel-snapshot", type=Path, required=True)
    parser.add_argument("--planning-config", type=Path, required=True)
    parser.add_argument("--scene-output", type=Path, required=True)
    parser.add_argument("--task-output", type=Path, required=True)
    parser.add_argument("--start-output", type=Path, required=True)
    parser.add_argument("--minimum-probability", type=float, default=0.20)
    args = parser.parse_args()

    candidates = [
        args.boxer_dir / "boxer_3dbbs_fused.csv",
        args.boxer_dir / "boxer_3dbbs.csv",
    ]
    source = next((path for path in candidates if path.is_file() and path.stat().st_size > 100), None)
    if source is None:
        raise SystemExit(f"no Boxer 3-D CSV under {args.boxer_dir}")
    boxes = read_boxes(source, args.minimum_probability)
    if not boxes:
        raise SystemExit(f"no valid Boxer detections in {source}")
    if source.name != "boxer_3dbbs_fused.csv":
        boxes = cluster_raw_boxes(boxes)

    profile = PlannerProfile.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.voxel_snapshot, profile)
    metadata = json.loads((args.voxel_snapshot / "metadata.json").read_text(encoding="utf-8"))
    extent = np.asarray(metadata["shape_xyz"], dtype=float) * float(metadata["resolution_m"])
    origin = np.asarray(metadata["origin_xyz_m"], dtype=float)
    upper = origin + extent
    boxes = [box for box in boxes if np.all(np.asarray(box["center_xyz_m"]) >= origin) and np.all(np.asarray(box["center_xyz_m"]) <= upper)]
    boxes.sort(key=lambda item: (-item["probability"], item["label"], item["center_xyz_m"]))
    if not boxes:
        raise SystemExit("all Boxer detections lie outside the voxel snapshot")

    room_id = "L1_R1"
    objects = []
    for index, box in enumerate(boxes):
        item = dict(box)
        item["id"] = f"L1_boxer_{index:03d}"
        item["source"] = "boxer_offline_rgbd"
        item["room_id"] = room_id
        objects.append(item)
    scene = {
        "format": "pre_map_vln.scene_graph.v1",
        "frame_id": "world",
        "source_boxer_csv": str(source.resolve()),
        "map_epoch_uuid": metadata["identity"]["map_epoch_uuid"],
        "rooms": [{
            "id": room_id, "floor_id": 1, "semantic_type": "flight_room",
            "space_role": "room", "adjacent_room_ids": [],
            "centroid_xy_m": ((origin[:2] + upper[:2]) * 0.5).tolist(),
            "camera_height_band_m": [0.65, 1.80], "objects": objects,
        }],
    }
    atomic_json(args.scene_output, scene)

    start = voxel_map.nearest_valid([0.0, 0.0, 1.0], radius_m=8.0)
    if start is None:
        indices = np.argwhere(voxel_map.inflated_free)
        if not len(indices):
            raise SystemExit("voxel snapshot has no inflated-free start")
        z, y, x = indices[len(indices) // 2]
        start = voxel_map.voxel_to_world([x, y, z])
    oracle = MotionCostOracle(CoarseAstar3D(voxel_map), args.start_output.parent / "selection_motion_cache.json")
    selected = None
    diagnostics = defaultdict(int)
    selection_objects = sorted(objects, key=lambda item: (
        PREFERRED_DEMO_LABELS.index(item["label"])
        if item["label"] in PREFERRED_DEMO_LABELS else len(PREFERRED_DEMO_LABELS),
        -int(item.get("observation_count", 1)),
        -float(item["probability"]),
    ))
    for obj in selection_objects:
        task_graph = valid_task(obj["label"], room_id)
        task = task_graph["tasks"][0]
        generated = [item for item in generate_candidates(oracle, scene, task, max_candidates=8) if item["object_id"] == obj["id"]]
        diagnostics["objects_tested"] += 1
        diagnostics["candidate_poses"] += len(generated)
        for candidate in generated:
            path = oracle.path(start, [candidate["pose"][key] for key in ("x", "y", "z")])
            if path is not None:
                selected = (obj, task_graph, candidate, path)
                break
        if selected:
            break
    if selected is None:
        raise SystemExit(f"no detected object has a reachable observation pose: {dict(diagnostics)}")
    obj, task_graph, candidate, path = selected
    atomic_json(args.task_output, task_graph)
    atomic_json(args.start_output, {
        "format": "pre_map_vln.real_planning_start.v1",
        "start_xyz_yaw": [*start, 0.0],
        "feasibility_probe_object_id": obj["id"], "selected_label": obj["label"],
        "selection_candidate_id": candidate["id"],
        "selection_path_length_m": path.length_m,
        "diagnostics": dict(diagnostics),
    })
    print(f"objects={len(objects)} selected={obj['id']} label={obj['label']} start={start} path={path.length_m:.2f}m")


if __name__ == "__main__":
    main()
