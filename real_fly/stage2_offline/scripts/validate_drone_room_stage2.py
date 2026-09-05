#!/usr/bin/env python3
"""Audit the immutable source and all Drone_room Stage-2 offline products."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.task_graph_scene_validation import (  # noqa: E402
    TaskSceneValidationError,
    validate_task_graph_against_scene,
)
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--mapping-bag", type=Path)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--boxer-dir", type=Path, required=True)
    parser.add_argument("--voxel-snapshot", type=Path, required=True)
    parser.add_argument("--planning-config", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--planning-dir", type=Path, required=True)
    parser.add_argument("--localization-summary", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    checks: list[dict[str, Any]] = []

    def check(name: str, condition: bool, detail: Any, required: bool = True) -> None:
        status = "pass" if condition else ("fail" if required else "warning")
        checks.append({"name": name, "status": status, "detail": detail})

    manifest = load(args.episode / "manifest.json")
    actual_hash = sha256(args.bag)
    check("source_bag_sha256", actual_hash == manifest["source_bag_sha256"], actual_hash)
    expected_frames = int(manifest["frames"])
    frame_counts = {
        kind: len(list((args.episode / "frames" / kind).glob(pattern)))
        for kind, pattern in (("color", "*.jpg"), ("depth", "*.png"), ("pose", "*.txt"))
    }
    frame_counts["npz"] = len(list(args.episode.glob("frame_*.npz")))
    check("episode_frame_counts", all(value == expected_frames for value in frame_counts.values()), frame_counts)
    if args.mapping_bag:
        mapping_hash = sha256(args.mapping_bag)
        check("mapping_bag_sha256", mapping_hash == manifest.get("mapping_bag_sha256"), mapping_hash)

    calibration = load(args.episode / "calibration" / "camera_extrinsic.json")
    before = float(calibration["validation_before"]["median_m"])
    after = float(calibration["validation_after"]["median_m"])
    target_based = calibration.get("method", "").startswith("FAST-Calib_target")
    check("calibrated_extrinsic", (
        calibration.get("status") == "accepted" and (target_based or after < before)
    ), {
        "status": calibration.get("status"), "method": calibration.get("method"),
        "target_based": target_based, "median_before_m": before, "median_after_m": after,
        "target_calibration_quality": (calibration.get("lidar_camera_calibration") or {}).get("quality"),
    }, required=False)

    scene = load(args.scene_graph)
    scene_source = Path(scene.get("source_boxer_csv", ""))
    if not scene_source.is_file():
        scene_source = args.boxer_dir / "boxer_3dbbs_fused.csv"
    if not scene_source.is_file():
        scene_source = args.boxer_dir / "boxer_3dbbs.csv"
    boxer_is_fused = scene_source.name.endswith("_fused.csv")
    with scene_source.open(newline="", encoding="utf-8") as handle:
        fused_rows = list(csv.DictReader(handle))
    check("boxer_3d_detections", len(fused_rows) > 0, {
        "rows": len(fused_rows), "path": str(scene_source),
        "fusion": boxer_is_fused,
    })
    processing_path = args.boxer_dir / "processing_manifest.json"
    if processing_path.is_file():
        processing = load(processing_path)
        processing_summary = processing.get("summary", {})
        check("boxer_complete_gpu_pass", (
            processing.get("status") == "completed"
            and processing.get("device") == "cuda"
            and int(processing_summary.get("success", 0)) == expected_frames
            and int(processing_summary.get("failed", 0)) == 0
        ), {"status": processing.get("status"), "device": processing.get("device"),
            "summary": processing_summary})
    else:
        check("boxer_complete_gpu_pass", False, "processing_manifest.json missing")
    if args.localization_summary:
        localization = load(args.localization_summary)
        check("depth_backprojection", int(localization.get("valid_depth_matches", 0)) > 0, {
            "valid_depth_matches": localization.get("valid_depth_matches"),
            "clustered_targets": localization.get("clustered_targets"),
            "rejected": localization.get("rejected"),
        })
        check("metric_extrinsic_ready", localization.get("extrinsic_status") == "accepted", {
            "extrinsic_status": localization.get("extrinsic_status"),
            "warning": localization.get("coordinate_warning"),
        }, required=False)

    profile = PlannerProfile.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.voxel_snapshot, profile)
    voxel_report = load(args.voxel_snapshot / "report.json")
    check("voxel_snapshot", voxel_report.get("status") == "ok" and voxel_report.get("unknown_is_blocked") is True, {
        "counts": voxel_report.get("counts"), "inflated_free_voxels": voxel_report.get("inflated_free_voxels"),
        "unknown_is_blocked": voxel_report.get("unknown_is_blocked"),
    })

    objects = [obj for room in scene.get("rooms", []) for obj in room.get("objects", [])]
    check("scene_graph", bool(objects), {"rooms": len(scene.get("rooms", [])), "objects": len(objects)})
    task = load(args.task_graph)
    check("task_graph", bool(task.get("tasks")) and "spatial_constraints" in task["tasks"][0], {
        "tasks": len(task.get("tasks", [])), "target": task.get("tasks", [{}])[0].get("target"),
    })
    try:
        grounding = validate_task_graph_against_scene(task, scene)
        check("task_scene_grounding", True, grounding)
    except TaskSceneValidationError as error:
        check("task_scene_grounding", False, str(error))

    mission = load(args.planning_dir / "mission_plan.json")
    candidates = load(args.planning_dir / "candidates.json")
    point_count = 0
    minimum_clearance = math.inf
    valid = bool(mission.get("visits")) and bool(mission.get("segments"))
    all_path_points: list[list[float]] = []
    for segment in mission.get("segments", []):
        points = segment.get("points_xyz_m", [])
        all_path_points.extend(points)
        point_count += len(points)
        minimum_clearance = min(minimum_clearance, float(segment.get("minimum_clearance_m", math.inf)))
        valid = valid and segment.get("map_epoch_uuid") == voxel_map.identity.map_epoch_uuid
        valid = valid and all(voxel_map.is_state_valid(point) for point in points)
        valid = valid and all(voxel_map.line_is_valid(first, second) for first, second in zip(points, points[1:]))
    check("offline_mission_collision_audit", valid, {
        "visits": len(mission.get("visits", [])), "path_points": point_count,
        "length_m": mission.get("total_path_length_m"),
        "minimum_clearance_m": None if math.isinf(minimum_clearance) else minimum_clearance,
        "candidate_tasks": len(candidates.get("by_task", {})),
    })

    # A compact human-readable artifact for reviewing the selected route over
    # the occupancy slice without opening ROS or RViz.
    if all_path_points:
        path_array = np.asarray(all_path_points, dtype=float)
        center_z = float(np.median(path_array[:, 2]))
        z0 = max(0, voxel_map.world_to_voxel([0, 0, center_z - 0.35])[2])
        z1 = min(voxel_map.raw.shape[0], voxel_map.world_to_voxel([0, 0, center_z + 0.35])[2] + 1)
        slab = voxel_map.raw[z0:z1]
        free = np.any(slab == 0, axis=0)
        occupied = np.any(slab == 100, axis=0)
        canvas = np.full((*free.shape, 3), (35, 35, 35), dtype=np.uint8)
        canvas[free] = (55, 90, 55)
        canvas[occupied] = (210, 210, 210)
        scale = 4
        canvas = cv2.resize(canvas, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

        def pixel(point: list[float]) -> tuple[int, int]:
            x, y, _ = voxel_map.world_to_voxel(point)
            return int((x + 0.5) * scale), int((y + 0.5) * scale)

        for segment in mission.get("segments", []):
            points = segment.get("points_xyz_m", [])
            if len(points) >= 2:
                cv2.polylines(canvas, [np.asarray([pixel(point) for point in points], np.int32)], False, (0, 190, 255), 3)
        cv2.circle(canvas, pixel(all_path_points[0]), 7, (0, 255, 0), -1)
        cv2.circle(canvas, pixel(all_path_points[-1]), 7, (0, 0, 255), -1)
        visualization = args.planning_dir / "topdown_plan.png"
        cv2.imwrite(str(visualization), np.flipud(canvas))
        check("planning_visualization", visualization.is_file(), {
            "path": str(visualization), "occupancy_slice_center_z_m": center_z,
        })

    status = "fail" if any(item["status"] == "fail" for item in checks) else (
        "provisional" if any(item["status"] == "warning" for item in checks) else "pass"
    )
    report = {
        "format": "pre_map_vln.real_stage2_validation.v1", "status": status,
        "completion_scope": (
            "complete_offline_package" if boxer_is_fused
            else "complete_offline_boxer_raw_with_custom_clustering"
        ),
        "safety_scope": "offline_only_no_flight_control", "checks": checks,
    }
    atomic_json(args.output, report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if status == "fail":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
