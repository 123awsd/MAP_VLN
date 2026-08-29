#!/usr/bin/env python3
"""Run a deterministic three-floor task planning acceptance test on 00337."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.astar_3d import CoarseAstar3D  # noqa: E402
from stage2.candidate_poses import generate_all_candidates  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.joint_planner import plan_fixed_order_baseline, plan_joint_mission  # noqa: E402
from stage2.motion_cost_oracle import MotionCostOracle  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def task(task_id: str, floor: int) -> dict:
    return {
        "id": task_id, "action": "inspect", "active_initially": True,
        "prerequisites": [], "verification_label": "bed",
        "target": {"label": "bed", "room": "bedroom", "floor_id": floor},
        "spatial_constraints": {
            "relation": None, "distance_m": [1.0, 2.0], "height_m": None,
            "height_range_m": None, "region_type": "auto", "observation_detail": "normal",
            "vertical_fov_deg": 70.0, "horizontal_fov_deg": 90.0,
            "yaw_tolerance_deg": 55.0, "face_target": True, "visibility_required": True,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scene_graph", type=Path)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--planning-config", type=Path,
        default=ROOT / "config/uav_3d_planning_habitat.yaml",
    )
    parser.add_argument("--max-candidates", type=int, default=4)
    args = parser.parse_args()
    profile = PlannerProfile.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.snapshot, profile)
    oracle = MotionCostOracle(CoarseAstar3D(voxel_map), args.output / "motion_cost_cache.json")
    scene = load_json(args.scene_graph)
    task_graph = {"format": "pre_map_vln.task_graph.v1", "tasks": [
        task("inspect_L3_bed", 3), task("inspect_L1_bed", 1), task("inspect_L2_bed", 2),
    ]}
    candidates = generate_all_candidates(voxel_map, scene, task_graph, args.max_candidates)
    missing = [task_id for task_id, values in candidates.items() if not values]
    if missing:
        raise RuntimeError(f"no 3-D candidates for: {missing}")
    start = [0.0, 0.0, 1.0, 0.0]
    joint = plan_joint_mission(oracle, task_graph, candidates, start)
    baseline = plan_fixed_order_baseline(oracle, task_graph, candidates, start)
    for segment in joint["segments"]:
        if "points_xyz_m" not in segment:
            raise AssertionError("joint plan emitted a non-3-D segment")
        if any(
            not voxel_map.line_is_valid(first, second)
            for first, second in zip(segment["points_xyz_m"], segment["points_xyz_m"][1:])
        ):
            raise AssertionError("joint plan failed fine FREE collision audit")
    visited_floors = [
        next(item["floor_id"] for item in candidates[visit["task_id"]] if item["id"] == visit["candidate_id"])
        for visit in joint["visits"]
    ]
    report = {
        "format": "pre_map_vln.stage2_multifloor_3d_validation.v1",
        "status": "passed", "start_xyz_yaw": start,
        "candidate_counts": {key: len(value) for key, value in candidates.items()},
        "joint_visit_order": [visit["task_id"] for visit in joint["visits"]],
        "joint_floor_order": visited_floors,
        "joint_path_length_m": joint["total_path_length_m"],
        "fixed_order_path_length_m": baseline["total_path_length_m"],
        "joint_improvement_m": baseline["total_path_length_m"] - joint["total_path_length_m"],
        "all_segments_xyz": True, "all_segments_fine_collision_free": True,
        "geometry_map_version": voxel_map.identity.geometry_map_version,
        "map_epoch_uuid": voxel_map.identity.map_epoch_uuid,
        "oracle": oracle.statistics(), "joint_plan": joint, "fixed_order_plan": baseline,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    atomic_json(args.output / "task_graph.json", task_graph)
    atomic_json(args.output / "candidates.json", {"format": "pre_map_vln.candidates.v2", "by_task": candidates})
    atomic_json(args.output / "report.json", report)
    print(json.dumps({key: value for key, value in report.items() if key not in {"joint_plan", "fixed_order_plan"}}, indent=2))


if __name__ == "__main__":
    main()
