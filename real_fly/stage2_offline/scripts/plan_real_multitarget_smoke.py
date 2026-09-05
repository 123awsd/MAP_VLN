#!/usr/bin/env python3
"""Plan a small set of semantic targets against the real voxel snapshot."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from stage2.candidate_poses import generate_all_candidates  # noqa: E402
from stage2.joint_planner import PlanningError, plan_joint_mission  # noqa: E402
from stage2.motion_cost_oracle import MotionCostOracle  # noqa: E402
from stage2.astar_3d import CoarseAstar3D  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def task(task_id: str, label: str) -> dict:
    return {
        "id": task_id, "action": "inspect", "target": {
            "label": label, "room": "flight_room", "room_id": "L1_R1", "floor_id": 1,
            "reference": None, "reference_secondary": None, "references": [],
        }, "verification_label": label,
        "spatial_constraints": {"relation": None, "distance_m": None, "region_type": "auto",
            "observation_detail": "normal", "vertical_fov_deg": 70, "horizontal_fov_deg": 90,
            "yaw_tolerance_deg": 55, "face_target": True, "visibility_required": True},
        "prerequisites": [], "active_initially": True, "success_outcome": "found",
        "search_policy": {"mode": "fixed", "maximum_location_hypotheses": 1},
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--voxel-snapshot", type=Path, required=True)
    parser.add_argument("--planning-config", type=Path, required=True)
    parser.add_argument("--start", type=float, nargs=4, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("labels", nargs="+", help="semantic labels to test")
    args = parser.parse_args()
    scene = json.loads(args.scene_graph.read_text(encoding="utf-8"))
    wanted = {label.strip().lower() for label in args.labels}
    # The smoke scene may contain many repeated unfused boxes. Keep only a
    # small high-confidence pool per requested label before candidate search.
    compact_scene = dict(scene)
    compact_scene["rooms"] = []
    for room in scene.get("rooms", []):
        compact_room = dict(room)
        matching = [obj for obj in room.get("objects", []) if str(obj.get("label", "")).lower() in wanted]
        compact_room["objects"] = sorted(matching, key=lambda item: -float(item.get("probability", 0.0)))[:4]
        compact_scene["rooms"].append(compact_room)
    profile = PlannerProfile.load(args.planning_config)
    voxel = VoxelMap3D.load(args.voxel_snapshot, profile)
    oracle = MotionCostOracle(CoarseAstar3D(voxel), args.output.parent / "multitarget_motion_cache.json")
    tasks = [task(f"inspect_{index:02d}", label.lower()) for index, label in enumerate(args.labels)]
    graph = {"format": "pre_map_vln.task_graph.v1", "instruction": "offline multi-target smoke", "tasks": tasks, "conditional_rules": []}
    candidates = generate_all_candidates(oracle, compact_scene, graph, max_candidates=2)
    start = list(args.start)
    resolved = {task_id: values[0]["object_id"] for task_id, values in candidates.items() if values}
    unresolved = [task_id for task_id in (item["id"] for item in tasks) if not candidates.get(task_id)]
    result = {"format": "pre_map_vln.real_multitarget_smoke.v1", "labels": args.labels,
        "candidate_counts": {key: len(value) for key, value in candidates.items()},
        "resolved_object_ids": resolved, "unresolved_tasks": unresolved, "status": "pass"}
    if not unresolved:
        try:
            result["mission"] = plan_joint_mission(oracle, graph, candidates, start)
        except PlanningError as error:
            result["status"] = "fail"; result["planning_error"] = str(error)
    else:
        result["status"] = "partial"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
