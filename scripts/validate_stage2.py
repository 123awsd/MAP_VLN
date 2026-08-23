#!/usr/bin/env python3
"""Automated acceptance checks for the complete second-stage system."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.grid_map import OccupancyGrid  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.task_graph import normalize_and_validate_task_graph  # noqa: E402


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, default=ROOT / "outputs/stage2/hm3d_example")
    parser.add_argument("--execution-dir", type=Path, default=ROOT / "outputs/stage2/hm3d_example/habitat_demo")
    parser.add_argument("--grid-prefix", type=Path, default=ROOT / "runtime/occusg/hm3d_stage1_grid")
    parser.add_argument("--bag-manifest", type=Path, default=ROOT / "outputs/bags/hm3d_stage2_complete.manifest.json")
    parser.add_argument("--multiscene-report", type=Path, default=ROOT / "outputs/stage2/multiscene/report.json")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/stage2/stage2_report.json")
    args = parser.parse_args()

    task_graph = normalize_and_validate_task_graph(load_json(args.run_dir / "task_graph.json"))
    candidates = load_json(args.run_dir / "candidates.json")["by_task"]
    mission = load_json(args.run_dir / "mission_plan.json")
    baseline = load_json(args.run_dir / "baseline_plan.json")
    comparison = load_json(args.run_dir / "comparison.json")
    execution = load_json(args.execution_dir / "habitat_execution.json")
    bag = load_json(args.bag_manifest)
    usage = load_json(ROOT / "outputs/stage2/api_usage.json")
    multiscene = load_json(args.multiscene_report)
    grid = OccupancyGrid.load(args.grid_prefix, inflation_m=0.10)

    task_ids = {task["id"] for task in task_graph["tasks"]}
    require(set(candidates) == task_ids, "candidate map does not cover every task")
    require(all(candidates[task_id] for task_id in task_ids), "at least one task has no feasible candidate")
    for values in candidates.values():
        for candidate in values:
            pose = candidate["pose"]
            require(grid.is_free([pose["x"], pose["y"]]), f"candidate {candidate['id']} is not in free space")
            require(candidate["line_of_sight"], f"candidate {candidate['id']} has no target line of sight")

    visit_order = {visit["task_id"]: visit["sequence"] for visit in mission["visits"]}
    initial_ids = {task["id"] for task in task_graph["tasks"] if task["active_initially"]}
    require(set(visit_order) == initial_ids, "joint plan does not cover exactly the initial active tasks")
    for task in task_graph["tasks"]:
        if task["id"] not in visit_order:
            continue
        for predecessor in task["prerequisites"]:
            if predecessor in visit_order:
                require(visit_order[predecessor] < visit_order[task["id"]], "joint plan violates precedence")
    require(mission["total_path_length_m"] < baseline["total_path_length_m"], "joint plan did not improve the fixed-order baseline")

    require(execution["status"] == "completed", "Habitat online execution is incomplete")
    require(all(value in {"completed", "skipped"} for value in execution["task_status"].values()), "some online tasks did not terminate")
    observations = {item["task_id"]: item for item in execution["observations"]}
    require(observations["observe_living_room_tv"]["outcome"] == "not_found", "conditional source did not produce not_found")
    require("inspect_cabinet" in observations, "not_found branch did not activate cabinet task")
    activation = [
        change
        for event in execution["events"] for change in event.get("state_changes", [])
        if change["task_id"] == "inspect_cabinet" and change["status"] == "pending"
    ]
    require(bool(activation), "condition did not generate an explicit activation event")
    require(sum(item["verification"]["found"] for item in execution["observations"]) >= 3, "too few targets were visually verified")
    require(len(execution["map_updates"]) == len(execution["observations"]), "semantic map was not updated after every task")
    require(execution["frame_count"] >= 100, "online RGB trajectory is unexpectedly short")
    frame_files = list((args.execution_dir / "frames").glob("frame_*.jpg"))
    require(len(frame_files) == execution["frame_count"], "full per-frame RGB was not saved")

    required_topics = {
        "/stage2/rgb", "/stage2/executed_path", "/stage2/global_tour",
        "/stage2/local_path", "/stage2/candidate_poses", "/stage2/semantic_boxes",
        "/stage2/task_status", "/stage2/uav_pose", "/uav_simulator/sensor_pose",
        "/voxel_mapping/occupancy_grid_occupied",
    }
    require(required_topics <= set(bag["topic_counts"]), "stage2 bag is missing visualization topics")
    require(bag["topic_counts"]["/stage2/rgb"] == len(execution["trajectory_xyz_yaw"]), "bag RGB is not frame-complete")
    require(bag["size_bytes"] > 50_000_000, "stage2 bag is unexpectedly small")
    require(float(usage["total_estimated_cny"]) <= 20.0, "Qwen cost ceiling exceeded")
    require(multiscene["status"] == "passed", "multi-scene planner benchmark failed")
    require(len(multiscene["scenes"]) >= 3, "fewer than three HM3D scenes were tested")
    require(all(item["candidate_count"] >= 20 for item in multiscene["scenes"]), "multi-scene candidate coverage is too low")
    require(all(item["dynamic_status"] == "completed" for item in multiscene["scenes"]), "multi-scene dynamic execution failed")

    report = {
        "format": "pre_map_vln.stage2_validation.v1",
        "status": "passed",
        "task_graph": task_graph["summary"],
        "candidate_count": sum(len(values) for values in candidates.values()),
        "initial_plan": {
            "task_count": len(mission["visits"]),
            "joint_path_length_m": mission["total_path_length_m"],
            "fixed_order_path_length_m": baseline["total_path_length_m"],
            "improvement_percent": comparison["improvement_percent"],
        },
        "online_execution": {
            "task_count": len(execution["observations"]),
            "replan_count": len(execution["replans"]),
            "path_length_m": execution["path_length_m"],
            "rgb_frame_count": execution["frame_count"],
            "verified_found_count": sum(item["verification"]["found"] for item in execution["observations"]),
            "condition_activated": True,
        },
        "bag": bag,
        "multiscene": multiscene["scenes"],
        "qwen_estimated_cny": usage["total_estimated_cny"],
    }
    atomic_json(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
