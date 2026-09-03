#!/usr/bin/env python3
"""Compare rolling representative planning with lazy representative planning.

This benchmark intentionally excludes Habitat rendering and visual detection.
It replays the dynamic "plan one visit, observe success, replan" loop on the
same 3-D map and reports the motion-oracle A* work and planning wall time.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.astar_3d import CoarseAstar3D  # noqa: E402
from stage2.candidate_poses import generate_all_candidates  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.joint_planner import plan_joint_mission  # noqa: E402
from stage2.lazy_representative_planner import LazyRepresentativePlanner  # noqa: E402
from stage2.mission_executor import MissionState  # noqa: E402
from stage2.motion_cost_oracle import MotionCostOracle  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.representative_viewpoints import select_location_representatives  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def run_dynamic_benchmark(
    mode: str,
    voxel_map: VoxelMap3D,
    scene_graph: dict,
    task_graph: dict,
    *,
    max_candidates: int,
    planning_horizon_tasks: int,
    start_xyz_yaw: list[float],
) -> dict:
    oracle = MotionCostOracle(CoarseAstar3D(voxel_map), None)
    candidates = generate_all_candidates(
        oracle, scene_graph, task_graph, max_candidates=max_candidates,
    )
    state = MissionState(task_graph)
    current = list(start_xyz_yaw)
    planning_wall_s = 0.0
    path_length_m = 0.0
    task_order = []
    replans = []
    lazy_planner = LazyRepresentativePlanner(oracle) if mode == "lazy" else None

    while state.active:
        active = state.available()
        planning_started = time.perf_counter()
        if mode == "lazy":
            plan = lazy_planner.plan_global(
                task_graph,
                candidates,
                current,
                active_task_ids=active,
                completed_task_ids=state.completed,
            )
        else:
            def lower_bound(task_id: str) -> float:
                return min((
                    math.dist(
                        current[:3],
                        [item["pose"][axis] for axis in ("x", "y", "z")],
                    ) + float(item.get("terminal_cost", 0.0))
                    for item in candidates.get(task_id, [])
                ), default=math.inf)

            planning_task_ids = set(sorted(
                active, key=lambda task_id: (lower_bound(task_id), task_id)
            )[:max(1, planning_horizon_tasks)])
            planning_input = select_location_representatives(
                candidates,
                current,
                planning_task_ids,
                maximum_per_location=1,
            )
            plan = plan_joint_mission(
                oracle,
                task_graph,
                planning_input,
                current,
                active_task_ids=planning_task_ids,
                completed_task_ids=state.completed,
            )
        elapsed = time.perf_counter() - planning_started
        planning_wall_s += elapsed
        visit = plan["visits"][0]
        segment = plan["segments"][0]
        task_order.append(visit["task_id"])
        path_length_m += float(segment["length_m"])
        replans.append({
            "task_id": visit["task_id"],
            "candidate_id": visit["candidate_id"],
            "planning_wall_s": elapsed,
            "path_length_m": float(segment["length_m"]),
            "astar_cache_misses_so_far": oracle.cache_misses,
        })
        current = [float(visit["pose"][axis]) for axis in ("x", "y", "z", "yaw")]
        # All-found is the controlled success oracle used only for the planning
        # comparison. Habitat/OWLv2 is deliberately not part of this metric.
        state.finish_task(visit["task_id"], "found")

    result = {
        "mode": mode,
        "status": state.result_status(),
        "task_order": task_order,
        "path_length_m": path_length_m,
        "planning_wall_s": planning_wall_s,
        "candidate_counts": {
            task_id: len(values) for task_id, values in candidates.items()
        },
        "replans": replans,
        "motion_cost_oracle": oracle.statistics(),
    }
    if lazy_planner is not None:
        result["lazy_representative_planner"] = lazy_planner.statistics()
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--voxel-snapshot", type=Path, required=True)
    parser.add_argument(
        "--planning-config", type=Path,
        default=ROOT / "config/uav_3d_planning_habitat.yaml",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-candidates", type=int, default=5)
    parser.add_argument("--planning-horizon-tasks", type=int, default=2)
    parser.add_argument(
        "--start", type=float, nargs=4,
        default=[0.0, 0.0, 1.0, 0.0],
        metavar=("X", "Y", "Z", "YAW"),
    )
    args = parser.parse_args()

    task_graph = load_json(args.task_graph)
    scene_graph = load_json(args.scene_graph)
    profile = PlannerProfile.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.voxel_snapshot, profile)
    old = run_dynamic_benchmark(
        "rolling", voxel_map, scene_graph, task_graph,
        max_candidates=args.max_candidates,
        planning_horizon_tasks=args.planning_horizon_tasks,
        start_xyz_yaw=list(args.start),
    )
    lazy = run_dynamic_benchmark(
        "lazy", voxel_map, scene_graph, task_graph,
        max_candidates=args.max_candidates,
        planning_horizon_tasks=args.planning_horizon_tasks,
        start_xyz_yaw=list(args.start),
    )
    old_path = float(old["path_length_m"])
    old_time = float(old["planning_wall_s"])
    old_astar = int(old["motion_cost_oracle"]["cache_misses"])
    comparison = {
        "path_delta_m_lazy_minus_rolling": float(lazy["path_length_m"]) - old_path,
        "path_improvement_percent": 100.0 * (old_path - float(lazy["path_length_m"])) / max(1e-9, old_path),
        "planning_wall_delta_s_lazy_minus_rolling": float(lazy["planning_wall_s"]) - old_time,
        "planning_wall_change_percent": 100.0 * (float(lazy["planning_wall_s"]) - old_time) / max(1e-9, old_time),
        "cold_astar_delta_lazy_minus_rolling": int(lazy["motion_cost_oracle"]["cache_misses"]) - old_astar,
        "cold_astar_change_percent": 100.0 * (int(lazy["motion_cost_oracle"]["cache_misses"]) - old_astar) / max(1, old_astar),
    }
    report = {
        "format": "pre_map_vln.stage2_planner_benchmark.v1",
        "scope": "3d_planning_only_dynamic_all_found_oracle",
        "max_candidates": args.max_candidates,
        "planning_horizon_tasks": args.planning_horizon_tasks,
        "start_xyz_yaw": list(args.start),
        "map_epoch_uuid": voxel_map.identity.map_epoch_uuid,
        "geometry_map_version": voxel_map.identity.geometry_map_version,
        "rolling_representative": old,
        "lazy_representative": lazy,
        "comparison": comparison,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(args.output, report)
    print(json.dumps({"comparison": comparison, "rolling": old, "lazy": lazy}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
