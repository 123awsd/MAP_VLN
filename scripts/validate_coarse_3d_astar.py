#!/usr/bin/env python3
"""Validate coarse 3-D A* on same-floor and reconstructed cross-floor pairs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.astar_3d import AstarFailure, CoarseAstar3D  # noqa: E402
from stage2.io_utils import atomic_json  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def trajectory_samples(path: Path, count: int = 8) -> list[list[float]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("trajectory contains no rows")
    indices = sorted({round(index * (len(rows) - 1) / max(1, count - 1)) for index in range(count)})
    return [[float(rows[index][axis]) for axis in "xyz"] for index in indices]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--planning-config", type=Path,
        default=ROOT / "config/uav_3d_planning_habitat.yaml",
    )
    args = parser.parse_args()
    profile = PlannerProfile.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.snapshot, profile)
    planner = CoarseAstar3D(voxel_map)
    transitions = json.loads(
        (args.run_dir / "transition_reconstruction/transitions.json").read_text(encoding="utf-8")
    )["transitions"]
    trajectory = trajectory_samples(args.run_dir / "bag_export/trajectory.csv")
    pairs = []
    for index, (start, goal) in enumerate(zip(trajectory, trajectory[1:])):
        pairs.append((f"trajectory_{index}", "executed_trajectory_sample", start, goal))
    for transition in transitions:
        pairs.append((
            transition["id"], "cross_floor_transition",
            transition["lower_entrance_xyz"], transition["upper_entrance_xyz"],
        ))
    if transitions:
        pairs.append((
            "initial_to_first_transition", "same_floor_to_transition",
            trajectory[0], transitions[0]["lower_entrance_xyz"],
        ))
    for first, second in zip(transitions, transitions[1:]):
        pairs.append((
            f'{first["id"]}_to_{second["id"]}', "between_transitions",
            first["upper_entrance_xyz"], second["lower_entrance_xyz"],
        ))

    results = []
    for pair_id, category, start, goal in pairs:
        result = planner.plan(start, goal)
        if isinstance(result, AstarFailure):
            results.append({
                "id": pair_id, "category": category, "start_xyz_m": start, "goal_xyz_m": goal,
                "status": result.status, "reason": result.reason,
                "expanded_nodes": result.expanded_nodes,
            })
        else:
            payload = result.to_json()
            payload.update({"id": pair_id, "category": category, "start_xyz_m": start, "goal_xyz_m": goal})
            if any(
                not voxel_map.line_is_valid(first_point, second_point)
                for first_point, second_point in zip(result.points_xyz_m, result.points_xyz_m[1:])
            ):
                raise AssertionError(f"{pair_id} path failed fine collision audit")
            results.append(payload)
    successful = [item for item in results if item["status"] == "success"]
    failed = [item for item in results if item["status"] != "success"]
    report = {
        "format": "pre_map_vln.coarse_3d_astar_validation.v1",
        "snapshot": str(args.snapshot.resolve()),
        "planning_config": str(args.planning_config.resolve()),
        "map_identity": voxel_map.identity.__dict__,
        "fine_inflated_free_voxels": int(voxel_map.inflated_free.sum()),
        "coarse_free_voxels": int(planner.free_zyx.sum()),
        "pair_count": len(results), "success_count": len(successful), "failure_count": len(failed),
        "cross_floor_success_count": sum(
            item["status"] == "success" and item["category"] == "cross_floor_transition"
            for item in results
        ),
        "all_successful_paths_fine_collision_free": True,
        "falcon_authoritative_parity": "pending_stage2_falcon_service",
        "results": results,
    }
    atomic_json(args.output, report)
    print(json.dumps({key: value for key, value in report.items() if key != "results"}, indent=2))
    for item in failed:
        print(f'FAIL {item["id"]}: {item["status"]} {item["reason"]}')
    if failed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
