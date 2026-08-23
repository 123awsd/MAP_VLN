#!/usr/bin/env python3
"""Generate terminal candidates and jointly optimize a stage-two mission."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.candidate_poses import generate_all_candidates  # noqa: E402
from stage2.grid_map import OccupancyGrid  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.joint_planner import plan_fixed_order_baseline, plan_joint_mission  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--grid-prefix", type=Path, required=True)
    parser.add_argument("--start", type=float, nargs=4, metavar=("X", "Y", "Z", "YAW"), default=[0.0, 0.0, 1.0, 0.0])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-candidates", type=int, default=6)
    parser.add_argument("--inflation", type=float, default=0.10)
    args = parser.parse_args()
    task_graph = load_json(args.task_graph)
    scene_graph = load_json(args.scene_graph)
    grid = OccupancyGrid.load(args.grid_prefix, inflation_m=args.inflation)
    candidates = generate_all_candidates(grid, scene_graph, task_graph, max_candidates=args.max_candidates)
    unresolved = [task_id for task_id, values in candidates.items() if not values]
    if unresolved:
        raise SystemExit(f"no feasible candidates: {', '.join(unresolved)}")
    joint = plan_joint_mission(grid, task_graph, candidates, list(args.start))
    baseline = plan_fixed_order_baseline(grid, task_graph, candidates, list(args.start))
    comparison = {
        "joint_path_length_m": joint["total_path_length_m"],
        "fixed_order_path_length_m": baseline["total_path_length_m"],
        "improvement_m": baseline["total_path_length_m"] - joint["total_path_length_m"],
        "improvement_percent": 100.0 * (baseline["total_path_length_m"] - joint["total_path_length_m"]) / max(1e-9, baseline["total_path_length_m"]),
    }
    atomic_json(args.output_dir / "candidates.json", {"format": "pre_map_vln.candidates.v1", "by_task": candidates})
    atomic_json(args.output_dir / "mission_plan.json", joint)
    atomic_json(args.output_dir / "baseline_plan.json", baseline)
    atomic_json(args.output_dir / "comparison.json", comparison)
    print(f"tasks={len(joint['visits'])} joint={joint['total_path_length_m']:.2f}m baseline={baseline['total_path_length_m']:.2f}m improvement={comparison['improvement_percent']:.1f}%")


if __name__ == "__main__":
    main()
