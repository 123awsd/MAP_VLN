#!/usr/bin/env python3
"""Materialize exact candidates and reference plans for benchmark episodes."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.candidate_poses import generate_candidates  # noqa: E402
from stage2.grid_map import OccupancyGrid  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.joint_planner import plan_fixed_order_baseline, plan_joint_mission  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/shared/PRE_MAP_VLN_benchmark_v1"))
    parser.add_argument("--inflation", type=float, default=0.10)
    parser.add_argument("--max-candidates", type=int, default=6)
    args = parser.parse_args()
    manifest = load_json(args.root / "tasks/manifest.json")
    prepared, failures = [], {}
    for item in manifest["episodes"]:
        episode = load_json(Path(item["path"]))
        episode_id, scene_id = episode["episode_id"], episode["scene_id"]
        output = args.root / "runs/prepared" / episode_id
        try:
            graph = episode["task_graph"]
            scene_graph = load_json(args.root / "maps/scene_graph" / f"{scene_id}.json")
            grid = OccupancyGrid.load(
                args.root / "maps/grids" / f"{scene_id}_navigation",
                inflation_m=args.inflation,
            )
            by_task = {}
            for task in graph["tasks"]:
                selected = episode["selected_objects"][task["id"]]
                values = generate_candidates(
                    grid, scene_graph, task, max_candidates=args.max_candidates,
                    allowed_object_ids={selected}, prefer_room=False,
                )
                if not values:
                    raise ValueError(f"selected object {selected} has no candidate for {task['id']}")
                if {value["object_id"] for value in values} != {selected}:
                    raise ValueError(f"candidate target drift for {task['id']}")
                by_task[task["id"]] = values
            joint = plan_joint_mission(grid, graph, by_task, [0.0, 0.0, 1.0, 0.0])
            baseline = plan_fixed_order_baseline(grid, graph, by_task, [0.0, 0.0, 1.0, 0.0])
            output.mkdir(parents=True, exist_ok=True)
            atomic_json(output / "episode.json", episode)
            atomic_json(output / "task_graph.json", graph)
            atomic_json(output / "candidates.json", {
                "format": "pre_map_vln.candidates.v1", "selection_mode": "frozen_map_anchor",
                "selected_objects": episode["selected_objects"], "by_task": by_task,
            })
            atomic_json(output / "mission_plan.json", joint)
            atomic_json(output / "baseline_plan.json", baseline)
            record = {
                "episode_id": episode_id, "scene_id": scene_id,
                "category": episode["category"], "status": "prepared",
                "joint_path_length_m": joint["total_path_length_m"],
                "fixed_path_length_m": baseline["total_path_length_m"],
                "controls": episode["controls"], "run_dir": str(output),
            }
            atomic_json(output / "status.json", record)
            prepared.append(record)
            print(f"{episode_id}: {joint['total_path_length_m']:.1f}m")
        except Exception as exc:
            failures[episode_id] = f"{type(exc).__name__}: {exc}"
            print(f"{episode_id}: FAILED {failures[episode_id]}", file=sys.stderr)
    report = {
        "format": "pre_map_vln.benchmark_preparation.v1",
        "prepared_count": len(prepared), "failure_count": len(failures),
        "failures": failures, "episodes": prepared,
    }
    atomic_json(args.root / "runs/preparation_report.json", report)
    print(json.dumps({key: report[key] for key in ("prepared_count", "failure_count")}, indent=2))
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
