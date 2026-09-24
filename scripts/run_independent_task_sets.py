#!/usr/bin/env python3
"""Run the generated single-target sets as independent MAP_VLN episodes."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / ".envs" / "habitat" / "bin" / "python"

SETS = {
    "00166": {
        "prepared": "p166",
        "task_graph": ROOT / "outputs/stage2_3d/task_sets/single_target/task_graph_00166.json",
    },
    "00409": {
        "prepared": "00409_full3d_v1_fixed",
        "task_graph": ROOT / "outputs/stage2_3d/task_sets/single_target/task_graph_00409.json",
    },
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-set", choices=["00166", "00409", "both"], default="both")
    parser.add_argument("--verification-mode", default="owlv2")
    parser.add_argument("--max-candidates", type=int, default=3)
    parser.add_argument("--owlv2-every", type=int, default=10)
    parser.add_argument("--save-every", type=int, default=10)
    args = parser.parse_args()

    selected = list(SETS) if args.scene_set == "both" else [args.scene_set]
    for scene_set in selected:
        config = SETS[scene_set]
        graph = json.loads(config["task_graph"].read_text(encoding="utf-8"))
        prepared = ROOT / "outputs" / "stage2_3d" / "prepared" / config["prepared"]
        for source_task in graph["tasks"]:
            task_id = source_task["id"]
            run_name = f"independent_{scene_set}_{task_id}_owlv2"
            run_root = ROOT / "outputs" / "stage2_3d" / "tasks" / run_name
            execution_dir = run_root / "execution"
            trace_path = execution_dir / "habitat_execution.json"
            if trace_path.exists():
                print(f"SKIP {run_name} (result exists)", flush=True)
                continue
            if run_root.exists():
                raise RuntimeError(f"refusing to reuse incomplete output directory: {run_root}")
            run_root.mkdir(parents=True)
            single_graph = {
                "format": "pre_map_vln.task_graph.v1",
                "instruction": f"{scene_set} independent single-target task {task_id}",
                "tasks": [source_task],
                "conditional_rules": [],
            }
            task_graph_path = run_root / "task_graph.json"
            task_graph_path.write_text(
                json.dumps(single_graph, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            command = [
                str(PYTHON), "-u", str(ROOT / "scripts" / "run_stage2_habitat.py"),
                "--task-graph", str(task_graph_path),
                "--scene-graph", str(prepared / "scene_graph.json"),
                "--voxel-snapshot", str(prepared / "voxel_snapshot"),
                "--generated-stage1-config", str(prepared / "generated_stage1_config.json"),
                "--planning-config", str(ROOT / "config" / "uav_3d_planning_habitat.yaml"),
                "--output-dir", str(execution_dir),
                "--verification-mode", args.verification_mode,
                "--max-candidates", str(args.max_candidates),
                "--planning-horizon-tasks", "1",
                "--owlv2-every", str(args.owlv2_every),
                "--save-every", str(args.save_every),
            ]
            print(f"RUN {scene_set} {task_id}", flush=True)
            result = subprocess.run(command, cwd=ROOT)
            if result.returncode != 0:
                print(f"FAILED_PROCESS {scene_set} {task_id} returncode={result.returncode}", flush=True)
                return result.returncode
            print(f"DONE {scene_set} {task_id}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
