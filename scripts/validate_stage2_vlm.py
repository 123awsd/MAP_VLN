#!/usr/bin/env python3
"""Acceptance checks for the Qwen-VL-driven online mission."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, default=ROOT / "outputs/stage2/hm3d_vlm")
    parser.add_argument("--bag-manifest", type=Path, default=ROOT / "outputs/bags/hm3d_stage2_vlm_complete.manifest.json")
    args = parser.parse_args()
    execution = load_json(args.run_dir / "habitat_demo/habitat_execution.json")
    bag = load_json(args.bag_manifest)
    observations = {item["task_id"]: item for item in execution["observations"]}
    require(execution["status"] == "completed", "VLM mission did not complete")
    require(execution["verification_mode"] == "qwen_vl", "mission was not controlled by Qwen-VL")
    require(len(observations) >= 4, "too few VLM-verified tasks")
    require(all(item["verification"]["vlm"] is not None for item in observations.values()), "missing VLM evidence")
    require(all(item["verification"]["vlm"]["confidence"] >= 0.65 for item in observations.values()), "a VLM decision is below threshold")
    require(observations["observe_living_room_tv"]["outcome"] == "found", "Qwen-VL did not recognize visible television")
    require(execution["task_status"]["inspect_cabinet"] == "skipped", "found television did not skip cabinet branch")
    require(bag["topic_counts"]["/stage2/rgb"] == len(execution["trajectory_xyz_yaw"]), "VLM bag RGB is incomplete")
    report = {
        "format": "pre_map_vln.stage2_vlm_validation.v1", "status": "passed",
        "task_count": len(observations), "path_length_m": execution["path_length_m"],
        "rgb_frame_count": execution["frame_count"], "replan_count": len(execution["replans"]),
        "tv_outcome": observations["observe_living_room_tv"]["outcome"],
        "cabinet_status": execution["task_status"]["inspect_cabinet"],
        "bag": bag,
    }
    atomic_json(args.run_dir / "validation_report.json", report)
    print(f"status=passed tasks={report['task_count']} tv=found cabinet=skipped path={report['path_length_m']:.2f}m")


if __name__ == "__main__":
    main()
