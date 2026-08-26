#!/usr/bin/env python3
"""Strict final acceptance gate for PRE-MAP-VLN-Bench-v1."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402


def require(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/shared/PRE_MAP_VLN_benchmark_v1"))
    parser.add_argument("--scene-count", type=int, default=0)
    parser.add_argument("--episode-count", type=int, default=0)
    parser.add_argument("--representative", nargs="+", default=[
        "Z6MFQCViBuw_01", "Z6MFQCViBuw_05", "Z6MFQCViBuw_08",
    ])
    args = parser.parse_args()
    stage1 = load_json(args.root / "reports/stage1_quality.json")
    manifest = load_json(args.root / "tasks/manifest.json")
    scene_count = args.scene_count or len(manifest["scenes"])
    episode_count = args.episode_count or 10 * scene_count
    preparation = load_json(args.root / "runs/preparation_report.json")
    benchmark = load_json(args.root / "reports/benchmark_report.json")
    require((args.root / "reports/episodes.csv").is_file(), "per-episode CSV is missing")
    require((args.root / "reports/summary.md").is_file(), "readable benchmark summary is missing")
    require((args.root / "reports/stage1_quality.csv").is_file(), "Stage-1 quality CSV is missing")
    require(stage1["scene_count"] == scene_count,
            f"Stage 1 does not contain {scene_count} scenes")
    require(stage1["passed_scene_count"] == scene_count,
            "at least one Stage-1 quality gate failed")
    require(manifest["summary"]["episode_count"] == episode_count,
            f"task manifest does not contain {episode_count} episodes")
    require(manifest["summary"]["categories"] == {
        "ordered_spatial": 4 * scene_count,
        "ordered_conditional": 3 * scene_count,
        "ordered_recovery": 3 * scene_count,
    }, "task-category distribution drifted")
    conditional_count = 3 * scene_count
    require(manifest["summary"]["branches"] == {
        "taken": (conditional_count + 1) // 2,
        "skipped": conditional_count // 2,
    },
            "condition outcomes are not balanced")
    require(manifest["summary"]["recoveries"] == {
        "rediscovered": 2 * scene_count, "exhausted": scene_count,
    },
            "recovery outcomes are not controlled as specified")
    require(preparation["prepared_count"] == episode_count and preparation["failure_count"] == 0,
            "candidate or geometric-plan preparation is incomplete")
    require(benchmark["overall"]["completed_count"] == episode_count,
            "not all Habitat benchmark episodes completed")
    require(benchmark["overall"]["order_constraint_satisfaction_rate"] == 1.0,
            "at least one precedence constraint was violated")
    require(benchmark["overall"]["branch_constraint_satisfaction_rate"] == 1.0,
            "at least one controlled condition branch was wrong")
    require(benchmark["overall"]["recovery_expectation_satisfaction_rate"] == 1.0,
            "at least one controlled recovery outcome was wrong")
    bag_evidence = []
    required_topics = {
        "/stage2/rgb", "/stage2/executed_path", "/stage2/global_tour",
        "/stage2/local_path", "/stage2/semantic_boxes", "/stage2/rooms",
        "/stage2/reached_goals", "/stage2/task_text", "/uav_simulator/sensor_pose",
        "/voxel_mapping/occupancy_grid_occupied",
    }
    for episode_id in args.representative:
        path = args.root / "bags/representative" / f"{episode_id}.manifest.json"
        bag = load_json(path)
        require(required_topics <= set(bag["topic_counts"]), f"{episode_id} bag misses topics")
        require(bag["topic_counts"]["/stage2/rgb"] > 100, f"{episode_id} bag has too little RGB")
        bag_path = Path(bag["bag"])
        if str(bag_path).startswith("/workspace/benchmark/"):
            bag_path = args.root / bag_path.relative_to("/workspace/benchmark")
        require(bag_path.is_file() and bag_path.stat().st_size > 1_000_000,
                f"{episode_id} bag is missing or too small")
        bag_evidence.append(bag)
    report = {
        "format": "pre_map_vln.benchmark_acceptance.v1", "status": "passed",
        "scene_count": scene_count, "episode_count": episode_count,
        "stage1_quality": stage1, "benchmark_summary": benchmark["overall"],
        "representative_bags": bag_evidence,
    }
    atomic_json(args.root / "reports/acceptance.json", report)
    print(json.dumps({
        "status": "passed", "scenes": scene_count, "episodes": episode_count,
        "representative_bags": len(bag_evidence),
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
