#!/usr/bin/env python3
"""Generate the frozen PRE-MAP-VLN-Bench-v1 task set from Stage-1 maps."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.benchmark_dataset import build_scene_episodes, validate_distribution  # noqa: E402
from stage2.grid_map import OccupancyGrid  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.vocabulary import load_semantic_aliases  # noqa: E402


def ground_truth_matched_ids(scene_graph, ground_truth, maximum_distance_m=1.5):
    aliases = load_semantic_aliases()
    result = set()
    gt_objects = ground_truth.get("objects", [])
    for room in scene_graph.get("rooms", []):
        for obj in room.get("objects", []):
            label = str(obj.get("label", "")).lower()
            accepted = aliases.get(label, {label}) | {label}
            matches = [
                item for item in gt_objects
                if str(item.get("label", "")).lower() in accepted
                or label in aliases.get(str(item.get("label", "")).lower(), {str(item.get("label", "")).lower()})
            ]
            if matches and min(
                math.dist(obj["center_xyz_m"], item["center_xyz_m"]) for item in matches
            ) <= maximum_distance_m:
                result.add(str(obj["id"]))
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/shared/PRE_MAP_VLN_benchmark_v1"))
    parser.add_argument("--scenes", nargs="+", required=True)
    parser.add_argument("--seed", type=int, default=20260825)
    parser.add_argument("--inflation", type=float, default=0.10)
    args = parser.parse_args()
    output = args.root / "tasks"
    output.mkdir(parents=True, exist_ok=True)
    episodes = []
    failures = {}
    for scene_index, scene_id in enumerate(args.scenes):
        try:
            graph = load_json(args.root / "maps/scene_graph" / f"{scene_id}.json")
            ground_truth = load_json(args.root / "scenes/ground_truth" / f"{scene_id}.json")
            matched_ids = ground_truth_matched_ids(graph, ground_truth)
            grid = OccupancyGrid.load(
                args.root / "maps/grids" / f"{scene_id}_navigation",
                inflation_m=args.inflation,
            )
            scene_episodes = build_scene_episodes(
                scene_id, graph, grid, scene_index, seed=args.seed,
                # The benchmark planner and task source use only the robot-built map.
                # Official semantics remain held out for validity/coverage reporting.
                allowed_object_ids=None,
            )
            scene_dir = output / "episodes" / scene_id
            scene_dir.mkdir(parents=True, exist_ok=True)
            for episode in scene_episodes:
                atomic_json(scene_dir / f"{episode['episode_id']}.json", episode)
            episodes.extend(scene_episodes)
            print(f"{scene_id}: {len(scene_episodes)} episodes, {len(matched_ids)} GT-matched mapped boxes")
        except Exception as exc:
            failures[scene_id] = f"{type(exc).__name__}: {exc}"
            print(f"{scene_id}: FAILED {failures[scene_id]}", file=sys.stderr)
    summary = validate_distribution(episodes, len(args.scenes)) if not failures else {
        "episode_count": len(episodes), "scene_count": len(args.scenes), "failures": failures,
    }
    manifest = {
        "format": "pre_map_vln.benchmark_manifest.v1",
        "name": "PRE-MAP-VLN-Bench-v1",
        "seed": args.seed,
        "scenes": list(args.scenes),
        "summary": summary,
        "episodes": [
            {
                "episode_id": item["episode_id"], "scene_id": item["scene_id"],
                "category": item["category"], "instruction": item["task_graph"]["instruction"],
                "path": str(output / "episodes" / item["scene_id"] / f"{item['episode_id']}.json"),
            }
            for item in episodes
        ],
    }
    atomic_json(output / "manifest.json", manifest)
    atomic_json(output / "generation_report.json", {"summary": summary, "failures": failures})
    if failures:
        raise SystemExit(1)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
