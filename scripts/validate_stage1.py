#!/usr/bin/env python3
"""Validate the artifacts produced by the stage-one HM3D demonstration."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", default="hm3d_stage1")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/stage1_report.json")
    args = parser.parse_args()
    episode_dir = ROOT / "data/episodes" / args.episode
    frames = sorted(episode_dir.glob("frame_*.npz"))
    require(len(frames) >= 2, "at least two Habitat keyframes are required")
    positions = np.asarray([np.load(frame)["position"] for frame in frames])
    path_length = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
    require(path_length > 1.0, "FALCON-driven trajectory did not move at least one meter")

    grid_path = ROOT / "runtime/occusg" / f"{args.episode}_grid.npy"
    grid_meta_path = grid_path.with_suffix(".json")
    grid, grid_meta = np.load(grid_path), json.loads(grid_meta_path.read_text())
    require(grid.shape == (grid_meta["height"], grid_meta["width"]), "grid shape mismatch")
    require(np.count_nonzero(grid == 0) > 0, "occupancy grid contains no free cells")

    boxes_path = ROOT / "outputs/boxer" / args.episode / "boxer_3dbbs_fused.csv"
    with boxes_path.open(newline="", encoding="utf-8") as handle:
        boxes = list(csv.DictReader(handle))
    require(boxes, "Boxer produced no fused static objects")

    regions_path = ROOT / "outputs/occusg" / args.episode / "regions.json"
    regions = json.loads(regions_path.read_text(encoding="utf-8"))
    require(regions["region_count"] > 0, "OccuSG produced no regions")

    graph_path = ROOT / "outputs/scene_graph" / f"{args.episode}.json"
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    summary = graph["summary"]
    require(summary["room_count"] == regions["region_count"], "scene graph room mismatch")
    require(summary["object_count"] == len(boxes), "scene graph object mismatch")

    report = {
        "format": "pre_map_vln.stage1_validation.v1",
        "status": "passed",
        "scene": json.loads((episode_dir / "manifest.json").read_text())["scene"],
        "episode": args.episode,
        "keyframe_count": len(frames),
        "trajectory_path_length_m": round(path_length, 4),
        "trajectory_extent_xyz_m": np.ptp(positions, axis=0).round(4).tolist(),
        "occupancy_grid": {
            "width": int(grid.shape[1]), "height": int(grid.shape[0]),
            "resolution_m": grid_meta["resolution_m"],
            "free_cells": grid_meta["free_cells"],
            "occupied_cells": grid_meta["occupied_cells"],
        },
        "fused_object_count": len(boxes),
        "room_count": regions["region_count"],
        "room_adjacency_edge_count": summary["adjacency_edge_count"],
        "semantic_room_count": sum(room["semantic_type"] != "unknown" for room in graph["rooms"]),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
