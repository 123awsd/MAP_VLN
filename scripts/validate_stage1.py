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
    parser.add_argument("--boxes", type=Path, default=None)
    parser.add_argument("--scene-graph", type=Path, default=None)
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
    require(grid_meta.get("map_role") == "navigation_occupancy", "planning grid is not the navigation map")
    single_floor = grid_meta.get("single_floor_policy", {})
    if args.episode == "hm3d_stage1_complete_v3":
        require(single_floor.get("excluded_navigation_semantic_ids") == [445, 462],
                "HM3D stair semantics are not excluded from the single-floor map")
        require(single_floor.get("excluded_cell_count", 0) > 0,
                "single-floor map contains no stair exclusion cells")

    boxes_path = args.boxes or ROOT / "outputs/boxer" / args.episode / "boxer_3dbbs_fused.csv"
    with boxes_path.open(newline="", encoding="utf-8") as handle:
        boxes = list(csv.DictReader(handle))
    require(boxes, "Boxer produced no fused static objects")

    graph_path = args.scene_graph or ROOT / "outputs/scene_graph" / f"{args.episode}.json"
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    structure_meta_path = Path(graph.get("sources", {}).get("grid_metadata", ""))
    if not structure_meta_path.is_absolute():
        structure_meta_path = ROOT / structure_meta_path
    structure_meta = json.loads(structure_meta_path.read_text(encoding="utf-8"))
    require(structure_meta.get("map_role") == "room_structure", "scene graph was not built from the structure map")
    structure_filter = structure_meta.get("object_filter", {})
    require(structure_filter.get("removable_instance_count", 0) > 0,
            "structure map removed no Qwen-approved objects")
    policy_path = Path(structure_filter.get("structure_policy", ""))
    if not policy_path.is_absolute():
        policy_path = ROOT / policy_path
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    require(len(policy.get("instances", [])) == len(boxes), "structure policy does not cover every box")
    regions_path = Path(graph.get("sources", {}).get(
        "regions", ROOT / "outputs/occusg" / args.episode / "regions.json"
    ))
    if not regions_path.is_absolute():
        regions_path = ROOT / regions_path
    regions = json.loads(regions_path.read_text(encoding="utf-8"))
    require(regions["region_count"] > 0, "OccuSG produced no regions")
    summary = graph["summary"]
    require(summary.get("source_region_count", summary.get("region_count")) == regions["region_count"],
            "scene graph source-region mismatch")
    require(0 < summary["room_count"] <= summary.get("region_count", regions["region_count"]),
            "scene graph major-room count is invalid")
    require(summary["object_count"] == len(boxes), "scene graph object mismatch")
    object_ids = [obj["id"] for room in graph["rooms"] for obj in room.get("objects", [])]
    object_ids.extend(obj["id"] for obj in graph.get("unassigned_objects", []))
    require(len(object_ids) == len(set(object_ids)), "an object appears in multiple canonical rooms")
    if "geometric_regions" in graph:
        require(summary.get("fragment_count") == 0, "room fragments leaked into canonical rooms")
        require(all(room.get("space_role") != "room_fragment" for room in graph["rooms"]),
                "canonical room list still contains a fragment")

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
            "single_floor_height_band_m": single_floor.get("endpoint_height_band_m"),
            "navigation_exclusion_cell_count": single_floor.get("excluded_cell_count", 0),
        },
        "structure_map": {
            "free_cells": structure_meta["free_cells"],
            "occupied_cells": structure_meta["occupied_cells"],
            "removable_instance_count": structure_filter["removable_instance_count"],
            "filter_method": structure_filter["method"],
        },
        "fused_object_count": len(boxes),
        "source_region_count": regions["region_count"],
        "region_count": summary.get("region_count", regions["region_count"]),
        "room_count": summary["room_count"],
        "corridor_count": summary.get("corridor_count", 0),
        "fragment_count": summary.get("fragment_count", 0),
        "source_fragment_count": summary.get("source_fragment_count", 0),
        "room_adjacency_edge_count": summary["adjacency_edge_count"],
        "semantic_room_count": sum(room["semantic_type"] != "unknown" for room in graph["rooms"]),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
