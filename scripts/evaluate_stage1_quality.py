#!/usr/bin/env python3
"""Generate Stage-1 coverage and semantic-map quality gates for benchmark scenes."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.build_benchmark_v1 import ground_truth_matched_ids  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/shared/PRE_MAP_VLN_benchmark_v1"))
    parser.add_argument("--scenes", nargs="+", required=True)
    parser.add_argument("--survey", type=Path)
    args = parser.parse_args()
    survey = load_json(args.survey or args.root / "scenes/mp3d_survey.json")
    survey_by_id = {item["scene_id"]: item for item in survey["scenes"]}
    rows = []
    for scene_id in args.scenes:
        try:
            status = load_json(args.root / "runs/stage1" / scene_id / "status.json")
            bag_audit = load_json(args.root / "runs/stage1" / scene_id / "bag_audit.json")
            graph = load_json(args.root / "maps/scene_graph" / f"{scene_id}.json")
            gt = load_json(args.root / "scenes/ground_truth" / f"{scene_id}.json")
            grid_meta = load_json(args.root / "maps/grids" / f"{scene_id}_navigation.json")
            survey_item = survey_by_id[scene_id]
        except (FileNotFoundError, KeyError, ValueError) as exc:
            rows.append({
                "scene_id": scene_id, "passed": False,
                "checks": {"stage1_artifacts_present": False},
                "error": f"{type(exc).__name__}: {exc}",
            })
            continue
        dominant_fraction = max(item["fraction"] for item in survey_item["floor_clusters"])
        estimated_floor_area = survey_item["navmesh_area_m2"] * dominant_fraction
        mapped_free_area = grid_meta["free_cells"] * grid_meta["resolution_m"] ** 2
        matched = ground_truth_matched_ids(graph, gt)
        object_count = int(graph["summary"]["object_count"])
        room_count = int(graph["summary"]["room_count"])
        free_area_ratio = mapped_free_area / max(estimated_floor_area, 1e-6)
        checks = {
            "stage1_passed": status.get("status") == "passed",
            "minimum_frames": int(status.get("frame_count", 0)) >= 100,
            "minimum_rooms": room_count >= 4,
            "minimum_objects": object_count >= 20,
            "minimum_gt_matched_boxes": len(matched) >= 8,
            "minimum_free_area_ratio": free_area_ratio >= 0.25,
            "frontier_audit_present": bag_audit["active_frontier_points"]["last_point_count"] is not None,
        }
        rows.append({
            "scene_id": scene_id, "passed": all(checks.values()), "checks": checks,
            "completion_source": status.get("exploration_completion") or "legacy_unattributed_completion_sentinel",
            "frame_count": status.get("frame_count"), "room_count": room_count,
            "mapped_object_count": object_count, "gt_matched_mapped_box_count": len(matched),
            "mapped_free_area_m2": mapped_free_area,
            "estimated_dominant_floor_navmesh_area_m2": estimated_floor_area,
            "free_area_ratio": free_area_ratio,
            "last_active_frontier_points": bag_audit["active_frontier_points"]["last_point_count"],
            "last_dormant_frontier_points": bag_audit["dormant_frontier_points"]["last_point_count"],
        })
    report = {
        "format": "pre_map_vln.stage1_quality.v1", "scene_count": len(rows),
        "passed_scene_count": sum(row["passed"] for row in rows), "scenes": rows,
    }
    report_dir = args.root / "reports"
    report_dir.mkdir(parents=True, exist_ok=True)
    atomic_json(report_dir / "stage1_quality.json", report)
    fields = [
        "scene_id", "passed", "frame_count", "room_count", "mapped_object_count",
        "gt_matched_mapped_box_count", "mapped_free_area_m2",
        "estimated_dominant_floor_navmesh_area_m2", "free_area_ratio",
        "last_active_frontier_points", "last_dormant_frontier_points",
        "completion_source", "error",
    ]
    with (report_dir / "stage1_quality.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in fields} for row in rows)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report["passed_scene_count"] != len(rows):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
