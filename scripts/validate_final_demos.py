#!/usr/bin/env python3
"""Acceptance checks and compact report for the six final demonstrations."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.task_graph import normalize_and_validate_task_graph  # noqa: E402


def require(value, message):
    if not value:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo-root", type=Path, default=ROOT / "outputs/final_demos")
    parser.add_argument("--bag-root", type=Path, default=ROOT / "outputs/bags/final_demos")
    parser.add_argument(
        "--scene-graph", type=Path,
        default=ROOT / "outputs/scene_graph/hm3d_stage1_complete_v3.json",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/final_demos/report.json")
    args = parser.parse_args()
    manifest = load_json(args.demo_root / "manifest.json")
    scene_graph = load_json(args.scene_graph)
    canonical_rooms = {
        room["id"]: {obj["id"] for obj in room.get("objects", [])}
        for room in scene_graph["rooms"]
    }
    require(
        all(room.get("space_role") != "room_fragment" for room in scene_graph["rooms"]),
        "scene graph still exposes fragments as canonical rooms",
    )
    require(len(manifest["demos"]) == 6, "exactly six demos are required")
    reports = []
    for entry in manifest["demos"]:
        name = entry["name"]
        run_dir = args.demo_root / name
        graph = normalize_and_validate_task_graph(load_json(run_dir / "task_graph.json"))
        execution = load_json(run_dir / "habitat_demo/habitat_execution.json")
        bag = load_json((args.bag_root / name).with_suffix(".manifest.json"))
        require(execution["status"] == "completed", f"{name}: execution incomplete")
        require(execution["frame_count"] == len(list((run_dir / "habitat_demo/frames").glob("frame_*.jpg"))),
                f"{name}: RGB is not frame-complete")
        require(bag["topic_counts"]["/stage2/rgb"] == len(execution["trajectory_xyz_yaw"]),
                f"{name}: bag RGB count mismatch")
        require(bag["topic_counts"].get("/stage2/recovery_search", 0) > 0,
                f"{name}: recovery visualization topic missing")
        recovery = execution.get("semantic_recovery", {})
        if name.startswith("recovery_"):
            require(recovery.get("enabled"), f"{name}: recovery was not enabled")
            require(len(recovery.get("events", [])) == 1, f"{name}: expected one VLM recovery call")
            event = recovery["events"][0]
            require(event["plan"].get("provenance", {}).get("planner") == "qwen_vlm",
                    f"{name}: candidate ranking is not VLM-provenanced")
            require(event["generated_candidate_ids"], f"{name}: no geometric viewpoints generated")
            for hypothesis in event["plan"].get("hypotheses", []):
                room_id = hypothesis["room_id"]
                require(room_id in canonical_rooms, f"{name}: recovery references non-canonical room {room_id}")
                require(hypothesis["anchor_object_id"] in canonical_rooms[room_id],
                        f"{name}: recovery anchor is not in canonical room {room_id}")
            require(any(item["outcome"] == "recovery_activated" for item in execution["observations"]),
                    f"{name}: activation event absent")
        else:
            require(graph["summary"]["conditional_rule_count"] >= 1,
                    f"{name}: no conditional constraint")
        reports.append({
            "name": name, "instruction": graph["instruction"],
            "status": execution["status"], "task_count": len(graph["tasks"]),
            "terminal_observations": len(execution["observations"]),
            "path_length_m": round(float(execution["path_length_m"]), 3),
            "frame_count": execution["frame_count"], "bag_size_bytes": bag["size_bytes"],
            "recovery_hypotheses": sum(len(item["plan"]["hypotheses"]) for item in recovery.get("events", [])),
            "final_outcomes": {item["task_id"]: item["verification_outcome"] for item in execution["observations"]
                               if item["outcome"] not in {"retry", "recovery_activated"}},
        })
    usage = load_json(ROOT / "outputs/stage2/api_usage.json")
    require(float(usage["total_estimated_cny"]) <= 20.0, "Qwen budget exceeded")
    report = {
        "format": "pre_map_vln.final_demo_report.v1", "status": "passed",
        "demo_count": 6, "conditional_demo_count": 3, "recovery_demo_count": 3,
        "qwen_total_estimated_cny": usage["total_estimated_cny"], "qwen_budget_cny": 20.0,
        "total_bag_bytes": sum(item["bag_size_bytes"] for item in reports), "demos": reports,
    }
    atomic_json(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
