#!/usr/bin/env python3
"""Create six fixed, auditable final-demo task graphs and their initial candidates."""

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
from stage2.task_graph import normalize_and_validate_task_graph  # noqa: E402


def task(task_id, label, *, prerequisites=(), room=None, reference=None,
         verification=None, search_mode="fixed"):
    return {
        "id": task_id, "action": "inspect",
        "target": {"label": label, "room": room, "reference": reference},
        "verification_label": verification or label,
        "spatial_constraints": {
            "relation": "near", "distance_m": [1.0, 2.2], "height_m": None,
            "face_target": True, "visibility_required": True,
        },
        "prerequisites": list(prerequisites),
        "success_outcome": "found",
        "search_policy": {"mode": search_mode, "maximum_location_hypotheses": 3},
    }


def graph(instruction, tasks, rules=()):
    return normalize_and_validate_task_graph({
        "instruction": instruction, "tasks": tasks, "conditional_rules": list(rules),
    })


def rule(source, activate, *, outcome="not_found", skip=()):
    return {
        "source_task_id": source, "if_outcome": outcome,
        "activate_task_ids": list(activate), "skip_task_ids": list(skip),
    }


def specifications():
    return {
        "conditional_01_branch": {
            "instruction": "我准备休息了，先带我看看卧室的床，再去客厅确认电视。要是电视没找到，就到另一边卧室的柜子旁再找找；路上也顺便看看那把椅子。",
            "tasks": [
                task("check_bed", "bed"),
                task("check_tv", "television", prerequisites=("check_bed",)),
                task("search_tv_near_cabinet", "cabinet", prerequisites=("check_tv",), verification="television"),
                task("check_chair", "chair"),
            ],
            "rules": [rule("check_tv", ("search_tv_near_cabinet",))],
            "anchors": {"check_bed": "boxer_4", "check_tv": "boxer_20", "search_tv_near_cabinet": "boxer_5", "check_chair": "boxer_22"},
            "controlled_stale": ["boxer_20"],
            "controlled_found": [],
        },
        "conditional_02_skip": {
            "instruction": "出门前帮我巡一遍：先确认卧室的床，再看看厨房那扇门，之后去检查远处客厅的电视。电视正常的话就不用再去架子附近折腾了，最后看一下床边的灯。",
            "tasks": [
                task("check_bed", "bed"), task("check_kitchen_door", "door", prerequisites=("check_bed",)),
                task("check_tv", "television", prerequisites=("check_kitchen_door",)),
                task("backup_shelf", "shelf", prerequisites=("check_tv",), verification="television"),
                task("check_lamp", "lamp", prerequisites=("check_tv",)),
            ],
            "rules": [rule("check_tv", ("backup_shelf",))],
            "anchors": {"check_bed": "boxer_6", "check_kitchen_door": "boxer_16", "check_tv": "boxer_26", "backup_shelf": "boxer_25", "check_lamp": "boxer_2"},
            "controlled_stale": [],
            "controlled_found": ["boxer_26"],
        },
        "conditional_03_parallel": {
            "instruction": "帮我做一次屋内巡查。床边的灯看过以后再确认卧室电视；椅子和门哪一个先看都可以。如果电视没有看到，再到厨房柜子附近补查一次。",
            "tasks": [
                task("check_lamp", "lamp"), task("check_bedroom_tv", "television", prerequisites=("check_lamp",)),
                task("check_chair", "chair"), task("check_door", "door"),
                task("backup_cabinet", "cabinet", prerequisites=("check_bedroom_tv",), verification="television"),
            ],
            "rules": [rule("check_bedroom_tv", ("backup_cabinet",))],
            "anchors": {"check_lamp": "boxer_2", "check_bedroom_tv": "boxer_9", "check_chair": "boxer_22", "check_door": "boxer_16", "backup_cabinet": "boxer_15"},
            "controlled_stale": ["boxer_9"],
            "controlled_found": [],
        },
        "recovery_01_tv": {
            "instruction": "去卧室柜子旁找一下电视；如果原来记的位置没有，就根据整张房屋语义地图继续找，找到为止。",
            "tasks": [task("find_tv", "cabinet", verification="television", search_mode="semantic_recovery")],
            "rules": [], "anchors": {"find_tv": "boxer_5"}, "controlled_stale": ["boxer_5"],
            "controlled_found": ["boxer_20"],
        },
        "recovery_02_lamp": {
            "instruction": "帮我找那盏灯，先去客厅架子旁看看；那里没有的话，就从其他语义相关的房间和位置继续搜索。",
            "tasks": [task("find_lamp", "shelf", verification="lamp", search_mode="semantic_recovery")],
            "rules": [], "anchors": {"find_lamp": "boxer_25"}, "controlled_stale": ["boxer_25"],
            "controlled_found": ["boxer_11"],
        },
        "recovery_03_exhausted": {
            "instruction": "去厨房柜子附近找一下微波炉；如果没在旧位置，就结合房间用途和周围家具继续找，没有也要明确告诉我。",
            "tasks": [task("find_microwave", "cabinet", verification="microwave", search_mode="semantic_recovery")],
            "rules": [], "anchors": {"find_microwave": "boxer_15"}, "controlled_stale": ["boxer_15"],
            "controlled_found": [],
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--grid-prefix", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    scene = load_json(args.scene_graph)
    grid = OccupancyGrid.load(args.grid_prefix, inflation_m=0.10)
    manifest = {"format": "pre_map_vln.final_demos.v1", "demos": []}
    for name, spec in specifications().items():
        run_dir = args.output_root / name
        task_graph = graph(spec["instruction"], spec["tasks"], spec["rules"])
        by_task = {}
        for current in task_graph["tasks"]:
            object_id = spec["anchors"][current["id"]]
            values = generate_candidates(grid, scene, current, max_candidates=6,
                                         allowed_object_ids={object_id}, prefer_room=False)
            if not values:
                raise RuntimeError(f"{name}/{current['id']} has no candidate for {object_id}")
            by_task[current["id"]] = values
        run_dir.mkdir(parents=True, exist_ok=True)
        atomic_json(run_dir / "task_graph.json", task_graph)
        atomic_json(run_dir / "candidates.json", {
            "format": "pre_map_vln.candidates.v1", "selection_mode": "fixed_auditable_demo",
            "selected_objects": spec["anchors"], "by_task": by_task,
        })
        atomic_json(run_dir / "demo_config.json", {
            "name": name, "controlled_stale_object_ids": spec["controlled_stale"],
            "controlled_found_object_ids": spec["controlled_found"],
            "semantic_recovery": name.startswith("recovery_"),
        })
        manifest["demos"].append({"name": name, "instruction": spec["instruction"], "directory": str(run_dir)})
    atomic_json(args.output_root / "manifest.json", manifest)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
