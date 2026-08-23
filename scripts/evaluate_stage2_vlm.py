#!/usr/bin/env python3
"""Compare Qwen3-VL terminal-frame decisions against Habitat semantic evidence."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.vlm_verifier import QwenImageVerifier  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, default=ROOT / "outputs/stage2/hm3d_example")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/stage2/vlm_perception/report.json")
    parser.add_argument("--no-cache", action="store_true")
    args = parser.parse_args()
    task_graph = load_json(args.run_dir / "task_graph.json")
    execution_dir = args.run_dir / "habitat_demo"
    execution = load_json(execution_dir / "habitat_execution.json")
    tasks = {task["id"]: task for task in task_graph["tasks"]}
    verifier = QwenImageVerifier()
    records = []
    for observation in execution["observations"]:
        task = tasks[observation["task_id"]]
        rgb_path = execution_dir / observation["rgb"]
        vlm = verifier.verify([rgb_path], task, use_cache=not args.no_cache)
        semantic = observation["verification"]
        records.append({
            "task_id": task["id"], "target_label": task["verification_label"],
            "rgb": str(rgb_path.relative_to(ROOT)),
            "semantic_found": bool(semantic.get("semantic_found", semantic["found"])),
            "semantic_pixels": int(semantic["matching_pixels"]),
            "semantic_threshold": int(semantic["minimum_pixels"]),
            "vlm_found": vlm["found"], "vlm_confidence": vlm["confidence"],
            "agreement": bool(semantic.get("semantic_found", semantic["found"])) == vlm["found"],
            "vlm": vlm,
        })
        print(f"task={task['id']} semantic={semantic['found']} vlm={vlm['found']} confidence={vlm['confidence']:.2f}", flush=True)
    agreement = sum(record["agreement"] for record in records)
    report = {
        "format": "pre_map_vln.stage2_vlm_perception.v1",
        "status": "passed",
        "model": "qwen3-vl-plus", "record_count": len(records),
        "agreement_count": agreement, "agreement_rate": agreement / max(1, len(records)),
        "records": records,
        "note": "Semantic decisions include the controlled television threshold; disagreements require evidence review.",
    }
    atomic_json(args.output, report)
    print(f"status=passed records={len(records)} agreement={agreement}/{len(records)} output={args.output}")


if __name__ == "__main__":
    main()
