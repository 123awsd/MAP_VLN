#!/usr/bin/env python3
"""Write a small human-readable instruction manifest beside a Stage 2 bag."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402


def build_manifest(task_graph: dict[str, Any], bag_path: Path) -> dict[str, Any]:
    return {
        "format": "pre_map_vln.stage2_bag_instruction.v1",
        "bag_filename": bag_path.name,
        "instruction": str(task_graph.get("instruction", "")).strip(),
        "tasks": [
            {
                "id": task.get("id"),
                "action": task.get("action"),
                "verification_label": task.get("verification_label"),
                "target": task.get("target"),
            }
            for task in task_graph.get("tasks", [])
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("task_graph", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    args = parser.parse_args()
    output = args.output or args.bag.with_suffix(".instruction.json")
    atomic_json(output, build_manifest(load_json(args.task_graph), args.bag))
    print(f"instruction_json={output.resolve()}")


if __name__ == "__main__":
    main()
