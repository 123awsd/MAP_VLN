#!/usr/bin/env python3
"""Freeze a conditional task graph for a controlled, fixed-layout demo."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    graph = json.loads(args.input.read_text(encoding="utf-8"))
    active = [task for task in graph.get("tasks", []) if task.get("active_initially", True)]
    for task in active:
        task["active_initially"] = True
    graph["tasks"] = active
    graph["conditional_rules"] = []
    graph["execution_policy"] = {
        "mode": "static_demo",
        "assumption": "The operator guarantees the cup is on the refrigerator before flight.",
        "online_target_verification_required": False,
        "observation_dwell_s": 3.0,
    }
    summary = dict(graph.get("summary", {}))
    summary["task_count"] = len(active)
    summary["conditional_rule_count"] = 0
    graph["summary"] = summary
    args.output.write_text(json.dumps(graph, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
