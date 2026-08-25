#!/usr/bin/env python3
"""Classify fused 3-D boxes for structural-map filtering."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage1.structure_policy import QwenStructurePolicy, atomic_json, read_boxes  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--boxes", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--budget-cny", type=float, default=20.0)
    parser.add_argument("--no-cache", action="store_true")
    args = parser.parse_args()
    policy = QwenStructurePolicy(budget_cny=args.budget_cny).classify(
        read_boxes(args.boxes), use_cache=not args.no_cache
    )
    atomic_json(args.output, policy)
    print(json.dumps({
        "output": str(args.output),
        "instance_count": len(policy["instances"]),
        "removed_instance_count": sum(item["remove_from_structure_map"] for item in policy["instances"]),
        "cache_hit": policy.get("provenance", {}).get("cache_hit", False),
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
