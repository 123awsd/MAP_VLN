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
    parser.add_argument(
        "--batch-size", type=int, default=40,
        help="maximum boxes per Qwen request; keeps large indoor vocabularies within the JSON output limit",
    )
    parser.add_argument("--no-cache", action="store_true")
    args = parser.parse_args()
    if args.batch_size < 1:
        parser.error("--batch-size must be positive")
    boxes = read_boxes(args.boxes)
    classifier = QwenStructurePolicy(budget_cny=args.budget_cny)
    batches = []
    for start in range(0, len(boxes), args.batch_size):
        batches.append(classifier.classify(
            boxes[start:start + args.batch_size], use_cache=not args.no_cache
        ))
    instances = [item for batch in batches for item in batch["instances"]]
    policy = {
        "format": "pre_map_vln.structure_policy.v1",
        "minimum_remove_confidence": batches[0]["minimum_remove_confidence"],
        "instances": instances,
        "provenance": {
            "classifier": "qwen_llm_batched",
            "batch_size": args.batch_size,
            "batch_count": len(batches),
            "cache_hit": all(
                batch.get("provenance", {}).get("cache_hit", False) for batch in batches
            ),
            "batches": [batch.get("provenance", {}) for batch in batches],
        },
    }
    atomic_json(args.output, policy)
    print(json.dumps({
        "output": str(args.output),
        "instance_count": len(policy["instances"]),
        "removed_instance_count": sum(item["remove_from_structure_map"] for item in policy["instances"]),
        "cache_hit": policy.get("provenance", {}).get("cache_hit", False),
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
