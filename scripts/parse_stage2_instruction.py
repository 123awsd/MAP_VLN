#!/usr/bin/env python3
"""Parse a long instruction into the canonical stage-two task graph."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.vlm_task_parser import QwenTaskParser  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("instruction")
    parser.add_argument("--scene-graph", type=Path, default=ROOT / "outputs/scene_graph/hm3d_stage1.json")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/stage2/task_graph.json")
    parser.add_argument("--budget-cny", type=float, default=20.0)
    parser.add_argument("--no-cache", action="store_true")
    args = parser.parse_args()
    graph = QwenTaskParser(budget_cny=args.budget_cny).parse(
        args.instruction,
        load_json(args.scene_graph),
        use_cache=not args.no_cache,
    )
    atomic_json(args.output, graph)
    print(f"tasks={graph['summary']['task_count']} rules={graph['summary']['conditional_rule_count']} output={args.output}")


if __name__ == "__main__":
    main()
