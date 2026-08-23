#!/usr/bin/env python3
"""Run the dynamic condition/replanning state machine on a built map."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.grid_map import OccupancyGrid  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.mission_executor import simulate_dynamic_execution  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--grid-prefix", type=Path, required=True)
    parser.add_argument("--start", type=float, nargs=4, default=[0.0, 0.0, 1.0, 0.0])
    parser.add_argument("--outcomes-json", default="{}")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--inflation", type=float, default=0.10)
    args = parser.parse_args()
    trace = simulate_dynamic_execution(
        OccupancyGrid.load(args.grid_prefix, inflation_m=args.inflation),
        load_json(args.task_graph),
        load_json(args.candidates)["by_task"],
        list(args.start),
        outcomes=json.loads(args.outcomes_json),
    )
    atomic_json(args.output, trace)
    print(f"status={trace['status']} visits={len(trace['executed_visits'])} replans={len(trace['replans'])} path={trace['total_path_length_m']:.2f}m")


if __name__ == "__main__":
    main()
