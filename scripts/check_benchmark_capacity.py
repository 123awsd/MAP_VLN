#!/usr/bin/env python3
"""Fail early instead of risking a host crash from benchmark storage pressure."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/shared/PRE_MAP_VLN_benchmark_v1"))
    parser.add_argument("--minimum-free-gb", type=float, required=True)
    args = parser.parse_args()
    usage = shutil.disk_usage(args.root)
    gib = 1024 ** 3
    payload = {
        "root": str(args.root), "total_gib": usage.total / gib,
        "used_gib": usage.used / gib, "free_gib": usage.free / gib,
        "minimum_free_gib": args.minimum_free_gb,
        "passed": usage.free / gib >= args.minimum_free_gb,
    }
    print(json.dumps(payload, indent=2))
    if not payload["passed"]:
        raise SystemExit(
            f"benchmark requires {args.minimum_free_gb:.1f} GiB free; "
            f"only {payload['free_gib']:.1f} GiB remains"
        )


if __name__ == "__main__":
    main()
