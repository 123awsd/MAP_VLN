#!/usr/bin/env python3
"""Write and print the external baseline readiness report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from baselines.external import inspect_external_baselines  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "outputs/baselines/readiness.json",
    )
    parser.add_argument("--runtime", action="store_true", help="also import each isolated stack")
    args = parser.parse_args()
    report = inspect_external_baselines(ROOT, runtime=args.runtime)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
