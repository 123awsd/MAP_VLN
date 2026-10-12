#!/usr/bin/env python3
"""Evaluate an inspection instruction against ordered task-time RGB images."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json  # noqa: E402
from stage2.vlm_mission_evaluator import (  # noqa: E402
    PROMPT_VERSION,
    SYSTEM_PROMPT,
    build_user_prompt,
    evaluate_mission_images,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send a natural-language inspection task and ordered RGB evidence images to Qwen-VL."
    )
    parser.add_argument("--instruction", required=True, help="Original task instruction; do not paraphrase it.")
    parser.add_argument(
        "--image", action="append", required=True, type=Path,
        help="Evidence image in task order; repeat this option for multiple tasks.",
    )
    parser.add_argument("--output", required=True, type=Path, help="JSON result path.")
    parser.add_argument("--api-key-file", type=Path, default=ROOT / ".secrets/dashscope_api_key")
    args = parser.parse_args()

    result = evaluate_mission_images(args.instruction, args.image, args.api_key_file)
    record = {
        "format": "pre_map_vln.vlm_mission_evaluation_record.v1",
        "instruction": args.instruction,
        "images": [str(path.resolve()) for path in args.image],
        "prompt_version": PROMPT_VERSION,
        "system_prompt": SYSTEM_PROMPT,
        "user_prompt": build_user_prompt(args.instruction, len(args.image)),
        "evaluation": result,
    }
    atomic_json(args.output, record)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"Saved: {args.output.resolve()}", file=sys.stderr)


if __name__ == "__main__":
    main()
