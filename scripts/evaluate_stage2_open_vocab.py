#!/usr/bin/env python3
"""Benchmark the persistent local OWLv2 detector on saved Habitat evidence."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json  # noqa: E402
from stage2.open_vocab_detector import LocalOpenVocabularyDetector, target_found  # noqa: E402


TARGETS = {
    "lamp": "lamp",
    "chair": "chair",
    "door": "door",
    "tv": "television",
    "cabinet": "cabinet",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames-dir", type=Path, default=ROOT / "outputs/stage2/hm3d_example/habitat_demo/frames")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/stage2/open_vocab_perception")
    parser.add_argument("--threshold", type=float, default=0.20)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    images = sorted(args.frames_dir.glob("terminal_*.jpg"))
    if not images:
        raise FileNotFoundError(f"no terminal frames under {args.frames_dir}")
    prompts = sorted(set(TARGETS.values()))
    records = []
    with LocalOpenVocabularyDetector(prompts, threshold=args.threshold, timeout_s=300.0) as detector:
        startup = detector.ready
        for path in images:
            target = next(value for key, value in TARGETS.items() if key in path.stem)
            result = detector.detect(path)
            found = target_found(result, target)
            records.append({"image": path.name, "target": target, "found": found, **result})
            canvas = Image.open(path).convert("RGB")
            draw = ImageDraw.Draw(canvas)
            for item in result["detections"]:
                x1, y1, x2, y2 = item["bbox_xyxy"]
                color = (20, 110, 220) if item["label"] == target else (100, 100, 100)
                draw.rectangle((x1, y1, x2, y2), outline=color, width=3)
                draw.text((max(0, x1), max(0, y1 - 14)), f"{item['label']} {item['score']:.2f}", fill=color)
            canvas.save(args.output_dir / path.name, quality=95)
    latencies = [item["latency_ms"] for item in records]
    report = {
        "format": "pre_map_vln.open_vocab_benchmark.v1",
        "backend": "local_owlv2",
        "threshold": args.threshold,
        "startup": startup,
        "image_count": len(records),
        "target_recall": sum(item["found"] for item in records) / len(records),
        "latency_ms": {
            "mean": statistics.mean(latencies),
            "median": statistics.median(latencies),
            "max": max(latencies),
        },
        "records": records,
    }
    atomic_json(args.output_dir / "report.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
