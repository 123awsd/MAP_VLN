#!/usr/bin/env python3
"""Create a deterministic class/frame count report for a Boxer run."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


def csv_summary(path: Path) -> dict:
    counts = Counter()
    frames = defaultdict(set)
    if not path.is_file():
        return {"exists": False, "total": 0, "by_class": {}, "frames_by_class": {}}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            label = row.get("name", "unknown").strip().lower()
            counts[label] += 1
            frames[label].add(row.get("time_ns", row.get("frame_id", "unknown")))
    return {
        "exists": True,
        "total": sum(counts.values()),
        "classes": len(counts),
        "by_class": dict(sorted(counts.items())),
        "frames_by_class": {key: len(frames[key]) for key in sorted(frames)},
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("boxer_scene", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    manifest_path = args.boxer_scene / "processing_manifest.json"
    processing = json.loads(manifest_path.read_text(encoding="utf-8"))
    report = {
        "format": "pre_map_vln.boxer_summary.v1",
        "boxer_scene": str(args.boxer_scene.resolve()),
        "processing": {
            "status": processing.get("status"),
            "device": processing.get("device"),
            "cuda_available": processing.get("cuda_available"),
            "total_input_frames": processing.get("total_input_frames"),
            "summary": processing.get("summary", {}),
        },
        "detections_2d": csv_summary(args.boxer_scene / "owl_2dbbs.csv"),
        "detections_3d": csv_summary(args.boxer_scene / "boxer_3dbbs.csv"),
        "fused_3d": csv_summary(args.boxer_scene / "boxer_3dbbs_fused.csv"),
    }
    report["box"] = {
        "detections_2d": report["detections_2d"]["by_class"].get("box", 0),
        "valid_2d_frames": report["detections_2d"]["frames_by_class"].get("box", 0),
        "detections_3d": report["detections_3d"]["by_class"].get("box", 0),
        "valid_3d_frames": report["detections_3d"]["frames_by_class"].get("box", 0),
        "fused_instances": report["fused_3d"]["by_class"].get("box", 0),
    }
    output = args.output or args.boxer_scene / "boxer_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
