#!/usr/bin/env python3
"""Compare two Boxer 2-D/3-D runs against an existing scene graph inventory."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def summarize(run_dir: Path, fused_name: str, reference_objects: list[dict]) -> dict:
    detections = read_rows(run_dir / "owl_2dbbs.csv")
    raw_3d = next(run_dir.glob("*_3dbbs.csv"), None)
    raw_rows = [] if raw_3d is None else read_rows(raw_3d)
    fused_rows = read_rows(run_dir / fused_name)
    manifest_path = run_dir / "processing_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    labels = Counter(row["name"] for row in detections)
    reference_by_label: dict[str, list[list[float]]] = {}
    for obj in reference_objects:
        reference_by_label.setdefault(str(obj["label"]), []).append(obj["center_xyz_m"])
    distances = []
    for row in fused_rows:
        candidates = reference_by_label.get(row["name"], [])
        if not candidates:
            continue
        center = [float(row[key]) for key in ("tx_world_object", "ty_world_object", "tz_world_object")]
        distances.append(min(math.dist(center, candidate) for candidate in candidates))
    frame_count = len({row["time_ns"] for row in detections})
    scores = [float(row["prob"]) for row in detections]
    reference_labels = set(reference_by_label)
    return {
        "run_dir": str(run_dir.resolve()),
        "detector": manifest.get("detector"),
        "detector_model": manifest.get("detector_model"),
        "sam2_enabled": bool(manifest.get("sam2_enabled")),
        "sampled_frames": int(manifest.get("total_input_frames", frame_count)),
        "detections_2d": len(detections),
        "detections_2d_per_frame": len(detections) / max(1, int(manifest.get("total_input_frames", frame_count))),
        "raw_boxes_3d": len(raw_rows),
        "detected_class_count": len(labels),
        "detections_in_reference_label_set": sum(count for label, count in labels.items() if label in reference_labels),
        "top_detected_classes": dict(labels.most_common(15)),
        "median_2d_score": None if not scores else statistics.median(scores),
        "fused_min2_instances": len(fused_rows),
        "fused_min2_class_count": len({row["name"] for row in fused_rows}),
        "fused_same_label_reference_matches": len(distances),
        "fused_within_1m_of_reference": sum(distance <= 1.0 for distance in distances),
        "median_nearest_same_label_reference_distance_m": (
            None if not distances else statistics.median(distances)
        ),
        "sam2_depth": manifest.get("summary", {}).get("sam2_depth"),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-dir", type=Path, required=True)
    parser.add_argument("--candidate-dir", type=Path, required=True)
    parser.add_argument("--baseline-fused", required=True)
    parser.add_argument("--candidate-fused", required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    scene = json.loads(args.scene_graph.read_text(encoding="utf-8"))
    report = {
        "format": "pre_map_vln.boxer_perception_comparison.v1",
        "reference_scene_graph": str(args.scene_graph.resolve()),
        "reference_is_ground_truth": False,
        "interpretation": (
            "The existing scene graph is a consistency reference, not semantic ground truth; "
            "same-label distances must not be reported as detector precision or recall."
        ),
        "baseline": summarize(args.baseline_dir, args.baseline_fused, scene.get("objects", [])),
        "candidate": summarize(args.candidate_dir, args.candidate_fused, scene.get("objects", [])),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
