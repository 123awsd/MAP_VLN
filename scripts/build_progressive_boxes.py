#!/usr/bin/env python3
"""Fuse Boxer detections and retain the first/last observation time per instance."""

import argparse
import csv
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party/boxer"))

from utils.fuse_3d_boxes import fuse_obbs_from_csv  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    parser.add_argument("--iou", type=float, default=0.2)
    parser.add_argument("--min-detections", type=int, default=2)
    parser.add_argument("--conf-threshold", type=float, default=0.3)
    args = parser.parse_args()

    with args.raw_csv.open(newline="", encoding="utf-8") as stream:
        raw_rows = list(csv.DictReader(stream))
    filtered_rows = [
        row for row in raw_rows if float(row["prob"]) >= args.conf_threshold
    ]

    instances = fuse_obbs_from_csv(
        str(args.raw_csv),
        str(args.output_csv),
        iou_threshold=args.iou,
        min_detections=args.min_detections,
        conf_threshold=args.conf_threshold,
    )
    if not instances:
        raise RuntimeError("Boxer fusion produced no progressive instances")

    with args.output_csv.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fused_rows = list(reader)
        fieldnames = list(reader.fieldnames or [])
    if len(fused_rows) != len(instances):
        raise RuntimeError("Fused CSV row count does not match fusion instances")

    for row, instance in zip(fused_rows, instances):
        detections = [filtered_rows[index] for index in instance.detection_indices]
        times = [int(item["time_ns"]) for item in detections]
        row["first_seen_ns"] = str(min(times))
        row["last_seen_ns"] = str(max(times))
        row["support_count"] = str(instance.support_count)

    fieldnames.extend(["first_seen_ns", "last_seen_ns", "support_count"])
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(fused_rows)
    print(f"wrote {len(fused_rows)} progressive instances to {args.output_csv}")


if __name__ == "__main__":
    main()
