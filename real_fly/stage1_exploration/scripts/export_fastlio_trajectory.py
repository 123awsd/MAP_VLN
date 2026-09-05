#!/usr/bin/env python3
"""Export FAST-LIO odometry as CSV/TUM plus a compact top-down preview."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--topic", default="/Odometry")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=[args.topic]):
            stamp = message.header.stamp.to_sec() or bag_time.to_sec()
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            rows.append([stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if not rows:
        raise SystemExit(f"no messages on {args.topic}")

    csv_path = args.output_dir / "trajectory.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_sec", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"])
        writer.writerows(rows)
    tum_path = args.output_dir / "trajectory_tum.txt"
    with tum_path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(" ".join(f"{value:.9f}" for value in row) + "\n")

    points = np.asarray([row[1:4] for row in rows], dtype=np.float64)
    lower = points[:, :2].min(axis=0)
    upper = points[:, :2].max(axis=0)
    extent = np.maximum(upper - lower, 1e-6)
    canvas = np.full((900, 1200, 3), 245, dtype=np.uint8)
    margin = 50
    scale = min((canvas.shape[1] - 2 * margin) / extent[0],
                (canvas.shape[0] - 2 * margin) / extent[1])
    pixels = np.empty((len(points), 2), dtype=np.int32)
    pixels[:, 0] = np.rint(margin + (points[:, 0] - lower[0]) * scale).astype(np.int32)
    pixels[:, 1] = np.rint(canvas.shape[0] - margin - (points[:, 1] - lower[1]) * scale).astype(np.int32)
    z_min, z_max = float(points[:, 2].min()), float(points[:, 2].max())
    z_span = max(1e-9, z_max - z_min)
    for first, second, z in zip(pixels, pixels[1:], points[1:, 2]):
        value = np.uint8(round(255 * (z - z_min) / z_span))
        color = tuple(int(item) for item in cv2.applyColorMap(np.asarray([[value]], np.uint8), cv2.COLORMAP_TURBO)[0, 0])
        cv2.line(canvas, tuple(first), tuple(second), color, 2, cv2.LINE_AA)
    cv2.circle(canvas, tuple(pixels[0]), 8, (0, 180, 0), -1)
    cv2.circle(canvas, tuple(pixels[-1]), 8, (0, 0, 220), -1)
    cv2.putText(canvas, f"FAST-LIO trajectory: {len(rows)} poses, z=[{z_min:.2f}, {z_max:.2f}] m",
                (30, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
    preview_path = args.output_dir / "trajectory_topdown.png"
    cv2.imwrite(str(preview_path), canvas)

    summary = {
        "poses": len(rows),
        "first_stamp_sec": rows[0][0],
        "last_stamp_sec": rows[-1][0],
        "duration_sec": rows[-1][0] - rows[0][0],
        "position_min_xyz_m": points.min(axis=0).tolist(),
        "position_max_xyz_m": points.max(axis=0).tolist(),
        "csv": str(csv_path.resolve()),
        "tum": str(tum_path.resolve()),
        "preview": str(preview_path.resolve()),
    }
    (args.output_dir / "trajectory_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
