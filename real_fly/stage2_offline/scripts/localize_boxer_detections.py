#!/usr/bin/env python3
"""Back-project Boxer 2D detections and cluster repeated real-world targets."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict, deque
from pathlib import Path

import numpy as np


def quaternion_xyzw_matrix(values) -> np.ndarray:
    x, y, z, w = np.asarray(values, dtype=np.float64)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm <= 0:
        raise ValueError("zero-length pose quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.asarray([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def connected_components(points: np.ndarray, radius: float) -> list[list[int]]:
    if len(points) == 0:
        return []
    distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    adjacency = distances <= radius
    unseen = set(range(len(points)))
    components = []
    while unseen:
        start = unseen.pop()
        queue = deque([start])
        component = [start]
        while queue:
            current = queue.popleft()
            neighbors = [int(index) for index in np.flatnonzero(adjacency[current]) if int(index) in unseen]
            for index in neighbors:
                unseen.remove(index)
                queue.append(index)
                component.append(index)
        components.append(component)
    return components


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--detections-2d", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-depth-m", type=float, default=0.35)
    parser.add_argument("--max-depth-m", type=float, default=8.0)
    parser.add_argument("--min-depth-valid-ratio", type=float, default=0.15)
    parser.add_argument("--inner-box-scale", type=float, default=1.00)
    parser.add_argument("--max-rgb-depth-dt", type=float, default=0.08)
    parser.add_argument("--max-depth-odometry-dt", type=float, default=0.08)
    parser.add_argument("--cluster-radius-m", type=float, default=0.80)
    parser.add_argument("--min-cluster-observations", type=int, default=2)
    args = parser.parse_args()

    manifest = json.loads((args.episode / "manifest.json").read_text(encoding="utf-8"))
    intrinsics = manifest["intrinsics"]
    fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
    cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
    records = {int(item["time_ns"]): item for item in manifest["frame_records"]}
    frame_paths = sorted(args.episode.glob("frame_*.npz"))
    frames = {}
    for path in frame_paths:
        with np.load(path) as data:
            time_ns = int(data["time_ns"])
            frames[time_ns] = {
                "depth": np.asarray(data["depth_m"], dtype=np.float32),
                "position": np.asarray(data["position"], dtype=np.float64),
                "rotation": quaternion_xyzw_matrix(data["orientation_xyzw"]),
            }

    raw_rows = []
    rejected = Counter()
    detections_2d_total = 0
    with args.detections_2d.open(newline="", encoding="utf-8") as stream:
        for detection_id, row in enumerate(csv.DictReader(stream)):
            detections_2d_total += 1
            time_ns = int(row["time_ns"])
            if time_ns not in frames or time_ns not in records:
                rejected["missing_episode_frame"] += 1
                continue
            record = records[time_ns]
            if record["rgb_depth_dt_sec"] > args.max_rgb_depth_dt:
                rejected["rgb_depth_time_error"] += 1
                continue
            if record["depth_odometry_dt_sec"] > args.max_depth_odometry_dt:
                rejected["depth_odometry_time_error"] += 1
                continue

            frame = frames[time_ns]
            depth = frame["depth"]
            source_width, source_height = depth.shape[1], depth.shape[0]
            detection_width = max(1, int(row["img_width"]))
            detection_height = max(1, int(row["img_height"]))
            x1 = float(row["x1"]) * source_width / detection_width
            x2 = float(row["x2"]) * source_width / detection_width
            y1 = float(row["y1"]) * source_height / detection_height
            y2 = float(row["y2"]) * source_height / detection_height
            center_x, center_y = (x1 + x2) * 0.5, (y1 + y2) * 0.5
            half_width = max(1.0, (x2 - x1) * args.inner_box_scale * 0.5)
            half_height = max(1.0, (y2 - y1) * args.inner_box_scale * 0.5)
            ix1 = max(0, int(math.floor(center_x - half_width)))
            ix2 = min(source_width, int(math.ceil(center_x + half_width)))
            iy1 = max(0, int(math.floor(center_y - half_height)))
            iy2 = min(source_height, int(math.ceil(center_y + half_height)))
            if ix2 <= ix1 or iy2 <= iy1:
                rejected["empty_box"] += 1
                continue

            patch = depth[iy1:iy2, ix1:ix2]
            valid = np.isfinite(patch) & (patch >= args.min_depth_m) & (patch <= args.max_depth_m)
            valid_ratio = float(valid.mean())
            if valid_ratio < args.min_depth_valid_ratio or not np.any(valid):
                rejected["insufficient_depth"] += 1
                continue
            vv_local, uu_local = np.nonzero(valid)
            z = patch[valid].astype(np.float64)
            median_z = float(np.median(z))
            mad = float(np.median(np.abs(z - median_z)))
            depth_gate = max(0.20, 3.0 * 1.4826 * mad)
            robust = np.abs(z - median_z) <= depth_gate
            if not np.any(robust):
                rejected["depth_outlier"] += 1
                continue
            u = uu_local[robust].astype(np.float64) + ix1
            v = vv_local[robust].astype(np.float64) + iy1
            z = z[robust]
            camera_points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
            camera_point = np.median(camera_points, axis=0)
            world_point = frame["rotation"] @ camera_point + frame["position"]
            if not np.isfinite(world_point).all():
                rejected["non_finite_world_point"] += 1
                continue

            raw_rows.append({
                "detection_id": detection_id,
                "time_ns": time_ns,
                "frame_index": int(record["index"]),
                "label": row["name"].strip().lower(),
                "confidence": float(row["prob"]),
                "x1": float(row["x1"]), "y1": float(row["y1"]),
                "x2": float(row["x2"]), "y2": float(row["y2"]),
                "depth_m": float(np.median(z)),
                "depth_valid_ratio": valid_ratio,
                "rgb_depth_dt_sec": float(record["rgb_depth_dt_sec"]),
                "depth_odometry_dt_sec": float(record["depth_odometry_dt_sec"]),
                "camera_x_m": float(camera_point[0]),
                "camera_y_m": float(camera_point[1]),
                "camera_z_m": float(camera_point[2]),
                "world_x_m": float(world_point[0]),
                "world_y_m": float(world_point[1]),
                "world_z_m": float(world_point[2]),
                "extrinsic_status": manifest.get("extrinsic_status", "unknown"),
            })

    clustered_rows = []
    grouped = defaultdict(list)
    for row in raw_rows:
        grouped[row["label"]].append(row)
    cluster_id = 0
    for label in sorted(grouped):
        items = grouped[label]
        points = np.asarray([[item["world_x_m"], item["world_y_m"], item["world_z_m"]] for item in items])
        for component in connected_components(points, args.cluster_radius_m):
            selected = [items[index] for index in component]
            weights = np.asarray([max(1e-6, item["confidence"] * item["depth_valid_ratio"]) for item in selected])
            center = np.average(points[component], axis=0, weights=weights)
            distances = np.linalg.norm(points[component] - center, axis=1)
            clustered_rows.append({
                "cluster_id": cluster_id,
                "label": label,
                "world_x_m": float(center[0]),
                "world_y_m": float(center[1]),
                "world_z_m": float(center[2]),
                "observations": len(selected),
                "unique_frames": len({item["frame_index"] for item in selected}),
                "confidence_mean": float(np.mean([item["confidence"] for item in selected])),
                "confidence_max": float(np.max([item["confidence"] for item in selected])),
                "depth_valid_ratio_mean": float(np.mean([item["depth_valid_ratio"] for item in selected])),
                "position_spread_p90_m": float(np.percentile(distances, 90)),
                "planning_eligible": len(selected) >= args.min_cluster_observations,
                "extrinsic_status": manifest.get("extrinsic_status", "unknown"),
            })
            cluster_id += 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "target_points_raw.csv", raw_rows)
    write_csv(args.output_dir / "target_points_clustered.csv", clustered_rows)
    (args.output_dir / "target_points_raw.json").write_text(
        json.dumps(raw_rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (args.output_dir / "target_points_clustered.json").write_text(
        json.dumps(clustered_rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    raw_class_counts = Counter(row["label"] for row in raw_rows)
    clustered_class_counts = Counter(row["label"] for row in clustered_rows)
    eligible_class_counts = Counter(row["label"] for row in clustered_rows if row["planning_eligible"])
    summary = {
        "format": "pre_map_vln.target_localization.v1",
        "episode_frames": len(frames),
        "detections_2d_total": detections_2d_total,
        "valid_depth_matches": len(raw_rows),
        "rejected": dict(sorted(rejected.items())),
        "raw_target_points": len(raw_rows),
        "raw_by_class": dict(sorted(raw_class_counts.items())),
        "clustered_targets": len(clustered_rows),
        "clustered_by_class": dict(sorted(clustered_class_counts.items())),
        "planning_eligible_targets": sum(row["planning_eligible"] for row in clustered_rows),
        "planning_eligible_by_class": dict(sorted(eligible_class_counts.items())),
        "box": {
            "raw_points": raw_class_counts.get("box", 0),
            "clusters": clustered_class_counts.get("box", 0),
            "planning_eligible_clusters": eligible_class_counts.get("box", 0),
        },
        "thresholds": {
            "min_depth_m": args.min_depth_m,
            "max_depth_m": args.max_depth_m,
            "min_depth_valid_ratio": args.min_depth_valid_ratio,
            "inner_box_scale": args.inner_box_scale,
            "max_rgb_depth_dt_sec": args.max_rgb_depth_dt,
            "max_depth_odometry_dt_sec": args.max_depth_odometry_dt,
            "cluster_radius_m": args.cluster_radius_m,
            "min_cluster_observations": args.min_cluster_observations,
        },
        "extrinsic_status": manifest.get("extrinsic_status", "unknown"),
        "coordinate_warning": (
            "Camera-to-body extrinsic is provisional; map-frame points are suitable for offline "
            "pipeline validation but require a measured rigid transform for metric flight use."
            if manifest.get("extrinsic_status") != "accepted" else ""
        ),
    }
    (args.output_dir / "localization_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
