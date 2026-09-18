#!/usr/bin/env python3
"""Compare RGB-D and registered-LiDAR localization for existing 2-D detections.

This is an offline diagnostic.  It never changes the existing semantic map.
The registered cloud and camera optical pose are both expressed in the
FAST-LIO world frame, so projecting one into the other directly audits the
calibration/synchronization chain.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
import rosbag


def stamp(message, bag_time) -> float:
    value = message.header.stamp.to_sec()
    return float(value if value > 0 else bag_time.to_sec())


def quaternion_matrix(values) -> np.ndarray:
    x, y, z, w = np.asarray(values, dtype=np.float64)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def cloud_xyz(message) -> np.ndarray:
    offsets = {field.name: field.offset for field in message.fields}
    count = message.width * message.height
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(count, message.point_step)
    xyz = np.column_stack([
        raw[:, offsets[name]:offsets[name]+4].copy().view("<f4").reshape(-1)
        for name in ("x", "y", "z")
    ]).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def load_pcd_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    marker = raw.find(b"DATA binary\n")
    if marker < 0:
        raise RuntimeError("only binary PCD is supported")
    header = raw[:marker].decode("ascii", errors="strict")
    fields = next(line.split()[1:] for line in header.splitlines() if line.startswith("FIELDS "))
    sizes = [int(v) for v in next(line.split()[1:] for line in header.splitlines() if line.startswith("SIZE "))]
    types = next(line.split()[1:] for line in header.splitlines() if line.startswith("TYPE "))
    counts = [int(v) for v in next(line.split()[1:] for line in header.splitlines() if line.startswith("COUNT "))]
    formats = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 4): "<u4", ("I", 4): "<i4"}
    dtype_fields, offset = [], 0
    for name, size, kind, count in zip(fields, sizes, types, counts):
        fmt = formats[(kind, size)]
        dtype_fields.append((name, fmt, (count,)) if count > 1 else (name, fmt))
        offset += size * count
    data = np.frombuffer(raw[marker+len(b"DATA binary\n"):], dtype=np.dtype(dtype_fields))
    return np.column_stack([data[name] for name in ("x", "y", "z")]).astype(np.float64)


def nearest_map_distance(point: np.ndarray, map_points: np.ndarray) -> float:
    """Small-query NumPy nearest neighbor, avoiding a SciPy runtime dependency."""
    best = math.inf
    for start in range(0, len(map_points), 50000):
        delta = map_points[start:start+50000] - point
        best = min(best, float(np.sqrt(np.min(np.einsum("ij,ij->i", delta, delta)))))
    return best


def inner_box(row, width: int, height: int, scale: float) -> tuple[float, float, float, float]:
    dw, dh = max(1, int(row["img_width"])), max(1, int(row["img_height"]))
    x1, x2 = float(row["x1"]) * width/dw, float(row["x2"]) * width/dw
    y1, y2 = float(row["y1"]) * height/dh, float(row["y2"]) * height/dh
    cx, cy = (x1+x2)/2, (y1+y2)/2
    hw, hh = max(2.0, (x2-x1)*scale/2), max(2.0, (y2-y1)*scale/2)
    return max(0.0, cx-hw), max(0.0, cy-hh), min(width-1.0, cx+hw), min(height-1.0, cy+hh)


def select_depth_cluster(points_camera: np.ndarray, indices: np.ndarray,
                         bin_width: float, min_points: int) -> np.ndarray:
    if len(indices) < min_points:
        return np.empty(0, dtype=np.int64)
    z = points_camera[indices, 2]
    bins = np.floor(z / bin_width).astype(np.int64)
    values, counts = np.unique(bins, return_counts=True)
    viable = [(int(count), int(value)) for value, count in zip(values, counts) if count >= min_points]
    if not viable:
        return np.empty(0, dtype=np.int64)
    # Prefer a well-supported surface; for similar support choose the nearer one.
    best_count = max(item[0] for item in viable)
    best_bin = min(value for count, value in viable if count >= max(min_points, int(0.65*best_count)))
    center = (best_bin + 0.5) * bin_width
    return indices[np.abs(z-center) <= bin_width]


def connected_components(points: np.ndarray, radius: float) -> list[list[int]]:
    if not len(points):
        return []
    adjacency = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2) <= radius
    unseen = set(range(len(points)))
    components = []
    while unseen:
        start = unseen.pop()
        queue, component = deque([start]), [start]
        while queue:
            current = queue.popleft()
            for index in np.flatnonzero(adjacency[current]):
                index = int(index)
                if index in unseen:
                    unseen.remove(index); queue.append(index); component.append(index)
        components.append(component)
    return components


def write_lidar_semantic_preview(rows: list[dict], output: Path, radius_m: float,
                                 minimum_observations: int) -> int:
    raw = [{
        "detection_id": row["detection_id"], "time_ns": row["time_ns"],
        "frame_index": row["frame_index"], "label": row["label"],
        "confidence": row["confidence"],
        "depth_valid_ratio": min(1.0, row["lidar_points_selected"] / 10.0),
        "world_x_m": row["lidar_world_x_m"], "world_y_m": row["lidar_world_y_m"],
        "world_z_m": row["lidar_world_z_m"], "localization_source": "registered_lidar",
    } for row in rows]
    (output / "target_points_raw.json").write_text(
        json.dumps(raw, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    grouped = defaultdict(list)
    for row in raw:
        grouped[row["label"]].append(row)
    clusters = []
    for label in sorted(grouped):
        items = grouped[label]
        points = np.asarray([[row[k] for k in ("world_x_m", "world_y_m", "world_z_m")]
                             for row in items], dtype=float)
        for component in connected_components(points, radius_m):
            selected = [items[index] for index in component]
            weights = np.asarray([max(1e-6, row["confidence"] * row["depth_valid_ratio"])
                                  for row in selected])
            center = np.average(points[component], axis=0, weights=weights)
            distances = np.linalg.norm(points[component] - center, axis=1)
            clusters.append({
                "cluster_id": len(clusters), "label": label,
                "world_x_m": float(center[0]), "world_y_m": float(center[1]),
                "world_z_m": float(center[2]), "observations": len(selected),
                "unique_frames": len({row["frame_index"] for row in selected}),
                "confidence_mean": float(np.mean([row["confidence"] for row in selected])),
                "confidence_max": float(np.max([row["confidence"] for row in selected])),
                "depth_valid_ratio_mean": float(np.mean([row["depth_valid_ratio"] for row in selected])),
                "position_spread_p90_m": float(np.percentile(distances, 90)),
                "planning_eligible": len(selected) >= minimum_observations,
                "extrinsic_status": "diagnostic_lidar_projection",
                "localization_source": "registered_lidar",
            })
    (output / "target_points_clustered.json").write_text(
        json.dumps(clusters, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return len(clusters)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--detections", type=Path, required=True)
    parser.add_argument("--mapping-bag", type=Path, required=True)
    parser.add_argument("--source-bag", type=Path,
                        help="Original sensor bag; used to recover color distortion coefficients")
    parser.add_argument("--map-pcd", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-cloud-dt", type=float, default=0.06)
    parser.add_argument("--inner-box-scale", type=float, default=0.70)
    parser.add_argument("--min-lidar-points", type=int, default=3)
    parser.add_argument("--depth-bin-m", type=float, default=0.30)
    parser.add_argument("--cluster-radius-m", type=float, default=0.80)
    parser.add_argument("--min-cluster-observations", type=int, default=2)
    parser.add_argument("--visualizations", type=int, default=18)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite existing output: {args.output}")
    args.output.mkdir(parents=True)
    viz_dir = args.output / "visualizations"
    viz_dir.mkdir()

    manifest = json.loads((args.episode / "manifest.json").read_text())
    intr = manifest["intrinsics"]
    K = np.asarray([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]], float)
    distortion = np.zeros(5, dtype=np.float64)
    if args.source_bag:
        with rosbag.Bag(str(args.source_bag)) as bag:
            for _, message, _ in bag.read_messages(topics=["/camera/color/camera_info"]):
                distortion = np.asarray(message.D, dtype=np.float64)
                break
    frames = {}
    for path in sorted(args.episode.glob("frame_*.npz")):
        with np.load(path) as data:
            time_ns = int(data["time_ns"])
            frames[time_ns] = {
                "index": int(path.stem.split("_")[-1]),
                "rgb": np.asarray(data["rgb"]),
                "depth": np.asarray(data["depth_m"], float),
                "position": np.asarray(data["position"], float),
                "rotation": quaternion_matrix(data["orientation_xyzw"]),
                "time": float(data["depth_time_sec"]),
            }
    frame_times = np.asarray([item["time"] for item in frames.values()])
    frame_keys = list(frames)
    aligned_depths, aligned_dt = {}, {key: math.inf for key in frames}
    if args.source_bag:
        with rosbag.Bag(str(args.source_bag)) as bag:
            for _, message, bag_time in bag.read_messages(topics=["/camera/aligned_depth_to_color/image_raw"]):
                current = stamp(message, bag_time)
                index = int(np.argmin(np.abs(frame_times-current)))
                key = frame_keys[index]
                delta = abs(frames[key]["time"]-current)
                if delta <= 0.10 and delta < aligned_dt[key]:
                    aligned_depths[key] = (np.frombuffer(message.data, dtype="<u2")
                                           .reshape(message.height, message.width).copy().astype(np.float32)/1000.0)
                    aligned_dt[key] = delta
    best_clouds, best_dt = {}, {key: math.inf for key in frames}
    with rosbag.Bag(str(args.mapping_bag)) as bag:
        for _, message, bag_time in bag.read_messages(topics=["/cloud_registered"]):
            current = stamp(message, bag_time)
            index = int(np.argmin(np.abs(frame_times-current)))
            key = frame_keys[index]
            delta = abs(frames[key]["time"]-current)
            if delta <= args.max_cloud_dt and delta < best_dt[key]:
                best_clouds[key] = cloud_xyz(message)
                best_dt[key] = delta

    map_points = load_pcd_xyz(args.map_pcd)
    detections = list(csv.DictReader(args.detections.open(newline="", encoding="utf-8")))
    rows, rejected = [], Counter()
    projected_cache = {}
    for detection_id, detection in enumerate(detections):
        key = int(detection["time_ns"])
        if key not in frames or key not in best_clouds:
            rejected["missing_frame_or_cloud"] += 1
            continue
        frame, world = frames[key], best_clouds[key]
        cache_key = key
        if cache_key not in projected_cache:
            camera = (world-frame["position"]) @ frame["rotation"]
            valid = (camera[:, 2] > 0.35) & (camera[:, 2] < 20.0)
            camera, world_valid = camera[valid], world[valid]
            uv, _ = cv2.projectPoints(camera.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, distortion)
            uv = uv.reshape(-1, 2)
            projected_cache[cache_key] = (camera, world_valid, uv)
        camera, world_valid, uv = projected_cache[cache_key]
        height, width = frame["rgb"].shape[:2]
        x1, y1, x2, y2 = inner_box(detection, width, height, args.inner_box_scale)
        inside = np.flatnonzero((uv[:, 0] >= x1) & (uv[:, 0] <= x2) &
                                (uv[:, 1] >= y1) & (uv[:, 1] <= y2))
        selected = select_depth_cluster(camera, inside, args.depth_bin_m, args.min_lidar_points)
        if not len(selected):
            rejected["insufficient_lidar_points"] += 1
            continue
        lidar_center = np.median(world_valid[selected], axis=0)
        patch = frame["depth"][int(y1):int(y2)+1, int(x1):int(x2)+1]
        valid_depth = patch[np.isfinite(patch) & (patch >= 0.35) & (patch <= 8.0)]
        rgbd_center = None
        if len(valid_depth):
            z = float(np.median(valid_depth))
            uc, vc = (x1+x2)/2, (y1+y2)/2
            camera_center = np.asarray([(uc-K[0, 2])*z/K[0, 0], (vc-K[1, 2])*z/K[1, 1], z])
            rgbd_center = frame["rotation"] @ camera_center + frame["position"]
        row = {
            "detection_id": detection_id, "frame_index": frame["index"], "time_ns": key,
            "label": detection["name"].strip().lower(), "confidence": float(detection["prob"]),
            "cloud_dt_ms": 1000.0*best_dt[key], "lidar_points_in_box": int(len(inside)),
            "lidar_points_selected": int(len(selected)), "lidar_depth_m": float(np.median(camera[selected, 2])),
            "lidar_world_x_m": float(lidar_center[0]), "lidar_world_y_m": float(lidar_center[1]),
            "lidar_world_z_m": float(lidar_center[2]),
            "lidar_to_map_m": nearest_map_distance(lidar_center, map_points),
            "rgbd_depth_m": None, "rgbd_lidar_delta_m": None,
        }
        if rgbd_center is not None:
            row["rgbd_depth_m"] = z
            row["rgbd_lidar_delta_m"] = float(np.linalg.norm(rgbd_center-lidar_center))
        rows.append(row)

    with (args.output / "lidar_localization.csv").open("w", newline="", encoding="utf-8") as stream:
        if rows:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (args.output / "lidar_localization.json").write_text(json.dumps(rows, indent=2)+"\n")

    comparable = [row for row in rows if row["rgbd_lidar_delta_m"] is not None]
    lidar_clusters = write_lidar_semantic_preview(
        rows, args.output, args.cluster_radius_m, args.min_cluster_observations)

    # Direct registration audit: compare RGB-D and LiDAR depth at the same
    # projected pixel, independent of semantic detections and box sampling.
    registration_rows = []
    hardware_aligned_rows = []
    for key, (camera, _, uv) in projected_cache.items():
        frame = frames[key]
        depth = frame["depth"]
        height, width = depth.shape
        px = np.rint(uv[:, 0]).astype(np.int64)
        py = np.rint(uv[:, 1]).astype(np.int64)
        valid = ((px >= 0) & (px < width) & (py >= 0) & (py < height)
                 & (camera[:, 2] >= 0.35) & (camera[:, 2] <= 8.0))
        indices = np.flatnonzero(valid)
        if not len(indices):
            continue
        rgbd = depth[py[indices], px[indices]]
        good = np.isfinite(rgbd) & (rgbd >= 0.35) & (rgbd <= 8.0)
        indices, rgbd = indices[good], rgbd[good]
        if not len(indices):
            continue
        residual = rgbd-camera[indices, 2]
        for value in residual:
            registration_rows.append(float(value))
        if key in aligned_depths:
            hardware = aligned_depths[key][py[indices], px[indices]]
            hardware_good = np.isfinite(hardware) & (hardware >= 0.35) & (hardware <= 8.0)
            hardware_residual = hardware[hardware_good]-camera[indices[hardware_good], 2]
            hardware_aligned_rows.extend(float(value) for value in hardware_residual)

    chosen = sorted(comparable, key=lambda row: row["rgbd_lidar_delta_m"], reverse=True)[:args.visualizations]
    tiles = []
    detection_by_id = {idx: row for idx, row in enumerate(detections)}
    for rank, row in enumerate(chosen):
        detection = detection_by_id[row["detection_id"]]
        key, frame = row["time_ns"], frames[row["time_ns"]]
        camera, _, uv = projected_cache[key]
        height, width = frame["rgb"].shape[:2]
        x1, y1, x2, y2 = inner_box(detection, width, height, args.inner_box_scale)
        canvas = cv2.cvtColor(frame["rgb"], cv2.COLOR_RGB2BGR)
        visible = ((uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height))
        for u, v in uv[visible][::2]: cv2.circle(canvas, (int(u), int(v)), 1, (255, 220, 0), -1)
        cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), (30, 30, 255), 2)
        cv2.putText(canvas, f"{row['label']}  RGBD/LiDAR delta={row['rgbd_lidar_delta_m']:.2f}m",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (20, 20, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"LiDAR z={row['lidar_depth_m']:.2f}m n={row['lidar_points_selected']} dt={row['cloud_dt_ms']:.1f}ms",
                    (8, 47), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 2, cv2.LINE_AA)
        depth = frame["depth"]
        scaled = np.clip(depth/8.0*255, 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        depth_color[~np.isfinite(depth) | (depth <= 0)] = 0
        cv2.rectangle(depth_color, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 2)
        tile = np.hstack([canvas, depth_color])
        cv2.imwrite(str(viz_dir / f"{rank:02d}_frame_{frame['index']:06d}_{row['label'].replace(' ', '_')}.png"), tile)
        tiles.append(cv2.resize(tile, (640, 240)))
    if tiles:
        cols = 2; rows_canvas = []
        for index in range(0, len(tiles), cols):
            group = tiles[index:index+cols]
            while len(group) < cols: group.append(np.full_like(tiles[0], 255))
            rows_canvas.append(np.hstack(group))
        cv2.imwrite(str(args.output / "largest_rgbd_lidar_disagreements.png"), np.vstack(rows_canvas))

    deltas = np.asarray([row["rgbd_lidar_delta_m"] for row in comparable], float)
    map_distances = np.asarray([row["lidar_to_map_m"] for row in rows], float)
    registration = np.asarray(registration_rows, float)
    registration_abs = np.abs(registration)
    hardware_registration = np.asarray(hardware_aligned_rows, float)
    hardware_abs = np.abs(hardware_registration)
    summary = {
        "format": "pre_map_vln.lidar_depth_localization_comparison.v1",
        "detections_total": len(detections), "lidar_localized": len(rows),
        "lidar_semantic_clusters": lidar_clusters,
        "comparable_with_rgbd": len(comparable), "rejected": dict(rejected),
        "rgbd_lidar_delta_m": ({"median": float(np.median(deltas)), "p90": float(np.percentile(deltas, 90)),
                                  "max": float(np.max(deltas))} if len(deltas) else None),
        "lidar_center_to_map_m": ({"median": float(np.median(map_distances)),
                                     "p90": float(np.percentile(map_distances, 90))} if len(map_distances) else None),
        "same_pixel_rgbd_minus_lidar_m": ({
            "samples": int(len(registration)),
            "signed_median": float(np.median(registration)),
            "absolute_median": float(np.median(registration_abs)),
            "absolute_p90": float(np.percentile(registration_abs, 90)),
            "within_0p10_fraction": float(np.mean(registration_abs <= 0.10)),
            "within_0p25_fraction": float(np.mean(registration_abs <= 0.25)),
        } if len(registration) else None),
        "same_pixel_hardware_aligned_minus_lidar_m": ({
            "matched_frames": len(aligned_depths),
            "median_frame_dt_ms": float(1000*np.median(list(aligned_dt[key] for key in aligned_depths))),
            "samples": int(len(hardware_registration)),
            "signed_median": float(np.median(hardware_registration)),
            "absolute_median": float(np.median(hardware_abs)),
            "absolute_p90": float(np.percentile(hardware_abs, 90)),
            "within_0p10_fraction": float(np.mean(hardware_abs <= 0.10)),
            "within_0p25_fraction": float(np.mean(hardware_abs <= 0.25)),
        } if len(hardware_registration) else None),
        "note": "Diagnostic only; existing RGB-D semantic map was not modified.",
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
