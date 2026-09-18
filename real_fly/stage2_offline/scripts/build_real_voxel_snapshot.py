#!/usr/bin/env python3
"""Build conservative UNKNOWN/FREE/OCCUPIED voxels from a real mapping bag."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
import sys
import uuid
from pathlib import Path

import cv2
import numpy as np
import rosbag
import yaml
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from stage2.planning_contract import OCCUPANCY_SEMANTICS_HASH, OccupancyState, PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402

from export_real_boxer_episode import load_pcd_xyz, nearest_index, pose_matrix, stamp, voxel_downsample


def cloud_xyz(msg) -> np.ndarray:
    offsets = {field.name: field.offset for field in msg.fields}
    if not all(name in offsets for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 lacks x/y/z")
    count = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(count, msg.point_step)
    columns = []
    for name in ("x", "y", "z"):
        columns.append(raw[:, offsets[name] : offsets[name] + 4].copy().view("<f4").reshape(-1))
    return np.column_stack(columns).astype(np.float64)


def atomic_json(path: Path, value) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(str(temporary), str(path))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("map_pcd", type=Path)
    parser.add_argument("planning_config", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--min-occupied-points", type=int, default=100)
    parser.add_argument("--scan-period", type=float, default=2.0)
    parser.add_argument("--max-ray-m", type=float, default=25.0)
    args = parser.parse_args()
    if args.output.exists(): raise SystemExit(f"refusing to overwrite: {args.output}")
    if args.min_occupied_points < 1:
        raise SystemExit("--min-occupied-points must be positive")
    profile = PlannerProfile.load(args.planning_config)
    if not math.isclose(profile.coarse_resolution_m / args.resolution, round(profile.coarse_resolution_m / args.resolution), abs_tol=1e-6):
        raise SystemExit("coarse resolution must be a multiple of voxel resolution")

    occupied_points = load_pcd_xyz(args.map_pcd)
    occupied_points = occupied_points[np.isfinite(occupied_points).all(axis=1)]
    lower = np.percentile(occupied_points, 0.25, axis=0) - [1.0, 1.0, 0.75]
    upper = np.percentile(occupied_points, 99.75, axis=0) + [1.0, 1.0, 0.75]
    origin = np.floor(lower / args.resolution) * args.resolution
    shape_xyz = np.ceil((upper - origin) / args.resolution).astype(int)
    if int(np.prod(shape_xyz)) > 12_000_000:
        raise SystemExit(f"voxel grid is unexpectedly large: {shape_xyz.tolist()}")
    raw = np.full(tuple(shape_xyz[::-1]), int(OccupancyState.UNKNOWN), dtype=np.int8)

    def indices(points):
        idx = np.floor((points - origin) / args.resolution).astype(np.int64)
        good = np.all((idx >= 0) & (idx < shape_xyz), axis=1)
        return idx[good]

    # Preserve raw PCD support per voxel for visualization. Planning still uses
    # the binary occupancy state below; density must never silently change the
    # collision contract.
    all_occupied_idx = indices(occupied_points)
    flat_idx = np.ravel_multi_index(
        (all_occupied_idx[:, 2], all_occupied_idx[:, 1], all_occupied_idx[:, 0]),
        raw.shape,
    )
    occupied_point_count = np.bincount(flat_idx, minlength=raw.size).reshape(raw.shape)
    occupied_point_count = np.minimum(occupied_point_count, np.iinfo(np.uint32).max).astype(np.uint32)

    odom_times, odom_positions = [], []
    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, msg, bt in bag.read_messages(topics=["/Odometry"]):
            odom_times.append(stamp(msg, bt)); odom_positions.append(pose_matrix(msg)[:3, 3])
    if not odom_times: raise SystemExit("/Odometry missing")

    scans, last = 0, -math.inf
    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, msg, bt in bag.read_messages(topics=["/cloud_registered"]):
            current = stamp(msg, bt)
            if current < odom_times[0] or current > odom_times[-1] or current - last < args.scan_period: continue
            last = current
            sensor = odom_positions[nearest_index(odom_times, current)]
            points = cloud_xyz(msg)
            points = points[np.isfinite(points).all(axis=1)]
            distance = np.linalg.norm(points - sensor, axis=1)
            points = points[(distance > 0.6) & (distance < args.max_ray_m)]
            points = voxel_downsample(points, args.resolution * 1.8)
            if len(points) > 3000: points = points[:: int(math.ceil(len(points) / 3000))]
            for endpoint in points:
                length = float(np.linalg.norm(endpoint - sensor))
                steps = max(2, int(math.ceil(length / (args.resolution * 0.8))))
                ray = sensor + (endpoint - sensor) * np.linspace(0.0, 0.96, steps)[:, None]
                idx = indices(ray)
                if len(idx): raw[idx[:, 2], idx[:, 1], idx[:, 0]] = int(OccupancyState.FREE)
            scans += 1
            if scans % 10 == 0: print(f"ray-integrated scans={scans}", flush=True)

    # A voxel needs several supporting raw PCD points before it becomes a hard
    # obstacle. The source PCD is untouched; this only affects planning occupancy.
    supported_occupied = occupied_point_count >= args.min_occupied_points
    raw[supported_occupied] = int(OccupancyState.OCCUPIED)
    esdf = distance_transform_edt(raw != int(OccupancyState.OCCUPIED), sampling=args.resolution).astype(np.float32)
    args.output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(
        args.output / "voxel_map.npz",
        raw_occupancy_zyx=raw,
        esdf_zyx_m=esdf,
        occupied_point_count_zyx=occupied_point_count,
    )
    counts = {name: int(np.count_nonzero(raw == value)) for name, value in (("unknown", -1), ("free", 0), ("occupied", 100))}
    provisional = {
        "format": "pre_map_vln.real_voxel_snapshot.v1", "frame_id": "world",
        "origin_xyz_m": origin.tolist(), "resolution_m": args.resolution, "shape_xyz": shape_xyz.tolist(),
        "counts": counts, "source_bag": str(args.bag.resolve()), "source_map": str(args.map_pcd.resolve()),
        "ray_integrated_scans": scans, "unknown_is_blocked": True,
        "minimum_esdf_distance_m": profile.minimum_esdf_distance_m,
        "preferred_esdf_distance_m": profile.preferred_esdf_distance_m,
        "planning_config": str(args.planning_config.resolve()),
        "min_occupied_points_per_voxel": args.min_occupied_points,
        "occupied_point_count": {
            "maximum": int(occupied_point_count.max()),
            "nonzero_voxels": int(np.count_nonzero(occupied_point_count)),
        },
    }
    content = hashlib.sha256(
        raw.tobytes() + esdf.tobytes() + occupied_point_count.tobytes()
        + json.dumps(provisional, sort_keys=True).encode()
    ).hexdigest()
    provisional["identity"] = {
        "map_epoch_uuid": str(uuid.uuid4()), "geometry_map_version": 1, "semantic_map_version": 1,
        "snapshot_content_hash": content, "occupancy_semantics_hash": OCCUPANCY_SEMANTICS_HASH,
        "planner_profile_hash": profile.profile_hash,
    }
    atomic_json(args.output / "metadata.json", provisional)
    loaded = VoxelMap3D.load(args.output, profile)
    valid = loaded.inflated_free
    floor_projection = np.zeros((raw.shape[1], raw.shape[2], 3), dtype=np.uint8)
    floor_projection[np.any(raw == -1, axis=0)] = [70, 70, 70]
    floor_projection[np.any(raw == 0, axis=0)] = [80, 180, 80]
    floor_projection[np.any(raw == 100, axis=0)] = [240, 240, 240]
    cv2.imwrite(str(args.output / "topdown_occupancy.png"), np.flipud(floor_projection))
    report = {**provisional, "inflated_free_voxels": int(np.count_nonzero(valid)), "planning_state_signature": loaded.planning_state_signature(), "status": "ok" if scans >= 20 and counts["free"] > 0 else "failed"}
    atomic_json(args.output / "report.json", report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__": main()
