#!/usr/bin/env python3
"""Offline quality audit for timing, map coverage, and collision-checked curves."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from stage2.bspline_3d import BsplineSettings, plan_collision_checked_bspline  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def stamp(msg: Any, bag_time: Any) -> float:
    value = getattr(getattr(getattr(msg, "header", None), "stamp", None), "to_sec", lambda: 0.0)()
    return float(value if value > 0 else bag_time.to_sec())


def pcd_points(path: Path) -> int:
    with path.open("rb") as handle:
        for line in handle:
            text = line.decode("ascii", errors="ignore").strip()
            if text.startswith("POINTS "):
                return int(text.split()[1])
            if text == "DATA binary":
                break
    return 0


def median_delta(first: list[float], second: list[float]) -> dict[str, float | int | None]:
    if not first or not second:
        return {"count": 0, "outside_overlap": 0, "median_ms": None, "p95_ms": None, "max_ms": None}
    values = []
    outside = 0
    for value in first:
        if value < second[0] or value > second[-1]:
            outside += 1
            continue
        insertion = bisect.bisect_left(second, value)
        choices = [max(0, insertion - 1), min(len(second) - 1, insertion)]
        index = min(choices, key=lambda item: abs(second[item] - value))
        values.append(1000.0 * abs(value - second[index]))
    if not values:
        return {"count": 0, "outside_overlap": outside, "median_ms": None, "p95_ms": None, "max_ms": None}
    return {"count": len(values), "outside_overlap": outside, "median_ms": float(np.median(values)), "p95_ms": float(np.percentile(values, 95)), "max_ms": float(max(values))}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--mapping-bag", type=Path,
                        help="regenerated bag providing complete /Odometry")
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--voxel-snapshot", type=Path, required=True)
    parser.add_argument("--planning-config", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    manifest = json.loads((args.episode / "manifest.json").read_text(encoding="utf-8"))
    requested_depth_topic = manifest.get("depth_source_topic", "/camera/aligned_depth_to_color/image_raw")
    streams: dict[str, list[float]] = {name: [] for name in ("rgb", "depth", "lidar", "livox_imu", "fc_imu", "odom")}
    source_topics = {
        "/camera/color/image_raw": "rgb", requested_depth_topic: "depth",
        "/livox/lidar": "lidar", "/livox/imu": "livox_imu", "/mavros/imu/data": "fc_imu",
    }
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=list(source_topics)):
            streams[source_topics[topic]].append(stamp(msg, bag_time))
    pose_bag = args.mapping_bag or args.bag
    with rosbag.Bag(str(pose_bag), "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=["/Odometry"]):
            streams["odom"].append(stamp(msg, bag_time))

    profile = PlannerProfile.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.voxel_snapshot, profile)
    mission = json.loads(args.mission.read_text(encoding="utf-8"))
    curve_results = []
    for segment in mission.get("segments", []):
        points = segment.get("points_xyz_m", [])
        try:
            curve = plan_collision_checked_bspline(points, voxel_map, BsplineSettings.load(args.planning_config))
            curve_results.append({
                "to_task_id": segment.get("to_task_id"), "status": "success", "mode": curve.mode,
                "degree": curve.degree, "piece_count": curve.piece_count,
                "sample_count": len(curve.points_xyz_m), "minimum_clearance_m": curve.minimum_clearance_m,
                "collision_free": True,
            })
        except (RuntimeError, TypeError, ValueError) as error:
            curve_results.append({"to_task_id": segment.get("to_task_id"), "status": "failed", "reason": str(error), "collision_free": False})

    timing = {
        "rgb_to_depth": median_delta(streams["rgb"], streams["depth"]),
        "depth_to_lidar": median_delta(streams["depth"], streams["lidar"]),
        "rgb_to_odom": median_delta(streams["rgb"], streams["odom"]),
        "livox_imu_to_odom": median_delta(streams["livox_imu"], streams["odom"]),
        "flight_controller_imu_to_odom": median_delta(streams["fc_imu"], streams["odom"]),
    }
    counts = {name: len(values) for name, values in streams.items()}
    voxel_counts = {name: int(np.count_nonzero(voxel_map.raw == value)) for name, value in (("unknown", -1), ("free", 0), ("occupied", 100))}
    report = {
        "format": "pre_map_vln.real_stage2_quality_audit.v1", "status": "pass",
        "safety_scope": "offline_only_no_flight_control", "source_bag_sha256": sha256(args.bag),
        "mapping_bag_sha256": sha256(pose_bag),
        "source_bag_bytes": args.bag.stat().st_size, "map_pcd_points": pcd_points(args.pcd),
        "stream_counts": counts, "depth_source_topic": requested_depth_topic,
        "depth_source": manifest.get("depth_source"),
        "timing_nearest_message_deltas": timing,
        "map": {"voxel_counts": voxel_counts, "inflated_free_voxels": int(voxel_map.inflated_free.sum()), "unknown_is_blocked": True},
        "bspline": {"segments": curve_results, "all_collision_free": all(item["collision_free"] for item in curve_results)},
        "notes": [
            "Nearest-message deltas are diagnostics, not a replacement for hardware timestamp calibration.",
            "The FC IMU stream is audited only because it exists in the bag; this report sends no FC command.",
        ],
    }
    atomic_json(args.output, report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["bspline"]["all_collision_free"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
