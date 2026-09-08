#!/usr/bin/env python3
"""Accumulate world-frame PointCloud2 scans into a voxelized global PCD."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import os
from pathlib import Path

import numpy as np
import rosbag


def stamp(message, bag_time) -> float:
    header = getattr(message, "header", None)
    value = header.stamp.to_sec() if header is not None else 0.0
    return float(value if value > 0 else bag_time.to_sec())


def cloud_xyzi(message) -> np.ndarray:
    offsets = {field.name: (field.offset, field.datatype, field.count) for field in message.fields}
    if not all(name in offsets for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 lacks x/y/z fields")
    for name in ("x", "y", "z"):
        if offsets[name][1:] != (7, 1):  # sensor_msgs/PointField.FLOAT32
            raise ValueError(f"unsupported PointCloud2 field {name}: {offsets[name]}")
    count = int(message.width * message.height)
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(count, message.point_step)
    endian = ">f4" if message.is_bigendian else "<f4"
    columns = [raw[:, offsets[name][0]:offsets[name][0] + 4].copy().view(endian).reshape(-1)
               for name in ("x", "y", "z")]
    if "intensity" in offsets and offsets["intensity"][1:] == (7, 1):
        intensity_offset = offsets["intensity"][0]
        intensity = raw[:, intensity_offset:intensity_offset + 4].copy().view(endian).reshape(-1)
    else:
        intensity = np.zeros(count, dtype=np.float32)
    return np.column_stack(columns + [intensity]).astype(np.float32, copy=False)


def pose_xyz(message) -> np.ndarray:
    position = message.pose.pose.position
    return np.asarray([position.x, position.y, position.z], dtype=np.float64)


def nearest_index(values: list[float], target: float) -> int:
    index = bisect.bisect_left(values, target)
    choices = (max(0, index - 1), min(len(values) - 1, index))
    return min(choices, key=lambda item: abs(values[item] - target))


def voxel_reduce(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if not len(points):
        return points
    finite = np.isfinite(points[:, :3]).all(axis=1)
    points = points[finite]
    keys = np.floor(points[:, :3].astype(np.float64) / voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def write_binary_pcd(path: Path, points: np.ndarray) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(header)
        stream.write(np.asarray(points, dtype="<f4").tobytes(order="C"))
    os.replace(str(temporary), str(path))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/cloud_registered")
    parser.add_argument("--pose-topic", default="/Odometry")
    parser.add_argument("--voxel-size", type=float, default=0.08)
    parser.add_argument("--min-sensor-range", type=float, default=0.3)
    parser.add_argument("--max-sensor-range", type=float, default=25.0)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--batch-points", type=int, default=1_000_000)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if not args.bag.is_file():
        raise SystemExit(f"bag not found: {args.bag}")
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite: {args.output}")
    if (args.voxel_size <= 0 or args.frame_stride < 1 or args.batch_points < 1
            or args.min_sensor_range < 0 or args.max_sensor_range <= args.min_sensor_range):
        raise SystemExit("voxel size, frame stride and batch size must be positive")

    accumulated = np.empty((0, 4), dtype=np.float32)
    pending: list[np.ndarray] = []
    pending_count = 0
    messages = selected_messages = input_points = range_rejected_points = 0
    first_sec = last_sec = None

    pose_times: list[float] = []
    pose_positions: list[np.ndarray] = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        topics = bag.get_type_and_topic_info()[1]
        if args.topic not in topics:
            raise SystemExit(f"topic missing from bag: {args.topic}")
        if args.pose_topic not in topics:
            raise SystemExit(f"pose topic missing from bag: {args.pose_topic}")
        for _, message, bag_time in bag.read_messages(topics=[args.pose_topic]):
            pose_times.append(stamp(message, bag_time))
            pose_positions.append(pose_xyz(message))
    if not pose_times:
        raise SystemExit(f"pose topic has no messages: {args.pose_topic}")

    def flush() -> None:
        nonlocal accumulated, pending, pending_count
        if not pending:
            return
        batch = voxel_reduce(np.concatenate(pending, axis=0), args.voxel_size)
        accumulated = voxel_reduce(np.concatenate((accumulated, batch), axis=0), args.voxel_size)
        pending = []
        pending_count = 0

    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=[args.topic]):
            index = messages
            messages += 1
            if index % args.frame_stride:
                continue
            points = cloud_xyzi(message)
            selected_messages += 1
            input_points += len(points)
            current = stamp(message, bag_time)
            sensor = pose_positions[nearest_index(pose_times, current)]
            distances = np.linalg.norm(points[:, :3].astype(np.float64) - sensor, axis=1)
            accepted = np.isfinite(distances) & (distances >= args.min_sensor_range) & (distances <= args.max_sensor_range)
            range_rejected_points += int(len(points) - np.count_nonzero(accepted))
            points = points[accepted]
            first_sec = current if first_sec is None else first_sec
            last_sec = current
            pending.append(points)
            pending_count += len(points)
            if pending_count >= args.batch_points:
                flush()
                print(f"accumulated scans={selected_messages}, voxels={len(accumulated)}", flush=True)
    flush()
    if not len(accumulated):
        raise SystemExit("no finite registered points were accumulated")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_binary_pcd(args.output, accumulated)
    report = {
        "format": "pre_map_vln.global_registered_cloud.v1",
        "status": "ok",
        "source_bag": str(args.bag.resolve()),
        "source_bag_sha256": sha256_file(args.bag),
        "source_topic": args.topic,
        "pose_topic": args.pose_topic,
        "frame_messages": messages,
        "selected_frame_messages": selected_messages,
        "frame_stride": args.frame_stride,
        "first_header_sec": first_sec,
        "last_header_sec": last_sec,
        "input_points": input_points,
        "range_rejected_points": range_rejected_points,
        "output_points": int(len(accumulated)),
        "voxel_size_m": args.voxel_size,
        "sensor_range_m": [args.min_sensor_range, args.max_sensor_range],
        "bounds_min_xyz_m": accumulated[:, :3].min(axis=0).astype(float).tolist(),
        "bounds_max_xyz_m": accumulated[:, :3].max(axis=0).astype(float).tolist(),
        "output_pcd": str(args.output.resolve()),
    }
    report_path = args.report or args.output.with_suffix(".json")
    temporary = report_path.with_suffix(report_path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    os.replace(str(temporary), str(report_path))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
