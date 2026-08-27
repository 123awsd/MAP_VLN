#!/usr/bin/env python3
"""Export auditable maps and planning traces from one recorded stage-1 Bag.

This script is intentionally ROS-only and is run inside the FALCON container.
It never changes the source Bag.  Point clouds are exported as ordinary ASCII
PCD so they remain inspectable without ROS, while the JSON/CSV summaries keep
the online planner evidence separate from Habitat truth metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import rosbag
from sensor_msgs import point_cloud2


MAP_TOPICS = {
    "occupied": "/voxel_mapping/occupancy_grid_occupied",
    "free": "/voxel_mapping/occupancy_grid_free",
    "unknown": "/voxel_mapping/occupancy_grid_unknown",
    "depth_pointcloud": "/voxel_mapping/depth_pointcloud",
}
MAP_TOPIC_TO_NAME = {topic: name for name, topic in MAP_TOPICS.items()}


def stamp_of(message, bag_stamp) -> float:
    header = getattr(message, "header", None)
    if header is not None and header.stamp is not None and header.stamp.to_sec() > 0:
        return float(header.stamp.to_sec())
    return float(bag_stamp.to_sec())


def cloud_points(message):
    fields = {field.name for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise RuntimeError(f"PointCloud2 has no x/y/z fields: {sorted(fields)}")
    return [
        (float(x), float(y), float(z))
        for x, y, z in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
    ]


def write_pcd(path: Path, points) -> int:
    points = list(points)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION .7\n")
        handle.write("FIELDS x y z\n")
        handle.write("SIZE 4 4 4\n")
        handle.write("TYPE F F F\n")
        handle.write("COUNT 1 1 1\n")
        handle.write(f"WIDTH {len(points)}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(points)}\n")
        handle.write("DATA ascii\n")
        for point in points:
            handle.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    return len(points)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    topic_counts = Counter()
    final_clouds = {}
    final_cloud_stamps = {}
    coverage = []
    frontier_rows = []
    replan_values = []
    status_values = []
    trajectory_rows = []
    bspline_count = 0
    bag_start = None
    bag_end = None

    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, bag_stamp in bag.read_messages():
            topic_counts[topic] += 1
            stamp = stamp_of(message, bag_stamp)
            bag_start = stamp if bag_start is None else min(bag_start, stamp)
            bag_end = stamp if bag_end is None else max(bag_end, stamp)

            if topic in MAP_TOPIC_TO_NAME:
                map_name = MAP_TOPIC_TO_NAME[topic]
                final_clouds[map_name] = message
                final_cloud_stamps[map_name] = stamp
            elif topic == "/voxel_mapping/map_coverage":
                coverage.append({"time": stamp, "coverage": float(message.data)})
            elif topic in {
                "/planning_vis/frontier_pcl",
                "/planning_vis/dormant_frontier_pcl",
            }:
                frontier_rows.append({
                    "time": stamp,
                    "active_points": int(
                        message.width * message.height
                        if topic.endswith("frontier_pcl") and not topic.endswith("dormant_frontier_pcl")
                        else 0
                    ),
                    "dormant_points": int(
                        message.width * message.height
                        if topic.endswith("dormant_frontier_pcl") else 0
                    ),
                })
            elif topic == "/planning/replan":
                replan_values.append({"time": stamp, "value": int(message.data)})
            elif topic == "/pre_map_vln/exploration_status":
                status_values.append({"time": stamp, "value": str(message.data)})
            elif topic == "/uav_simulator/sensor_pose":
                translation = message.transform.translation
                rotation = message.transform.rotation
                trajectory_rows.append({
                    "time": stamp,
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "z": float(translation.z),
                    "qx": float(rotation.x),
                    "qy": float(rotation.y),
                    "qz": float(rotation.z),
                    "qw": float(rotation.w),
                })
            elif topic == "/planning/bspline":
                bspline_count += 1

    maps = {}
    for name, topic in MAP_TOPICS.items():
        message = final_clouds.get(name)
        if message is None:
            continue
        points = cloud_points(message)
        maps[name] = {
            "topic": topic,
            "stamp": final_cloud_stamps[name],
            "points": write_pcd(args.output_dir / f"map_{name}.pcd", points),
            "fields": [field.name for field in message.fields],
        }

    with (args.output_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        columns = ["time", "x", "y", "z", "qx", "qy", "qz", "qw"]
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(trajectory_rows)

    with (args.output_dir / "frontier_series.csv").open("w", newline="", encoding="utf-8") as handle:
        columns = ["time", "active_points", "dormant_points"]
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        # Both frontier topics are published at the same planning ticks. Merge
        # the two streams so a row represents one online frontier snapshot.
        merged = {}
        for row in frontier_rows:
            key = round(row["time"], 5)
            target = merged.setdefault(key, {"time": row["time"], "active_points": 0, "dormant_points": 0})
            target["active_points"] = max(target["active_points"], row["active_points"])
            target["dormant_points"] = max(target["dormant_points"], row["dormant_points"])
        writer.writerows(sorted(merged.values(), key=lambda item: item["time"]))

    (args.output_dir / "coverage_series.json").write_text(
        json.dumps(coverage, indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "replan_series.json").write_text(
        json.dumps(replan_values, indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "status_series.json").write_text(
        json.dumps(status_values, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    report = {
        "format": "pre_map_vln.stage1_bag_export.v1",
        "source_bag": str(args.bag),
        "bag_start": bag_start,
        "bag_end": bag_end,
        "duration_seconds": None if bag_start is None else bag_end - bag_start,
        "topic_counts": dict(sorted(topic_counts.items())),
        "maps": maps,
        "trajectory_rows": len(trajectory_rows),
        "coverage_samples": len(coverage),
        "coverage_first": None if not coverage else coverage[0]["coverage"],
        "coverage_last": None if not coverage else coverage[-1]["coverage"],
        "coverage_max": None if not coverage else max(row["coverage"] for row in coverage),
        "frontier_snapshots": len(frontier_rows),
        "active_frontier_point_max": max(
            (row["active_points"] for row in frontier_rows), default=0
        ),
        "dormant_frontier_point_max": max(
            (row["dormant_points"] for row in frontier_rows), default=0
        ),
        "replan_value_counts": dict(Counter(row["value"] for row in replan_values)),
        "status_values": status_values,
        "bspline_messages": bspline_count,
        "note": "Frontier point counts and FALCON coverage are online planner diagnostics, not Habitat ground truth.",
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
