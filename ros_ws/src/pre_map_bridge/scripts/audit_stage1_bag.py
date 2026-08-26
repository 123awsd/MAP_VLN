#!/usr/bin/env python3
"""Extract final Frontier and occupancy evidence from a Stage-1 ROS bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rosbag


def point_count(message) -> int:
    if hasattr(message, "width") and hasattr(message, "height"):
        return int(message.width) * int(message.height)
    if hasattr(message, "points"):
        return len(message.points)
    if hasattr(message, "markers"):
        return sum(len(marker.points) for marker in message.markers)
    return 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    topics = {
        "/planning_vis/frontier_pcl": "active_frontier_points",
        "/planning_vis/dormant_frontier_pcl": "dormant_frontier_points",
        "/planning_vis/frontier": "frontier_marker_points",
        "/voxel_mapping/occupancy_grid_occupied": "occupied_points",
    }
    values = {name: {"message_count": 0, "last_point_count": None, "maximum_point_count": 0}
              for name in topics.values()}
    first_stamp = last_stamp = None
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, stamp in bag.read_messages(topics=list(topics)):
            seconds = stamp.to_sec()
            first_stamp = seconds if first_stamp is None else min(first_stamp, seconds)
            last_stamp = seconds if last_stamp is None else max(last_stamp, seconds)
            count = point_count(message)
            record = values[topics[topic]]
            record["message_count"] += 1
            record["last_point_count"] = count
            record["maximum_point_count"] = max(record["maximum_point_count"], count)
    payload = {
        "format": "pre_map_vln.stage1_bag_audit.v1", "bag": str(args.bag),
        "duration_s": None if first_stamp is None else last_stamp - first_stamp,
        **values,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":
    main()
