#!/usr/bin/env python3
"""Make a replayable, bounded-size stage-1 Bag.

The live recorder also stores several growing PointCloud2 topics at every map
update.  Those are useful while debugging a run but make a raw Bag enormous.
This filter keeps the RGB/pose/planning streams, keeps coverage samples, and
retains only the final map clouds and final frontier snapshots.  The source is
never modified.

Run inside the FALCON ROS container because the host does not provide ROS.
"""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

import rosbag


KEEP_ALL = {
    "/uav_simulator/odometry",
    "/uav_simulator/sensor_pose",
    "/pre_map_vln/agent_path",
    "/pre_map_vln/exploration_status",
    "/planning/replan",
    "/planning/bspline",
    "/planning/travel_traj",
    "/voxel_mapping/map_coverage",
}
FINAL_ONLY = {
    "/planning_vis/frontier",
    "/planning_vis/frontier_pcl",
    "/planning_vis/dormant_frontier_pcl",
    "/voxel_mapping/occupancy_grid_occupied",
    "/voxel_mapping/occupancy_grid_free",
    "/voxel_mapping/occupancy_grid_unknown",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--rgb-stride", type=int, default=5)
    args = parser.parse_args()
    if args.rgb_stride < 1:
        parser.error("--rgb-stride must be positive")
    if args.output_bag.exists():
        raise FileExistsError(args.output_bag)

    args.output_bag.parent.mkdir(parents=True, exist_ok=True)
    final_messages = {}
    final_stamps = {}
    counts = Counter()
    kept_counts = Counter()
    rgb_index = 0
    last_kept_stamp = None

    with rosbag.Bag(str(args.input_bag), "r") as source, rosbag.Bag(
        str(args.output_bag), "w", compression=rosbag.Compression.LZ4
    ) as target:
        for topic, message, stamp in source.read_messages():
            counts[topic] += 1
            if topic == "/habitat/rgb":
                if rgb_index % args.rgb_stride == 0:
                    target.write(topic, message, stamp)
                    kept_counts[topic] += 1
                    last_kept_stamp = stamp
                rgb_index += 1
                continue
            if topic in KEEP_ALL:
                target.write(topic, message, stamp)
                kept_counts[topic] += 1
                last_kept_stamp = stamp
                continue
            if topic in FINAL_ONLY:
                final_messages[topic] = message
                final_stamps[topic] = stamp

        if last_kept_stamp is not None:
            for topic in sorted(final_messages):
                # Put final map/frontier snapshots at the end of the replay
                # interval.  This makes them visible in RViz and keeps the
                # compact Bag compatible with progressive-box injection.
                target.write(topic, final_messages[topic], last_kept_stamp)
                kept_counts[topic] += 1

    print({
        "source": str(args.input_bag),
        "output": str(args.output_bag),
        "source_counts": dict(sorted(counts.items())),
        "kept_counts": dict(sorted(kept_counts.items())),
        "rgb_stride": args.rgb_stride,
        "final_snapshot_topics": sorted(final_messages),
    })


if __name__ == "__main__":
    main()
