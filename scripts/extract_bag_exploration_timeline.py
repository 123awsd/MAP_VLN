#!/usr/bin/env python3
"""Extract compact occupied-map snapshots and odometry from a ROS1 bag."""
import argparse
from pathlib import Path

import numpy as np
import rosbag


def xyz_from_cloud(message):
    offsets = {field.name: field.offset for field in message.fields}
    if not all(name in offsets for name in ("x", "y", "z")):
        raise RuntimeError("PointCloud2 does not contain x/y/z")
    count = message.width * message.height
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [endian + "f4"] * 3,
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": message.point_step,
    })
    raw = np.frombuffer(message.data, dtype=dtype, count=count)
    return np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--steps", type=int, default=101)
    parser.add_argument("--cloud-topic", default="/voxel_mapping/occupancy_grid_occupied")
    parser.add_argument("--odom-topic", default="/uav_simulator/odometry")
    args = parser.parse_args()

    with rosbag.Bag(str(args.bag)) as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        targets = np.linspace(start, end, args.steps)
        snapshots = [None] * args.steps
        snapshot_times = np.full(args.steps, np.nan, dtype=np.float64)
        next_target = 0
        trajectory, trajectory_times = [], []
        last_cloud = None
        last_cloud_time = start
        for topic, message, stamp in bag.read_messages(topics=[args.cloud_topic, args.odom_topic]):
            current = stamp.to_sec()
            if topic == args.odom_topic:
                position = message.pose.pose.position
                trajectory.append((position.x, position.y, position.z))
                trajectory_times.append(current)
                continue
            cloud = xyz_from_cloud(message)
            last_cloud, last_cloud_time = cloud, current
            while next_target < args.steps and current >= targets[next_target]:
                snapshots[next_target] = cloud.copy()
                snapshot_times[next_target] = current
                next_target += 1
        if last_cloud is None:
            raise RuntimeError("No occupied point cloud messages found")
        while next_target < args.steps:
            snapshots[next_target] = last_cloud.copy()
            snapshot_times[next_target] = last_cloud_time
            next_target += 1

    payload = {
        "format": np.asarray("pre_map_vln.bag_exploration_timeline.v1"),
        "bag_start": np.asarray(start),
        "bag_end": np.asarray(end),
        "fractions": np.linspace(0.0, 1.0, args.steps).astype(np.float32),
        "snapshot_times": snapshot_times,
        "trajectory": np.asarray(trajectory, dtype=np.float32),
        "trajectory_times": np.asarray(trajectory_times, dtype=np.float64),
    }
    for index, cloud in enumerate(snapshots):
        payload[f"cloud_{index:03d}"] = cloud
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **payload)
    print("saved %d snapshots and %d trajectory samples to %s" % (
        args.steps, len(trajectory), args.output
    ))


if __name__ == "__main__":
    main()
