#!/usr/bin/env python3
"""Extract the geometry needed by the HM3D navmesh comparison from a ROS1 bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


DEPTH_TOPIC = "/uav_simulator/depth_image"
POSE_TOPIC = "/uav_simulator/sensor_pose"
PATH_TOPIC = "/pre_map_vln/agent_path"
MAP_TOPIC = "/voxel_mapping/occupancy_grid_occupied"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scene", required=True)
    parser.add_argument(
        "--keyframe-step",
        type=int,
        default=5,
        help="save every Nth depth frame; 5 matches the validated 10 Hz to 2 Hz map",
    )
    parser.add_argument(
        "--max-sync-ms",
        type=float,
        default=5.0,
        help="maximum pose/depth timestamp difference when an exact match is unavailable",
    )
    args = parser.parse_args()
    if args.keyframe_step < 1:
        parser.error("--keyframe-step must be positive")
    if args.max_sync_ms < 0:
        parser.error("--max-sync-ms cannot be negative")
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")

    args.output.mkdir(parents=True)
    (args.output / "manifest.json").write_text(
        json.dumps(
            {
                "format": "pre_map_vln.habitat_episode.v1",
                "scene": args.scene,
                "width": 640,
                "height": 480,
                "fx": 320.0,
                "fy": 320.0,
                "cx": 320.0,
                "cy": 240.0,
                "coordinate_frame": "falcon_world_z_up_camera_optical",
                "source_bag": str(args.bag),
                "semantic": False,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    # First pass: collect the complete pose timeline. ROS connections can be
    # interleaved in either order inside a bag, so a streaming "latest pose"
    # lookup can silently pair a depth frame with the preceding camera pose.
    poses_by_stamp = {}
    final_path = None
    final_cloud = None
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, _ in bag.read_messages(
            topics=[POSE_TOPIC, PATH_TOPIC, MAP_TOPIC]
        ):
            if topic == POSE_TOPIC:
                poses_by_stamp[message.header.stamp.to_nsec()] = message.transform
            elif topic == PATH_TOPIC:
                final_path = message
            else:
                final_cloud = message
    if not poses_by_stamp:
        raise RuntimeError(f"bag contains no {POSE_TOPIC}")

    pose_stamps = np.asarray(sorted(poses_by_stamp), dtype=np.int64)
    depth_messages = 0
    saved_frames = 0
    exact_pairs = 0
    nearest_pairs = 0
    unmatched_depth = 0
    maximum_sync_error_ms = 0.0
    # Second pass: match each depth header stamp against the complete timeline.
    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[DEPTH_TOPIC]):
            depth_messages += 1
            stamp_ns = message.header.stamp.to_nsec()
            pose = poses_by_stamp.get(stamp_ns)
            sync_error_ms = 0.0
            if pose is not None:
                exact_pairs += 1
            else:
                insertion = int(np.searchsorted(pose_stamps, stamp_ns))
                candidates = pose_stamps[max(0, insertion - 1):min(len(pose_stamps), insertion + 1)]
                if len(candidates):
                    nearest_stamp = int(candidates[np.argmin(np.abs(candidates - stamp_ns))])
                    sync_error_ms = abs(stamp_ns - nearest_stamp) / 1.0e6
                    if sync_error_ms <= args.max_sync_ms:
                        pose = poses_by_stamp[nearest_stamp]
                        nearest_pairs += 1
            if pose is None:
                unmatched_depth += 1
                continue
            maximum_sync_error_ms = max(maximum_sync_error_ms, sync_error_ms)
            if (depth_messages - 1) % args.keyframe_step != 0:
                continue
            depth = np.frombuffer(message.data, dtype="<u2").reshape(
                message.height, message.width
            ).astype(np.float32) / 1000.0
            translation = pose.translation
            rotation = pose.rotation
            np.savez_compressed(
                args.output / f"frame_{saved_frames:06d}.npz",
                depth_m=depth,
                position=np.asarray(
                    [translation.x, translation.y, translation.z], dtype=np.float32
                ),
                orientation_xyzw=np.asarray(
                    [rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float32
                ),
                time_ns=np.asarray(stamp_ns, dtype=np.int64),
            )
            saved_frames += 1

    if saved_frames == 0:
        raise RuntimeError("bag contains no synchronized depth and sensor-pose frames")
    if final_path is None:
        raise RuntimeError(f"bag contains no {PATH_TOPIC}")
    if final_cloud is None:
        raise RuntimeError(f"bag contains no {MAP_TOPIC}")

    trajectory = np.asarray(
        [
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in final_path.poses
        ],
        dtype=np.float32,
    )
    cloud = np.asarray(
        list(
            point_cloud2.read_points(
                final_cloud, field_names=("x", "y", "z"), skip_nans=True
            )
        ),
        dtype=np.float32,
    )
    np.savez_compressed(
        args.output / "recorded_geometry.npz",
        cloud_xyz=cloud,
        trajectory_xyz=trajectory,
    )
    report = {
        "format": "pre_map_vln.mapping_bag_extraction.v2",
        "depth_messages": depth_messages,
        "saved_frames": saved_frames,
        "keyframe_step": args.keyframe_step,
        "exact_pose_depth_pairs": exact_pairs,
        "nearest_pose_depth_pairs": nearest_pairs,
        "unmatched_depth_messages": unmatched_depth,
        "maximum_sync_error_ms": maximum_sync_error_ms,
        "trajectory_points": len(trajectory),
        "cloud_points": len(cloud),
        "output": str(args.output),
    }
    (args.output / "extraction_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report))


if __name__ == "__main__":
    main()
