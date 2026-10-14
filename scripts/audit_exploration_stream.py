#!/usr/bin/env python3
"""Audit recorded RGB-D/pose alignment and actual motion, without planner input.

Height compliance is only a scalar constraint check, not proof of roof clearance
or room coverage. No scene truth, semantic labels, or room coordinates are used.
Run in the ROS container, whose Python provides rosbag and numpy.
"""
import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-flight-z", required=True, type=float)
    parser.add_argument("--result", type=Path)
    args = parser.parse_args()
    topics = {
        "depth": "/uav_simulator/depth_image",
        "rgb": "/habitat/rgb",
        "pose": "/uav_simulator/sensor_pose",
        "odom": "/uav_simulator/odometry",
        "coverage": "/voxel_mapping/map_coverage",
    }
    stamps = {key: Counter() for key in ("depth", "rgb", "pose", "odom")}
    positions = {}
    odometry = {}
    odom_velocity = {}
    bad_encoding = Counter()
    coverage = []
    reverse = {value: key for key, value in topics.items()}
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, _ in bag.read_messages(topics=list(reverse)):
            key = reverse[topic]
            if key == "coverage":
                coverage.append(float(message.data))
                continue
            stamp = message.header.stamp.to_nsec()
            stamps[key][stamp] += 1
            if key == "pose":
                p = message.transform.translation
                positions[stamp] = [p.x, p.y, p.z]
            elif key == "odom":
                p = message.pose.pose.position
                odometry[stamp] = [p.x, p.y, p.z]
                v = message.twist.twist.linear
                odom_velocity[stamp] = [v.x, v.y, v.z]
            else:
                expected = "16UC1" if key == "depth" else "rgb8"
                if message.encoding != expected:
                    bad_encoding[key] += 1
    depth_stamps = sorted(stamps["depth"])
    missing = {
        key: sum(stamps["depth"][s] for s in depth_stamps if s not in stamps[key])
        for key in ("rgb", "pose", "odom")
    }
    aligned = [s for s in depth_stamps if s in positions and s in odometry]
    p = np.asarray([positions[s] for s in aligned], dtype=float).reshape(-1, 3)
    o = np.asarray([odometry[s] for s in aligned], dtype=float).reshape(-1, 3)
    finite = bool(len(p) and np.isfinite(p).all() and np.isfinite(o).all())
    duration = (depth_stamps[-1] - depth_stamps[0]) * 1e-9 if depth_stamps else None
    gaps = np.diff(depth_stamps) * 1e-9
    motion = np.linalg.norm(np.diff(p, axis=0), axis=1)
    aligned_dt = np.diff(np.asarray(aligned, dtype=np.int64)) * 1e-9
    speeds = motion / aligned_dt if len(motion) else np.empty(0)
    z_violations = int((p[:, 2] > args.max_flight_z + 1e-5).sum())
    reported_speeds = np.linalg.norm(np.asarray([odom_velocity[s] for s in aligned], dtype=float).reshape(-1, 3), axis=1)
    velocity_residuals = np.linalg.norm(np.asarray([odom_velocity[s] for s in aligned[1:]], dtype=float).reshape(-1, 3) - np.diff(p, axis=0) / aligned_dt[:, None], axis=1) if len(aligned) > 1 else np.empty(0)
    result = json.loads(args.result.read_text()) if args.result and args.result.exists() else None
    report = {
        "format": "pre_map_vln.exploration_stream_audit.v1",
        "source_bag": str(args.bag),
        "message_counts": {k: sum(v.values()) for k, v in stamps.items()},
        "unique_stamp_counts": {k: len(v) for k, v in stamps.items()},
        "depth_frames_missing_exact_stamp": missing,
        "unexpected_encoding_counts": dict(bad_encoding),
        "finite_aligned_positions": finite,
        "pose_odom_max_position_difference_m": float(np.linalg.norm(p-o, axis=1).max()) if len(p) else None,
        "depth_duration_s": duration,
        "depth_gap_median_s": float(np.median(gaps)) if len(gaps) else None,
        "depth_gap_max_s": float(gaps.max()) if len(gaps) else None,
        "maximum_reported_executed_speed_mps": float(reported_speeds.max()) if len(reported_speeds) else None,
        "aligned_frames_exceeding_2_01_mps": int((reported_speeds > 2.01).sum()),
        "reported_vs_stamp_derived_velocity_max_error_mps": float(velocity_residuals.max()) if len(velocity_residuals) else None,
        "actual_depth_pose_path_m": float(motion.sum()),
        "max_adjacent_depth_pose_displacement_m": float(motion.max()) if len(motion) else None,
        "max_stamp_derived_depth_pose_speed_mps": float(speeds.max()) if len(speeds) else None,
        "adjacent_depth_pose_displacements_over_1m": int((motion > 1.).sum()),
        "stationary_depth_intervals_under_1cm": int((motion < .01).sum()),
        "sensor_z_range_m": [float(p[:, 2].min()), float(p[:, 2].max())] if len(p) else None,
        "max_flight_z_m": args.max_flight_z,
        "depth_pose_height_violation_count": z_violations,
        "last_reported_known_volume_m3": coverage[-1] if coverage else None,
        "simulator_result": result,
        "stream_alignment_passed": bool(finite and not any(missing.values()) and not bad_encoding and np.allclose(p, o, atol=1e-6, rtol=0)),
        "scalar_height_check_passed": bool(finite and z_violations == 0),
        "full_pipeline_validated": False,
        "limitations": [
            "Exact header timestamps verify recorded alignment, not physical hardware exposure timing.",
            "Scalar height compliance does not prove local sloped-roof clearance or building containment.",
            "Stationary intervals are not necessarily wasted: yaw scans can acquire new observations.",
            "Stamp-derived speed diagnoses recorded pose jumps; it is not a full dynamic flight feasibility certificate.",
            "Known volume and trajectory length do not establish per-floor room/wall completeness.",
            "Timeout is not exploration completion; FINISH still requires independent coverage validation.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
