#!/usr/bin/env python3
"""Capture a stable, disarmed world pose and predict PX4Ctrl's hover pose.

Read-only: this node subscribes to odometry and FCU state and publishes nothing.
"""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import yaml


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def load_takeoff_height(path):
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    try:
        value = float(document["auto_takeoff_land"]["takeoff_height"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"missing auto_takeoff_land/takeoff_height in {path}") from error
    if not 0.2 <= value <= 3.0:
        raise ValueError(f"unsafe takeoff_height outside [0.2, 3.0] m: {value}")
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--px4ctrl-config", type=Path, required=True)
    parser.add_argument("--odom-topic", default="/ekf_quat/ekf_odom")
    parser.add_argument("--sample-sec", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--max-position-span", type=float, default=0.10)
    parser.add_argument("--max-speed", type=float, default=0.10)
    args = parser.parse_args()
    if not args.px4ctrl_config.is_file():
        raise SystemExit(f"missing PX4Ctrl config: {args.px4ctrl_config}")
    takeoff_height = load_takeoff_height(args.px4ctrl_config)

    import rospy
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry

    rospy.init_node("pre_map_vln_capture_planning_start", anonymous=True,
                    disable_signals=True)
    state = rospy.wait_for_message("/mavros/state", State, timeout=args.timeout)
    if not state.connected:
        raise SystemExit("FCU is not connected")
    if state.armed:
        raise SystemExit("refusing to generate a new mission while the FCU is armed")

    samples = []
    deadline = time.monotonic() + args.sample_sec
    while time.monotonic() < deadline:
        message = rospy.wait_for_message(args.odom_topic, Odometry, timeout=args.timeout)
        if message.header.frame_id != "world":
            raise SystemExit(
                f"odometry frame is {message.header.frame_id!r}, expected 'world'"
            )
        p = message.pose.pose.position
        v = message.twist.twist.linear
        values = [p.x, p.y, p.z, v.x, v.y, v.z]
        if not all(math.isfinite(value) for value in values):
            raise SystemExit("odometry contains non-finite values")
        speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        samples.append((p.x, p.y, p.z, yaw_from_quaternion(message.pose.pose.orientation), speed))

    if len(samples) < 5:
        raise SystemExit(f"too few odometry samples: {len(samples)}")
    spans = [max(row[i] for row in samples) - min(row[i] for row in samples) for i in range(3)]
    position_span = math.sqrt(sum(value * value for value in spans))
    maximum_speed = max(row[4] for row in samples)
    if position_span > args.max_position_span:
        raise SystemExit(
            f"stationary position span {position_span:.3f} m exceeds "
            f"{args.max_position_span:.3f} m"
        )
    if maximum_speed > args.max_speed:
        raise SystemExit(
            f"stationary speed {maximum_speed:.3f} m/s exceeds {args.max_speed:.3f} m/s"
        )

    middle = samples[len(samples) // 2]
    ground = list(middle[:4])
    hover = [ground[0], ground[1], ground[2] + takeoff_height, ground[3]]
    document = {
        "format": "pre_map_vln.runtime_planning_start.v1",
        "frame_id": "world",
        "odom_topic": args.odom_topic,
        "ground_xyz_yaw": ground,
        "takeoff_height_m": takeoff_height,
        "planned_hover_xyz_yaw": hover,
        "sample_count": len(samples),
        "stationary_position_span_m": position_span,
        "maximum_reported_speed_mps": maximum_speed,
        "px4ctrl_config": str(args.px4ctrl_config.resolve()),
        "px4ctrl_config_sha256": sha256(args.px4ctrl_config),
        "fcu_connected": True,
        "fcu_armed": False,
        "safety_scope": "read-only pose capture; no topic or service output",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print("ground x y z yaw =", *ground)
    print("PX4Ctrl takeoff height =", takeoff_height)
    print("planned hover x y z yaw =", *hover)


if __name__ == "__main__":
    main()
