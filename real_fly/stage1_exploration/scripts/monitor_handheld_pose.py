#!/usr/bin/env python3
"""Print the live FAST-LIO mapping pose without publishing ROS data."""

import argparse
import math
import threading

import rospy
from nav_msgs.msg import Odometry


def yaw_degrees(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/Odometry")
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--jump-warning-m", type=float, default=1.5)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be positive")

    rospy.init_node("stage1_handheld_pose_monitor", anonymous=True)
    lock = threading.Lock()
    latest = [None]

    def callback(message):
        with lock:
            latest[0] = message

    rospy.Subscriber(args.topic, Odometry, callback, queue_size=1)
    rate = rospy.Rate(args.rate_hz)
    start_xyz = None
    previous_xyz = None
    waiting_printed = False

    while not rospy.is_shutdown():
        with lock:
            message = latest[0]
        if message is None:
            if not waiting_printed:
                print(f"[MAP POSE] waiting for {args.topic} ...", flush=True)
                waiting_printed = True
            rate.sleep()
            continue

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        frame = message.header.frame_id or "--"

        if not all(math.isfinite(value) for value in values):
            print(f"[MAP POSE][WARN] frame={frame} contains non-finite values", flush=True)
            rate.sleep()
            continue

        xyz = (position.x, position.y, position.z)
        if start_xyz is None:
            start_xyz = xyz
        distance = math.dist(xyz, start_xyz)
        step = math.dist(xyz, previous_xyz) if previous_xyz is not None else 0.0
        previous_xyz = xyz
        warnings = []
        if frame != "world":
            warnings.append(f"frame={frame}")
        if step > args.jump_warning_m:
            warnings.append(f"jump={step:.2f}m")
        suffix = " WARN:" + ",".join(warnings) if warnings else ""

        print(
            "[MAP POSE] "
            f"x={xyz[0]:8.3f}  y={xyz[1]:8.3f}  z={xyz[2]:7.3f}  "
            f"yaw={yaw_degrees(orientation):7.1f}deg  "
            f"from_start={distance:7.2f}m  step={step:5.2f}m{suffix}",
            flush=True,
        )
        rate.sleep()


if __name__ == "__main__":
    main()
