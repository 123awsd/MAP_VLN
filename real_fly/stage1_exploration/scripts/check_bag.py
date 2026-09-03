#!/usr/bin/env python3
"""Read-only ROS bag integrity, timing, and RGB/depth coverage checks."""

import argparse
import json
import math
import os
import re
import sys
from collections import OrderedDict

try:
    import rosbag
except ImportError as exc:
    raise SystemExit("rosbag Python module is unavailable; source ROS Noetic first") from exc


UNSAFE = re.compile(
    r"cmd_vel|setpoint|position_command|trajectory|mavros|flight|motor|takeoff|"
    r"offboard|px4|ardupilot|mavlink|habitat|uav_simulator|sensor_pose",
    re.IGNORECASE,
)


def read_env(path):
    values = {}
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def stamp_seconds(message, bag_time):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        value = stamp.to_sec()
        if value > 0:
            return float(value)
    return float(bag_time.to_sec())


def new_topic(msg_type):
    return OrderedDict(
        type=msg_type,
        count=0,
        first_time=None,
        last_time=None,
        backwards=0,
        image_count=0,
        image_width=None,
        image_height=None,
        image_encoding=None,
        empty_image_count=0,
        pose_count=0,
        first_pose=None,
        last_pose=None,
    )


def update_topic(info, message, bag_time):
    current = stamp_seconds(message, bag_time)
    if info["last_time"] is not None and current < info["last_time"]:
        info["backwards"] += 1
    if info["first_time"] is None:
        info["first_time"] = current
    info["last_time"] = current
    info["count"] += 1

    if hasattr(message, "width") and hasattr(message, "height") and hasattr(message, "data"):
        info["image_count"] += 1
        info["image_width"] = int(message.width)
        info["image_height"] = int(message.height)
        info["image_encoding"] = str(getattr(message, "encoding", ""))
        if int(message.width) <= 0 or int(message.height) <= 0 or len(message.data) == 0:
            info["empty_image_count"] += 1

    pose = getattr(getattr(message, "pose", None), "pose", None)
    position = getattr(pose, "position", None)
    if position is not None and all(hasattr(position, axis) for axis in ("x", "y", "z")):
        values = [float(position.x), float(position.y), float(position.z)]
        info["pose_count"] += 1
        if all(math.isfinite(value) for value in values):
            if info["first_pose"] is None:
                info["first_pose"] = values
            info["last_pose"] = values


def summarize(info):
    result = OrderedDict(info)
    first = info["first_time"]
    last = info["last_time"]
    duration = (last - first) if first is not None and last is not None else 0.0
    result["duration_sec"] = duration
    result["rate_hz"] = ((info["count"] - 1) / duration) if duration > 0 and info["count"] > 1 else 0.0
    return result


def tf_graph(bag_path, topics):
    edges = set()
    frames = set()
    with rosbag.Bag(bag_path, "r") as bag:
        for topic in (topics.get("TF_TOPIC", "/tf"), topics.get("TF_STATIC_TOPIC", "/tf_static")):
            if not topic:
                continue
            for _, message, _ in bag.read_messages(topics=[topic]):
                for transform in getattr(message, "transforms", []):
                    parent = str(getattr(getattr(transform, "header", None), "frame_id", ""))
                    child = str(getattr(transform, "child_frame_id", ""))
                    if parent and child:
                        frames.update((parent, child))
                        edges.add((parent, child))
    adjacency = {frame: set() for frame in frames}
    children = set()
    for parent, child in edges:
        adjacency[parent].add(child)
        adjacency[child].add(parent)
        children.add(child)
    components = 0
    unseen = set(frames)
    while unseen:
        components += 1
        stack = [unseen.pop()]
        while stack:
            current = stack.pop()
            for neighbor in adjacency[current]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    stack.append(neighbor)
    return OrderedDict(
        frames=sorted(frames),
        edges=[OrderedDict(parent=parent, child=child) for parent, child in sorted(edges)],
        root_frames=sorted(frames - children),
        component_count=components,
        status="ok" if edges else "missing",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topics-config")
    parser.add_argument("--output")
    parser.add_argument("--require-rgb", action="store_true")
    args = parser.parse_args()

    expected = read_env(args.topics_config) if args.topics_config else {}
    required = OrderedDict(
        lidar=(expected.get("LIDAR_TOPIC", ""), expected.get("LIDAR_TYPE", "")),
        imu=(expected.get("IMU_TOPIC", ""), expected.get("IMU_TYPE", "")),
    )
    if args.require_rgb:
        required.update(
            rgb=(expected.get("RGB_TOPIC", ""), expected.get("RGB_TYPE", "")),
            depth=(expected.get("DEPTH_TOPIC", ""), expected.get("DEPTH_TYPE", "")),
            rgb_camera_info=(expected.get("RGB_CAMERA_INFO_TOPIC", ""), expected.get("RGB_CAMERA_INFO_TYPE", "")),
            depth_camera_info=(expected.get("DEPTH_CAMERA_INFO_TOPIC", ""), expected.get("DEPTH_CAMERA_INFO_TYPE", "")),
        )
        camera_imu_topic = expected.get("CAMERA_IMU_TOPIC", "")
        if camera_imu_topic:
            required["camera_imu"] = (camera_imu_topic, expected.get("CAMERA_IMU_TYPE", ""))
        else:
            required["camera_gyro"] = (expected.get("CAMERA_GYRO_TOPIC", ""), expected.get("CAMERA_GYRO_TYPE", ""))
            required["camera_accel"] = (expected.get("CAMERA_ACCEL_TOPIC", ""), expected.get("CAMERA_ACCEL_TYPE", ""))
        required["tf"] = (expected.get("TF_TOPIC", ""), expected.get("TF_TYPE", ""))
        required["tf_static"] = (expected.get("TF_STATIC_TOPIC", ""), expected.get("TF_STATIC_TYPE", ""))

    topics = OrderedDict()
    unsafe_topics = []
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, bag_time in bag.read_messages():
            if UNSAFE.search(topic):
                unsafe_topics.append(topic)
            msg_type = getattr(message, "_type", message.__class__.__name__)
            if topic not in topics:
                topics[topic] = new_topic(msg_type)
            update_topic(topics[topic], message, bag_time)

    report = OrderedDict(
        bag=os.path.abspath(args.bag),
        topics=OrderedDict((topic, summarize(info)) for topic, info in topics.items()),
        tf_graph=tf_graph(args.bag, expected),
        required=OrderedDict(),
        errors=[],
        warnings=[],
    )
    if unsafe_topics:
        report["errors"].append("unsafe/control-like topics found: " + ", ".join(sorted(set(unsafe_topics))))

    for role, (topic, expected_type) in required.items():
        if not topic:
            report["required"][role] = {"status": "unconfigured"}
            report["errors"].append("missing topic configuration for " + role)
            continue
        info = report["topics"].get(topic)
        if info is None:
            report["required"][role] = {"topic": topic, "status": "missing"}
            report["errors"].append("required topic absent: " + topic)
            continue
        result = OrderedDict(topic=topic, status="ok", type=info["type"], count=info["count"], rate_hz=info["rate_hz"])
        if expected_type and info["type"] != expected_type:
            result["status"] = "type_mismatch"
            report["errors"].append("type mismatch for %s: expected %s, got %s" % (topic, expected_type, info["type"]))
        if info["count"] < 2:
            report["warnings"].append("too few messages on " + topic)
        if info["backwards"]:
            report["warnings"].append("non-monotonic timestamps on %s: %d" % (topic, info["backwards"]))
        report["required"][role] = result

    image_topics = [topic for topic, info in report["topics"].items() if info["image_count"] > 0]
    report["rgb_coverage"] = OrderedDict(
        verification_level="temporal_stream_only",
        image_topics=image_topics,
        note="Geometric point-to-pixel coverage requires measured camera-LiDAR extrinsics and projection; this check does not claim it.",
    )
    for role in ("rgb", "depth"):
        topic = required.get(role, ("", ""))[0]
        if topic in report["topics"]:
            info = report["topics"][topic]
            report["rgb_coverage"][role] = OrderedDict(
                topic=topic,
                frames=info["image_count"],
                width=info["image_width"],
                height=info["image_height"],
                encoding=info["image_encoding"],
                empty_frames=info["empty_image_count"],
                start_sec=info["first_time"],
                end_sec=info["last_time"],
            )

    report["status"] = "ok" if not report["errors"] else "failed"
    payload = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        with open(args.output, "x", encoding="utf-8") as handle:
            handle.write(payload)
        print(args.output)
    else:
        print(payload, end="")
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
