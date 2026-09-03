#!/usr/bin/env python3
"""Measure recorded RGB/depth temporal coverage without claiming colorization."""

import argparse
import json
import math
import os
import sys
from collections import OrderedDict

try:
    import rosbag
except ImportError as exc:
    raise SystemExit("rosbag Python module is unavailable; source ROS Noetic first") from exc


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


def stamp(message, bag_time):
    header = getattr(message, "header", None)
    value = getattr(getattr(header, "stamp", None), "to_sec", lambda: 0.0)()
    return float(value) if value > 0 else float(bag_time.to_sec())


def empty_summary(topic):
    return OrderedDict(
        topic=topic,
        count=0,
        first_sec=None,
        last_sec=None,
        duration_sec=0.0,
        rate_hz=0.0,
        backwards=0,
        empty_frames=0,
        width=None,
        height=None,
        encoding=None,
    )


def summarize(bag_path, topic):
    info = empty_summary(topic)
    with rosbag.Bag(bag_path, "r") as bag:
        for current_topic, message, bag_time in bag.read_messages(topics=[topic]):
            current = stamp(message, bag_time)
            if info["last_sec"] is not None and current < info["last_sec"]:
                info["backwards"] += 1
            if info["first_sec"] is None:
                info["first_sec"] = current
            info["last_sec"] = current
            info["count"] += 1
            if hasattr(message, "width") and hasattr(message, "height") and hasattr(message, "data"):
                info["width"] = int(message.width)
                info["height"] = int(message.height)
                info["encoding"] = str(getattr(message, "encoding", ""))
                if info["width"] <= 0 or info["height"] <= 0 or len(message.data) == 0:
                    info["empty_frames"] += 1
    if info["first_sec"] is not None and info["last_sec"] is not None:
        info["duration_sec"] = max(0.0, info["last_sec"] - info["first_sec"])
    if info["duration_sec"] > 0 and info["count"] > 1:
        info["rate_hz"] = (info["count"] - 1) / info["duration_sec"]
    return info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topics-config", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    if not os.path.isfile(args.bag):
        raise SystemExit("bag not found: " + args.bag)
    expected = read_env(args.topics_config)
    keys = OrderedDict(
        rgb="RGB_TOPIC",
        depth="DEPTH_TOPIC",
        rgb_camera_info="RGB_CAMERA_INFO_TOPIC",
        depth_camera_info="DEPTH_CAMERA_INFO_TOPIC",
    )
    summaries = OrderedDict()
    errors = []
    for role, key in keys.items():
        topic = expected.get(key, "")
        if not topic:
            errors.append("missing topic configuration for " + role)
            continue
        summaries[role] = summarize(args.bag, topic)
        if summaries[role]["count"] == 0:
            errors.append("topic absent: " + topic)
        if summaries[role]["backwards"]:
            errors.append("non-monotonic timestamps: " + topic)

    image_roles = [role for role in ("rgb", "depth") if role in summaries and summaries[role]["count"]]
    overlap_start = None
    overlap_end = None
    if image_roles:
        overlap_start = max(summaries[role]["first_sec"] for role in image_roles)
        overlap_end = min(summaries[role]["last_sec"] for role in image_roles)
    overlap_duration = max(0.0, overlap_end - overlap_start) if overlap_start is not None and overlap_end is not None else 0.0

    result = OrderedDict(
        bag=os.path.abspath(args.bag),
        streams=summaries,
        temporal_coverage=OrderedDict(
            status="ok" if image_roles and overlap_duration > 0 and not errors else "failed",
            overlap_start_sec=overlap_start,
            overlap_end_sec=overlap_end,
            overlap_duration_sec=overlap_duration,
        ),
        geometric_projection=OrderedDict(
            status="not_run",
            colored_point_ratio=None,
            note="Camera-LiDAR extrinsics, distortion, time offset, TF, and a projection implementation are required; this script deliberately does not invent them.",
        ),
        errors=errors,
        status="ok" if not errors and image_roles and overlap_duration > 0 else "failed",
    )
    payload = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        if os.path.exists(args.output):
            raise SystemExit("refusing to overwrite output: " + args.output)
        with open(args.output, "x", encoding="utf-8") as handle:
            handle.write(payload)
        print(args.output)
    else:
        print(payload, end="")
    return 0 if result["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
