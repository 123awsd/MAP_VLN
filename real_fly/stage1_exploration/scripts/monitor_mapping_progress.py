#!/usr/bin/env python3
"""Record the latest FAST-LIO odometry stamp for offline queue draining."""

import argparse
import json
import os
import tempfile
import time

import rospy
from nav_msgs.msg import Odometry


def atomic_write(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".mapping-progress-", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic", default="/Odometry")
    args = parser.parse_args()

    state = {
        "topic": args.topic,
        "message_count": 0,
        "first_header_stamp": None,
        "last_header_stamp": None,
        "last_wall_time": None,
    }

    def callback(message):
        stamp = message.header.stamp.to_sec()
        state["message_count"] += 1
        if state["first_header_stamp"] is None:
            state["first_header_stamp"] = stamp
        state["last_header_stamp"] = stamp
        state["last_wall_time"] = time.time()
        atomic_write(args.output, state)

    rospy.init_node("stage1_mapping_progress", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.topic, Odometry, callback, queue_size=10000)
    atomic_write(args.output, state)
    rospy.spin()


if __name__ == "__main__":
    main()
