#!/usr/bin/env python3
"""Extract sparse, timestamp-aligned RGB samples without replaying a full bag."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import rosbag
import rospy
from cv_bridge import CvBridge


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("timeline", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--topic", default="/habitat/rgb")
    args = parser.parse_args()

    timeline = np.load(args.timeline, allow_pickle=False)
    targets = timeline["snapshot_times"][:: max(1, args.stride)].astype(float)
    args.output.mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()
    saved = []

    with rosbag.Bag(str(args.bag)) as bag:
        for index, target in enumerate(targets):
            chosen = None
            best_delta = float("inf")
            for _, message, stamp in bag.read_messages(
                topics=[args.topic],
                start_time=rospy.Time.from_sec(max(bag.get_start_time(), target - 12.0)),
                end_time=rospy.Time.from_sec(min(bag.get_end_time(), target + 12.0)),
            ):
                delta = abs(stamp.to_sec() - target)
                if delta < best_delta:
                    chosen, best_delta = message, delta
            if chosen is None:
                raise RuntimeError("No RGB frame near %.3f" % target)
            rgb = bridge.imgmsg_to_cv2(chosen, desired_encoding="rgb8")
            path = args.output / ("rgb_%03d.jpg" % index)
            cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
            saved.append((index, target, best_delta, path.name))
            print("%03d/%03d delta=%.3fs %s" % (index + 1, len(targets), best_delta, path.name), flush=True)

    with (args.output / "timestamps.csv").open("w", encoding="utf-8") as stream:
        stream.write("index,target_time_s,delta_s,file\n")
        for index, target, delta, name in saved:
            stream.write("%d,%.9f,%.6f,%s\n" % (index, target, delta, name))


if __name__ == "__main__":
    main()
