#!/usr/bin/env python3
"""Drive the cached RViz timeline at a fixed, low frame rate."""

import argparse
import time

import rospy
from std_msgs.msg import Int32


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--fps", type=float, default=4.0)
    parser.add_argument("--lead", type=float, default=1.0)
    args = parser.parse_args()
    rospy.init_node("timeline_index_driver", anonymous=True)
    publisher = rospy.Publisher("/pre_map_vln/timeline_index", Int32, queue_size=1)
    time.sleep(max(0.0, args.lead))
    period = 1.0 / max(0.1, args.fps)
    for index in range(0, 101, max(1, args.stride)):
        if rospy.is_shutdown():
            break
        publisher.publish(Int32(data=index))
        rospy.loginfo("timeline frame %d", index)
        time.sleep(period)
    time.sleep(period)


if __name__ == "__main__":
    main()
