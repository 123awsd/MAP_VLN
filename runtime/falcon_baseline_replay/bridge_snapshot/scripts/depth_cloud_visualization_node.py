#!/usr/bin/env python3
"""Publish a cached paper-figure point cloud for interactive RViz inspection."""
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def main():
    rospy.init_node("depth_cloud_visualization")
    path = Path(rospy.get_param("~cloud_cache"))
    frame_id = rospy.get_param("~frame_id", "world")
    with np.load(path) as data:
        points = data["points"].astype(np.float32)
    publisher = rospy.Publisher(
        "/pre_map_vln/paper_depth_cloud", PointCloud2, queue_size=1, latch=True
    )
    message = point_cloud2.create_cloud_xyz32(
        Header(stamp=rospy.Time.now(), frame_id=frame_id), points.tolist()
    )
    publisher.publish(message)
    rospy.loginfo("Published %d depth-only points from %s", len(points), path)
    rospy.spin()


if __name__ == "__main__":
    main()
