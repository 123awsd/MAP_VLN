#!/usr/bin/env python3
"""Record a bounded-size stage-1 replay Bag while FALCON is running.

RGB, pose and planning streams are written online. Growing occupancy and
frontier clouds are kept only in memory and written once during shutdown, so
the replay Bag contains the final map without storing every large map update.
"""

from __future__ import annotations

import threading
from pathlib import Path

import rosbag
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32, Int32, String
from trajectory.msg import Bspline
from visualization_msgs.msg import Marker


class CompactRecorder:
    def __init__(self) -> None:
        bag_path = Path(rospy.get_param("~bag_path"))
        if bag_path.exists():
            raise FileExistsError(bag_path)
        bag_path.parent.mkdir(parents=True, exist_ok=True)
        self.bag = rosbag.Bag(str(bag_path), "w", compression=rosbag.Compression.LZ4)
        self.lock = threading.RLock()
        self.rgb_stride = max(1, int(rospy.get_param("~rgb_stride", 5)))
        self.rgb_count = 0
        self.last_stamp = None
        self.final_messages = {}
        self.subscribers = []

        self.subscribe("/habitat/rgb", Image, self.record_rgb)
        self.subscribe("/uav_simulator/odometry", Odometry, self.record)
        self.subscribe("/uav_simulator/sensor_pose", TransformStamped, self.record)
        self.subscribe("/pre_map_vln/agent_path", RosPath, self.record)
        self.subscribe("/pre_map_vln/exploration_status", String, self.record)
        self.subscribe("/planning/replan", Int32, self.record)
        self.subscribe("/planning/bspline", Bspline, self.record)
        self.subscribe("/planning/travel_traj", Marker, self.record)
        self.subscribe("/voxel_mapping/map_coverage", Float32, self.record)

        self.subscribe_final("/planning_vis/frontier", Marker)
        self.subscribe_final("/planning_vis/frontier_pcl", PointCloud2)
        self.subscribe_final("/planning_vis/dormant_frontier_pcl", PointCloud2)
        self.subscribe_final("/voxel_mapping/occupancy_grid_occupied", PointCloud2)
        self.subscribe_final("/voxel_mapping/occupancy_grid_free", PointCloud2)
        self.subscribe_final("/voxel_mapping/occupancy_grid_unknown", PointCloud2)
        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "Compact stage-1 Bag recorder writing %s (RGB stride %d)",
            bag_path,
            self.rgb_stride,
        )

    @staticmethod
    def stamp(message):
        header = getattr(message, "header", None)
        if header is not None and header.stamp.to_sec() > 0:
            return header.stamp
        return rospy.Time.now()

    def subscribe(self, topic, message_type, callback):
        self.subscribers.append(
            rospy.Subscriber(topic, message_type, callback, callback_args=topic, queue_size=20)
        )

    def subscribe_final(self, topic, message_type):
        self.subscribers.append(
            rospy.Subscriber(
                topic, message_type, self.final_callback, callback_args=topic, queue_size=1
            )
        )

    def record_rgb(self, message, topic):
        index = self.rgb_count
        self.rgb_count += 1
        if index % self.rgb_stride == 0:
            self.record(message, topic)

    def record(self, message, topic):
        stamp = self.stamp(message)
        with self.lock:
            self.bag.write(topic, message, stamp)
            self.last_stamp = stamp

    def final_callback(self, message, topic):
        with self.lock:
            self.final_messages[topic] = message

    def close(self):
        with self.lock:
            if self.bag is None:
                return
            if self.last_stamp is not None:
                for topic in sorted(self.final_messages):
                    self.bag.write(topic, self.final_messages[topic], self.last_stamp)
            self.bag.close()
            self.bag = None
            rospy.loginfo(
                "Compact stage-1 Bag closed with %d final snapshots",
                len(self.final_messages),
            )


if __name__ == "__main__":
    rospy.init_node("compact_bag_recorder")
    CompactRecorder()
    rospy.spin()
