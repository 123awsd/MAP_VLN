#!/usr/bin/env python3
"""Highlight the UAV's current floor while retaining other floors as gray context."""

import json
import struct

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String


FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("rgb", 12, PointField.FLOAT32, 1),
]


def packed_rgb(r, g, b):
    return struct.unpack("f", struct.pack("I", (int(r) << 16) | (int(g) << 8) | int(b)))[0]


class FloorHighlighter:
    def __init__(self):
        graph = json.load(open(rospy.get_param("~scene_graph"), encoding="utf-8"))
        self.max_floor = int(rospy.get_param("~max_floor", 3))
        self.floors = sorted(
            (item for item in graph.get("floors", []) if int(item["floor_id"]) <= self.max_floor),
            key=lambda item: float(item["floor_z_m"]),
        )
        if not self.floors:
            raise RuntimeError("scene graph contains no visible floor definitions")
        spacings = [float(b["floor_z_m"]) - float(a["floor_z_m"]) for a, b in zip(self.floors, self.floors[1:])]
        self.typical_spacing = float(np.median(spacings)) if spacings else 2.8
        self.maximum_visible_z = float(self.floors[-1]["floor_z_m"]) + self.typical_spacing
        self.cloud = None
        self.active_floor = None
        self.complete = False
        self.active_pub = rospy.Publisher("/stage2/floor_active", PointCloud2, queue_size=1, latch=True)
        self.context_pub = rospy.Publisher("/stage2/floor_context", PointCloud2, queue_size=1, latch=True)
        cloud_topic = rospy.get_param("~cloud_topic", "/pre_map_vln/roofless_map")
        rospy.Subscriber(cloud_topic, PointCloud2, self.on_cloud, queue_size=1)
        rospy.Subscriber("/stage2/uav_pose", Odometry, self.on_pose, queue_size=5)
        rospy.Subscriber("/stage2/mission_state", String, self.on_mission_state, queue_size=2)

    def floor_for_z(self, z):
        for floor in self.floors:
            low, high = floor.get("camera_height_band_m", [floor["floor_z_m"], floor["floor_z_m"] + 2.2])
            if float(low) <= z <= float(high):
                return int(floor["floor_id"])
        return int(min(self.floors, key=lambda item: abs(z - (float(item["floor_z_m"]) + 1.0)))["floor_id"])

    def point_floor(self, z):
        selected = self.floors[0]
        for floor in self.floors:
            if z >= float(floor["floor_z_m"]) - 0.25:
                selected = floor
            else:
                break
        return int(selected["floor_id"])

    def on_mission_state(self, message):
        complete = message.data == "complete"
        if complete != self.complete:
            self.complete = complete
            if self.cloud is not None:
                self.publish()

    def on_cloud(self, message):
        self.cloud = [(float(x), float(y), float(z)) for x, y, z in point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)]
        if self.active_floor is not None:
            self.publish()

    def on_pose(self, message):
        floor_id = self.floor_for_z(float(message.pose.pose.position.z))
        if floor_id != self.active_floor:
            self.active_floor = floor_id
            if self.cloud is not None:
                self.publish()

    def publish(self):
        # Distinct cool colors for floor context; deliberately no orange.
        floor_colors = {1: (40, 155, 235), 2: (150, 95, 225), 3: (35, 190, 125)}
        active, context = [], []
        for x, y, z in self.cloud:
            if z > self.maximum_visible_z:
                continue
            floor_id = self.point_floor(z)
            floor = next(item for item in self.floors if int(item["floor_id"]) == floor_id)
            base_z = float(floor["floor_z_m"])
            relative = max(0.0, min(1.0, (z - base_z) / max(0.1, self.typical_spacing)))
            context.append((x, y, z, packed_rgb(*floor_colors.get(floor_id, (130, 130, 150)))))
        header = Header(stamp=rospy.Time.now(), frame_id="world")
        self.active_pub.publish(point_cloud2.create_cloud(header, FIELDS, active))
        self.context_pub.publish(point_cloud2.create_cloud(header, FIELDS, context))


if __name__ == "__main__":
    rospy.init_node("stage2_floor_highlight")
    FloorHighlighter()
    rospy.spin()
