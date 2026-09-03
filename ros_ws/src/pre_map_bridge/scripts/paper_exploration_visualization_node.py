#!/usr/bin/env python3
"""Publish an editable RViz view of the floor-wise exploration concept."""
import csv
import json
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def load_ascii_pcd(path):
    skip = None
    with Path(path).open("rb") as stream:
        for index, line in enumerate(stream):
            if line.strip().lower() == b"data ascii":
                skip = index + 1
                break
    if skip is None:
        raise RuntimeError("Only ASCII PCD is supported: %s" % path)
    values = np.loadtxt(path, dtype=np.float32, skiprows=skip, ndmin=2)
    return values[:, :3]


def floor_indices(points, floor_z, top_clearance):
    difference = points[:, None, 2] - floor_z[None, :]
    valid = (difference >= -0.12) & (difference <= top_clearance)
    cost = np.where(valid, np.abs(difference - 1.0), np.inf)
    result = np.argmin(cost, axis=1)
    result[~np.any(valid, axis=1)] = -1
    return result


def explode(points, indices, floor_z, spacing, lift=0.0):
    result = points.copy()
    result[:, 2] = points[:, 2] - floor_z[indices] + spacing * indices + lift
    return result


class PaperExplorationView:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        spacing = float(rospy.get_param("~floor_spacing", 3.7))
        clearance = float(rospy.get_param("~top_clearance", 2.3))
        lift = float(rospy.get_param("~trajectory_lift", 0.18))
        width = float(rospy.get_param("~trajectory_width", 0.10))

        floor_data = json.loads(Path(rospy.get_param("~floors_path")).read_text())
        floor_z = np.asarray([item["floor_z_m"] for item in floor_data], dtype=np.float32)
        points = load_ascii_pcd(rospy.get_param("~cloud_path"))
        indices = floor_indices(points, floor_z, clearance)
        keep = indices >= 0
        points, indices = points[keep], indices[keep]
        if len(floor_z) < 2 or not np.any(indices == 1):
            raise RuntimeError("The paper view needs at least two populated floors")
        split_y = float(np.median(points[indices == 1, 1]))
        explored = (indices == 0) | ((indices == 1) & (points[:, 1] <= split_y))
        shown = explode(points, indices, floor_z, spacing)

        with Path(rospy.get_param("~trajectory_path")).open() as stream:
            rows = list(csv.DictReader(stream))
        trajectory = np.asarray(
            [[float(row[key]) for key in ("x", "y", "z")] for row in rows],
            dtype=np.float32,
        )
        trajectory_floor = np.argmin(
            np.abs(trajectory[:, None, 2] - (floor_z[None, :] + 1.2)), axis=1
        )
        candidates = np.flatnonzero(
            (trajectory_floor == 1) & (trajectory[:, 1] <= split_y)
        )
        cutoff = int(candidates[0]) if len(candidates) else len(trajectory) - 1
        trajectory = trajectory[: cutoff + 1]
        trajectory_floor = trajectory_floor[: cutoff + 1]
        trajectory_shown = explode(
            trajectory, trajectory_floor, floor_z, spacing, lift=lift
        )

        self.explored_pub = rospy.Publisher(
            "/pre_map_vln/paper_explored", PointCloud2, queue_size=1, latch=True
        )
        self.unexplored_pub = rospy.Publisher(
            "/pre_map_vln/paper_unexplored", PointCloud2, queue_size=1, latch=True
        )
        self.trajectory_pub = rospy.Publisher(
            "/pre_map_vln/paper_trajectory", MarkerArray, queue_size=1, latch=True
        )
        self.explored = self.cloud(shown[explored])
        self.unexplored = self.cloud(shown[~explored])
        self.markers = self.trajectory_markers(
            trajectory_shown, trajectory_floor, width
        )
        rospy.Timer(rospy.Duration(1.0), self.publish)
        self.publish()
        rospy.loginfo(
            "Paper exploration view: %d explored, %d unexplored, %d trajectory points; L2 split y=%.3f",
            int(explored.sum()), int((~explored).sum()), len(trajectory), split_y,
        )

    def cloud(self, points):
        header = Header(frame_id=self.frame_id, stamp=rospy.Time.now())
        return point_cloud2.create_cloud_xyz32(header, points.tolist())

    def trajectory_markers(self, points, floors, width):
        result = MarkerArray()
        for floor in np.unique(floors):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.ns = "executed_trajectory"
            marker.id = int(floor)
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.30, 0.02, 1.0
            for index in range(len(points) - 1):
                if floors[index] == floor and floors[index + 1] == floor:
                    marker.points.extend([Point(*points[index]), Point(*points[index + 1])])
            result.markers.append(marker)
        current = Marker()
        current.header.frame_id = self.frame_id
        current.ns = "current_position"
        current.id = 100
        current.type = Marker.SPHERE
        current.action = Marker.ADD
        current.pose.position = Point(*points[-1])
        current.pose.orientation.w = 1.0
        current.scale.x = current.scale.y = current.scale.z = 0.32
        current.color.r, current.color.g, current.color.b, current.color.a = 0.85, 0.02, 0.04, 1.0
        result.markers.append(current)
        return result

    def publish(self, _event=None):
        stamp = rospy.Time.now()
        self.explored.header.stamp = stamp
        self.unexplored.header.stamp = stamp
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.explored_pub.publish(self.explored)
        self.unexplored_pub.publish(self.unexplored)
        self.trajectory_pub.publish(self.markers)


if __name__ == "__main__":
    rospy.init_node("paper_exploration_visualization")
    PaperExplorationView()
    rospy.spin()
