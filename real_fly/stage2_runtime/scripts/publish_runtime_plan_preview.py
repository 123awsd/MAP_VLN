#!/usr/bin/env python3
"""Publish an NX-generated mission and approved semantics for RViz only."""
import argparse
import colorsys
import json
import math
import zlib
from pathlib import Path

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import Marker, MarkerArray


def color(label):
    hue = (zlib.crc32(label.encode("utf-8")) % 360) / 360.0
    return colorsys.hsv_to_rgb(hue, 0.78, 1.0)


def point(values):
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def add_frustums(markers, mission, candidates):
    by_id = {
        item["id"]: item
        for values in candidates.get("by_task", {}).values()
        for item in values
    }
    for index, visit in enumerate(mission.get("visits", [])):
        candidate = by_id.get(visit.get("candidate_id"))
        if candidate is None:
            continue
        pose = visit["pose"]
        origin = [float(pose[key]) for key in ("x", "y", "z")]
        target = [float(value) for value in candidate.get("target_xyz_m", origin)]
        yaw = float(pose["yaw"])
        depth = min(3.0, max(0.5, math.dist(origin, target)))
        half_width = depth * math.tan(math.radians(float(candidate.get("horizontal_fov_deg", 90.0))) / 2.0)
        half_height = depth * math.tan(math.radians(float(candidate.get("vertical_fov_deg", 70.0))) / 2.0)
        forward = [math.cos(yaw), math.sin(yaw), 0.0]
        right = [-math.sin(yaw), math.cos(yaw), 0.0]
        center = [origin[i] + depth * forward[i] for i in range(3)]
        corners = [
            [center[0] + side * half_width * right[0],
             center[1] + side * half_width * right[1],
             center[2] + vertical * half_height]
            for side, vertical in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        ]
        edges = [(origin, corner) for corner in corners]
        edges += [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
        edges.append((origin, target))
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "observation_frustums"
        marker.id = index
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.82, 0.1, 0.95
        marker.points = [point(value) for edge in edges for value in edge]
        markers.markers.append(marker)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    args = parser.parse_args()
    scene = json.loads(args.scene.read_text(encoding="utf-8"))
    mission = json.loads(args.mission.read_text(encoding="utf-8"))
    candidates = json.loads(args.candidates.read_text(encoding="utf-8"))
    rospy.init_node("pre_map_vln_runtime_plan_preview", anonymous=False)

    path = RosPath()
    path.header.frame_id = "world"
    for segment in mission.get("segments", []):
        validated = segment.get("validated_trajectory") or {}
        for xyz in validated.get("points_xyz_m") or segment.get("points_xyz_m", []):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
    path_pub = rospy.Publisher("/pre_map_vln/preview/planned_path", RosPath,
                               queue_size=1, latch=True)
    path_pub.publish(path)

    markers = MarkerArray()
    marker_id = 0
    for room in scene.get("rooms", []):
        polygon = room.get("polygon_xy_m") or []
        if len(polygon) >= 3:
            boundary = Marker()
            boundary.header.frame_id = "world"
            boundary.ns = "approved_rooms"
            boundary.id = marker_id
            marker_id += 1
            boundary.type = Marker.LINE_STRIP
            boundary.action = Marker.ADD
            boundary.pose.orientation.w = 1.0
            boundary.scale.x = 0.06
            boundary.color.r, boundary.color.g, boundary.color.b, boundary.color.a = 0.1, 0.7, 1.0, 0.9
            z = float(room.get("z_min_m", 0.0)) + 0.05
            boundary.points = [Point(x=float(x), y=float(y), z=z) for x, y in polygon + [polygon[0]]]
            markers.markers.append(boundary)
        for obj in room.get("objects", []):
            center = obj.get("center_xyz_m")
            size = obj.get("size_xyz_m")
            orientation = obj.get("orientation_wxyz")
            if not center or not size or not orientation:
                continue
            cube = Marker()
            cube.header.frame_id = "world"
            cube.ns = "approved_objects"
            cube.id = marker_id
            marker_id += 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = map(float, center)
            cube.pose.orientation.w = float(orientation[0])
            cube.pose.orientation.x = float(orientation[1])
            cube.pose.orientation.y = float(orientation[2])
            cube.pose.orientation.z = float(orientation[3])
            cube.scale.x, cube.scale.y, cube.scale.z = [max(0.05, float(value)) for value in size]
            cube.color.r, cube.color.g, cube.color.b = color(str(obj.get("label", "object")))
            cube.color.a = 0.30
            markers.markers.append(cube)
            label = Marker()
            label.header.frame_id = "world"
            label.ns = "approved_object_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(center[0])
            label.pose.position.y = float(center[1])
            label.pose.position.z = float(center[2]) + float(size[2]) / 2.0 + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(obj.get("label", "object"))
            markers.markers.append(label)

    start = mission.get("start_xyz_yaw", [])
    if len(start) == 4:
        start_marker = Marker()
        start_marker.header.frame_id = "world"
        start_marker.ns = "planned_start"
        start_marker.id = marker_id
        marker_id += 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x, start_marker.pose.position.y, start_marker.pose.position.z = map(float, start[:3])
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.24
        start_marker.color.r, start_marker.color.g, start_marker.color.b, start_marker.color.a = 0.2, 1.0, 0.2, 1.0
        markers.markers.append(start_marker)
    for index, visit in enumerate(mission.get("visits", [])):
        pose = visit.get("pose", {})
        goal = Marker()
        goal.header.frame_id = "world"
        goal.ns = "observation_goals"
        goal.id = marker_id + index
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.position.x = float(pose["x"])
        goal.pose.position.y = float(pose["y"])
        goal.pose.position.z = float(pose["z"])
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.22
        goal.color.r, goal.color.g, goal.color.b, goal.color.a = 1.0, 0.25, 0.1, 1.0
        markers.markers.append(goal)
    add_frustums(markers, mission, candidates)
    marker_pub = rospy.Publisher("/pre_map_vln/preview/markers", MarkerArray,
                                 queue_size=1, latch=True)
    marker_pub.publish(markers)
    rospy.loginfo("Preview published: %d path poses, %d markers", len(path.poses), len(markers.markers))
    rospy.spin()


if __name__ == "__main__":
    main()
