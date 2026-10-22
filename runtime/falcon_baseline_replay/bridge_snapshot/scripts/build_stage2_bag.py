#!/usr/bin/env python3
"""Build a self-contained ROS bag for the stage-two Habitat mission replay."""

from __future__ import annotations

import argparse
import copy
import json
import math
import re
from pathlib import Path

import cv2
import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def stamp_message(message, stamp):
    if hasattr(message, "header"):
        message.header.stamp = stamp
        if not message.header.frame_id:
            message.header.frame_id = "world"
    return message


def final_topic_message(bag_path: Path, topic: str):
    result = None
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[topic]):
            result = message
    if result is None:
        raise RuntimeError(f"topic {topic} not found in {bag_path}")
    return result


def yaw_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def matrix_to_xyzw(matrix: np.ndarray):
    trace = float(np.trace(matrix))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (matrix[2, 1] - matrix[1, 2]) / s, (matrix[0, 2] - matrix[2, 0]) / s, (matrix[1, 0] - matrix[0, 1]) / s
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w, x, y, z = (matrix[2, 1] - matrix[1, 2]) / s, 0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s
        elif index == 1:
            s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w, x, y, z = (matrix[0, 2] - matrix[2, 0]) / s, (matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w, x, y, z = (matrix[1, 0] - matrix[0, 1]) / s, (matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s
    return x, y, z, w


def optical_quaternion(yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    body = np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
    optical_to_body = np.asarray([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
    return matrix_to_xyzw(body @ optical_to_body)


def box_marker(obj, marker_id: int, stamp, status: str = "base"):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = stamp
    marker.ns = "stage2_context_boxes" if status == "context" else "stage2_task_boxes"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    is_context = status == "context"
    marker.scale.x = 0.022 if is_context else 0.100
    colors = {
        "context": (0.08, 0.32, 0.48, 0.32),
        "pending": (0.48, 0.52, 0.58, 0.48),
        "current": (0.02, 0.48, 0.92, 1.00),
        "found": (0.02, 0.72, 0.18, 1.00),
        "not_found": (0.88, 0.08, 0.05, 1.00),
        "skipped": (0.42, 0.42, 0.42, 0.25),
    }
    color = colors.get(status, (0.95, 0.02, 0.45, 1.00))
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.lifetime = rospy.Duration(0)
    center = np.asarray(obj["center_xyz_m"], dtype=np.float64)
    size = np.asarray(obj["size_xyz_m"], dtype=np.float64)
    # RViz depth-tests marker lines against the occupied cloud. Expand only the
    # display wireframe so all edges remain visible; planning geometry is unchanged.
    display_margin = 0.035 if is_context else 0.075
    size = size + 2.0 * display_margin
    w, x, y, z = [float(value) for value in obj["orientation_wxyz"]]
    rotation = np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    corners = []
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            for sz in (-0.5, 0.5):
                corners.append(center + rotation @ (size * np.asarray([sx, sy, sz])))
    edges = []
    for index, corner in enumerate(corners):
        for bit in (1, 2, 4):
            other = index ^ bit
            if index < other:
                edges.append((corner, corners[other]))
    for start, end in edges:
        marker.points.extend([Point(*start.tolist()), Point(*end.tolist())])
    return marker


def open_vocab_box_marker(obj, marker_id: int, stamp):
    converted = {
        "center_xyz_m": obj["center"],
        "size_xyz_m": obj["size"],
        "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    marker = box_marker(converted, marker_id, stamp, "context")
    marker.ns = "stage2_open_vocab_boxes"
    marker.scale.x = 0.025
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.02, 0.30, 0.72, 0.72
    return marker


def open_vocab_label_marker(obj, marker_id: int, stamp):
    """Readable category/confidence label for an online OWLv2 box."""
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = stamp
    marker.ns = "stage2_open_vocab_labels"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    center = np.asarray(obj["center"], dtype=np.float64)
    size = np.asarray(obj["size"], dtype=np.float64)
    marker.pose.position.x = float(center[0])
    marker.pose.position.y = float(center[1])
    marker.pose.position.z = float(center[2] + size[2] * 0.5 + 0.24)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.18
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.00, 0.16, 0.72, 1.00
    label = str(obj.get("label") or "object").upper()
    score = obj.get("score")
    marker.text = label if score is None else f"{label}  {float(score):.2f}"
    marker.lifetime = rospy.Duration(0)
    return marker


def target_label_marker(obj, marker_id: int, stamp, label: str, status: str):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = stamp
    marker.ns = "stage2_task_labels"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    center = np.asarray(obj["center_xyz_m"], dtype=np.float64)
    size = np.asarray(obj["size_xyz_m"], dtype=np.float64)
    marker.pose.position.x = float(center[0])
    marker.pose.position.y = float(center[1])
    marker.pose.position.z = float(center[2] + size[2] * 0.5 + 0.25)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.18
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.72, 0.00, 0.32, 1.00
    symbols = {"pending": "WAIT", "current": "SEARCHING", "found": "✓ FOUND", "not_found": "✗ NOT FOUND", "skipped": "SKIPPED"}
    marker.text = f"{label.upper()}  [{symbols.get(status, status.upper())}]"
    colors = {
        "pending": (0.35, 0.38, 0.42), "current": (0.02, 0.35, 0.90),
        "found": (0.00, 0.58, 0.12), "not_found": (0.82, 0.04, 0.04),
        "skipped": (0.38, 0.38, 0.38),
    }
    marker.color.r, marker.color.g, marker.color.b = colors.get(status, colors["pending"])
    return marker


def context_label_marker(obj, marker_id: int, stamp):
    """Persistent category + stable object id for every pre-map semantic box."""
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = stamp
    marker.ns = "stage2_context_labels"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    center = np.asarray(obj["center_xyz_m"], dtype=np.float64)
    size = np.asarray(obj["size_xyz_m"], dtype=np.float64)
    marker.pose.position.x = float(center[0])
    marker.pose.position.y = float(center[1])
    marker.pose.position.z = float(center[2] + size[2] * 0.5 + 0.22)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.16
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.03, 0.18, 0.30, 0.96
    marker.text = f"{obj.get('label', 'object')}  [{obj.get('id', marker_id)}]"
    marker.lifetime = rospy.Duration(0)
    return marker


def task_box_markers(
    task_order, task_objects, task_candidate_objects, task_references,
    objects, statuses, current_task, stamp,
):
    result = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    visible_tasks = {task_id for task_id in task_order if statuses.get(task_id) == "found"}
    if current_task:
        visible_tasks.add(current_task)
    anchor_ids = set()
    for task_id in visible_tasks:
        anchor_ids.update(task_references.get(task_id, []))
    for object_id in sorted(anchor_ids):
        if object_id not in objects:
            continue
        marker_id, obj = objects[object_id]
        result.markers.append(box_marker(obj, marker_id, stamp, "context"))
        result.markers.append(context_label_marker(obj, marker_id, stamp))
    # Keep every physical location hypothesis visible for the lifetime of its
    # task. Previously ``task_objects`` held only the current instance, so an
    # already checked bed disappeared as soon as the planner selected another
    # bed. Aggregate first in case two tasks happen to share one anchor.
    visible_object_states = {}
    state_priority = {
        "skipped": 0, "pending": 1, "not_found": 2, "current": 3, "found": 4,
    }
    for task_id in task_order:
        object_ids = list(task_candidate_objects.get(task_id, []))
        selected_object_id = task_objects.get(task_id)
        if selected_object_id and selected_object_id not in object_ids:
            object_ids.append(selected_object_id)
        task_status = statuses.get(task_id, "pending")
        for object_id in object_ids:
            if object_id not in objects:
                continue
            if task_status == "not_found":
                object_status = "not_found"
            elif task_status == "found":
                object_status = "found" if object_id == selected_object_id else "pending"
            elif task_status == "skipped":
                object_status = "skipped"
            elif task_id == current_task and object_id == selected_object_id:
                object_status = "current"
            else:
                object_status = "pending"
            previous = visible_object_states.get(object_id)
            if (
                previous is None
                or state_priority[object_status] > state_priority[previous[0]]
            ):
                visible_object_states[object_id] = (object_status, task_id)

    for object_id, (status, task_id) in sorted(
        visible_object_states.items(), key=lambda item: objects[item[0]][0]
    ):
        marker_id, obj = objects[object_id]
        label = obj.get("label", task_id)
        result.markers.append(box_marker(obj, marker_id, stamp, status))
        result.markers.append(target_label_marker(obj, marker_id, stamp, label, status))
    return result


def reached_goal_markers(executed_views, task_order, stamp):
    result = MarkerArray()
    for marker_id, reached in enumerate(executed_views):
        task_id = reached["task_id"]
        x, y, z, yaw = [float(value) for value in reached["pose"]]
        outcome = reached["outcome"]
        color = {
            "retry": (0.18, 0.48, 0.78, 0.28),
            "found": (0.00, 0.55, 0.12, 0.78),
            "not_found": (0.78, 0.16, 0.10, 0.45),
            "done": (0.00, 0.55, 0.12, 0.65),
        }.get(outcome, (0.35, 0.40, 0.48, 0.25))

        frustum = Marker()
        frustum.header.frame_id = "world"
        frustum.header.stamp = stamp
        frustum.ns = "stage2_reached_goal_frustums"
        frustum.id = marker_id
        frustum.type = Marker.LINE_LIST
        frustum.action = Marker.ADD
        frustum.pose.orientation.w = 1.0
        frustum.scale.x = 0.025 if outcome == "retry" else 0.040
        frustum.color.r, frustum.color.g, frustum.color.b, frustum.color.a = color
        origin = np.asarray([x, y, z], dtype=np.float64)
        forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
        right = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0])
        up = np.asarray([0.0, 0.0, 1.0])
        depth, half_width, half_height = 0.72, 0.40, 0.30
        center = origin + depth * forward
        corners = [
            center - half_width * right - half_height * up,
            center + half_width * right - half_height * up,
            center + half_width * right + half_height * up,
            center - half_width * right + half_height * up,
        ]
        for corner in corners:
            frustum.points.extend([Point(*origin.tolist()), Point(*corner.tolist())])
        for index in range(4):
            frustum.points.extend([Point(*corners[index].tolist()), Point(*corners[(index + 1) % 4].tolist())])
        result.markers.append(frustum)

    return result


def room_markers(scene_graph, stamp):
    result = MarkerArray()
    room_colors = {
        "bedroom": (0.36, 0.24, 0.72),
        "living_room": (0.02, 0.52, 0.62),
        "kitchen": (0.88, 0.48, 0.04),
        "bathroom": (0.12, 0.42, 0.82),
        "corridor": (0.38, 0.40, 0.44),
        "unknown": (0.42, 0.44, 0.48),
    }
    for index, room in enumerate(scene_graph.get("rooms", [])):
        room_type = str(room.get("semantic_type") or "unknown")
        color = room_colors.get(room_type, room_colors["unknown"])
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "stage2_room_boundaries"
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        is_fragment = room.get("space_role") == "room_fragment"
        marker.scale.x = 0.018 if is_fragment else 0.035
        alpha = 0.24 if is_fragment else 0.52
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color[0], color[1], color[2], alpha
        polygon = room.get("polygon_xy_m", [])
        for xy in polygon + polygon[:1]:
            marker.points.append(Point(float(xy[0]), float(xy[1]), 0.12))
        result.markers.append(marker)

        centroid = room.get("centroid_xy_m")
        if not centroid and polygon:
            centroid = np.mean(np.asarray(polygon, dtype=np.float64), axis=0).tolist()
        if centroid and not is_fragment:
            label = Marker()
            label.header.frame_id = "world"
            label.header.stamp = stamp
            label.ns = "stage2_room_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(centroid[0])
            label.pose.position.y = float(centroid[1])
            label.pose.position.z = 1.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.r, label.color.g, label.color.b, label.color.a = color[0] * 0.65, color[1] * 0.65, color[2] * 0.65, 0.92
            label.text = f"R{room['id']}  {room_type.replace('_', ' ').upper()}"
            result.markers.append(label)
    return result


def candidate_markers(candidates, stamp, current_task=None, selected_id=None, limit=5):
    result = MarkerArray()
    marker_id = 0
    task_items = [(current_task, candidates.get(current_task, []))] if current_task else candidates.items()
    for task_id, values in task_items:
        ranked = sorted(values, key=lambda item: float(item.get("terminal_cost", 1e9)))[:limit]
        for rank, candidate in enumerate(ranked, start=1):
            pose = candidate["pose"]
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = stamp
            marker.ns = "stage2_candidate_viewpoints"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose["x"], pose["y"], pose["z"]
            marker.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
            marker.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.28, 0.055, 0.09
            selected = candidate.get("id") == selected_id
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = ((0.02, 0.78, 0.18, 0.98) if selected else (0.10, 0.48, 0.90, 0.58))
            result.markers.append(marker)
            label = Marker()
            label.header.frame_id, label.header.stamp = "world", stamp
            label.ns, label.id = "stage2_candidate_labels", marker_id
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = pose["x"], pose["y"], pose["z"] + 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r, label.color.g, label.color.b, label.color.a = marker.color.r, marker.color.g, marker.color.b, 0.96
            label.text = f"V{rank} {'SELECTED' if selected else 'feasible'}\n{candidate.get('object_label', task_id)}  L{candidate.get('floor_id', '?')}\ncost={float(candidate.get('terminal_cost', 0.0)):.2f}"
            result.markers.append(label)
            marker_id += 1
    return result


def recovery_search_markers(events, objects, stamp):
    """Show failed expected anchors and VLM-ranked search hypotheses progressively."""
    result = MarkerArray()
    marker_id = 0
    for event in events:
        failed_id = event.get("failed_expected_object_id")
        if failed_id in objects:
            _, obj = objects[failed_id]
            center = obj["center_xyz_m"]
            failed = Marker()
            failed.header.frame_id, failed.header.stamp = "world", stamp
            failed.ns, failed.id, failed.type, failed.action = "stage2_failed_expected", marker_id, Marker.LINE_LIST, Marker.ADD
            failed.pose.orientation.w = 1.0
            failed.scale.x = 0.075
            failed.color.r, failed.color.g, failed.color.b, failed.color.a = 0.78, 0.08, 0.06, 0.95
            radius = 0.32
            failed.points.extend([
                Point(center[0] - radius, center[1] - radius, center[2] + 0.5),
                Point(center[0] + radius, center[1] + radius, center[2] + 0.5),
                Point(center[0] - radius, center[1] + radius, center[2] + 0.5),
                Point(center[0] + radius, center[1] - radius, center[2] + 0.5),
            ])
            result.markers.append(failed)
            marker_id += 1
        for rank, hypothesis in enumerate(event.get("plan", {}).get("hypotheses", []), start=1):
            object_id = hypothesis.get("anchor_object_id")
            if object_id not in objects:
                continue
            _, obj = objects[object_id]
            center = obj["center_xyz_m"]
            ring = Marker()
            ring.header.frame_id, ring.header.stamp = "world", stamp
            ring.ns, ring.id, ring.type, ring.action = "stage2_recovery_hypotheses", marker_id, Marker.CYLINDER, Marker.ADD
            ring.pose.position.x, ring.pose.position.y, ring.pose.position.z = center[0], center[1], 0.18
            ring.pose.orientation.w = 1.0
            ring.scale.x = ring.scale.y = 0.42 - 0.05 * min(rank - 1, 2)
            ring.scale.z = 0.045
            colors = [(0.96, 0.55, 0.03), (0.95, 0.76, 0.05), (0.72, 0.62, 0.12)]
            ring.color.r, ring.color.g, ring.color.b = colors[rank - 1]
            ring.color.a = 0.90
            result.markers.append(ring)
            label = target_label_marker(obj, marker_id, stamp, f"SEARCH {rank}: {obj['label']}", "current")
            label.ns = "stage2_recovery_labels"
            label.scale.z = 0.13
            label.color.r, label.color.g, label.color.b, label.color.a = ring.color.r, ring.color.g, ring.color.b, 0.95
            result.markers.append(label)
            marker_id += 1
    return result


def plan_markers(plan, stamp):
    tour = Marker()
    tour.header.frame_id = "world"
    tour.header.stamp = stamp
    tour.ns = "stage2_global_tour"
    tour.id = 0
    tour.type = Marker.LINE_STRIP
    tour.action = Marker.ADD
    tour.pose.orientation.w = 1.0
    tour.scale.x = 0.045
    tour.color.r, tour.color.g, tour.color.b, tour.color.a = 0.50, 0.08, 0.72, 0.92
    if plan.get("segments"):
        first = plan["segments"][0].get("from_xyz_m", plan["segments"][0].get("from_xy_m"))
        tour.points.append(Point(float(first[0]), float(first[1]), float(first[2]) if len(first) > 2 else 1.05))
    for visit in plan.get("visits", []):
        pose = visit["pose"]
        tour.points.append(Point(float(pose["x"]), float(pose["y"]), float(pose["z"])))

    local = Marker()
    local.header.frame_id = "world"
    local.header.stamp = stamp
    local.ns = "stage2_local_path"
    local.id = 0
    local.type = Marker.LINE_STRIP
    local.action = Marker.ADD
    local.pose.orientation.w = 1.0
    local.scale.x = 0.060
    local.color.r, local.color.g, local.color.b, local.color.a = 0.96, 0.42, 0.02, 0.96
    if plan.get("segments"):
        destination = plan["visits"][0]["pose"]
        points = plan["segments"][0].get("points_xyz_m", plan["segments"][0].get("points_xy_m", []))
        for index, point in enumerate(points):
            ratio = index / max(1, len(points) - 1)
            z = float(point[2]) if len(point) > 2 else 1.0 + ratio * (float(destination["z"]) - 1.0)
            local.points.append(Point(float(point[0]), float(point[1]), z))

    selected = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    selected.markers.append(clear)
    for index, visit in enumerate(plan.get("visits", [])):
        pose = visit["pose"]
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "stage2_selected_goals"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose["x"], pose["y"], pose["z"]
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.18
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.98, 0.76, 0.02, 0.92
        selected.markers.append(marker)
    return tour, local, selected


def task_text_marker(text: str, pose, stamp):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = stamp
    marker.ns = "stage2_task_status"
    marker.id = 0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose[0], pose[1], pose[2] + 0.65
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.17
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.08, 0.08, 0.10, 0.96
    marker.text = text
    return marker


def task_summary_text(task_order, task_labels, statuses, current_task, instruction=""):
    symbols = {
        "pending": "[ ]",
        "found": "[✓ FOUND]",
        "not_found": "[✗ NOT FOUND]",
        "skipped": "[SKIPPED]",
    }
    lines = []
    for task_id in task_order:
        status = statuses.get(task_id, "pending")
        prefix = ">" if task_id == current_task and status == "pending" else " "
        state = "[▶ SEARCHING]" if prefix == ">" else symbols.get(status, f"[{status.upper()}]")
        lines.append(f"{prefix} {task_labels[task_id].upper():<10} {state}")
    instruction_line = " ".join(str(instruction).split())
    if len(instruction_line) > 88:
        instruction_line = instruction_line[:85] + "..."
    heading = f"QWEN INPUT: {instruction_line}\nMISSION STATUS" if instruction_line else "MISSION STATUS"
    return heading + "\n" + "\n".join(lines)


def image_message(path: Path, stamp):
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(path)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    message = Image()
    message.header.frame_id = "camera_optical"
    message.header.stamp = stamp
    message.height, message.width = rgb.shape[:2]
    message.encoding = "rgb8"
    message.is_bigendian = False
    message.step = message.width * 3
    message.data = rgb.tobytes()
    return message


def array_image_message(rgb, stamp):
    message = Image()
    message.header.frame_id = "camera_optical"
    message.header.stamp = stamp
    message.height, message.width = rgb.shape[:2]
    message.encoding = "rgb8"
    message.is_bigendian = False
    message.step = message.width * 3
    message.data = np.ascontiguousarray(rgb).tobytes()
    return message


def annotated_evidence(path, observation):
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(path)
    verification = observation.get("verification", {})
    detections = verification.get("owlv2", {}).get("detections", [])
    for detection in detections:
        x1, y1, x2, y2 = [int(round(value)) for value in detection["bbox_xyxy"]]
        cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 40), 5)
        text = f"FOUND {detection.get('label', '')}  {float(detection.get('score', 0)):.2f}"
        cv2.rectangle(bgr, (x1, max(0, y1 - 32)), (min(bgr.shape[1] - 1, x1 + 280), y1), (0, 160, 20), -1)
        cv2.putText(bgr, text, (x1 + 5, max(22, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(bgr, "TASK EVIDENCE  ✓", (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 40), 2, cv2.LINE_AA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def task_panel_image(task_order, task_labels, statuses, current_task, instruction):
    height, width = 92 + 44 * len(task_order), 720
    canvas = np.full((height, width, 3), (247, 249, 252), dtype=np.uint8)
    cv2.putText(canvas, "QWEN TASK EXECUTION", (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (32, 42, 58), 2, cv2.LINE_AA)
    short = " ".join(str(instruction).split())[:72]
    cv2.putText(canvas, short, (18, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 86, 96), 1, cv2.LINE_AA)
    for index, task_id in enumerate(task_order):
        status = statuses.get(task_id, "pending")
        current = task_id == current_task and status == "pending"
        y = 94 + index * 44
        if current:
            cv2.rectangle(canvas, (8, y - 28), (width - 8, y + 10), (226, 241, 255), -1)
        symbol = "DONE" if status == "found" else "SKIP" if status == "skipped" else "NOW" if current else "WAIT"
        color = (25, 150, 45) if status == "found" else (230, 110, 20) if current else (125, 130, 138)
        cv2.putText(canvas, symbol, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)
        cv2.putText(canvas, f"T{index + 1}  {task_labels[task_id]}", (102, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (32, 42, 58), 1, cv2.LINE_AA)
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-bag", type=Path, required=True)
    parser.add_argument("--execution", type=Path, required=True)
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--frames-dir", type=Path, required=True)
    parser.add_argument("--output-bag", type=Path, required=True)
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()

    execution = load_json(args.execution)
    task_graph = load_json(args.task_graph)
    candidates = load_json(args.candidates)["by_task"]
    scene_graph = load_json(args.scene_graph)
    objects = {obj["id"]: (index, obj) for index, obj in enumerate(obj for room in scene_graph["rooms"] for obj in room["objects"])}
    task_order = [task["id"] for task in task_graph["tasks"]]
    task_labels = {task["id"]: task.get("verification_label") or task["target"]["label"] for task in task_graph["tasks"]}
    task_objects = {
        task_id: values[0]["object_id"]
        for task_id, values in candidates.items()
        if values
    }
    task_candidate_objects = {
        task_id: list(dict.fromkeys(
            value["object_id"] for value in values if value.get("object_id") in objects
        ))
        for task_id, values in candidates.items()
    }
    task_references = {
        task_id: list(dict.fromkeys(object_id for value in values for object_id in value.get("reference_object_ids", [])))
        for task_id, values in candidates.items()
    }
    task_states = {task_id: "pending" for task_id in task_order}
    executed_views = []
    current_task = execution["replans"][0]["plan"]["visits"][0]["task_id"]
    room_points = [xy for room in scene_graph["rooms"] for xy in room.get("polygon_xy_m", [])]
    room_x = [float(xy[0]) for xy in room_points]
    room_y = [float(xy[1]) for xy in room_points]
    status_anchor = [
        (min(room_x) + max(room_x)) * 0.5,
        (min(room_y) + max(room_y)) * 0.5,
        3.35,
    ]
    occupied = final_topic_message(args.source_bag, "/voxel_mapping/occupancy_grid_occupied")
    saved_frames = {}
    for path in args.frames_dir.glob("frame_*.jpg"):
        match = re.fullmatch(r"frame_(\d+)\.jpg", path.name)
        if match:
            saved_frames[int(match.group(1))] = path
    if not saved_frames:
        raise RuntimeError(f"no frame_*.jpg images found in {args.frames_dir}")
    saved_indices = sorted(saved_frames)

    def nearest_saved_frame(index):
        # Habitat output may intentionally retain only every Nth keyframe.
        return saved_frames[min(saved_indices, key=lambda value: abs(value - index))]
    start = rospy.Time.from_sec(1000.0)
    period = 1.0 / args.hz
    args.output_bag.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(args.output_bag), "w", compression="lz4") as bag:
        initial_stamp = start + rospy.Duration.from_sec(0.05)
        cloud = copy.deepcopy(occupied)
        stamp_message(cloud, initial_stamp)
        bag.write("/voxel_mapping/occupancy_grid_occupied", cloud, initial_stamp)
        boxes = task_box_markers(
            task_order, task_objects, task_candidate_objects, task_references,
            objects, task_states, current_task, initial_stamp,
        )
        bag.write("/stage2/semantic_boxes", boxes, initial_stamp)
        first_visit = execution["replans"][0]["plan"]["visits"][0]
        bag.write("/stage2/candidate_poses", candidate_markers(candidates, initial_stamp, current_task, first_visit["candidate_id"]), initial_stamp)
        bag.write("/stage2/reached_goals", MarkerArray(), initial_stamp)
        active_recovery_events = []
        bag.write("/stage2/recovery_search", MarkerArray(), initial_stamp)
        blank_evidence = np.full((480, 640, 3), 245, dtype=np.uint8)
        cv2.putText(blank_evidence, "Waiting for verified target...", (95, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (90, 90, 90), 2, cv2.LINE_AA)
        blank_evidence = cv2.cvtColor(blank_evidence, cv2.COLOR_BGR2RGB)
        active_evidence = None

        event_by_frame = {int(item["frame_index"]): item for item in execution["observations"]}
        open_vocab_by_frame = {
            int(item["frame_index"]): item
            for item in execution.get("open_vocab_observations", [])
        }
        detected_objects = {}
        online_object_ids = sorted({
            detected.get("associated_object_id") or detected.get("id")
            for item in execution.get("open_vocab_observations", [])
            for detected in item.get("fused_objects_3d", item.get("detections_3d", []))
            if detected.get("associated_object_id") or detected.get("id")
        })
        online_marker_ids = {object_id: index for index, object_id in enumerate(online_object_ids)}
        replan_by_frame = {0: execution["replans"][0]}
        for index, observation in enumerate(execution["observations"][:-1], start=1):
            replan_by_frame[int(observation["frame_index"])] = execution["replans"][index]
        path = NavPath()
        path.header.frame_id = "world"
        for frame_index, pose in enumerate(execution["trajectory_xyz_yaw"]):
            stamp = start + rospy.Duration.from_sec(frame_index * period)
            x, y, z, yaw = [float(value) for value in pose]
            qx, qy, qz, qw = yaw_quaternion(yaw)
            odom = Odometry()
            odom.header.frame_id = "world"
            odom.child_frame_id = "uav"
            odom.header.stamp = stamp
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qx, qy, qz, qw
            bag.write("/stage2/uav_pose", odom, stamp)

            pose_stamped = PoseStamped()
            pose_stamped.header = odom.header
            pose_stamped.pose = odom.pose.pose
            path.header.stamp = stamp
            path.poses.append(pose_stamped)
            bag.write("/stage2/executed_path", copy.deepcopy(path), stamp)

            sensor_pose = TransformStamped()
            sensor_pose.header.frame_id = "world"
            sensor_pose.header.stamp = stamp
            sensor_pose.child_frame_id = "camera_optical"
            sensor_pose.transform.translation.x, sensor_pose.transform.translation.y, sensor_pose.transform.translation.z = x, y, z
            oqx, oqy, oqz, oqw = optical_quaternion(yaw)
            sensor_pose.transform.rotation.x, sensor_pose.transform.rotation.y, sensor_pose.transform.rotation.z, sensor_pose.transform.rotation.w = oqx, oqy, oqz, oqw
            bag.write("/uav_simulator/sensor_pose", sensor_pose, stamp)

            rgb_index = min(max(0, frame_index - 1), int(execution["frame_count"]) - 1)
            image_path = nearest_saved_frame(rgb_index)
            if frame_index in event_by_frame:
                event = event_by_frame[frame_index]
                terminal = args.frames_dir / f"terminal_{int(event['sequence']):02d}_{event['task_id']}.jpg"
                if terminal.exists():
                    image_path = terminal
            live_rgb = cv2.cvtColor(cv2.imread(str(image_path), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            if frame_index in event_by_frame and event_by_frame[frame_index].get("outcome") == "found":
                active_evidence = annotated_evidence(image_path, event_by_frame[frame_index])
                live_rgb = active_evidence
            bag.write("/stage2/rgb", array_image_message(live_rgb, stamp), stamp)
            # Keep the latest verified observation until the next target replaces it.
            evidence_rgb = active_evidence if active_evidence is not None else blank_evidence
            bag.write("/stage2/evidence", array_image_message(evidence_rgb, stamp), stamp)

            if frame_index in replan_by_frame:
                replan = replan_by_frame[frame_index]
                tour, local, selected = plan_markers(replan["plan"], stamp)
                bag.write("/stage2/global_tour", tour, stamp)
                bag.write("/stage2/local_path", local, stamp)
                bag.write("/stage2/selected_goals", selected, stamp)
                next_task = replan["plan"]["visits"][0]["task_id"]
                selected_id = replan["plan"]["visits"][0]["candidate_id"]
                current_task = next_task
                task_objects[next_task] = replan["plan"]["visits"][0]["object_id"]
                bag.write(
                    "/stage2/semantic_boxes",
                    task_box_markers(
                        task_order, task_objects, task_candidate_objects, task_references,
                        objects, task_states, current_task, stamp,
                    ),
                    stamp,
                )
                bag.write("/stage2/task_status", String(data=json.dumps({"next_task": next_task, "active_task_ids": replan["active_task_ids"]})), stamp)
                bag.write("/stage2/candidate_poses", candidate_markers(candidates, stamp, next_task, selected_id), stamp)

            if frame_index in open_vocab_by_frame:
                fused_values = open_vocab_by_frame[frame_index].get(
                    "fused_objects_3d",
                    open_vocab_by_frame[frame_index].get("detections_3d", []),
                )
                for detected in fused_values:
                    object_id = detected.get("associated_object_id") or detected.get("id")
                    previous = detected_objects.get(object_id)
                    if previous is None or float(detected["score"]) >= float(previous["score"]):
                        detected_objects[object_id] = detected
                markers = MarkerArray()
                for object_id in sorted(detected_objects):
                    detected = detected_objects[object_id]
                    marker_id = online_marker_ids[object_id]
                    markers.markers.append(open_vocab_box_marker(detected, marker_id, stamp))
                    markers.markers.append(open_vocab_label_marker(detected, marker_id, stamp))
                bag.write("/stage2/open_vocab_boxes", markers, stamp)

            if frame_index in event_by_frame:
                event = event_by_frame[frame_index]
                executed_views.append({
                    "task_id": event["task_id"],
                    "pose": event["pose"],
                    "outcome": event["outcome"],
                })
                terminal_outcomes = {"found", "not_found", "done", "skipped"}
                if event["outcome"] in terminal_outcomes:
                    task_states[event["task_id"]] = event["outcome"]
                elif event["outcome"] == "recovery_activated":
                    task_states[event["task_id"]] = "searching"
                for rule in task_graph.get("conditional_rules", []):
                    if rule["source_task_id"] == event["task_id"] and rule["if_outcome"] != event["outcome"]:
                        for skipped_id in rule.get("activate_task_ids", []):
                            task_states[skipped_id] = "skipped"
                next_plan_index = min(event["sequence"] + 1, len(execution["replans"]) - 1)
                next_visits = execution["replans"][next_plan_index].get("plan", {}).get("visits", [])
                current_task = next_visits[0]["task_id"] if next_visits else None
                update = task_box_markers(
                    task_order, task_objects, task_candidate_objects, task_references,
                    objects, task_states, current_task, stamp,
                )
                bag.write("/stage2/semantic_boxes", update, stamp)
                bag.write("/stage2/reached_goals", reached_goal_markers(executed_views, task_order, stamp), stamp)
                bag.write("/stage2/task_status", String(data=json.dumps({"task_id": event["task_id"], "outcome": event["outcome"]})), stamp)
                for recovery_event in execution.get("semantic_recovery", {}).get("events", []):
                    if int(recovery_event.get("sequence", -1)) == int(event["sequence"]):
                        active_recovery_events.append(recovery_event)
                bag.write("/stage2/recovery_search", recovery_search_markers(active_recovery_events, objects, stamp), stamp)
            summary = task_summary_text(task_order, task_labels, task_states, current_task, task_graph.get("instruction", ""))
            bag.write("/stage2/task_text", task_text_marker(summary, status_anchor, stamp), stamp)
            panel = task_panel_image(task_order, task_labels, task_states, current_task, task_graph.get("instruction", ""))
            bag.write("/stage2/task_panel", array_image_message(panel, stamp), stamp)
            mission_state = "complete" if all(value in {"found", "not_found", "skipped", "done"} for value in task_states.values()) else "running"
            bag.write("/stage2/mission_state", String(data=mission_state), stamp)
    topic_counts = {
        "/voxel_mapping/occupancy_grid_occupied": 1,
        "/stage2/semantic_boxes": 1 + len(execution["observations"]) + len(execution["replans"]),
        "/stage2/open_vocab_boxes": len(execution.get("open_vocab_observations", [])),
        "/stage2/candidate_poses": 1 + len(execution["replans"]),
        "/stage2/reached_goals": 1 + len(execution["observations"]),
        "/stage2/recovery_search": 1 + len(execution["observations"]),
        "/stage2/executed_path": len(execution["trajectory_xyz_yaw"]),
        "/stage2/rgb": len(execution["trajectory_xyz_yaw"]),
        "/stage2/evidence": len(execution["trajectory_xyz_yaw"]),
        "/stage2/task_panel": len(execution["trajectory_xyz_yaw"]),
        "/stage2/mission_state": len(execution["trajectory_xyz_yaw"]),
        "/stage2/uav_pose": len(execution["trajectory_xyz_yaw"]),
        "/uav_simulator/sensor_pose": len(execution["trajectory_xyz_yaw"]),
        "/stage2/task_text": len(execution["trajectory_xyz_yaw"]),
        "/stage2/global_tour": len(execution["replans"]),
        "/stage2/local_path": len(execution["replans"]),
        "/stage2/selected_goals": len(execution["replans"]),
        "/stage2/task_status": 2 * len(execution["observations"]),
    }
    manifest = {
        "format": "pre_map_vln.stage2_bag.v1",
        "bag": str(args.output_bag),
        "duration_s": (len(execution["trajectory_xyz_yaw"]) - 1) / args.hz,
        "hz": args.hz,
        "message_count": sum(topic_counts.values()),
        "topic_counts": topic_counts,
        "size_bytes": args.output_bag.stat().st_size,
    }
    args.output_bag.with_suffix(".manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"bag={args.output_bag} duration={len(execution['trajectory_xyz_yaw']) / args.hz:.2f}s frames={len(execution['trajectory_xyz_yaw'])}")


if __name__ == "__main__":
    main()
