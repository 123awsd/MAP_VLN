#!/usr/bin/env python3
"""Build a self-contained ROS bag for the stage-two Habitat mission replay."""

from __future__ import annotations

import argparse
import copy
import json
import math
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
    color = (0.08, 0.32, 0.48, 0.72) if is_context else (0.95, 0.02, 0.45, 1.00)
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.lifetime = rospy.Duration(0)
    center = np.asarray(obj["center_xyz_m"], dtype=np.float64)
    size = np.asarray(obj["size_xyz_m"], dtype=np.float64)
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
    marker.pose.position.z = float(center[2] + size[2] * 0.5 + 0.18)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.15
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.72, 0.00, 0.32, 1.00
    symbols = {"pending": "WAIT", "current": "TARGET", "found": "FOUND", "not_found": "NOT FOUND", "skipped": "SKIPPED"}
    marker.text = f"{label.upper()}  [{symbols.get(status, status.upper())}]"
    return marker


def task_box_markers(task_order, task_objects, objects, statuses, current_task, stamp):
    result = MarkerArray()
    for object_id, (marker_id, obj) in sorted(objects.items(), key=lambda item: item[1][0]):
        result.markers.append(box_marker(obj, marker_id, stamp, "context"))
    for marker_id, task_id in enumerate(task_order):
        object_id = task_objects.get(task_id)
        if object_id not in objects:
            continue
        _, obj = objects[object_id]
        status = statuses.get(task_id, "pending")
        if task_id == current_task and status == "pending":
            status = "current"
        label = obj.get("label", task_id)
        result.markers.append(box_marker(obj, marker_id, stamp, "task"))
        result.markers.append(target_label_marker(obj, marker_id, stamp, label, status))
    return result


def reached_goal_markers(reached_goals, task_order, stamp):
    result = MarkerArray()
    task_indices = {task_id: index for index, task_id in enumerate(task_order)}
    for task_id, reached in reached_goals.items():
        x, y, z, _ = [float(value) for value in reached["pose"]]
        outcome = reached["outcome"]
        color = (0.00, 0.55, 0.12, 1.00) if outcome == "found" else (0.78, 0.04, 0.04, 1.00)
        marker_id = task_indices[task_id]

        star = Marker()
        star.header.frame_id = "world"
        star.header.stamp = stamp
        star.ns = "stage2_reached_goal_stars"
        star.id = marker_id
        star.type = Marker.LINE_STRIP
        star.action = Marker.ADD
        star.pose.orientation.w = 1.0
        star.scale.x = 0.070
        star.color.r, star.color.g, star.color.b, star.color.a = color
        for point_index in range(11):
            vertex = point_index % 10
            radius = 0.28 if vertex % 2 == 0 else 0.12
            angle = math.pi / 2.0 + vertex * math.pi / 5.0
            star.points.append(Point(x + radius * math.cos(angle), y + radius * math.sin(angle), z + 0.08))
        result.markers.append(star)

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


def candidate_markers(candidates, stamp):
    result = MarkerArray()
    marker_id = 0
    for task_id, values in candidates.items():
        for candidate in values:
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
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.12, 0.48, 0.82, 0.38
            result.markers.append(marker)
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
    tour.scale.x = 0.075
    tour.color.r, tour.color.g, tour.color.b, tour.color.a = 0.50, 0.08, 0.72, 0.92
    if plan.get("segments"):
        first = plan["segments"][0]["from_xy_m"]
        tour.points.append(Point(float(first[0]), float(first[1]), 1.05))
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
        points = plan["segments"][0]["points_xy_m"]
        for index, xy in enumerate(points):
            ratio = index / max(1, len(points) - 1)
            local.points.append(Point(float(xy[0]), float(xy[1]), 1.0 + ratio * (float(destination["z"]) - 1.0)))

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


def task_summary_text(task_order, task_labels, statuses, current_task):
    symbols = {
        "pending": "[ ]",
        "found": "[FOUND]",
        "not_found": "[NOT FOUND]",
        "skipped": "[SKIPPED]",
    }
    lines = []
    for task_id in task_order:
        status = statuses.get(task_id, "pending")
        prefix = ">" if task_id == current_task and status == "pending" else " "
        state = "[TARGET]" if prefix == ">" else symbols.get(status, f"[{status.upper()}]")
        lines.append(f"{prefix} {task_labels[task_id].upper():<10} {state}")
    return "MISSION STATUS\n" + "\n".join(lines)


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
    for replan in execution["replans"]:
        for visit in replan.get("plan", {}).get("visits", []):
            task_objects[visit["task_id"]] = visit["object_id"]
    task_states = {task_id: "pending" for task_id in task_order}
    reached_goals = {}
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
    start = rospy.Time.from_sec(1000.0)
    period = 1.0 / args.hz
    args.output_bag.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(args.output_bag), "w", compression="lz4") as bag:
        initial_stamp = start + rospy.Duration.from_sec(0.05)
        cloud = copy.deepcopy(occupied)
        stamp_message(cloud, initial_stamp)
        bag.write("/voxel_mapping/occupancy_grid_occupied", cloud, initial_stamp)
        boxes = task_box_markers(task_order, task_objects, objects, task_states, current_task, initial_stamp)
        bag.write("/stage2/semantic_boxes", boxes, initial_stamp)
        bag.write("/stage2/rooms", room_markers(scene_graph, initial_stamp), initial_stamp)
        bag.write("/stage2/candidate_poses", candidate_markers(candidates, initial_stamp), initial_stamp)
        bag.write("/stage2/reached_goals", MarkerArray(), initial_stamp)

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
            bag.write("/stage2/rgb", image_message(args.frames_dir / f"frame_{rgb_index:06d}.jpg", stamp), stamp)

            if frame_index in replan_by_frame:
                replan = replan_by_frame[frame_index]
                tour, local, selected = plan_markers(replan["plan"], stamp)
                bag.write("/stage2/global_tour", tour, stamp)
                bag.write("/stage2/local_path", local, stamp)
                bag.write("/stage2/selected_goals", selected, stamp)
                next_task = replan["plan"]["visits"][0]["task_id"]
                current_task = next_task
                bag.write("/stage2/task_status", String(data=json.dumps({"next_task": next_task, "active_task_ids": replan["active_task_ids"]})), stamp)

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
                    markers.markers.append(open_vocab_box_marker(
                        detected_objects[object_id], online_marker_ids[object_id], stamp
                    ))
                bag.write("/stage2/open_vocab_boxes", markers, stamp)

            if frame_index in event_by_frame:
                event = event_by_frame[frame_index]
                task_states[event["task_id"]] = event["outcome"]
                reached_goals[event["task_id"]] = {"pose": event["pose"], "outcome": event["outcome"]}
                for rule in task_graph.get("conditional_rules", []):
                    if rule["source_task_id"] == event["task_id"] and rule["if_outcome"] != event["outcome"]:
                        for skipped_id in rule.get("activate_task_ids", []):
                            task_states[skipped_id] = "skipped"
                next_plan_index = min(event["sequence"] + 1, len(execution["replans"]) - 1)
                next_visits = execution["replans"][next_plan_index].get("plan", {}).get("visits", [])
                current_task = next_visits[0]["task_id"] if next_visits else None
                update = task_box_markers(task_order, task_objects, objects, task_states, current_task, stamp)
                bag.write("/stage2/semantic_boxes", update, stamp)
                bag.write("/stage2/reached_goals", reached_goal_markers(reached_goals, task_order, stamp), stamp)
                bag.write("/stage2/task_status", String(data=json.dumps({"task_id": event["task_id"], "outcome": event["outcome"]})), stamp)
            summary = task_summary_text(task_order, task_labels, task_states, current_task)
            bag.write("/stage2/task_text", task_text_marker(summary, status_anchor, stamp), stamp)
    topic_counts = {
        "/voxel_mapping/occupancy_grid_occupied": 1,
        "/stage2/semantic_boxes": 1 + len(execution["observations"]),
        "/stage2/open_vocab_boxes": len(execution.get("open_vocab_observations", [])),
        "/stage2/rooms": 1,
        "/stage2/candidate_poses": 1,
        "/stage2/reached_goals": 1 + len(execution["observations"]),
        "/stage2/executed_path": len(execution["trajectory_xyz_yaw"]),
        "/stage2/rgb": len(execution["trajectory_xyz_yaw"]),
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
