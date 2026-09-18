#!/usr/bin/env python3
"""Publish Drone_room offline outputs for RViz; no sensors or flight nodes."""
import csv
import colorsys
import json
import math
import struct
import sys
import zlib
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

LABEL_ZH = {
    "bed": "床", "chair": "椅子", "sofa": "沙发", "table": "桌子",
    "coffee table": "茶几", "desk": "书桌", "cabinet": "柜子",
    "television": "电视", "television stand": "电视柜", "refrigerator": "冰箱",
    "microwave": "微波炉", "pot": "锅", "cup": "水杯", "bottle": "水瓶",
    "toilet": "马桶", "sink": "洗手台", "toilet paper": "卫生纸",
    "laptop": "笔记本电脑", "mobile phone": "手机", "speaker": "音箱",
    "fire extinguisher": "灭火器", "dumbbell": "哑铃", "pool table": "台球桌",
    "door": "门", "window": "窗户", "bedside table": "床头柜",
}

def display_label(label):
    original = str(label).strip()
    return original


def read_binary_pcd(path):
    raw = path.read_bytes()
    marker = raw.find(b"DATA binary\n")
    if marker < 0:
        raise RuntimeError("Only binary PCD is supported")
    header = raw[:marker].decode("ascii", errors="strict")
    lines = header.splitlines()
    fields = next(line.split()[1:] for line in lines if line.startswith("FIELDS "))
    sizes = [int(value) for value in next(line.split()[1:] for line in lines if line.startswith("SIZE "))]
    counts = [int(value) for value in next(line.split()[1:] for line in lines if line.startswith("COUNT "))]
    points = int(next(line.split()[1] for line in lines if line.startswith("POINTS ")))
    payload = raw[marker + len(b"DATA binary\n"):]
    offsets = {}
    offset = 0
    for name, size, count in zip(fields, sizes, counts):
        offsets[name] = offset
        offset += size * count
    stride = offset
    if not all(name in offsets for name in ("x", "y", "z")):
        raise RuntimeError("PCD lacks x/y/z fields")
    intensity_offset = offsets.get("intensity")
    return [(
        struct.unpack_from("<f", payload, i * stride + offsets["x"])[0],
        struct.unpack_from("<f", payload, i * stride + offsets["y"])[0],
        struct.unpack_from("<f", payload, i * stride + offsets["z"])[0],
        struct.unpack_from("<f", payload, i * stride + intensity_offset)[0]
        if intensity_offset is not None else 0.0,
    ) for i in range(points)]


def load_path(csv_path, topic):
    msg = RosPath()
    msg.header.frame_id = "map"
    with csv_path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.from_sec(float(row["stamp_sec"]))
            pose.pose.position.x = float(row["x_m"])
            pose.pose.position.y = float(row["y_m"])
            pose.pose.position.z = float(row["z_m"])
            pose.pose.orientation.x = float(row["qx"])
            pose.pose.orientation.y = float(row["qy"])
            pose.pose.orientation.z = float(row["qz"])
            pose.pose.orientation.w = float(row["qw"])
            msg.poses.append(pose)
    pub = rospy.Publisher(topic, RosPath, queue_size=1, latch=True)
    pub.publish(msg)
    return pub


def load_box_geometry(clusters_path, raw_targets_path, boxer_3d_path, min_observations):
    """Attach one representative Boxer OBB geometry to each semantic cluster."""
    clusters = [item for item in json.loads(clusters_path.read_text())
                if int(item["observations"]) >= min_observations]
    raw = json.loads(raw_targets_path.read_text())
    boxer_by_time = {}
    with boxer_3d_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            boxer_by_time.setdefault((int(row["time_ns"]), row["name"].strip().lower()), []).append(row)
    result = []
    for cluster in clusters:
        if "size_x_m" in cluster and "orientation_wxyz" in cluster:
            qw, qx, qy, qz = cluster["orientation_wxyz"]
            result.append((cluster, {
                "qw_world_object": qw, "qx_world_object": qx,
                "qy_world_object": qy, "qz_world_object": qz,
                "scale_x": cluster["size_x_m"], "scale_y": cluster["size_y_m"],
                "scale_z": cluster["size_z_m"],
            }))
            continue
        center = (float(cluster["world_x_m"]), float(cluster["world_y_m"]), float(cluster["world_z_m"]))
        nearby = sorted((item for item in raw if item["label"] == cluster["label"]),
                        key=lambda item: sum((float(item[k]) - center[i]) ** 2
                            for i, k in enumerate(("world_x_m", "world_y_m", "world_z_m"))))
        representative = nearby[0] if nearby else None
        geometry = None
        if representative:
            candidates = boxer_by_time.get((int(representative["time_ns"]), cluster["label"]), [])
            if candidates:
                geometry = min(candidates, key=lambda row: sum(
                    (float(row[k]) - float(representative[rk])) ** 2
                    for k, rk in zip(("tx_world_object", "ty_world_object", "tz_world_object"),
                                     ("world_x_m", "world_y_m", "world_z_m"))))
        result.append((cluster, geometry))
    return result


def semantic_color(label):
    hue = (zlib.crc32(label.encode("utf-8")) % 360) / 360.0
    return colorsys.hsv_to_rgb(hue, 0.78, 1.0)


def add_observation_frustums(marker_array, mission, mission_path, frame):
    """Draw the selected candidate's nominal camera FoV at every visit."""
    candidates_path = mission_path.parent / "candidates.json"
    if not candidates_path.is_file():
        return 0
    document = json.loads(candidates_path.read_text())
    candidate_by_id = {
        candidate["id"]: candidate
        for values in document.get("by_task", {}).values()
        for candidate in values
    }
    count = 0
    for index, visit in enumerate(mission.get("visits", [])):
        candidate = candidate_by_id.get(visit.get("candidate_id"))
        if candidate is None:
            continue
        pose = visit["pose"]
        origin = [float(pose[key]) for key in ("x", "y", "z")]
        yaw = float(pose["yaw"])
        target = [float(value) for value in candidate.get("target_xyz_m", origin)]
        horizontal_fov = math.radians(float(candidate.get("horizontal_fov_deg", 90.0)))
        vertical_fov = math.radians(float(candidate.get("vertical_fov_deg", 70.0)))
        target_range = math.dist(origin, target)
        depth = min(3.0, max(0.5, target_range))
        half_width = depth * math.tan(horizontal_fov * 0.5)
        half_height = depth * math.tan(vertical_fov * 0.5)
        forward = [math.cos(yaw), math.sin(yaw), 0.0]
        right = [-math.sin(yaw), math.cos(yaw), 0.0]
        center = [origin[axis] + depth * forward[axis] for axis in range(3)]
        corners = [
            [center[0] + side * half_width * right[0],
             center[1] + side * half_width * right[1],
             center[2] + vertical * half_height]
            for side, vertical in ((-1.0, -1.0), (1.0, -1.0),
                                   (1.0, 1.0), (-1.0, 1.0))
        ]
        lines = [(origin, corner) for corner in corners]
        lines.extend((corners[i], corners[(i + 1) % 4]) for i in range(4))
        lines.append((origin, target))

        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = "planned_observation_frustums"
        marker.id = index
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.82
        marker.color.b = 0.10
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0)
        marker.points = [Point(x=a[0], y=a[1], z=a[2])
                         for line in lines for a in line]
        marker_array.markers.append(marker)
        count += 1
    return count


def add_clearance_markers(marker_array, mission, voxel_snapshot, frame):
    """Mark each segment's minimum-ESDF point and its nearest occupied voxel."""
    if voxel_snapshot is None:
        return 0
    metadata_path = voxel_snapshot / "metadata.json"
    arrays_path = voxel_snapshot / "voxel_map.npz"
    if not metadata_path.is_file() or not arrays_path.is_file():
        return 0
    metadata = json.loads(metadata_path.read_text())
    arrays = np.load(arrays_path)
    esdf = arrays["esdf_zyx_m"]
    raw_occupancy = arrays["raw_occupancy_zyx"]
    origin = np.asarray(metadata["origin_xyz_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    threshold = float(metadata.get("minimum_esdf_distance_m", 0.0))

    # np.argwhere is z/y/x. Convert occupied voxel indices to world x/y/z
    # centers once, then use the small local neighbourhood around each
    # bottleneck for an exact nearest-voxel marker.
    occupied_zyx = np.argwhere(raw_occupancy > 0)
    occupied_xyz = origin + (occupied_zyx[:, ::-1].astype(float) + 0.5) * resolution

    def clearance(point):
        index = np.floor((np.asarray(point, dtype=float) - origin) / resolution).astype(int)
        x, y, z = [int(value) for value in index]
        if z < 0 or y < 0 or x < 0 or z >= esdf.shape[0] or y >= esdf.shape[1] or x >= esdf.shape[2]:
            return 0.0
        return float(esdf[z, y, x])

    count = 0
    global_bottleneck = None
    for segment_index, segment in enumerate(mission.get("segments", [])):
        validated = segment.get("validated_trajectory") or {}
        points = validated.get("points_xyz_m") or segment.get("points_xyz_m", [])
        if not points:
            continue
        values = [(clearance(point), point) for point in points]
        minimum, point = min(values, key=lambda item: item[0])
        if global_bottleneck is None or minimum < global_bottleneck[0]:
            global_bottleneck = (minimum, segment_index, np.asarray(point, dtype=float))

        sphere = Marker()
        sphere.header.frame_id = frame
        sphere.ns = "trajectory_clearance"
        sphere.id = segment_index * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = map(float, point)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
        sphere.color.r = 1.0
        sphere.color.g = 0.05 if minimum < threshold else 0.75
        sphere.color.b = 0.05
        sphere.color.a = 1.0
        marker_array.markers.append(sphere)

        label = Marker()
        label.header.frame_id = frame
        label.ns = "trajectory_clearance_labels"
        label.id = segment_index * 2 + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(point[0])
        label.pose.position.y = float(point[1])
        label.pose.position.z = float(point[2]) + 0.18
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f"seg {segment_index + 1}: clearance={minimum:.2f}m"
        marker_array.markers.append(label)

        path_point = np.asarray(point, dtype=float)
        local_mask = np.all(np.abs(occupied_xyz - path_point) <= max(1.0, minimum + resolution), axis=1)
        local_occupied = occupied_xyz[local_mask]
        if len(local_occupied):
            distances = np.linalg.norm(local_occupied - path_point, axis=1)
            obstacle_point = local_occupied[int(np.argmin(distances))]

            obstacle = Marker()
            obstacle.header.frame_id = frame
            obstacle.ns = "trajectory_nearest_obstacles"
            obstacle.id = segment_index
            obstacle.type = Marker.CUBE
            obstacle.action = Marker.ADD
            obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z = map(float, obstacle_point)
            obstacle.pose.orientation.w = 1.0
            obstacle.scale.x = obstacle.scale.y = obstacle.scale.z = max(0.14, resolution)
            obstacle.color.r = 1.0
            obstacle.color.g = 0.0
            obstacle.color.b = 1.0
            obstacle.color.a = 1.0
            marker_array.markers.append(obstacle)

            connector = Marker()
            connector.header.frame_id = frame
            connector.ns = "trajectory_clearance_vectors"
            connector.id = segment_index
            connector.type = Marker.LINE_LIST
            connector.action = Marker.ADD
            connector.pose.orientation.w = 1.0
            connector.scale.x = 0.045
            connector.color.r = 0.0
            connector.color.g = 1.0
            connector.color.b = 1.0
            connector.color.a = 1.0
            connector.points = [Point(x=float(path_point[0]), y=float(path_point[1]), z=float(path_point[2])),
                                Point(x=float(obstacle_point[0]), y=float(obstacle_point[1]), z=float(obstacle_point[2]))]
            marker_array.markers.append(connector)
        count += 1

    if global_bottleneck is not None:
        minimum, segment_index, point = global_bottleneck

        highlight = Marker()
        highlight.header.frame_id = frame
        highlight.ns = "global_bottleneck_highlight"
        highlight.id = 0
        highlight.type = Marker.SPHERE
        highlight.action = Marker.ADD
        highlight.pose.position.x, highlight.pose.position.y, highlight.pose.position.z = map(float, point)
        highlight.pose.orientation.w = 1.0
        highlight.scale.x = highlight.scale.y = highlight.scale.z = 0.46
        highlight.color.r = 1.0
        highlight.color.g = 0.0
        highlight.color.b = 0.0
        highlight.color.a = 0.88
        marker_array.markers.append(highlight)

        arrow = Marker()
        arrow.header.frame_id = frame
        arrow.ns = "global_bottleneck_arrow"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = 0.10
        arrow.scale.y = 0.22
        arrow.scale.z = 0.28
        arrow.color.r = 1.0
        arrow.color.g = 0.05
        arrow.color.b = 0.05
        arrow.color.a = 1.0
        arrow.points = [Point(x=float(point[0]), y=float(point[1]), z=float(point[2]) + 1.35),
                        Point(x=float(point[0]), y=float(point[1]), z=float(point[2]) + 0.25)]
        marker_array.markers.append(arrow)

        warning = Marker()
        warning.header.frame_id = frame
        warning.ns = "global_bottleneck_label"
        warning.id = 0
        warning.type = Marker.TEXT_VIEW_FACING
        warning.action = Marker.ADD
        warning.pose.position.x = float(point[0])
        warning.pose.position.y = float(point[1])
        warning.pose.position.z = float(point[2]) + 1.55
        warning.pose.orientation.w = 1.0
        warning.scale.z = 0.30
        warning.color.r = 1.0
        warning.color.g = 0.12
        warning.color.b = 0.12
        warning.color.a = 1.0
        warning.text = (f"BOTTLENECK  SEG {segment_index + 1}  {minimum:.2f} m\n"
                        f"({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")
        marker_array.markers.append(warning)
    return count


def add_round_planned_path(marker_array, mission, frame):
    """Render the planned route as round samples instead of an RViz billboard ribbon."""
    points = []
    for segment in mission.get("segments", []):
        validated = segment.get("validated_trajectory") or {}
        points.extend(validated.get("points_xyz_m") or segment.get("points_xyz_m", []))
    if not points:
        return 0

    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = "round_planned_path"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.11
    marker.color.r = 0.10
    marker.color.g = 0.85
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.points = [Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])) for xyz in points]
    marker_array.markers.append(marker)
    return len(points)


def publish_traversability(voxel_snapshot, frame):
    """Publish filtered occupancy and traversability voxel centers for RViz."""
    if voxel_snapshot is None:
        return [], 0, 0, 0, 0.0
    metadata_path = voxel_snapshot / "metadata.json"
    arrays_path = voxel_snapshot / "voxel_map.npz"
    if not metadata_path.is_file() or not arrays_path.is_file():
        return [], 0, 0, 0, 0.0

    metadata = json.loads(metadata_path.read_text())
    arrays = np.load(arrays_path)
    raw = arrays["raw_occupancy_zyx"]
    esdf = arrays["esdf_zyx_m"]
    origin = np.asarray(metadata["origin_xyz_m"], dtype=np.float32)
    resolution = float(metadata["resolution_m"])
    threshold = float(metadata["minimum_esdf_distance_m"])

    observed_free = raw == 0
    filtered_occupied = raw > 0
    inflated_free = observed_free & (esdf + 1e-6 >= threshold)
    inflation_excluded = observed_free & ~inflated_free
    header = Header(frame_id=frame, stamp=rospy.Time.now())
    publishers = []

    def publish_mask(mask, topic):
        # np.argwhere returns z/y/x; RViz needs center coordinates in x/y/z.
        zyx = np.argwhere(mask)
        xyz = zyx[:, ::-1].astype(np.float32)
        xyz = origin + (xyz + 0.5) * resolution
        message = point_cloud2.create_cloud_xyz32(header, xyz)
        publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
        publisher.publish(message)
        publishers.append(publisher)
        return len(xyz)

    occupied_count = publish_mask(filtered_occupied, "/drone_room/filtered_occupied")
    free_count = publish_mask(inflated_free, "/drone_room/inflated_free")
    excluded_count = publish_mask(inflation_excluded, "/drone_room/inflation_excluded")
    return publishers, occupied_count, free_count, excluded_count, threshold


def setup_clicked_point_display(frame):
    """Show Publish Point coordinates directly in the RViz scene."""
    publisher = rospy.Publisher(
        "/drone_room/clicked_point", MarkerArray, queue_size=1, latch=True
    )

    def clicked(message):
        point = message.point
        markers = MarkerArray()
        sphere = Marker()
        sphere.header.frame_id = message.header.frame_id or frame
        sphere.header.stamp = rospy.Time.now()
        sphere.ns = "clicked_point"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.16
        sphere.color.r = 0.10
        sphere.color.g = 0.85
        sphere.color.b = 1.0
        sphere.color.a = 1.0
        markers.markers.append(sphere)

        label = Marker()
        label.header = sphere.header
        label.ns = "clicked_point_coordinate"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = point.x
        label.pose.position.y = point.y
        label.pose.position.z = point.z + 0.24
        label.pose.orientation.w = 1.0
        label.scale.z = 0.20
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f"x={point.x:.2f}  y={point.y:.2f}  z={point.z:.2f}"
        markers.markers.append(label)
        publisher.publish(markers)
        rospy.loginfo("Clicked world point: x=%.3f y=%.3f z=%.3f", point.x, point.y, point.z)

    subscriber = rospy.Subscriber("/clicked_point", PointStamped, clicked, queue_size=1)
    return publisher, subscriber


def main():
    if len(sys.argv) not in (8, 9):
        raise SystemExit("usage: publish_drone_room_visualization.py MAP_PCD CLUSTERS_JSON RAW_TARGETS_JSON BOXER_3D_CSV TRAJECTORY_CSV MISSION_JSON MIN_OBSERVATIONS [VOXEL_SNAPSHOT]")
    map_path, clusters_path, raw_targets_path, boxer_3d_path, trajectory_path, mission_path = map(Path, sys.argv[1:7])
    min_observations = int(sys.argv[7])
    voxel_snapshot = Path(sys.argv[8]) if len(sys.argv) == 9 else None
    rospy.init_node("drone_room_offline_visualization", anonymous=False)
    frame = "map"

    fields = [PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1),
              PointField("z", 8, PointField.FLOAT32, 1),
              PointField("intensity", 12, PointField.FLOAT32, 1)]
    cloud_header = Header(frame_id=frame, stamp=rospy.Time.now())
    cloud = point_cloud2.create_cloud(cloud_header, fields, read_binary_pcd(map_path))
    cloud_pub = rospy.Publisher("/drone_room/map", PointCloud2, queue_size=1, latch=True)
    cloud_pub.publish(cloud)
    traversability_pubs, occupied_count, free_count, excluded_count, inflation_threshold = (
        publish_traversability(voxel_snapshot, frame)
    )
    clicked_point_pub, clicked_point_sub = setup_clicked_point_display(frame)

    semantic_markers = MarkerArray()
    boxes = load_box_geometry(clusters_path, raw_targets_path, boxer_3d_path, min_observations)
    for item, geometry in boxes:
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = "semantic_boxes"
        marker.id = int(item["cluster_id"])
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(item["world_x_m"])
        marker.pose.position.y = float(item["world_y_m"])
        marker.pose.position.z = float(item["world_z_m"])
        if geometry:
            marker.pose.orientation.w = float(geometry["qw_world_object"])
            marker.pose.orientation.x = float(geometry["qx_world_object"])
            marker.pose.orientation.y = float(geometry["qy_world_object"])
            marker.pose.orientation.z = float(geometry["qz_world_object"])
            marker.scale.x = min(2.5, max(0.08, abs(float(geometry["scale_x"]))))
            marker.scale.y = min(2.5, max(0.08, abs(float(geometry["scale_y"]))))
            marker.scale.z = min(2.5, max(0.08, abs(float(geometry["scale_z"]))))
        else:
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.30
        marker.color.r, marker.color.g, marker.color.b = semantic_color(item["label"])
        marker.color.a = 0.34
        marker.lifetime = rospy.Duration(0)
        semantic_markers.markers.append(marker)
        text = Marker()
        text.header.frame_id = frame
        text.ns = "semantic_labels"
        text.id = int(item["cluster_id"])
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = marker.pose.position.x
        text.pose.position.y = marker.pose.position.y
        text.pose.position.z = marker.pose.position.z + marker.scale.z * 0.5 + 0.15
        text.pose.orientation.w = 1.0
        text.scale.z = 0.18
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a = 1.0
        text.text = display_label(item["label"])
        semantic_markers.markers.append(text)
    trajectory_pub = load_path(trajectory_path, "/drone_room/trajectory")
    mission = json.loads(mission_path.read_text())
    planning_markers = MarkerArray()
    frustum_count = add_observation_frustums(planning_markers, mission, mission_path, frame)
    clearance_count = add_clearance_markers(planning_markers, mission, voxel_snapshot, frame)
    round_path_count = add_round_planned_path(planning_markers, mission, frame)
    semantic_marker_pub = rospy.Publisher(
        "/drone_room/semantic_markers", MarkerArray, queue_size=1, latch=True
    )
    planning_marker_pub = rospy.Publisher(
        "/drone_room/planning_markers", MarkerArray, queue_size=1, latch=True
    )
    semantic_marker_pub.publish(semantic_markers)
    planning_marker_pub.publish(planning_markers)

    plan = RosPath()
    plan.header.frame_id = frame
    for segment in mission.get("segments", []):
        validated = segment.get("validated_trajectory") or {}
        points = validated.get("points_xyz_m") or segment.get("points_xyz_m", [])
        for xyz in points:
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
            pose.pose.orientation.w = 1.0
            plan.poses.append(pose)
    plan_pub = rospy.Publisher("/drone_room/planned_path", RosPath, queue_size=1, latch=True)
    plan_pub.publish(plan)
    rospy.loginfo("Published map (%d points), filtered-occupied=%d, inflated-free=%d, inflation-excluded=%d "
                  "(minimum clearance %.2fm), %d semantic 3D boxes, %d observation frustums, "
                  "%d clearance markers, round plan (%d samples), trajectory and validated plan (%d poses)",
                  len(cloud.data) // cloud.point_step, occupied_count, free_count, excluded_count,
                  inflation_threshold, len(boxes), frustum_count,
                  clearance_count, round_path_count, len(plan.poses))
    rospy.spin()


if __name__ == "__main__":
    main()
