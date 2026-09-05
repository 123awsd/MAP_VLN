#!/usr/bin/env python3
"""Publish Drone_room offline outputs for RViz; no sensors or flight nodes."""
import csv
import colorsys
import json
import struct
import sys
import zlib
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def read_binary_pcd(path):
    raw = path.read_bytes()
    marker = raw.find(b"DATA binary\n")
    if marker < 0:
        raise RuntimeError("Only binary PCD is supported")
    header = raw[:marker].decode("ascii", errors="strict")
    points = int(next(line.split()[1] for line in header.splitlines() if line.startswith("POINTS ")))
    payload = raw[marker + len(b"DATA binary\n"):]
    stride = 32
    return [struct.unpack_from("<fff", payload, i * stride) for i in range(points)]


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


def main():
    if len(sys.argv) != 8:
        raise SystemExit("usage: publish_drone_room_visualization.py MAP_PCD CLUSTERS_JSON RAW_TARGETS_JSON BOXER_3D_CSV TRAJECTORY_CSV MISSION_JSON MIN_OBSERVATIONS")
    map_path, clusters_path, raw_targets_path, boxer_3d_path, trajectory_path, mission_path = map(Path, sys.argv[1:7])
    min_observations = int(sys.argv[7])
    rospy.init_node("drone_room_offline_visualization", anonymous=False)
    frame = "map"

    fields = [PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1),
              PointField("z", 8, PointField.FLOAT32, 1)]
    cloud_header = Header(frame_id=frame, stamp=rospy.Time.now())
    cloud = point_cloud2.create_cloud(cloud_header, fields, read_binary_pcd(map_path))
    cloud_pub = rospy.Publisher("/drone_room/map", PointCloud2, queue_size=1, latch=True)
    cloud_pub.publish(cloud)

    marker_array = MarkerArray()
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
        marker_array.markers.append(marker)
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
        text.text = "%s #%d  obs=%d  conf=%.2f" % (
            item["label"], item["cluster_id"], item["observations"], item["confidence_max"])
        marker_array.markers.append(text)
    marker_pub = rospy.Publisher("/drone_room/targets", MarkerArray, queue_size=1, latch=True)
    marker_pub.publish(marker_array)

    trajectory_pub = load_path(trajectory_path, "/drone_room/trajectory")
    mission = json.loads(mission_path.read_text())
    plan = RosPath()
    plan.header.frame_id = frame
    for segment in mission.get("segments", []):
        for xyz in segment.get("points_xyz_m", []):
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
            pose.pose.orientation.w = 1.0
            plan.poses.append(pose)
    plan_pub = rospy.Publisher("/drone_room/planned_path", RosPath, queue_size=1, latch=True)
    plan_pub.publish(plan)
    rospy.loginfo("Published map (%d points), %d semantic 3D boxes, trajectory and plan (%d poses)",
                  len(cloud.data) // cloud.point_step, len(boxes), len(plan.poses))
    rospy.spin()


if __name__ == "__main__":
    main()
