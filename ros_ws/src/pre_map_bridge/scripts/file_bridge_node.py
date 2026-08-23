#!/usr/bin/env python3
"""Exchange synchronized Habitat observations and FALCON commands via atomic files."""

import csv
import json
import os
from pathlib import Path

import rospy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


class FileBridge:
    def __init__(self):
        self.root = Path(rospy.get_param("~bridge_dir", "/workspace/shared/bridge"))
        self.root.mkdir(parents=True, exist_ok=True)
        self.last_sequence = -1
        self.latest = None
        self.depth_pub = rospy.Publisher("/uav_simulator/depth_image", Image, queue_size=1)
        self.rgb_pub = rospy.Publisher("/habitat/rgb", Image, queue_size=1)
        self.odom_pub = rospy.Publisher("/uav_simulator/odometry", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/uav_simulator/sensor_pose", TransformStamped, queue_size=10)
        self.path_pub = rospy.Publisher("/pre_map_vln/agent_path", RosPath, queue_size=1, latch=True)
        self.box_pub = rospy.Publisher(
            "/pre_map_vln/box_markers", MarkerArray, queue_size=1, latch=True
        )
        self.path = RosPath()
        self.path.header.frame_id = "world"
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.command_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.pose_timer)
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.frame_timer)
        rospy.Timer(rospy.Duration(1.0), self.publish_boxes)

    @staticmethod
    def atomic_json(path, value):
        temp = path.with_suffix(path.suffix + ".tmp")
        with temp.open("w", encoding="utf-8") as stream:
            json.dump(value, stream)
        os.replace(str(temp), str(path))

    def command_callback(self, msg):
        self.atomic_json(self.root / "command.json", {
            "stamp": rospy.get_time(), "trajectory_id": int(msg.trajectory_id),
            "position": [msg.position.x, msg.position.y, msg.position.z],
            "velocity": [msg.velocity.x, msg.velocity.y, msg.velocity.z],
            "acceleration": [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            "yaw": msg.yaw, "yaw_dot": msg.yaw_dot,
        })

    def load_frame(self):
        state_path = self.root / "state.json"
        depth_path = self.root / "depth_u16.raw"
        rgb_path = self.root / "rgb_u8.raw"
        if not state_path.exists() or not depth_path.exists() or not rgb_path.exists():
            return None
        try:
            state = json.loads(state_path.read_text(encoding="utf-8"))
            sequence = int(state["sequence"])
            if sequence == self.last_sequence:
                return None
            depth = depth_path.read_bytes()
            rgb = rgb_path.read_bytes()
            if len(depth) != int(state["width"]) * int(state["height"]) * 2:
                rospy.logwarn_throttle(2.0, "Depth frame size mismatch")
                return None
            if len(rgb) != int(state["width"]) * int(state["height"]) * 3:
                rospy.logwarn_throttle(2.0, "RGB frame size mismatch")
                return None
            return state, depth, rgb
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Bridge frame not ready: %s", exc)
            return None

    @staticmethod
    def fill_pose(state, stamp, body=False):
        p = state["position"]
        q = (
            state.get("body_orientation_xyzw", state["orientation_xyzw"])
            if body else state["orientation_xyzw"]
        )
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link" if body else "camera_optical"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = p
        transform.transform.rotation.x, transform.transform.rotation.y = q[0], q[1]
        transform.transform.rotation.z, transform.transform.rotation.w = q[2], q[3]
        return transform

    def publish_pose(self, stamp):
        if self.latest is None:
            return
        optical_transform = self.fill_pose(self.latest, stamp)
        body_transform = self.fill_pose(self.latest, stamp, body=True)
        self.pose_pub.publish(optical_transform)
        odom = Odometry()
        odom.header = body_transform.header
        odom.child_frame_id = "base_link"
        odom.pose.pose.position = body_transform.transform.translation
        odom.pose.pose.orientation = body_transform.transform.rotation
        self.odom_pub.publish(odom)

    def publish_boxes(self, _event):
        csv_path = Path(rospy.get_param(
            "~box_csv",
            "/workspace/shared/outputs/boxer/hm3d_stage1/boxer_3dbbs_visual.csv",
        ))
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "world"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        try:
            with csv_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        except OSError as exc:
            rospy.logwarn("Unable to load Boxer markers from %s: %s", csv_path, exc)
            self.box_pub.publish(markers)
            return

        palette = [
            (0.12, 0.72, 1.00), (1.00, 0.45, 0.12), (0.45, 0.90, 0.25),
            (0.85, 0.30, 0.90), (1.00, 0.82, 0.15), (0.25, 0.85, 0.75),
        ]
        stamp = rospy.Time.now()
        for index, row in enumerate(rows):
            red, green, blue = palette[index % len(palette)]
            cube = Marker()
            cube.header.stamp, cube.header.frame_id = stamp, "world"
            cube.ns, cube.id, cube.type, cube.action = "boxer_3d", index, Marker.CUBE, Marker.ADD
            cube.pose.position.x = float(row["tx_world_object"])
            cube.pose.position.y = float(row["ty_world_object"])
            cube.pose.position.z = float(row["tz_world_object"])
            cube.pose.orientation.w = float(row["qw_world_object"])
            cube.pose.orientation.x = float(row["qx_world_object"])
            cube.pose.orientation.y = float(row["qy_world_object"])
            cube.pose.orientation.z = float(row["qz_world_object"])
            cube.scale.x = float(row["scale_x"])
            cube.scale.y = float(row["scale_y"])
            cube.scale.z = float(row["scale_z"])
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = red, green, blue, 0.10
            cube.lifetime = rospy.Duration(0)
            markers.markers.append(cube)

            outline = Marker()
            outline.header.stamp, outline.header.frame_id = stamp, "world"
            outline.ns, outline.id = "boxer_outlines", index
            outline.type, outline.action = Marker.LINE_LIST, Marker.ADD
            outline.pose = cube.pose
            outline.scale.x = 0.055
            outline.color.r, outline.color.g, outline.color.b, outline.color.a = (
                red, green, blue, 1.0
            )
            hx, hy, hz = cube.scale.x * 0.5, cube.scale.y * 0.5, cube.scale.z * 0.5
            corners = [
                (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
                (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
            ]
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ]
            for start, end in edges:
                outline.points.append(Point(*corners[start]))
                outline.points.append(Point(*corners[end]))
            outline.lifetime = rospy.Duration(0)
            markers.markers.append(outline)

            label = Marker()
            label.header.stamp, label.header.frame_id = stamp, "world"
            label.ns, label.id, label.type, label.action = "boxer_labels", index, Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x = cube.pose.position.x
            label.pose.position.y = cube.pose.position.y
            label.pose.position.z = cube.pose.position.z + cube.scale.z * 0.5 + 0.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.34
            label.color.r, label.color.g, label.color.b, label.color.a = red, green, blue, 1.0
            label.text = "%s %.2f" % (row["name"], float(row["prob"]))
            label.lifetime = rospy.Duration(0)
            markers.markers.append(label)
        self.box_pub.publish(markers)
        rospy.loginfo_throttle(
            10.0, "Published %d Boxer 3D boxes from %s", len(rows), csv_path
        )

    def pose_timer(self, _event):
        self.publish_pose(rospy.Time.now())

    def frame_timer(self, _event):
        loaded = self.load_frame()
        if loaded is None:
            return
        state, depth, rgb = loaded
        self.latest = state
        self.last_sequence = int(state["sequence"])
        stamp = rospy.Time.now()
        self.publish_pose(stamp)
        image = Image()
        image.header.stamp, image.header.frame_id = stamp, "camera"
        image.height, image.width = int(state["height"]), int(state["width"])
        image.encoding, image.is_bigendian, image.step = "16UC1", False, image.width * 2
        image.data = depth
        self.depth_pub.publish(image)

        rgb_image = Image()
        rgb_image.header.stamp, rgb_image.header.frame_id = stamp, "camera"
        rgb_image.height, rgb_image.width = image.height, image.width
        rgb_image.encoding, rgb_image.is_bigendian, rgb_image.step = "rgb8", False, image.width * 3
        rgb_image.data = rgb
        self.rgb_pub.publish(rgb_image)

        pose = PoseStamped()
        pose.header = image.header
        transform = self.fill_pose(state, stamp, body=True)
        pose.header.frame_id = "world"
        pose.pose.position = transform.transform.translation
        pose.pose.orientation = transform.transform.rotation
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        rospy.loginfo_throttle(2.0, "Published Habitat frame %d", self.last_sequence)


if __name__ == "__main__":
    rospy.init_node("habitat_file_bridge")
    FileBridge()
    rospy.spin()
