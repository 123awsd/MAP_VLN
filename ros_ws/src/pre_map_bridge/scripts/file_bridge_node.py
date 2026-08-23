#!/usr/bin/env python3
"""Exchange synchronized Habitat observations and FALCON commands via atomic files."""

import json
import os
from pathlib import Path

import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Image


class FileBridge:
    def __init__(self):
        self.root = Path(rospy.get_param("~bridge_dir", "/workspace/shared/bridge"))
        self.root.mkdir(parents=True, exist_ok=True)
        self.last_sequence = -1
        self.latest = None
        self.depth_pub = rospy.Publisher("/uav_simulator/depth_image", Image, queue_size=1)
        self.odom_pub = rospy.Publisher("/uav_simulator/odometry", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/uav_simulator/sensor_pose", TransformStamped, queue_size=10)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.command_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.pose_timer)
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.frame_timer)

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
        if not state_path.exists() or not depth_path.exists():
            return None
        try:
            state = json.loads(state_path.read_text(encoding="utf-8"))
            sequence = int(state["sequence"])
            if sequence == self.last_sequence:
                return None
            depth = depth_path.read_bytes()
            if len(depth) != int(state["width"]) * int(state["height"]) * 2:
                rospy.logwarn_throttle(2.0, "Depth frame size mismatch")
                return None
            return state, depth
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Bridge frame not ready: %s", exc)
            return None

    @staticmethod
    def fill_pose(state, stamp):
        p, q = state["position"], state["orientation_xyzw"]
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "camera"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = p
        transform.transform.rotation.x, transform.transform.rotation.y = q[0], q[1]
        transform.transform.rotation.z, transform.transform.rotation.w = q[2], q[3]
        return transform

    def publish_pose(self, stamp):
        if self.latest is None:
            return
        transform = self.fill_pose(self.latest, stamp)
        self.pose_pub.publish(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = "base_link"
        odom.pose.pose.position = transform.transform.translation
        odom.pose.pose.orientation = transform.transform.rotation
        self.odom_pub.publish(odom)

    def pose_timer(self, _event):
        self.publish_pose(rospy.Time.now())

    def frame_timer(self, _event):
        loaded = self.load_frame()
        if loaded is None:
            return
        state, depth = loaded
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
        rospy.loginfo_throttle(2.0, "Published Habitat frame %d", self.last_sequence)


if __name__ == "__main__":
    rospy.init_node("habitat_file_bridge")
    FileBridge()
    rospy.spin()
