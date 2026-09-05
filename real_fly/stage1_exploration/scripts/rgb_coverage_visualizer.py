#!/usr/bin/env python3
"""Show sampled D435 RGB camera frustums along the FAST-LIO trajectory."""

import math

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker


R_I_L = np.array([
    [0.9659258, 0.0, 0.2588190],
    [0.0, 1.0, 0.0],
    [-0.2588190, 0.0, 0.9659258],
], dtype=np.float64)
T_I_L = np.array([-0.011, -0.02329, 0.04412], dtype=np.float64)

# FAST-Calib calib_01 multi-scene result: p_camera = R_C_L*p_lidar + T_C_L.
R_C_L = np.array([
    [-0.021484, -0.999724, -0.009470],
    [0.256997, 0.003631, -0.966405],
    [0.966173, -0.023196, 0.256849],
], dtype=np.float64)
T_C_L = np.array([0.037228, -0.121695, -0.033376], dtype=np.float64)


def rigid(rotation, translation):
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = rotation
    value[:3, 3] = translation
    return value


def ros_point(value):
    point = Point()
    point.x, point.y, point.z = map(float, value)
    return point


class CoverageVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~fixed_frame", "world")
        self.depth = float(rospy.get_param("~frustum_depth", 3.0))
        self.min_translation = float(rospy.get_param("~min_translation", 0.40))
        self.min_rotation = math.radians(float(rospy.get_param("~min_rotation_deg", 15.0)))
        self.max_samples = int(rospy.get_param("~max_samples", 400))
        self.fx = float(rospy.get_param("~fx", 605.2134399414062))
        self.fy = float(rospy.get_param("~fy", 605.4168090820312))
        self.cx = float(rospy.get_param("~cx", 330.0566101074219))
        self.cy = float(rospy.get_param("~cy", 245.53988647460938))
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))

        t_i_l = rigid(R_I_L, T_I_L)
        t_c_l = rigid(R_C_L, T_C_L)
        self.t_i_c = t_i_l @ np.linalg.inv(t_c_l)
        self.latest_odom = None
        self.samples = []

        self.frustums_pub = rospy.Publisher("/rgb_coverage/frustums", Marker, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher("/rgb_coverage/camera_path", Marker, queue_size=1, latch=True)
        self.current_pub = rospy.Publisher("/rgb_coverage/current_frustum", Marker, queue_size=1)
        self.count_pub = rospy.Publisher("/rgb_coverage/count", UInt32, queue_size=1, latch=True)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)

    def odom_callback(self, message):
        self.latest_odom = message

    def image_callback(self, image):
        if self.latest_odom is None:
            return
        odom = self.latest_odom
        if image.header.stamp and odom.header.stamp:
            if abs((image.header.stamp - odom.header.stamp).to_sec()) > 0.5:
                return
        pose = odom.pose.pose
        t_w_i = quaternion_matrix([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ])
        t_w_i[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        t_w_c = t_w_i @ self.t_i_c
        self.publish_current(t_w_c, image.header.stamp)
        if not self.should_sample(t_w_c):
            return
        self.samples.append(t_w_c)
        if len(self.samples) > self.max_samples:
            self.samples.pop(0)
        self.publish_history(image.header.stamp)

    def should_sample(self, current):
        if not self.samples:
            return True
        previous = self.samples[-1]
        translation = np.linalg.norm(current[:3, 3] - previous[:3, 3])
        relative = previous[:3, :3].T @ current[:3, :3]
        angle = math.acos(float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)))
        return translation >= self.min_translation or angle >= self.min_rotation

    def camera_corners(self):
        pixels = ((0.0, 0.0), (self.width, 0.0),
                  (self.width, self.height), (0.0, self.height))
        return [np.array([(u - self.cx) * self.depth / self.fx,
                          (v - self.cy) * self.depth / self.fy,
                          self.depth, 1.0]) for u, v in pixels]

    def frustum_lines(self, transform):
        origin = transform[:3, 3]
        corners = [(transform @ corner)[:3] for corner in self.camera_corners()]
        lines = []
        for corner in corners:
            lines.extend((ros_point(origin), ros_point(corner)))
        for index in range(4):
            lines.extend((ros_point(corners[index]), ros_point(corners[(index + 1) % 4])))
        return lines

    def base_marker(self, namespace, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp or rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_current(self, transform, stamp):
        marker = self.base_marker("rgb_current", 0, Marker.LINE_LIST, stamp)
        marker.scale.x = 0.055
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.15, 0.05, 0.95
        marker.points = self.frustum_lines(transform)
        self.current_pub.publish(marker)

    def publish_history(self, stamp):
        frustums = self.base_marker("rgb_seen", 0, Marker.LINE_LIST, stamp)
        frustums.scale.x = 0.025
        frustums.color.r, frustums.color.g, frustums.color.b, frustums.color.a = 0.0, 0.85, 1.0, 0.35
        for transform in self.samples:
            frustums.points.extend(self.frustum_lines(transform))
        self.frustums_pub.publish(frustums)

        path = self.base_marker("rgb_camera_path", 0, Marker.LINE_STRIP, stamp)
        path.scale.x = 0.07
        path.color.r, path.color.g, path.color.b, path.color.a = 1.0, 0.75, 0.0, 1.0
        path.points = [ros_point(transform[:3, 3]) for transform in self.samples]
        self.path_pub.publish(path)
        self.count_pub.publish(UInt32(data=len(self.samples)))


if __name__ == "__main__":
    rospy.init_node("rgb_coverage_visualizer")
    CoverageVisualizer()
    rospy.loginfo("RGB coverage visualization ready: cyan=sampled, red=current, yellow=path")
    rospy.spin()
