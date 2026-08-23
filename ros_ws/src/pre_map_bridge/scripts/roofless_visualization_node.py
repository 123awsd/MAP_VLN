#!/usr/bin/env python3
"""Publish derived, balanced RViz layers for exploration visualization."""

import copy
import math

import numpy as np
import rospy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from trajectory.msg import Bspline
from visualization_msgs.msg import Marker, MarkerArray


class RooflessVisualization:
    def __init__(self):
        self.fallback_floor_z = float(rospy.get_param("~floor_z", 0.0))
        self.sensor_height = float(rospy.get_param("~sensor_height", 1.0))
        self.floor_clearance = float(rospy.get_param("~floor_clearance", 0.25))
        self.ceiling_min_height = float(rospy.get_param("~ceiling_min_height", 1.65))
        self.ceiling_thickness = float(rospy.get_param("~ceiling_thickness", 0.60))
        self.voxel_resolution = float(rospy.get_param("~voxel_resolution", 0.10))
        self.frustum_length = float(rospy.get_param("~frustum_length", 0.9))
        self.camera_hfov = math.radians(float(rospy.get_param("~camera_hfov", 90.0)))
        self.camera_aspect = float(rospy.get_param("~camera_aspect", 4.0 / 3.0))
        self.box_line_width = float(rospy.get_param("~box_line_width", 0.050))
        self.box_color_scale = float(rospy.get_param("~box_color_scale", 0.72))
        self.box_alpha = float(rospy.get_param("~box_alpha", 0.62))
        self.floor_z = self.fallback_floor_z
        self.plan_samples = max(8, int(rospy.get_param("~plan_samples", 64)))
        self.cloud_pub = rospy.Publisher(
            "/pre_map_vln/roofless_map", PointCloud2, queue_size=1
        )
        self.plan_pub = rospy.Publisher(
            "/pre_map_vln/current_plan", Marker, queue_size=1, latch=True
        )
        self.frustum_pub = rospy.Publisher(
            "/pre_map_vln/camera_frustum", Marker, queue_size=1
        )
        self.box_pub = rospy.Publisher(
            "/pre_map_vln/styled_box_markers", MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/voxel_mapping/occupancy_grid_occupied",
            PointCloud2,
            self.cloud_callback,
            queue_size=1,
        )
        rospy.Subscriber("/planning/bspline", Bspline, self.plan_callback, queue_size=1)
        rospy.Subscriber(
            "/uav_simulator/odometry", Odometry, self.odom_callback, queue_size=1
        )
        rospy.Subscriber(
            "/uav_simulator/sensor_pose",
            TransformStamped,
            self.sensor_pose_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/pre_map_vln/box_markers",
            MarkerArray,
            self.box_callback,
            queue_size=1,
        )
        rospy.loginfo(
            "Envelope filter: floor +%.2f m, roof top %.2f m, min ceiling %.2f m",
            self.floor_clearance,
            self.ceiling_thickness,
            self.ceiling_min_height,
        )

    def box_callback(self, message):
        """Restyle outline markers without changing their progressive timing."""
        output = copy.deepcopy(message)
        for marker in output.markers:
            if marker.ns != "boxer_outlines" or marker.action != Marker.ADD:
                continue
            marker.scale.x = max(self.box_line_width, marker.scale.x)
            marker.color.r = max(0.0, min(1.0, marker.color.r * self.box_color_scale))
            marker.color.g = max(0.0, min(1.0, marker.color.g * self.box_color_scale))
            marker.color.b = max(0.0, min(1.0, marker.color.b * self.box_color_scale))
            marker.color.a = max(marker.color.a, self.box_alpha)
        self.box_pub.publish(output)

    def odom_callback(self, message):
        self.floor_z = float(message.pose.pose.position.z) - self.sensor_height

    @staticmethod
    def rotate_vector(vector, quaternion):
        q_vector = np.asarray(
            [quaternion.x, quaternion.y, quaternion.z], dtype=np.float64
        )
        norm = math.sqrt(float(np.dot(q_vector, q_vector)) + quaternion.w ** 2)
        if norm < 1e-12:
            return vector
        q_vector /= norm
        q_w = quaternion.w / norm
        return vector + 2.0 * np.cross(q_vector, np.cross(q_vector, vector) + q_w * vector)

    def sensor_pose_callback(self, message):
        origin = np.asarray(
            [
                message.transform.translation.x,
                message.transform.translation.y,
                message.transform.translation.z,
            ],
            dtype=np.float64,
        )
        half_width = self.frustum_length * math.tan(0.5 * self.camera_hfov)
        half_height = half_width / max(0.1, self.camera_aspect)
        # camera_optical follows ROS convention: +X right, +Y down, +Z forward.
        local_corners = [
            np.asarray([x, y, self.frustum_length], dtype=np.float64)
            for x, y in (
                (-half_width, -half_height),
                (half_width, -half_height),
                (half_width, half_height),
                (-half_width, half_height),
            )
        ]
        corners = [
            origin + self.rotate_vector(corner, message.transform.rotation)
            for corner in local_corners
        ]
        center = origin + self.rotate_vector(
            np.asarray([0.0, 0.0, self.frustum_length], dtype=np.float64),
            message.transform.rotation,
        )

        marker = Marker()
        marker.header = message.header
        marker.header.frame_id = message.header.frame_id or "world"
        marker.ns = "camera_fov"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.018
        marker.color.r = 0.95
        marker.color.g = 0.12
        marker.color.b = 0.04
        marker.color.a = 0.88
        segments = []
        for corner in corners:
            segments.append((origin, corner))
        for index in range(4):
            segments.append((corners[index], corners[(index + 1) % 4]))
        segments.append((origin, center))
        for start, end in segments:
            marker.points.append(Point(*start))
            marker.points.append(Point(*end))
        self.frustum_pub.publish(marker)

    def cloud_callback(self, message):
        xyz_fields = {
            field.name: field
            for field in message.fields
            if field.name in ("x", "y", "z")
        }
        if len(xyz_fields) != 3 or any(
            field.datatype != PointField.FLOAT32 for field in xyz_fields.values()
        ):
            rospy.logwarn_throttle(5.0, "Cannot filter cloud: XYZ are not FLOAT32")
            return
        if message.point_step <= 0 or not message.data:
            return
        point_count = len(message.data) // message.point_step
        endian = ">f4" if message.is_bigendian else "<f4"
        coordinates = {
            axis: np.ndarray(
                shape=(point_count,),
                dtype=endian,
                buffer=message.data,
                offset=xyz_fields[axis].offset,
                strides=(message.point_step,),
            )
            for axis in ("x", "y", "z")
        }
        finite = np.isfinite(coordinates["x"]) & np.isfinite(coordinates["y"]) & np.isfinite(
            coordinates["z"]
        )
        valid_indices = np.flatnonzero(finite)
        if not len(valid_indices):
            return
        x_values = coordinates["x"][valid_indices]
        y_values = coordinates["y"][valid_indices]
        z_values = coordinates["z"][valid_indices]

        # Each XY voxel column has its own upper envelope. Removing an upper
        # layer below that envelope follows horizontal or sloped ceilings while
        # retaining vertical walls and obstacle sides below it.
        x_cells = np.rint(x_values / self.voxel_resolution).astype(np.int64)
        y_cells = np.rint(y_values / self.voxel_resolution).astype(np.int64)
        y_min = int(y_cells.min())
        y_span = int(y_cells.max()) - y_min + 1
        column_keys = (x_cells - int(x_cells.min())) * y_span + (y_cells - y_min)
        _, inverse = np.unique(column_keys, return_inverse=True)
        column_top = np.full(int(inverse.max()) + 1, -np.inf, dtype=np.float32)
        np.maximum.at(column_top, inverse, z_values)

        floor_mask = z_values <= self.floor_z + self.floor_clearance
        ceiling_columns = column_top[inverse] >= self.floor_z + self.ceiling_min_height
        ceiling_mask = ceiling_columns & (
            z_values >= column_top[inverse] - self.ceiling_thickness
        )
        visible = ~(floor_mask | ceiling_mask)
        visible_z = z_values[visible]
        height_span = max(0.1, self.ceiling_min_height - self.floor_clearance)
        height_ratio = np.clip(
            (visible_z - self.floor_z - self.floor_clearance) / height_span,
            0.0,
            1.0,
        )

        # FUEL's office demo is dominated by cyan explored surfaces and magenta
        # upper boundaries. Use the same restrained palette instead of RViz's
        # full rainbow, which makes a roofless wall-only view visually noisy.
        red = np.interp(height_ratio, (0.0, 0.55, 1.0), (0, 35, 235)).astype(np.uint32)
        green = np.interp(height_ratio, (0.0, 0.55, 1.0), (210, 70, 0)).astype(np.uint32)
        blue = np.interp(height_ratio, (0.0, 0.55, 1.0), (230, 210, 210)).astype(np.uint32)
        rgba = (np.uint32(255) << np.uint32(24)) | (red << 16) | (green << 8) | blue
        endian = ">" if message.is_bigendian else "<"
        colored = np.empty(
            int(visible.sum()),
            dtype=[("x", endian + "f4"), ("y", endian + "f4"),
                   ("z", endian + "f4"), ("rgba", endian + "u4")],
        )
        colored["x"] = x_values[visible]
        colored["y"] = y_values[visible]
        colored["z"] = visible_z
        colored["rgba"] = rgba

        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = len(colored)
        output.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgba", 12, PointField.UINT32, 1),
        ]
        output.is_bigendian = message.is_bigendian
        output.point_step = 16
        output.row_step = output.point_step * output.width
        output.data = colored.tobytes()
        output.is_dense = True
        self.cloud_pub.publish(output)
        rospy.loginfo_throttle(
            5.0,
            "Roofless cloud: %d -> %d points (floor %.2f m)",
            point_count,
            output.width,
            self.floor_z,
        )

    @staticmethod
    def de_boor(control, knots, degree, value):
        count = len(control)
        if value >= knots[count]:
            span = count - 1
        else:
            span = degree
            while span + 1 < count and value >= knots[span + 1]:
                span += 1
        work = [control[span - degree + index].copy() for index in range(degree + 1)]
        for level in range(1, degree + 1):
            for index in range(degree, level - 1, -1):
                knot_index = span - degree + index
                denominator = knots[knot_index + degree - level + 1] - knots[knot_index]
                alpha = 0.0 if abs(denominator) < 1e-12 else (value - knots[knot_index]) / denominator
                work[index] = (1.0 - alpha) * work[index - 1] + alpha * work[index]
        return work[degree]

    def plan_callback(self, message):
        control = [np.asarray([point.x, point.y, point.z], dtype=np.float64) for point in message.pos_pts]
        knots = np.asarray(message.knots, dtype=np.float64)
        degree = int(message.order)
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "falcon_current_plan"
        marker.id = 0
        if len(control) <= degree or len(knots) < len(control) + degree + 1:
            marker.action = Marker.DELETE
            self.plan_pub.publish(marker)
            return
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 0.95
        marker.color.g = 0.48
        marker.color.b = 0.02
        marker.color.a = 0.95
        start, end = float(knots[degree]), float(knots[len(control)])
        for value in np.linspace(start, end, self.plan_samples):
            marker.points.append(Point(*self.de_boor(control, knots, degree, value)))
        self.plan_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("roofless_visualization")
    RooflessVisualization()
    rospy.spin()
