#!/usr/bin/env python3
"""Publish a roofless occupied cloud and the currently active FALCON B-spline."""

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from trajectory.msg import Bspline
from visualization_msgs.msg import Marker


class RooflessVisualization:
    def __init__(self):
        self.fallback_floor_z = float(rospy.get_param("~floor_z", 0.0))
        self.sensor_height = float(rospy.get_param("~sensor_height", 1.0))
        self.floor_clearance = float(rospy.get_param("~floor_clearance", 0.25))
        self.ceiling_min_height = float(rospy.get_param("~ceiling_min_height", 1.65))
        self.ceiling_thickness = float(rospy.get_param("~ceiling_thickness", 0.60))
        self.voxel_resolution = float(rospy.get_param("~voxel_resolution", 0.10))
        self.floor_z = self.fallback_floor_z
        self.plan_samples = max(8, int(rospy.get_param("~plan_samples", 64)))
        self.cloud_pub = rospy.Publisher(
            "/pre_map_vln/roofless_map", PointCloud2, queue_size=1
        )
        self.plan_pub = rospy.Publisher(
            "/pre_map_vln/current_plan", Marker, queue_size=1, latch=True
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
        rospy.loginfo(
            "Envelope filter: floor +%.2f m, roof top %.2f m, min ceiling %.2f m",
            self.floor_clearance,
            self.ceiling_thickness,
            self.ceiling_min_height,
        )

    def odom_callback(self, message):
        self.floor_z = float(message.pose.pose.position.z) - self.sensor_height

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
