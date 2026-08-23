#!/usr/bin/env python3
"""Publish a roofless occupied cloud and the currently active FALCON B-spline."""

import copy

import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from trajectory.msg import Bspline
from visualization_msgs.msg import Marker


class RooflessVisualization:
    def __init__(self):
        self.min_z = float(rospy.get_param("~min_z", 0.50))
        self.max_z = float(rospy.get_param("~max_z", 2.25))
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
        rospy.loginfo(
            "Roofless visualization keeps %.2f <= world Z <= %.2f m",
            self.min_z,
            self.max_z,
        )

    def cloud_callback(self, message):
        z_fields = [field for field in message.fields if field.name == "z"]
        if len(z_fields) != 1 or z_fields[0].datatype != PointField.FLOAT32:
            rospy.logwarn_throttle(5.0, "Cannot filter cloud: z is not FLOAT32")
            return
        if message.point_step <= 0 or not message.data:
            return
        point_count = len(message.data) // message.point_step
        endian = ">f4" if message.is_bigendian else "<f4"
        z_values = np.ndarray(
            shape=(point_count,),
            dtype=endian,
            buffer=message.data,
            offset=z_fields[0].offset,
            strides=(message.point_step,),
        )
        keep = np.isfinite(z_values) & (z_values >= self.min_z) & (z_values <= self.max_z)
        packed = np.frombuffer(message.data, dtype=np.uint8).reshape(
            point_count, message.point_step
        )
        filtered = np.ascontiguousarray(packed[keep])
        output = copy.copy(message)
        output.height = 1
        output.width = int(filtered.shape[0])
        output.row_step = output.point_step * output.width
        output.data = filtered.tobytes()
        output.is_dense = True
        self.cloud_pub.publish(output)

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
        marker.color.r = 1.0
        marker.color.g = 0.76
        marker.color.b = 0.08
        marker.color.a = 0.95
        start, end = float(knots[degree]), float(knots[len(control)])
        for value in np.linspace(start, end, self.plan_samples):
            marker.points.append(Point(*self.de_boor(control, knots, degree, value)))
        self.plan_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("roofless_visualization")
    RooflessVisualization()
    rospy.spin()
