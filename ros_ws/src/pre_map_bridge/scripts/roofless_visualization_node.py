#!/usr/bin/env python3
"""Publish derived, balanced RViz layers for exploration visualization."""

import copy
from collections import deque
import math

import numpy as np
import rospy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
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
        self.multi_floor = bool(rospy.get_param("~multi_floor", False))
        requested_floor_layers = int(rospy.get_param("~floor_layer_count", 0))
        # Zero means fully automatic: publish as many layers as the observed
        # map supports. A positive value is an optional safety cap.
        self.floor_layer_limit = requested_floor_layers if requested_floor_layers > 0 else None
        self.floor_detection_bin = float(rospy.get_param("~floor_detection_bin", 0.10))
        self.floor_detection_min_separation = float(
            rospy.get_param("~floor_detection_min_separation", 1.0)
        )
        self.floor_detection_min_samples = max(
            20, int(rospy.get_param("~floor_detection_min_samples", 100))
        )
        self.multi_floor_z_min = float(rospy.get_param("~multi_floor_z_min", -1.0))
        self.multi_floor_z_max = float(rospy.get_param("~multi_floor_z_max", 10.0))
        if self.multi_floor_z_min >= self.multi_floor_z_max:
            rospy.logwarn("multi_floor_z_min must be below multi_floor_z_max; using -1..10 m")
            self.multi_floor_z_min, self.multi_floor_z_max = -1.0, 10.0
        self.floor_z = self.fallback_floor_z
        self.z_samples = deque(maxlen=20000)
        self.floor_surface_samples = np.empty(0, dtype=np.float64)
        self.floor_levels = []
        self.map_complete = False
        self.last_visible_cloud = None
        self.plan_samples = max(8, int(rospy.get_param("~plan_samples", 64)))
        self.cloud_pub = rospy.Publisher(
            "/pre_map_vln/roofless_map", PointCloud2, queue_size=1
        )
        self.floor_pubs = []
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
            "/pre_map_vln/exploration_status", String,
            self.status_callback, queue_size=1,
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
            "Envelope filter: floor +%.2f m, roof top %.2f m, min ceiling %.2f m; multi_floor=%s",
            self.floor_clearance,
            self.ceiling_thickness,
            self.ceiling_min_height,
            self.multi_floor,
        )
        if self.multi_floor:
            rospy.loginfo(
                "Multi-floor layers: %s topics /pre_map_vln/floor_1..floor_%s; "
                "automatic height clustering enabled",
                "auto" if self.floor_layer_limit is None else self.floor_layer_limit,
                "N",
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
        if self.multi_floor:
            self.z_samples.append(float(message.pose.pose.position.z))
            if len(self.z_samples) % 20 == 0:
                self.update_floor_levels()

    def status_callback(self, message):
        status = str(message.data).strip().lower()
        if status not in {"complete", "finished", "done"}:
            return
        self.map_complete = True
        if self.multi_floor:
            rospy.loginfo(
                "Exploration complete: final floor detection from %d surface samples and cached cloud=%s",
                len(self.floor_surface_samples),
                self.last_visible_cloud is not None,
            )
            self.update_floor_levels(self.floor_surface_samples)
            self.publish_floor_layers()

    def update_floor_levels(self, floor_samples=None):
        """Estimate discrete floor heights from the accumulated UAV trajectory.

        The lowest occupied point in each XY column is a floor-surface cue;
        trajectory heights provide a fallback while the map is still sparse.
        Histogram peaks are therefore tied to physical floors, while the point
        cloud itself remains untouched in multi-floor mode.
        """
        if not self.map_complete:
            return
        source = self.floor_surface_samples if floor_samples is None else floor_samples
        if len(source) < self.floor_detection_min_samples:
            source = np.asarray(self.z_samples, dtype=np.float64)
        if len(source) < self.floor_detection_min_samples:
            return
        values = np.asarray(source, dtype=np.float64)
        values = values[np.isfinite(values)]
        if len(values) < self.floor_detection_min_samples:
            return
        bin_size = max(0.05, self.floor_detection_bin)
        lower = math.floor(float(values.min()) / bin_size) * bin_size - bin_size
        upper = math.ceil(float(values.max()) / bin_size) * bin_size + bin_size
        edges = np.arange(lower, upper + bin_size * 0.5, bin_size)
        if len(edges) < 3:
            return
        histogram, _ = np.histogram(values, bins=edges)
        # Smooth over 0.6 m so small command oscillations do not create layers.
        window = max(3, int(round(0.6 / bin_size)))
        if window % 2 == 0:
            window += 1
        smoothed = np.convolve(histogram.astype(np.float64), np.ones(window), mode="same")
        if not np.any(smoothed):
            return
        threshold = max(20.0, float(smoothed.max()) * 0.08)
        candidates = [
            index for index in range(1, len(smoothed) - 1)
            if smoothed[index] >= smoothed[index - 1]
            and smoothed[index] >= smoothed[index + 1]
            and smoothed[index] >= threshold
        ]
        candidates.sort(key=lambda index: smoothed[index], reverse=True)
        min_bins = max(1, int(round(max(1.5, self.floor_detection_min_separation) / bin_size)))
        selected = []
        for index in candidates:
            if all(abs(index - other) >= min_bins for other in selected):
                selected.append(index)
            if self.floor_layer_limit is not None and len(selected) >= self.floor_layer_limit:
                break
        if not selected:
            return
        candidates_z = [float((edges[index] + edges[index + 1]) * 0.5) for index in selected]
        # Quantize final floor surfaces before publishing. A 0.25 m bin absorbs
        # voxel/sloped-floor noise while remaining far below floor separation.
        levels = sorted(round(candidate / 0.25) * 0.25 for candidate in candidates_z)
        if levels != self.floor_levels:
            self.floor_levels = levels
            rospy.loginfo(
                "Detected multi-floor levels (world Z, final floor surfaces): %s",
                ", ".join(f"{z:.2f}" for z in levels),
            )

    def ensure_floor_publishers(self, count):
        while len(self.floor_pubs) < count:
            index = len(self.floor_pubs)
            self.floor_pubs.append(rospy.Publisher(
                f"/pre_map_vln/floor_{index + 1}", PointCloud2, queue_size=1, latch=True
            ))

    def publish_floor_layers(self):
        if not self.multi_floor or not self.floor_levels or self.last_visible_cloud is None:
            return
        message, x_values, y_values, visible_z = self.last_visible_cloud
        self.ensure_floor_publishers(len(self.floor_levels))
        boundaries = [
            (self.floor_levels[index] + self.floor_levels[index + 1]) * 0.5
            for index in range(len(self.floor_levels) - 1)
        ]
        floor_indices = np.searchsorted(boundaries, visible_z, side="right")
        counts = []
        for index, publisher in enumerate(self.floor_pubs):
            mask = floor_indices == index
            counts.append(int(np.count_nonzero(mask)))
            if np.any(mask):
                publisher.publish(self.colored_cloud(
                    message, x_values[mask], y_values[mask], visible_z[mask],
                    self.multi_floor_z_min, self.multi_floor_z_max,
                ))
        rospy.loginfo(
            "Published %d floor layers: %s points",
            len(self.floor_levels), ", ".join(str(count) for count in counts),
        )

    @staticmethod
    def colored_cloud(message, x_values, y_values, z_values, z_min, z_max):
        """Create an RViz RGB8 cloud without changing the source XYZ values."""
        height_ratio = np.clip((z_values - z_min) / max(0.1, z_max - z_min), 0.0, 1.0)
        red = np.interp(height_ratio, (0.0, 0.55, 1.0), (0, 35, 235)).astype(np.uint32)
        green = np.interp(height_ratio, (0.0, 0.55, 1.0), (210, 70, 0)).astype(np.uint32)
        blue = np.interp(height_ratio, (0.0, 0.55, 1.0), (230, 210, 210)).astype(np.uint32)
        rgba = (np.uint32(255) << np.uint32(24)) | (red << 16) | (green << 8) | blue
        endian = ">" if message.is_bigendian else "<"
        colored = np.empty(
            len(z_values),
            dtype=[("x", endian + "f4"), ("y", endian + "f4"),
                   ("z", endian + "f4"), ("rgba", endian + "u4")],
        )
        colored["x"], colored["y"], colored["z"], colored["rgba"] = (
            x_values, y_values, z_values, rgba
        )
        output = PointCloud2()
        output.header = message.header
        output.height, output.width = 1, len(colored)
        output.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgba", 12, PointField.UINT32, 1),
        ]
        output.is_bigendian = message.is_bigendian
        output.point_step, output.row_step = 16, 16 * output.width
        output.data, output.is_dense = colored.tobytes(), True
        return output

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

        if self.multi_floor:
            # Multi-floor mode deliberately preserves every finite XYZ point.
            # The old floor/ceiling envelope is only appropriate for a single
            # active floor and would hide other levels after a Z transition.
            visible = np.ones(len(z_values), dtype=bool)
            # Computing a per-column minimum is relatively expensive for the
            # growing map. Defer it until the terminal status has arrived; the
            # trajectory remains available as a lightweight fallback, and the
            # final cloud callback refines the levels from actual floor points.
            if self.map_complete:
                x_cells = np.rint(x_values / self.voxel_resolution).astype(np.int64)
                y_cells = np.rint(y_values / self.voxel_resolution).astype(np.int64)
                y_min, y_span = int(y_cells.min()), int(y_cells.max() - y_cells.min() + 1)
                column_keys = (x_cells - int(x_cells.min())) * y_span + (y_cells - y_min)
                _, inverse = np.unique(column_keys, return_inverse=True)
                minima = np.full(int(inverse.max()) + 1, np.inf, dtype=np.float32)
                np.minimum.at(minima, inverse, z_values)
                self.floor_surface_samples = minima[np.isfinite(minima)].astype(np.float64)
                self.update_floor_levels(self.floor_surface_samples)
        else:
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
        if self.multi_floor:
            output = self.colored_cloud(
                message, x_values[visible], y_values[visible], visible_z,
                self.multi_floor_z_min, self.multi_floor_z_max,
            )
        else:
            height_span = max(0.1, self.ceiling_min_height - self.floor_clearance)
            output = self.colored_cloud(
                message, x_values[visible], y_values[visible], visible_z,
                self.floor_z + self.floor_clearance,
                self.floor_z + self.floor_clearance + height_span,
            )
        self.cloud_pub.publish(output)
        if self.multi_floor:
            self.last_visible_cloud = (
                message, x_values[visible], y_values[visible], visible_z
            )
        if self.multi_floor:
            self.publish_floor_layers()
        rospy.loginfo_throttle(
            5.0,
            "%s cloud: %d -> %d points (floor %.2f m)",
            "Multi-floor" if self.multi_floor else "Roofless",
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
