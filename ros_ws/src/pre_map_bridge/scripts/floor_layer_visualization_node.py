#!/usr/bin/env python3
"""Publish a selectable, true-coordinate multi-floor map for RViz."""

import csv
import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
    MarkerArray,
)


PALETTE = (
    (0, 174, 239),
    (138, 43, 226),
    (0, 166, 118),
    (245, 124, 0),
    (211, 47, 47),
    (70, 90, 110),
)


def packed_rgba(red, green, blue, alpha=255):
    return (
        (np.uint32(alpha) << np.uint32(24))
        | (np.uint32(red) << np.uint32(16))
        | (np.uint32(green) << np.uint32(8))
        | np.uint32(blue)
    )


class FloorLayerVisualization:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.cloud_path = Path(rospy.get_param("~cloud_path"))
        self.boxes_path = Path(rospy.get_param("~boxes_path"))
        self.trajectory_path = Path(rospy.get_param("~trajectory_path"))
        self.floor_json_path = Path(rospy.get_param("~floor_json_path", ""))
        self.floor_detection_mode = rospy.get_param("~floor_detection_mode", "falcon")
        self.selected_floor = int(rospy.get_param("~selected_floor", 0))
        self.max_floor = int(rospy.get_param("~max_floor", 0))
        self.voxel_resolution = float(rospy.get_param("~voxel_resolution", 0.10))
        self.minimum_slab_area = float(rospy.get_param("~minimum_slab_area", 2.0))

        self.active_cloud_pub = rospy.Publisher(
            "/pre_map_vln/floor_active_map", PointCloud2, queue_size=1, latch=True
        )
        self.context_cloud_pub = rospy.Publisher(
            "/pre_map_vln/floor_context_map", PointCloud2, queue_size=1, latch=True
        )
        self.box_pub = rospy.Publisher(
            "/pre_map_vln/floor_box_markers", MarkerArray, queue_size=1, latch=True
        )
        self.trajectory_pub = rospy.Publisher(
            "/pre_map_vln/floor_trajectory", MarkerArray, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/pre_map_vln/floor_view_status", Marker, queue_size=1, latch=True
        )

        self.points = self.load_pcd(self.cloud_path)
        self.trajectory = self.load_trajectory(self.trajectory_path)
        if self.floor_detection_mode == "comparison_json":
            self.floor_data = self.load_floors(self.floor_json_path)
        elif self.floor_detection_mode == "falcon":
            self.floor_data = self.detect_falcon_floors(self.points, self.trajectory)
        else:
            raise ValueError(
                f"Unsupported floor_detection_mode: {self.floor_detection_mode}"
            )
        self.floor_z = np.asarray(
            [item["falcon_floor_z_m"] for item in self.floor_data], dtype=np.float32
        )
        if not len(self.floor_z):
            raise RuntimeError("No floors were detected")
        self.visible_floor_count = (
            len(self.floor_z)
            if self.max_floor <= 0
            else min(self.max_floor, len(self.floor_z))
        )
        if self.selected_floor > self.visible_floor_count:
            raise ValueError(
                f"selected_floor={self.selected_floor} exceeds visible floor count "
                f"{self.visible_floor_count}"
            )
        self.boundaries = self.floor_boundaries(self.floor_z)
        self.point_floors = self.assign_floors(self.points[:, 2])
        self.structure_visible_mask, self.slab_bands = self.adaptive_structure_mask(
            self.points
        )
        self.boxes = self.load_boxes(self.boxes_path)
        self.trajectory_floors = (
            self.assign_floors(self.trajectory[:, 2])
            if len(self.trajectory)
            else np.empty(0, dtype=np.int32)
        )

        self.selector = InteractiveMarkerServer("stage1_floor_selector")
        self.create_selector()
        self.publish_view()
        rospy.loginfo(
            "Floor viewer ready: %d points, %d boxes, %d trajectory points, %d levels",
            len(self.points), len(self.boxes), len(self.trajectory), len(self.floor_z),
        )
        rospy.loginfo(
            "Floor planes (%s): %s",
            self.floor_detection_mode,
            ", ".join(f"L{i + 1}={z:.2f}m" for i, z in enumerate(self.floor_z)),
        )
        rospy.loginfo(
            "Adaptive slab bands: %s",
            ", ".join(
                f"L{i + 1}=[{lower:.2f},{upper:.2f}]m"
                for i, (lower, upper) in enumerate(self.slab_bands)
            ),
        )

    @staticmethod
    def load_floors(path):
        with path.open(encoding="utf-8") as stream:
            document = json.load(stream)
        floors = document.get("floor_projection", {}).get("floors", [])
        return sorted(floors, key=lambda item: float(item["falcon_floor_z_m"]))

    @staticmethod
    def detect_falcon_floors(points, trajectory):
        """Infer floors using only executed FALCON trajectory and occupied voxels.

        Long-dwell trajectory height modes identify visited levels. Each mode is
        aligned to the strongest occupied-map horizontal slice below it. This
        does not consume Habitat navmesh, mesh geometry, or semantic truth.
        """
        if len(trajectory) < 10:
            raise RuntimeError("FALCON-only floor detection needs a trajectory")
        trajectory_z = trajectory[:, 2]
        trajectory_z = trajectory_z[np.isfinite(trajectory_z)]
        point_z = points[:, 2]
        point_z = point_z[np.isfinite(point_z)]
        if not len(trajectory_z) or not len(point_z):
            raise RuntimeError("Floor detection received no finite z values")

        bin_width = 0.10
        lower = math.floor(float(trajectory_z.min()) / bin_width) * bin_width
        upper = math.ceil(float(trajectory_z.max()) / bin_width) * bin_width
        edges = np.arange(lower, upper + 1.5 * bin_width, bin_width)
        histogram, _ = np.histogram(trajectory_z, edges)
        centers = (edges[:-1] + edges[1:]) * 0.5
        smooth = np.convolve(histogram, np.ones(5, dtype=np.float64), mode="same")
        peak_minimum = max(0.01 * len(trajectory_z), 0.12 * float(smooth.max()))
        order = np.argsort(smooth)[::-1]
        flight_levels = []
        for index in order:
            if smooth[index] < peak_minimum:
                break
            height = float(centers[index])
            if any(abs(height - accepted) < 1.80 for accepted in flight_levels):
                continue
            flight_levels.append(height)
        flight_levels.sort()
        if not flight_levels:
            raise RuntimeError("No persistent flight-height modes were detected")

        occupied_bins = np.round(point_z / bin_width) * bin_width
        occupied_z, occupied_counts = np.unique(occupied_bins, return_counts=True)
        global_support = int(occupied_counts.max())
        floors = []
        for flight_z in flight_levels:
            # The flight height is not assumed fixed. Search a broad clearance
            # interval and let the occupied-map support select the floor plane.
            candidates = np.flatnonzero(
                (occupied_z >= flight_z - 1.60)
                & (occupied_z <= flight_z - 0.35)
                & (occupied_counts >= 0.20 * global_support)
            )
            if not len(candidates):
                continue
            best = int(candidates[np.argmax(occupied_counts[candidates])])
            floor_z = float(occupied_z[best])
            support = int(occupied_counts[best])
            if any(abs(floor_z - item["falcon_floor_z_m"]) < 1.50 for item in floors):
                continue
            floors.append({
                "falcon_floor_z_m": floor_z,
                "flight_height_mode_m": flight_z,
                "falcon_floor_support_points": support,
                "detection_source": "falcon_trajectory+falcon_occupied_pointcloud",
            })
        floors.sort(key=lambda item: item["falcon_floor_z_m"])
        if not floors:
            raise RuntimeError("Trajectory modes did not match occupied-map floor planes")
        return floors

    @staticmethod
    def floor_boundaries(floor_z):
        # Assign every point to a level while retaining the full point cloud.
        # A new level starts slightly below its detected floor slab.
        if len(floor_z) == 1:
            return np.empty(0, dtype=np.float32)
        return floor_z[1:] - np.float32(0.30)

    def assign_floors(self, z_values):
        return np.searchsorted(self.boundaries, z_values, side="right").astype(np.int32) + 1

    def adaptive_structure_mask(self, points):
        """Remove broad horizontal slabs while retaining vertical structure.

        The occupied map is voxelized once. A slab candidate needs broad local
        XY support, limited vertical continuity, and strong map-wide support at
        a detected floor plane. The selected band gets one extra voxel of
        display-only margin for a cleaner cut.
        """
        resolution = self.voxel_resolution
        grid_min = np.rint(points.min(axis=0) / resolution).astype(np.int32)
        grid_ids = np.rint(points / resolution).astype(np.int32) - grid_min
        shape = tuple((grid_ids.max(axis=0) + 1).tolist())
        occupied = np.zeros(shape, dtype=bool)
        occupied[tuple(grid_ids.T)] = True

        horizontal = np.zeros(shape, dtype=np.uint8)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                source_x = slice(max(0, dx), min(shape[0], shape[0] + dx))
                target_x = slice(max(0, -dx), min(shape[0], shape[0] - dx))
                source_y = slice(max(0, dy), min(shape[1], shape[1] + dy))
                target_y = slice(max(0, -dy), min(shape[1], shape[1] - dy))
                horizontal[target_x, target_y, :] += occupied[
                    source_x, source_y, :
                ]

        vertical = np.zeros(shape, dtype=np.uint8)
        vertical_radius = max(4, int(round(0.80 / resolution)))
        for dz in range(-vertical_radius, vertical_radius + 1):
            source_z = slice(max(0, dz), min(shape[2], shape[2] + dz))
            target_z = slice(max(0, -dz), min(shape[2], shape[2] - dz))
            vertical[:, :, target_z] += occupied[:, :, source_z]

        # A 0.4 m square neighborhood contains 25 cells at 0.1 m resolution.
        # Sparse slab fringes can have only four occupied neighbors on this
        # map's interleaved voxel lattice. Walls and columns are still
        # protected by their longer vertical runs.
        plane_candidates = occupied & (horizontal >= 4) & (vertical <= 7)
        support = plane_candidates.sum(axis=(0, 1))
        z_values = (np.arange(shape[2]) + grid_min[2]) * resolution
        minimum_cells = max(20, int(round(self.minimum_slab_area / resolution**2)))
        slab_bins = np.zeros(shape[2], dtype=bool)
        bands = []
        search_bins = max(3, int(round(0.60 / resolution)))
        peak_radius = max(1, int(round(0.30 / resolution)))
        margin_bins = max(1, int(round(0.10 / resolution)))

        for floor_z in self.floor_z:
            center = int(np.argmin(np.abs(z_values - floor_z)))
            near = np.arange(
                max(0, center - peak_radius), min(shape[2], center + peak_radius + 1)
            )
            peak = int(near[np.argmax(support[near])])
            threshold = max(minimum_cells, 0.18 * float(support[peak]))
            window = np.arange(
                max(0, peak - search_bins), min(shape[2], peak + search_bins + 1)
            )
            strong = window[support[window] >= threshold]
            if not len(strong):
                lower = upper = peak
            else:
                lower = max(0, int(strong.min()) - margin_bins)
                upper = min(shape[2] - 1, int(strong.max()) + margin_bins)
            slab_bins[lower : upper + 1] = True
            bands.append((float(z_values[lower]), float(z_values[upper])))

        remove_voxels = plane_candidates & slab_bins[None, None, :]
        remove_points = remove_voxels[tuple(grid_ids.T)]
        rospy.loginfo(
            "Adaptive slab filter removed %d/%d displayed source points (%.1f%%)",
            int(remove_points.sum()), len(points), 100.0 * float(remove_points.mean()),
        )
        return ~remove_points, bands

    @staticmethod
    def load_pcd(path):
        data_line = None
        with path.open("rb") as stream:
            for index, line in enumerate(stream):
                if line.strip().lower() == b"data ascii":
                    data_line = index + 1
                    break
        if data_line is None:
            raise RuntimeError(f"Only ASCII PCD is supported: {path}")
        points = np.loadtxt(path, dtype=np.float32, skiprows=data_line, ndmin=2)
        if points.shape[1] < 3:
            raise RuntimeError(f"PCD has fewer than three fields: {path}")
        return points[:, :3]

    @staticmethod
    def load_boxes(path):
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        boxes = []
        for row in rows:
            boxes.append({
                "name": row["name"],
                "prob": float(row["prob"]),
                "center": np.asarray([
                    float(row["tx_world_object"]),
                    float(row["ty_world_object"]),
                    float(row["tz_world_object"]),
                ], dtype=np.float32),
                "quaternion": np.asarray([
                    float(row["qw_world_object"]),
                    float(row["qx_world_object"]),
                    float(row["qy_world_object"]),
                    float(row["qz_world_object"]),
                ], dtype=np.float32),
                "scale": np.asarray([
                    float(row["scale_x"]), float(row["scale_y"]), float(row["scale_z"])
                ], dtype=np.float32),
            })
        return boxes

    @staticmethod
    def load_trajectory(path):
        if not path.is_file():
            return np.empty((0, 3), dtype=np.float32)
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        return np.asarray(
            [[float(row[axis]) for axis in ("x", "y", "z")] for row in rows],
            dtype=np.float32,
        )

    def create_selector(self):
        lower = self.points.min(axis=0)
        upper = self.points.max(axis=0)
        origin = np.asarray([lower[0], lower[1] - 1.0, upper[2] + 0.8], dtype=np.float32)
        labels = ["ALL"] + [
            f"L{index}" for index in range(1, self.visible_floor_count + 1)
        ]
        for floor_id, label in enumerate(labels):
            interactive = InteractiveMarker()
            interactive.header.frame_id = self.frame_id
            interactive.name = f"floor_{floor_id}"
            interactive.description = label
            interactive.scale = 0.75
            interactive.pose.position.x = float(origin[0] + floor_id * 0.95)
            interactive.pose.position.y = float(origin[1])
            interactive.pose.position.z = float(origin[2])

            control = InteractiveMarkerControl()
            control.name = "select"
            control.interaction_mode = InteractiveMarkerControl.BUTTON
            control.always_visible = True
            cube = Marker()
            cube.type = Marker.CUBE
            cube.scale.x = 0.65
            cube.scale.y = 0.45
            cube.scale.z = 0.22
            cube.color.r = 0.10
            cube.color.g = 0.48
            cube.color.b = 0.82
            cube.color.a = 0.92
            control.markers.append(cube)
            interactive.controls.append(control)
            self.selector.insert(interactive, self.selector_feedback)
        self.selector.applyChanges()

    def selector_feedback(self, feedback):
        try:
            floor_id = int(feedback.marker_name.rsplit("_", 1)[1])
        except (ValueError, IndexError):
            return
        if floor_id < 0 or floor_id > self.visible_floor_count:
            return
        self.selected_floor = floor_id
        rospy.set_param("~selected_floor", floor_id)
        self.publish_view()

    def make_cloud(self, points, floor_ids, context=False):
        colored = np.empty(
            len(points),
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgba", "<u4")],
        )
        if len(points):
            colored["x"], colored["y"], colored["z"] = points.T
            if context:
                colored["rgba"] = packed_rgba(145, 150, 158)
            elif self.selected_floor > 0:
                floor_z = float(self.slab_bands[self.selected_floor - 1][1])
                if self.selected_floor < len(self.floor_z):
                    ceiling_z = float(self.slab_bands[self.selected_floor][0])
                else:
                    ceiling_z = floor_z + 2.50
                height_ratio = np.clip(
                    (points[:, 2] - floor_z) / max(0.5, ceiling_z - floor_z),
                    0.0,
                    1.0,
                )
                red = np.interp(height_ratio, (0.0, 0.55, 1.0), (0, 35, 235)).astype(np.uint32)
                green = np.interp(height_ratio, (0.0, 0.55, 1.0), (210, 70, 0)).astype(np.uint32)
                blue = np.interp(height_ratio, (0.0, 0.55, 1.0), (230, 210, 210)).astype(np.uint32)
                colored["rgba"] = (
                    (np.uint32(255) << np.uint32(24))
                    | (red << np.uint32(16))
                    | (green << np.uint32(8))
                    | blue
                )
            else:
                colors = np.asarray(PALETTE, dtype=np.uint32)
                selected = colors[(floor_ids - 1) % len(colors)]
                colored["rgba"] = (
                    (np.uint32(255) << np.uint32(24))
                    | (selected[:, 0] << np.uint32(16))
                    | (selected[:, 1] << np.uint32(8))
                    | selected[:, 2]
                )
        message = PointCloud2()
        message.header.frame_id = self.frame_id
        message.header.stamp = rospy.Time.now()
        message.height = 1
        message.width = len(colored)
        message.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgba", 12, PointField.UINT32, 1),
        ]
        message.point_step = 16
        message.row_step = 16 * len(colored)
        message.data = colored.tobytes()
        message.is_dense = True
        return message

    @staticmethod
    def label_color(label):
        digest = hashlib.sha1(label.encode("utf-8")).digest()
        return tuple(0.25 + 0.70 * channel / 255.0 for channel in digest[:3])

    @staticmethod
    def quaternion_matrix(quaternion):
        w, x, y, z = quaternion.astype(np.float64)
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-9:
            return np.eye(3)
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        return np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def box_markers(self):
        result = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        result.markers.append(delete)
        edges = ((0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7))
        marker_id = 1
        for box in self.boxes:
            floor_id = int(self.assign_floors(np.asarray([box["center"][2]]))[0])
            if floor_id > self.visible_floor_count:
                continue
            active = self.selected_floor == 0 or floor_id == self.selected_floor
            rotation = self.quaternion_matrix(box["quaternion"])
            half = np.maximum(box["scale"], 0.02) * 0.5
            local = np.asarray([
                [sx * half[0], sy * half[1], sz * half[2]]
                for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)
            ])
            corners = local @ rotation.T + box["center"]
            outline = Marker()
            outline.header.frame_id = self.frame_id
            outline.header.stamp = rospy.Time.now()
            outline.ns = "active_boxes" if active else "context_boxes"
            outline.id = marker_id
            marker_id += 1
            outline.type = Marker.LINE_LIST
            outline.action = Marker.ADD
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.045 if active else 0.025
            color = self.label_color(box["name"]) if active else (0.48, 0.50, 0.54)
            outline.color.r, outline.color.g, outline.color.b = color
            outline.color.a = 0.96 if active else 0.16
            for start, end in edges:
                outline.points.append(Point(*corners[start]))
                outline.points.append(Point(*corners[end]))
            result.markers.append(outline)

            if active:
                label = Marker()
                label.header = outline.header
                label.ns = "active_labels"
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position = Point(
                    float(box["center"][0]), float(box["center"][1]),
                    float(box["center"][2] + half[2] + 0.12),
                )
                label.pose.orientation.w = 1.0
                label.scale.z = 0.16
                label.color.r = label.color.g = label.color.b = 0.08
                label.color.a = 0.92
                label.text = f'{box["name"]} {box["prob"]:.2f} | L{floor_id}'
                result.markers.append(label)
        return result

    def trajectory_markers(self):
        result = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        result.markers.append(delete)
        for active, name, color, alpha, width in (
            (False, "context_trajectory", (0.55, 0.57, 0.60), 0.18, 0.025),
            (True, "active_trajectory", (0.86, 0.08, 0.08), 0.95, 0.055),
        ):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = name
            marker.id = 1 if active else 0
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = alpha
            for index in range(max(0, len(self.trajectory) - 1)):
                segment_floor = int(self.trajectory_floors[index])
                if segment_floor > self.visible_floor_count:
                    continue
                is_active = self.selected_floor == 0 or segment_floor == self.selected_floor
                if is_active != active:
                    continue
                marker.points.append(Point(*self.trajectory[index]))
                marker.points.append(Point(*self.trajectory[index + 1]))
            result.markers.append(marker)
        return result

    def publish_status(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "floor_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.points[:, 0].min())
        marker.pose.position.y = float(self.points[:, 1].min() - 1.0)
        marker.pose.position.z = float(self.points[:, 2].max() + 1.55)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.32
        marker.color.r = marker.color.g = marker.color.b = 0.05
        marker.color.a = 0.96
        marker.text = (
            "All floors" if self.selected_floor == 0
            else f"Level {self.selected_floor} highlighted | true coordinates"
        )
        marker.text += " | FALCON map-derived levels"
        if self.visible_floor_count < len(self.floor_z):
            marker.text += f" | showing L1-L{self.visible_floor_count}"
        self.status_pub.publish(marker)

    def publish_view(self):
        visible_mask = (
            (self.point_floors <= self.visible_floor_count)
            & self.structure_visible_mask
        )
        active_mask = (
            visible_mask
            if self.selected_floor == 0
            else visible_mask & (self.point_floors == self.selected_floor)
        )
        context_mask = visible_mask & ~active_mask
        self.active_cloud_pub.publish(
            self.make_cloud(self.points[active_mask], self.point_floors[active_mask])
        )
        self.context_cloud_pub.publish(
            self.make_cloud(
                self.points[context_mask], self.point_floors[context_mask], context=True
            )
        )
        self.box_pub.publish(self.box_markers())
        self.trajectory_pub.publish(self.trajectory_markers())
        self.publish_status()
        rospy.loginfo("Selected floor view: %s", "ALL" if self.selected_floor == 0 else self.selected_floor)


if __name__ == "__main__":
    rospy.init_node("floor_layer_visualization")
    FloorLayerVisualization()
    rospy.spin()
