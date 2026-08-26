#!/usr/bin/env python3
"""Publish paper-oriented room and semantic-box markers from a scene graph.

This node is deliberately visualization-only: polygon simplification/smoothing is
applied to RViz markers and never written back to the scene graph used by the
planner.
"""

import json
import math
from pathlib import Path

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


ROOM_PALETTE = (
    (0.20, 0.47, 0.78),
    (0.90, 0.46, 0.13),
    (0.20, 0.64, 0.43),
    (0.65, 0.40, 0.76),
    (0.92, 0.70, 0.12),
    (0.10, 0.65, 0.70),
    (0.83, 0.34, 0.43),
    (0.43, 0.50, 0.62),
)

BOX_PALETTE = (
    (0.12, 0.36, 0.72),
    (0.82, 0.25, 0.20),
    (0.08, 0.56, 0.38),
    (0.63, 0.30, 0.72),
    (0.91, 0.55, 0.08),
    (0.05, 0.58, 0.66),
    (0.76, 0.24, 0.48),
)


def stable_palette_color(text, palette):
    value = sum((index + 1) * byte for index, byte in enumerate(text.encode("utf-8")))
    return palette[value % len(palette)]


def point_segment_distance(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def simplify_closed_polygon(points, tolerance):
    """Remove raster stair-step vertices while preserving the closed shape."""
    result = [(float(point[0]), float(point[1])) for point in points]
    if len(result) > 1 and result[0] == result[-1]:
        result.pop()
    changed = True
    while changed and len(result) > 3:
        changed = False
        kept = []
        count = len(result)
        for index, point in enumerate(result):
            previous = result[(index - 1) % count]
            following = result[(index + 1) % count]
            if point_segment_distance(point, previous, following) <= tolerance:
                changed = True
            else:
                kept.append(point)
        if len(kept) < 3:
            break
        result = kept
    return result


def chaikin_closed(points, iterations):
    result = list(points)
    for _ in range(max(0, iterations)):
        refined = []
        for index, start in enumerate(result):
            end = result[(index + 1) % len(result)]
            refined.append((0.75 * start[0] + 0.25 * end[0], 0.75 * start[1] + 0.25 * end[1]))
            refined.append((0.25 * start[0] + 0.75 * end[0], 0.25 * start[1] + 0.75 * end[1]))
        result = refined
    return result


def signed_area(points):
    return 0.5 * sum(
        points[index][0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * points[index][1]
        for index in range(len(points))
    )


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def point_in_triangle(point, a, b, c):
    first = cross(a, b, point)
    second = cross(b, c, point)
    third = cross(c, a, point)
    return (first >= -1e-9 and second >= -1e-9 and third >= -1e-9) or (
        first <= 1e-9 and second <= 1e-9 and third <= 1e-9
    )


def point_in_polygon(point, polygon):
    """Return whether a point lies inside a simple polygon (ray casting)."""
    x_value, y_value = float(point[0]), float(point[1])
    inside = False
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        if (current[1] > y_value) == (previous[1] > y_value):
            continue
        crossing_x = (
            (previous[0] - current[0])
            * (y_value - current[1])
            / (previous[1] - current[1])
            + current[0]
        )
        if x_value < crossing_x:
            inside = not inside
    return inside


def triangulate_polygon(points):
    """Triangulate a simple polygon for a translucent RViz floor marker."""
    if len(points) < 3:
        return []
    vertices = list(range(len(points)))
    if signed_area(points) < 0.0:
        vertices.reverse()
    triangles = []
    guard = len(vertices) * len(vertices)
    while len(vertices) > 3 and guard > 0:
        guard -= 1
        ear_found = False
        for offset, current in enumerate(vertices):
            previous = vertices[offset - 1]
            following = vertices[(offset + 1) % len(vertices)]
            a, b, c = points[previous], points[current], points[following]
            if cross(a, b, c) <= 1e-10:
                continue
            if any(
                point_in_triangle(points[candidate], a, b, c)
                for candidate in vertices
                if candidate not in (previous, current, following)
            ):
                continue
            triangles.append((a, b, c))
            del vertices[offset]
            ear_found = True
            break
        if not ear_found:
            return []
    if len(vertices) == 3:
        triangles.append(tuple(points[index] for index in vertices))
    return triangles


def box_edges(size):
    sx, sy, sz = (0.5 * max(0.01, float(value)) for value in size)
    corners = [
        (-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),
        (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz),
    ]
    indices = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
               (0, 4), (1, 5), (2, 6), (3, 7))
    return [(corners[start], corners[end]) for start, end in indices]


def box_footprint_world(obj):
    center_x, center_y = [float(value) for value in obj["center_xyz_m"][:2]]
    size_x, size_y = [float(value) for value in obj["size_xyz_m"][:2]]
    w_value, x_value, y_value, z_value = [
        float(value) for value in obj.get("orientation_wxyz", [1, 0, 0, 0])
    ]
    yaw = math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    result = []
    for local_x, local_y in (
        (-0.5 * size_x, -0.5 * size_y),
        (0.5 * size_x, -0.5 * size_y),
        (0.5 * size_x, 0.5 * size_y),
        (-0.5 * size_x, 0.5 * size_y),
    ):
        result.append(
            (
                center_x + cosine * local_x - sine * local_y,
                center_y + sine * local_x + cosine * local_y,
            )
        )
    return result


class PaperMapVisualization:
    def __init__(self):
        graph_path = Path(rospy.get_param("~scene_graph_path"))
        self.simplify_tolerance = float(rospy.get_param("~room_simplify_tolerance", 0.09))
        self.smoothing_iterations = int(rospy.get_param("~room_smoothing_iterations", 2))
        self.room_z = float(rospy.get_param("~room_z", 0.06))
        self.room_line_width = float(rospy.get_param("~room_line_width", 0.075))
        self.room_fill_alpha = float(rospy.get_param("~room_fill_alpha", 0.10))
        self.box_line_width = float(rospy.get_param("~box_line_width", 0.045))
        self.graph = json.loads(graph_path.read_text(encoding="utf-8"))
        self.objects_by_room = {}
        for room in self.graph.get("rooms", []):
            polygon = room.get("polygon_xy_m", [])
            self.objects_by_room[room.get("id")] = [
                obj
                for obj in room.get("objects", [])
                if point_in_polygon(obj["center_xyz_m"][:2], polygon)
            ]
        self.room_pub = rospy.Publisher(
            "/pre_map_vln/paper_rooms", MarkerArray, queue_size=1, latch=True
        )
        self.box_pub = rospy.Publisher(
            "/pre_map_vln/paper_boxes", MarkerArray, queue_size=1, latch=True
        )
        rospy.Timer(rospy.Duration(0.5), self.publish_once, oneshot=True)
        rospy.loginfo("Paper map visualization loaded %s", graph_path)

    @staticmethod
    def set_color(marker, color, alpha):
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha

    def room_markers(self):
        output = MarkerArray()
        for index, room in enumerate(self.graph.get("rooms", [])):
            raw_polygon = room.get("polygon_xy_m", [])
            if room.get("space_role") == "room":
                envelope_points = [tuple(map(float, point)) for point in raw_polygon]
                for obj in self.objects_by_room.get(room.get("id"), []):
                    envelope_points.extend(box_footprint_world(obj))
                x_values = [point[0] for point in envelope_points]
                y_values = [point[1] for point in envelope_points]
                padding = 0.08
                polygon = [
                    (min(x_values) - padding, min(y_values) - padding),
                    (max(x_values) + padding, min(y_values) - padding),
                    (max(x_values) + padding, max(y_values) + padding),
                    (min(x_values) - padding, max(y_values) + padding),
                ]
                smooth = polygon
            else:
                polygon = simplify_closed_polygon(raw_polygon, self.simplify_tolerance)
                smooth = chaikin_closed(polygon, self.smoothing_iterations)
            if len(polygon) < 3:
                continue
            color = ROOM_PALETTE[index % len(ROOM_PALETTE)]

            fill = Marker()
            fill.header.frame_id = "world"
            fill.ns = "paper_room_fills"
            fill.id = index
            fill.type = Marker.TRIANGLE_LIST
            fill.action = Marker.ADD
            fill.pose.orientation.w = 1.0
            self.set_color(fill, color, self.room_fill_alpha)
            for triangle in triangulate_polygon(smooth):
                for x, y in triangle:
                    fill.points.append(Point(x, y, self.room_z))
            if fill.points:
                output.markers.append(fill)

            outline = Marker()
            outline.header.frame_id = "world"
            outline.ns = "paper_room_outlines"
            outline.id = index
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose.orientation.w = 1.0
            outline.scale.x = self.room_line_width
            self.set_color(outline, color, 0.96)
            for x, y in smooth + smooth[:1]:
                outline.points.append(Point(x, y, self.room_z + 0.015))
            output.markers.append(outline)

            centroid = room.get("centroid_xy_m")
            if centroid:
                label = Marker()
                label.header.frame_id = "world"
                label.ns = "paper_room_labels"
                label.id = index
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position = Point(float(centroid[0]), float(centroid[1]), self.room_z + 0.08)
                label.pose.orientation.w = 1.0
                label.scale.z = 0.28
                self.set_color(label, tuple(channel * 0.62 for channel in color), 1.0)
                room_type = str(room.get("semantic_type", "unknown")).replace("_", " ").title()
                label.text = "R{}  {}".format(room.get("id", index), room_type)
                output.markers.append(label)
        return output

    def box_markers(self):
        output = MarkerArray()
        objects = []
        seen = set()
        rejected = 0
        for room in self.graph.get("rooms", []):
            kept = self.objects_by_room.get(room.get("id"), [])
            rejected += len(room.get("objects", [])) - len(kept)
            for obj in kept:
                if obj.get("id") not in seen:
                    objects.append(obj)
                    seen.add(obj.get("id"))

        for index, obj in enumerate(objects):
            center = [float(value) for value in obj["center_xyz_m"]]
            size = [float(value) for value in obj["size_xyz_m"]]
            orientation = [float(value) for value in obj.get("orientation_wxyz", [1, 0, 0, 0])]
            label_text = str(obj.get("label", "object"))
            color = stable_palette_color(label_text, BOX_PALETTE)

            fill = Marker()
            fill.header.frame_id = "world"
            fill.ns = "paper_box_fills"
            fill.id = index
            fill.type = Marker.CUBE
            fill.action = Marker.ADD
            fill.pose.position = Point(*center)
            fill.pose.orientation.w, fill.pose.orientation.x, fill.pose.orientation.y, fill.pose.orientation.z = orientation
            fill.scale.x, fill.scale.y, fill.scale.z = size
            self.set_color(fill, color, 0.075)
            output.markers.append(fill)

            outline = Marker()
            outline.header.frame_id = "world"
            outline.ns = "paper_box_outlines"
            outline.id = index
            outline.type = Marker.LINE_LIST
            outline.action = Marker.ADD
            outline.pose = fill.pose
            outline.scale.x = self.box_line_width
            self.set_color(outline, color, 0.98)
            for start, end in box_edges(size):
                outline.points.append(Point(*start))
                outline.points.append(Point(*end))
            output.markers.append(outline)

            label = Marker()
            label.header.frame_id = "world"
            label.ns = "paper_box_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(center[0], center[1], center[2] + 0.5 * size[2] + 0.12)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            self.set_color(label, tuple(channel * 0.68 for channel in color), 1.0)
            label.text = label_text.replace("_", " ")
            output.markers.append(label)
        rospy.loginfo_once(
            "Paper view kept %d room-consistent boxes and hid %d fallback associations",
            len(objects),
            rejected,
        )
        return output

    def publish_once(self, _event):
        rooms = self.room_markers()
        boxes = self.box_markers()
        self.room_pub.publish(rooms)
        self.box_pub.publish(boxes)
        rospy.loginfo(
            "Published %d paper room markers and %d paper box markers",
            len(rooms.markers),
            len(boxes.markers),
        )


if __name__ == "__main__":
    rospy.init_node("paper_map_visualization")
    PaperMapVisualization()
    rospy.spin()
