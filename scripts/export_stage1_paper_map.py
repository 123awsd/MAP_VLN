#!/usr/bin/env python3
"""Export a clean rooms-and-boxes Stage 1 figure without the RViz chrome."""

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon


ROOM_COLORS = (
    "#3378C7", "#E67521", "#33A36E", "#A666C2",
    "#EAB31F", "#1AA6B3", "#D4576E", "#6E809E",
)
BOX_COLORS = ("#1F5DB8", "#D33F35", "#168F61", "#963FB2", "#E18A13", "#1595A6", "#BD3B78")
SALIENT_LABELS = {
    "bed", "sofa", "couch", "chair", "table", "coffee_table", "desk",
    "television", "tv", "cabinet", "dresser", "shelf", "lamp", "toilet",
    "sink", "refrigerator", "microwave", "bathtub",
}


def stable_color(text):
    value = sum((index + 1) * byte for index, byte in enumerate(text.encode("utf-8")))
    return BOX_COLORS[value % len(BOX_COLORS)]


def point_segment_distance(point, start, end):
    point, start, end = np.asarray(point), np.asarray(start), np.asarray(end)
    delta = end - start
    denominator = float(np.dot(delta, delta))
    ratio = 0.0 if denominator <= 1e-12 else np.clip(np.dot(point - start, delta) / denominator, 0.0, 1.0)
    return float(np.linalg.norm(point - (start + ratio * delta)))


def smooth_polygon(points, tolerance=0.09, iterations=2):
    points = [tuple(map(float, point)) for point in points]
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    changed = True
    while changed and len(points) > 3:
        changed = False
        kept = []
        for index, point in enumerate(points):
            if point_segment_distance(point, points[index - 1], points[(index + 1) % len(points)]) <= tolerance:
                changed = True
            else:
                kept.append(point)
        if len(kept) < 3:
            break
        points = kept
    for _ in range(iterations):
        refined = []
        for index, start in enumerate(points):
            end = points[(index + 1) % len(points)]
            refined.extend(((0.75 * start[0] + 0.25 * end[0], 0.75 * start[1] + 0.25 * end[1]),
                            (0.25 * start[0] + 0.75 * end[0], 0.25 * start[1] + 0.75 * end[1])))
        points = refined
    return np.asarray(points)


def yaw_from_wxyz(quaternion):
    w, x, y, z = [float(value) for value in quaternion]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def box_footprint(obj):
    cx, cy = [float(value) for value in obj["center_xyz_m"][:2]]
    sx, sy = [float(value) for value in obj["size_xyz_m"][:2]]
    local = np.asarray([[-sx, -sy], [sx, -sy], [sx, sy], [-sx, sy]]) * 0.5
    yaw = yaw_from_wxyz(obj.get("orientation_wxyz", [1, 0, 0, 0]))
    rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return local @ rotation.T + np.asarray([cx, cy])


def point_in_polygon(point, polygon):
    x_value, y_value = [float(value) for value in point]
    inside = False
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        if (current[1] > y_value) == (previous[1] > y_value):
            continue
        crossing_x = ((previous[0] - current[0]) * (y_value - current[1])
                      / (previous[1] - current[1]) + current[0])
        if x_value < crossing_x:
            inside = not inside
    return inside


def collect_objects(graph):
    objects, seen = [], set()
    for room in graph.get("rooms", []):
        for obj in room.get("objects", []):
            if not point_in_polygon(obj["center_xyz_m"][:2], room.get("polygon_xy_m", [])):
                continue
            if obj.get("id") not in seen:
                objects.append(obj)
                seen.add(obj.get("id"))
    return objects


def display_room_polygon(room, room_objects):
    raw = np.asarray(room.get("polygon_xy_m", []), dtype=np.float64)
    if room.get("space_role") != "room":
        return smooth_polygon(raw)
    points = [raw]
    points.extend(box_footprint(obj) for obj in room_objects)
    combined = np.vstack(points)
    padding = 0.08
    minimum = combined.min(axis=0) - padding
    maximum = combined.max(axis=0) + padding
    return np.asarray([
        [minimum[0], minimum[1]], [maximum[0], minimum[1]],
        [maximum[0], maximum[1]], [minimum[0], maximum[1]],
    ])


def draw_rooms(axis, graph, objects_by_room, show_labels=True):
    for index, room in enumerate(graph.get("rooms", [])):
        polygon = display_room_polygon(room, objects_by_room.get(room.get("id"), []))
        if len(polygon) < 3:
            continue
        color = ROOM_COLORS[index % len(ROOM_COLORS)]
        axis.add_patch(Polygon(polygon, closed=True, facecolor=color, edgecolor=color,
                               alpha=0.12, linewidth=0.0, zorder=1))
        closed = np.vstack((polygon, polygon[0]))
        axis.plot(closed[:, 0], closed[:, 1], color=color, linewidth=2.4, zorder=3)
        if show_labels and room.get("centroid_xy_m"):
            x, y = room["centroid_xy_m"]
            room_type = str(room.get("semantic_type", "unknown")).replace("_", " ").title()
            axis.text(x, y, "R{}  {}".format(room.get("id", index), room_type),
                      ha="center", va="center", fontsize=8.5, weight="semibold", color=color,
                      bbox=dict(boxstyle="round,pad=0.24", fc="white", ec=color, alpha=0.92), zorder=7)


def draw_boxes(axis, objects, label_mode):
    outlines = []
    outline_colors = []
    for obj in objects:
        footprint = box_footprint(obj)
        closed = np.vstack((footprint, footprint[0]))
        color = stable_color(str(obj.get("label", "object")))
        outlines.extend(zip(closed[:-1], closed[1:]))
        outline_colors.extend([color] * 4)
        axis.add_patch(Polygon(footprint, closed=True, facecolor=color, edgecolor="none", alpha=0.10, zorder=4))
        label = str(obj.get("label", "object"))
        should_label = label_mode == "all" or (label_mode == "salient" and label in SALIENT_LABELS)
        if should_label:
            cx, cy = obj["center_xyz_m"][:2]
            axis.text(cx, cy, label.replace("_", " "), fontsize=6.2, color="#20242A",
                      ha="center", va="center", zorder=8,
                      bbox=dict(boxstyle="round,pad=0.12", fc="white", ec=color, alpha=0.82, lw=0.7))
    axis.add_collection(LineCollection(outlines, colors=outline_colors, linewidths=1.35, alpha=0.96, zorder=5))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-graph", type=Path,
                        default=Path("outputs/scene_graph/hm3d_stage1_complete_v3_indoor_v1.json"))
    parser.add_argument("--output", type=Path,
                        default=Path("outputs/paper_figures/stage1_rooms_boxes.png"))
    parser.add_argument("--label-mode", choices=("none", "salient", "all"), default="salient")
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()

    graph = json.loads(args.scene_graph.read_text(encoding="utf-8"))
    objects = collect_objects(graph)
    object_ids = {obj.get("id") for obj in objects}
    objects_by_room = {
        room.get("id"): [obj for obj in room.get("objects", []) if obj.get("id") in object_ids]
        for room in graph.get("rooms", [])
    }
    figure, axis = plt.subplots(figsize=(7.8, 7.8), constrained_layout=True)
    draw_rooms(axis, graph, objects_by_room)
    draw_boxes(axis, objects, args.label_mode)
    axis.set_aspect("equal", adjustable="box")
    axis.set_facecolor("white")
    axis.margins(0.035)
    axis.axis("off")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=args.dpi, facecolor="white", bbox_inches="tight")
    print("wrote {} ({} rooms, {} objects)".format(args.output, len(graph.get("rooms", [])), len(objects)))


if __name__ == "__main__":
    main()
