#!/usr/bin/env python3
"""Render standalone L2 room-segmentation and Boxer layers from source data."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOM_COLORS = {
    "bedroom": "#84A6CC",
    "bathroom": "#EF7775",
    "living_room": "#83B77B",
    "kitchen": "#F4A64F",
    "entry_hall": "#91C7C3",
    "corridor": "#91C7C3",
    "unknown": "#B8C6D4",
}

BOX_COLORS = {
    "sleep": "#9BBCE0",
    "opening": "#F2AAA5",
    "storage": "#C8B5E6",
    "seating": "#A9D8B8",
    "surface": "#F5C78E",
    "fixture": "#9ED9D5",
    "appliance": "#F1DB91",
    "decor": "#DAB5CC",
    "other": "#B9CBD5",
}

CATEGORY_GROUPS = {
    "sleep": {"bed", "crib", "mattress", "pillow"},
    "opening": {"curtain", "door", "railing", "window"},
    "storage": {"cabinet", "dresser", "vanity"},
    "seating": {"armchair", "bench", "chair", "sofa"},
    "surface": {"coffee table", "countertop", "kitchen island", "table"},
    "fixture": {"bathtub", "heater", "sink", "toilet", "toilet paper"},
    "appliance": {"dishwasher", "microwave", "television"},
    "decor": {"fireplace", "lamp", "mirror", "picture", "rug"},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-graph", required=True, type=Path)
    parser.add_argument("--wall-grid", required=True, type=Path)
    parser.add_argument("--wall-meta", required=True, type=Path)
    parser.add_argument("--boxes", required=True, type=Path)
    parser.add_argument("--room-output", required=True, type=Path)
    parser.add_argument("--box-output", required=True, type=Path)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=26.0)
    parser.add_argument("--height-scale", type=float, default=0.62)
    parser.add_argument("--width", type=int, default=2208)
    parser.add_argument("--height", type=int, default=1600)
    parser.add_argument("--dpi", type=int, default=200)
    parser.add_argument("--dash-cells", type=int, nargs=2, default=(7, 5))
    return parser.parse_args()


def load_sources(args: argparse.Namespace):
    scene = json.loads(args.scene_graph.read_text(encoding="utf-8"))
    meta = json.loads(args.wall_meta.read_text(encoding="utf-8"))
    # Occupancy encoding is {-1: unknown, 0: free, 100: measured wall}.
    wall = np.load(args.wall_grid) == 100
    with args.boxes.open(newline="", encoding="utf-8") as handle:
        boxes = list(csv.DictReader(handle))
    return scene, meta, wall, boxes


def room_type_map(scene: dict) -> dict[int, str]:
    decisions = scene.get("room_semantics", {}).get("decisions", [])
    return {
        int(item["room_id"]): item.get("room_type", "unknown")
        for item in decisions if item.get("accepted", False)
    }


def bounds_from_meta(meta: dict) -> tuple[float, float, float, float]:
    ox, oy = map(float, meta["origin_xy_m"])
    resolution = float(meta["resolution_m"])
    return ox, ox + meta["width"] * resolution, oy, oy + meta["height"] * resolution


def setup_axis(args: argparse.Namespace, bounds, zmax: float):
    fig = plt.figure(
        figsize=(args.width / args.dpi, args.height / args.dpi),
        dpi=args.dpi,
        facecolor="white",
    )
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    xmin, xmax, ymin, ymax = bounds
    xpad = (xmax - xmin) * 0.035
    ypad = (ymax - ymin) * 0.035
    ax.set_xlim(xmin - xpad, xmax + xpad)
    ax.set_ylim(ymin - ypad, ymax + ypad)
    ax.set_zlim(-0.03, max(0.25, zmax))
    ax.view_init(elev=args.elevation, azim=args.azimuth)
    ax.set_proj_type("ortho")
    ax.set_box_aspect((xmax - xmin, ymax - ymin, max(0.8, zmax * 1.5)))
    ax.set_axis_off()
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    return fig, ax


def render_rooms(args, scene, meta, wall, bounds):
    fig, ax = setup_axis(args, bounds, zmax=0.7)
    types = room_type_map(scene)
    for room in scene.get("rooms", []):
        polygon = np.asarray(room.get("polygon_xy_m", []), dtype=float)
        if len(polygon) < 3:
            continue
        color = ROOM_COLORS.get(types.get(int(room["id"]), "unknown"), ROOM_COLORS["unknown"])
        vertices = [[(float(x), float(y), 0.02) for x, y in polygon]]
        ax.add_collection3d(Poly3DCollection(
            vertices, facecolors=to_rgba(color, 0.96), edgecolors="#F8FAFC",
            linewidths=1.15, zorder=2,
        ))

    ox, oy = map(float, meta["origin_xy_m"])
    resolution = float(meta["resolution_m"])
    ys = oy + (np.arange(wall.shape[0]) + 0.5) * resolution
    xs = ox + (np.arange(wall.shape[1]) + 0.5) * resolution
    # The structural wall grid is drawn last so room colors remain fully filled
    # while the measured wall cells retain their original neutral gray role.
    ax.contourf(
        xs, ys, wall.astype(np.uint8), zdir="z", offset=0.035,
        levels=[0.5, 1.5], colors=["#747E8D"], antialiased=False, zorder=3,
    )
    args.room_output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.room_output, dpi=args.dpi, facecolor="white", bbox_inches=None)
    plt.close(fig)


def category_color(label: str) -> str:
    label = label.strip().lower()
    for group, labels in CATEGORY_GROUPS.items():
        if label in labels:
            return BOX_COLORS[group]
    return BOX_COLORS["other"]


def cuboid_faces(row: dict, floor_z: float, height_scale: float):
    cx, cy, cz = (float(row[k]) for k in ("tx_world_object", "ty_world_object", "tz_world_object"))
    sx, sy, sz = (float(row[k]) for k in ("scale_x", "scale_y", "scale_z"))
    qw, qz = float(row["qw_world_object"]), float(row["qz_world_object"])
    yaw = 2.0 * math.atan2(qz, qw)
    local = np.asarray([[-sx/2, -sy/2], [sx/2, -sy/2], [sx/2, sy/2], [-sx/2, sy/2]])
    rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    xy = local @ rotation.T + np.asarray([cx, cy])
    bottom_world = cz - sz * 0.5
    z0 = max(0.04, bottom_world - floor_z)
    z1 = z0 + sz * height_scale
    lower = [(float(x), float(y), z0) for x, y in xy]
    upper = [(float(x), float(y), z1) for x, y in xy]
    return [lower, upper] + [
        [lower[i], lower[(i + 1) % 4], upper[(i + 1) % 4], upper[i]] for i in range(4)
    ], z1


def footprint_contours(scene: dict, meta: dict, wall: np.ndarray):
    mask = np.zeros_like(wall, dtype=np.uint8)
    ox, oy = map(float, meta["origin_xy_m"])
    resolution = float(meta["resolution_m"])
    for room in scene.get("rooms", []):
        polygon = np.asarray(room.get("polygon_xy_m", []), dtype=float)
        if len(polygon) < 3:
            continue
        pixels = np.rint((polygon - np.asarray([ox, oy])) / resolution).astype(np.int32)
        cv2.fillPoly(mask, [pixels], 255)
    mask[wall] = 255
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    result = []
    for contour in contours:
        if cv2.contourArea(contour) < 30:
            continue
        points = contour[:, 0, :].astype(float)
        points[:, 0] = ox + points[:, 0] * resolution
        points[:, 1] = oy + points[:, 1] * resolution
        result.append(np.vstack([points, points[0]]))
    return result


def draw_dashed_closed(ax, contour: np.ndarray, z: float, on: float, off: float):
    draw = True
    remaining = on
    for start, end in zip(contour[:-1], contour[1:]):
        vector = end - start
        length = float(np.linalg.norm(vector))
        if length <= 1e-9:
            continue
        direction = vector / length
        cursor = 0.0
        while cursor < length - 1e-9:
            step = min(remaining, length - cursor)
            if draw:
                a, b = start + direction * cursor, start + direction * (cursor + step)
                ax.plot([a[0], b[0]], [a[1], b[1]], [z, z], color="#657F99",
                        linewidth=1.7, solid_capstyle="butt", zorder=1)
            cursor += step
            remaining -= step
            if remaining <= 1e-9:
                draw = not draw
                remaining = on if draw else off


def render_boxes(args, scene, meta, wall, boxes, bounds):
    floor_z = float(meta["floor_z_m"])
    prepared = []
    maximum_z = 0.5
    for row in boxes:
        faces, top = cuboid_faces(row, floor_z, args.height_scale)
        prepared.append((row, faces))
        maximum_z = max(maximum_z, top)
    fig, ax = setup_axis(args, bounds, zmax=maximum_z * 1.14)

    resolution = float(meta["resolution_m"])
    for contour in footprint_contours(scene, meta, wall):
        draw_dashed_closed(
            ax, contour, 0.025,
            args.dash_cells[0] * resolution, args.dash_cells[1] * resolution,
        )

    for row, faces in sorted(prepared, key=lambda item: float(item[0]["tz_world_object"])):
        color = category_color(row["name"])
        ax.add_collection3d(Poly3DCollection(
            faces, facecolors=to_rgba(color, 0.22), edgecolors=to_rgba(color, 0.72),
            linewidths=0.85, zorder=3,
        ))
    args.box_output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.box_output, dpi=args.dpi, facecolor="white", bbox_inches=None)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    scene, meta, wall, boxes = load_sources(args)
    bounds = bounds_from_meta(meta)
    render_rooms(args, scene, meta, wall, bounds)
    render_boxes(args, scene, meta, wall, boxes, bounds)
    print(args.room_output.resolve())
    print(args.box_output.resolve())


if __name__ == "__main__":
    main()
