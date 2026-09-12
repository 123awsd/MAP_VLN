#!/usr/bin/env python3
"""Render floor-assigned oriented 3D-box footprints as a matched floor stack."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors as mpl_colors
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection


PASTEL_GROUPS = {
    "sleep": "#9BBCE0", "opening": "#F2AAA5", "storage": "#C8B5E6",
    "seating": "#A9D8B8", "surface": "#F5C78E", "fixture": "#9ED9D5",
    "appliance": "#F1DB91", "decor": "#DAB5CC", "other": "#B9CBD5",
}


def category_group(name: str) -> str:
    name = name.lower()
    groups = {
        "sleep": {"bed", "crib", "mattress", "pillow"},
        "opening": {"door", "window", "curtain"},
        "storage": {"cabinet", "dresser", "vanity", "shelf", "wardrobe"},
        "seating": {"sofa", "armchair", "chair", "bench"},
        "surface": {"table", "coffee table", "countertop", "kitchen island", "desk"},
        "fixture": {"sink", "toilet", "bathtub", "shower", "toilet paper"},
        "appliance": {"television", "microwave", "dishwasher", "heater"},
        "decor": {"picture", "mirror", "rug", "lamp", "fireplace", "railing"},
    }
    return next((group for group, names in groups.items() if name in names), "other")


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def box_corners(row: dict[str, str]) -> np.ndarray:
    center = np.asarray([float(row["tx_world_object"]), float(row["ty_world_object"])])
    half_x, half_y = float(row["scale_x"]) / 2.0, float(row["scale_y"]) / 2.0
    local = np.asarray([
        [-half_x, -half_y], [half_x, -half_y],
        [half_x, half_y], [-half_x, half_y],
    ])
    yaw = yaw_from_quaternion(*(
        float(row[key]) for key in
        ("qw_world_object", "qx_world_object", "qy_world_object", "qz_world_object")
    ))
    rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]])
    return local @ rotation.T + center


def box_floor_raster(grid: np.ndarray, origin: np.ndarray, resolution: float,
                     rows: list[dict[str, str]]) -> np.ndarray:
    image = np.zeros((*grid.shape, 4), dtype=float)
    observed = grid >= 0
    image[observed, :3] = np.asarray((248, 250, 252)) / 255.0
    image[observed, 3] = 1.0
    image[grid == 100, :3] = np.asarray((148, 163, 184)) / 255.0
    image[grid == 100, 3] = 1.0
    fill_rgb = np.asarray((251, 146, 60), dtype=float) / 255.0
    edge_rgb = np.asarray((194, 65, 12), dtype=float) / 255.0
    for row in sorted(rows, key=lambda item: float(item["prob"])):
        polygon = np.round((box_corners(row) - origin) / resolution).astype(np.int32)
        mask = np.zeros(grid.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        confidence = float(row["prob"])
        alpha = float(np.clip(.22 + .55 * (confidence - .45) / .35, .22, .77))
        use = mask.astype(bool)
        image[use, :3] = image[use, :3] * (1.0 - alpha) + fill_rgb * alpha
        image[use, 3] = 1.0
        edge = np.zeros(grid.shape, dtype=np.uint8)
        cv2.polylines(edge, [polygon], True, 1, thickness=2, lineType=cv2.LINE_8)
        use_edge = edge.astype(bool)
        image[use_edge, :3] = edge_rgb
        image[use_edge, 3] = 1.0
    return image


def dashed_free_outline(grid: np.ndarray, dash_on: int, dash_off: int) -> np.ndarray:
    """Return a one-pixel dashed outline along every free-space boundary."""
    outline = np.zeros(grid.shape, dtype=np.uint8)
    contours, _ = cv2.findContours(
        (grid == 0).astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE,
    )
    period = max(1, dash_on + dash_off)
    for contour in contours:
        points = contour[:, 0, :]
        if len(points) == 0:
            continue
        # CHAIN_APPROX_NONE provides an ordered one-pixel boundary, so masking
        # this sequence creates a true single-cell dash without wall thickness.
        keep = (np.arange(len(points)) % period) < dash_on
        selected = points[keep]
        outline[selected[:, 1], selected[:, 0]] = 1
    return outline.astype(bool)


def dashed_room_outline(shape: tuple[int, int], rooms: list[dict],
                        origin: np.ndarray, resolution: float,
                        dash_on: int, dash_off: int) -> np.ndarray:
    """Draw clean one-cell dashes from semantic room polygons."""
    outline = np.zeros(shape, dtype=np.uint8)
    period = max(1, dash_on + dash_off)
    for room in rooms:
        polygon = np.asarray(room.get("polygon_xy_m", []), dtype=float)
        if len(polygon) < 3:
            continue
        points = np.round((polygon - origin) / resolution).astype(int)
        phase = 0
        for start, end in zip(points, np.roll(points, -1, axis=0)):
            delta = end - start
            steps = max(1, int(np.ceil(np.linalg.norm(delta))))
            samples = np.rint(
                start + np.linspace(0.0, 1.0, steps + 1)[:, None] * delta
            ).astype(int)
            sequence = np.arange(phase, phase + len(samples))
            samples = samples[(sequence % period) < dash_on]
            valid = ((samples[:, 0] >= 0) & (samples[:, 0] < shape[1]) &
                     (samples[:, 1] >= 0) & (samples[:, 1] < shape[0]))
            samples = samples[valid]
            outline[samples[:, 1], samples[:, 0]] = 1
            phase += steps
    return outline.astype(bool)


def closed_outer_outline(grid: np.ndarray) -> np.ndarray:
    """Return only the continuous, one-cell outer boundary of each floor."""
    observed = (grid >= 0).astype(np.uint8)
    observed = cv2.morphologyEx(
        observed, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    contours, _ = cv2.findContours(
        observed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
    )
    outline = np.zeros(grid.shape, dtype=np.uint8)
    cv2.drawContours(outline, contours, -1, 1, thickness=1, lineType=cv2.LINE_8)
    return outline.astype(bool)


def dashed_outer_outline(grid: np.ndarray, dash_on: int,
                         dash_off: int) -> np.ndarray:
    """Return only dashed one-cell external contours, each following a closed loop."""
    observed = (grid >= 0).astype(np.uint8)
    observed = cv2.morphologyEx(
        observed, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    contours, _ = cv2.findContours(
        observed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE,
    )
    outline = np.zeros(grid.shape, dtype=np.uint8)
    period = max(1, dash_on + dash_off)
    for contour in contours:
        points = contour[:, 0, :]
        if len(points) == 0:
            continue
        keep = (np.arange(len(points)) % period) < dash_on
        selected = points[keep]
        outline[selected[:, 1], selected[:, 0]] = 1
    return outline.astype(bool)


def base_floor_raster(grid: np.ndarray, wall_thin: int, outline_mode: str,
                      dash_on: int, dash_off: int, rooms: list[dict] | None,
                      origin: np.ndarray, resolution: float,
                      outline_color: str) -> np.ndarray:
    image = np.zeros((*grid.shape, 4), dtype=float)
    if outline_mode == "outer-dashed":
        wall = dashed_outer_outline(grid, dash_on, dash_off)
    elif outline_mode == "outer-closed":
        wall = closed_outer_outline(grid)
    elif outline_mode == "room-dashed":
        wall = dashed_room_outline(
            grid.shape, rooms or [], origin, resolution, dash_on, dash_off,
        )
    elif outline_mode == "dashed":
        wall = dashed_free_outline(grid, dash_on, dash_off)
    else:
        wall = (grid == 100).astype(np.uint8)
        if wall_thin > 0:
            wall = cv2.erode(
                wall, np.ones((3, 3), dtype=np.uint8), iterations=wall_thin,
            )
        wall = wall.astype(bool)
    image[wall, :3] = mpl_colors.to_rgb(outline_color)
    image[wall, 3] = .96 if outline_mode in (
        "outer-closed", "outer-dashed", "dashed", "room-dashed",
    ) else .68
    return image


def draw_cuboid(axis, row: dict[str, str], display_floor_z: float,
                source_floor_z: float, height_scale: float) -> None:
    xy = box_corners(row)
    center_z = float(row["tz_world_object"])
    half_z = float(row["scale_z"]) / 2.0
    raw_low = display_floor_z + (center_z - half_z - source_floor_z) * height_scale
    raw_high = display_floor_z + (center_z + half_z - source_floor_z) * height_scale
    low = max(display_floor_z + .035, raw_low)
    high = max(low + .035, raw_high)
    vertices = np.vstack([
        np.column_stack([xy, np.full(4, low)]),
        np.column_stack([xy, np.full(4, high)]),
    ])
    face_indices = [
        [0, 1, 2, 3], [4, 5, 6, 7],
        [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
    ]
    edge_indices = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    color = np.asarray(mpl_colors.to_rgb(PASTEL_GROUPS[category_group(row["name"])]))
    confidence = float(row["prob"])
    face_alpha = float(np.clip(.13 + (confidence - .45) * .26, .13, .22))
    edge_color = color * .72 + .28
    faces = [[vertices[i] for i in indices] for indices in face_indices]
    edges = [[vertices[a], vertices[b]] for a, b in edge_indices]
    axis.add_collection3d(Poly3DCollection(
        faces, facecolors=[(*color, face_alpha)], edgecolors="none",
        linewidths=0, shade=False, rasterized=True,
    ))
    axis.add_collection3d(Line3DCollection(
        edges, colors=[(*edge_color, .82)], linewidths=.72,
        rasterized=True,
    ))


def add_soft_image_shadow(path: Path, strength: float, blur_px: float,
                          offset_px: tuple[int, int]) -> None:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    darkness = 255.0 - image.min(axis=2).astype(np.float32)
    foreground_alpha = np.clip((darkness - 2.0) / 18.0, 0.0, 1.0)
    mask = (darkness > 5.0).astype(np.uint8) * 255
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
    )
    shadow = cv2.GaussianBlur(mask, (0, 0), blur_px).astype(np.float32) / 255.0
    transform = np.float32([[1, 0, offset_px[0]], [0, 1, offset_px[1]]])
    shadow = cv2.warpAffine(shadow, transform, (image.shape[1], image.shape[0]))
    shadow_alpha = np.clip(shadow * strength, 0.0, 1.0)[..., None]
    background = 255.0 * (1.0 - shadow_alpha) + 72.0 * shadow_alpha
    alpha = foreground_alpha[..., None]
    composed = image.astype(np.float32) * alpha + background * (1.0 - alpha)
    cv2.imwrite(str(path), np.clip(composed, 0, 255).astype(np.uint8))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("box_dir", type=Path)
    parser.add_argument("grid_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scene-dir", type=Path,
                        help="Scene-graph directory used for clean room outlines")
    parser.add_argument("--floors", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--stack-gap", type=float, default=4.2)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=26.0)
    parser.add_argument("--z-aspect", type=float, default=1.5)
    parser.add_argument("--min-prob", type=float, default=0.0)
    parser.add_argument("--render-mode", choices=("footprint", "cuboid"), default="cuboid")
    parser.add_argument("--height-scale", type=float, default=.62,
                        help="Visual scale applied to physical box elevation and height")
    parser.add_argument("--wall-thin", type=int, default=0,
                        help="Erode wall display by this many pixels; source grid is unchanged")
    parser.add_argument("--outline-mode", choices=(
        "wall", "dashed", "room-dashed", "outer-closed", "outer-dashed",
    ), default="wall",
                        help="Draw wall bands or a one-cell dashed room outline")
    parser.add_argument("--dash-on", type=int, default=5,
                        help="Number of visible cells per outline dash")
    parser.add_argument("--dash-off", type=int, default=4,
                        help="Number of blank cells between outline dashes")
    parser.add_argument("--outline-color", default="#7890a8")
    parser.add_argument("--shadow-strength", type=float, default=.14)
    parser.add_argument("--shadow-blur", type=float, default=18.0)
    parser.add_argument("--shadow-offset", type=int, nargs=2, default=[12, 12])
    args = parser.parse_args()

    floor_data, all_xy, reports = [], [], {}
    for index, floor in enumerate(sorted(args.floors)):
        metadata = json.loads((args.grid_dir / f"L{floor}_wall_grid.json").read_text())
        grid = np.load(args.grid_dir / f"L{floor}_wall_grid.npy")
        origin = np.asarray(metadata["origin_xy_m"], dtype=float)
        resolution = float(metadata["resolution_m"])
        with (args.box_dir / f"L{floor}_boxes.csv").open(newline="") as stream:
            all_rows = list(csv.DictReader(stream))
        rows = [row for row in all_rows if float(row["prob"]) >= args.min_prob]
        rooms = []
        if args.scene_dir is not None:
            graph = json.loads(
                (args.scene_dir / f"L{floor}_scene_graph_qwen.json").read_text()
            )
            rooms = graph.get("rooms", [])
        if args.outline_mode == "room-dashed" and not rooms:
            parser.error("--outline-mode room-dashed requires --scene-dir")
        rgba = (box_floor_raster(grid, origin, resolution, rows)
                if args.render_mode == "footprint" else base_floor_raster(
                    grid, args.wall_thin, args.outline_mode,
                    args.dash_on, args.dash_off, rooms, origin, resolution,
                    args.outline_color,
                ))
        height, width = grid.shape
        x_edges = origin[0] + np.arange(width + 1) * resolution
        y_edges = origin[1] + np.arange(height + 1) * resolution
        xx, yy = np.meshgrid(x_edges, y_edges)
        floor_data.append((index, floor, xx, yy, rgba, rows, float(metadata["floor_z_m"])))
        all_xy.append(np.asarray([[x_edges[0], y_edges[0]], [x_edges[-1], y_edges[-1]]]))
        reports[str(floor)] = {
            "source_boxes": len(all_rows), "rendered_boxes": len(rows),
            "categories": dict(sorted(Counter(row["name"] for row in rows).items())),
        }

    xy = np.concatenate(all_xy)
    xmin, ymin = xy.min(axis=0); xmax, ymax = xy.max(axis=0)
    padx, pady = (xmax - xmin) * .05, (ymax - ymin) * .05
    figure = plt.figure(figsize=(12, 10), facecolor="white")
    axis = figure.add_subplot(111, projection="3d")
    for index, floor, xx, yy, rgba, rows, source_floor_z in floor_data:
        z = index * args.stack_gap
        axis.plot_surface(
            xx, yy, np.full_like(xx, z - .035), rstride=1, cstride=1,
            facecolors=rgba, shade=False, antialiased=False,
            linewidth=0, rasterized=True,
        )
        if args.render_mode == "cuboid":
            for row in rows:
                draw_cuboid(axis, row, z, source_floor_z, args.height_scale)
    axis.set_xlim(xmin - padx, xmax + padx); axis.set_ylim(ymin - pady, ymax + pady)
    axis.set_zlim(-args.stack_gap * .2, args.stack_gap * (len(floor_data) - 1) + args.stack_gap * .6)
    axis.set_box_aspect((xmax - xmin, ymax - ymin, args.stack_gap * len(floor_data) * args.z_aspect))
    axis.set_proj_type("ortho"); axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.set_axis_off(); figure.subplots_adjust(left=0, right=1, bottom=0, top=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=220, facecolor="white", bbox_inches="tight", pad_inches=.02)
    plt.close(figure)
    if args.shadow_strength > 0:
        add_soft_image_shadow(args.output, args.shadow_strength, args.shadow_blur,
                              tuple(args.shadow_offset))
    report = {
        "format": "pre_map_vln.multifloor_box_stack.v2",
        "box_dir": str(args.box_dir.resolve()), "grid_dir": str(args.grid_dir.resolve()),
        "floors": sorted(args.floors), "stack_gap": args.stack_gap,
        "azimuth": args.azimuth, "elevation": args.elevation,
        "z_aspect": args.z_aspect, "min_prob": args.min_prob,
        "render_mode": args.render_mode, "height_scale": args.height_scale,
        "wall_thin_px": args.wall_thin,
        "outline_mode": args.outline_mode,
        "dash_cells": [args.dash_on, args.dash_off],
        "outline_color": args.outline_color,
        "scene_dir": str(args.scene_dir.resolve()) if args.scene_dir else None,
        "category_palette": PASTEL_GROUPS,
        "reports": reports, "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
