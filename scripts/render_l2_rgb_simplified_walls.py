#!/usr/bin/env python3
"""Render the official L2 RGB floor texture with simplified wall-grid solids."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import proj3d
from PIL import Image


def clean_wall_mask(grid: np.ndarray, minimum_cells: int) -> tuple[np.ndarray, np.ndarray]:
    mask = (grid == 100).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    keep = np.zeros_like(mask, dtype=bool)
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= minimum_cells:
            keep |= labels == label
    # Morphological skeleton: reduce thick occupied bands to a one-cell wall
    # centerline. This keeps the paper figure legible without inventing paths.
    work = keep.astype(np.uint8)
    skeleton = np.zeros_like(work)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while np.any(work):
        opened = cv2.morphologyEx(work, cv2.MORPH_OPEN, element)
        skeleton |= cv2.subtract(work, opened)
        work = cv2.erode(work, element)
    # Connect diagonal skeleton samples into a continuous narrow ribbon. At the
    # current 5 cm grid resolution this is about 15 cm, still much thinner than
    # the original occupied bands.
    thin_wall = cv2.dilate(skeleton, np.ones((3, 3), np.uint8), iterations=1)
    return keep, thin_wall.astype(bool)


def wall_contour_faces(mask: np.ndarray, x0: float, y0: float, resolution: float,
                       wall_height: float, external_only: bool = False):
    """Merge cell boundaries into simplified continuous vertical wall faces."""
    faces = []
    outward_normals = []
    paths = []
    retrieval = cv2.RETR_EXTERNAL if external_only else cv2.RETR_CCOMP
    contours, _ = cv2.findContours(mask.astype(np.uint8), retrieval,
                                   cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour = cv2.approxPolyDP(contour, epsilon=1.0, closed=True)
        uv = contour[:, 0, :]
        if len(uv) < 2:
            continue
        xy = np.column_stack((x0 + (uv[:, 0] + .5) * resolution,
                              y0 + (uv[:, 1] + .5) * resolution))
        paths.append(xy)
        for index in range(len(xy)):
            a = xy[index]
            b = xy[(index + 1) % len(xy)]
            direction = b - a
            length = float(np.linalg.norm(direction))
            if length < 1e-9:
                continue
            normal = np.asarray([-direction[1], direction[0]]) / length
            midpoint = (a + b) / 2

            def is_wall(point):
                col = int(np.floor((point[0] - x0) / resolution))
                row = int(np.floor((point[1] - y0) / resolution))
                return (0 <= row < mask.shape[0] and 0 <= col < mask.shape[1]
                        and bool(mask[row, col]))

            probe = max(2.2 * resolution, .11)
            plus_wall = is_wall(midpoint + normal * probe)
            minus_wall = is_wall(midpoint - normal * probe)
            if plus_wall and not minus_wall:
                normal = -normal
            elif plus_wall == minus_wall:
                # OpenCV contour order is stable, but the mask probe above is
                # preferred because it also handles concave outlines.
                normal = -normal
            faces.append([(a[0], a[1], 0), (b[0], b[1], 0),
                          (b[0], b[1], wall_height),
                          (a[0], a[1], wall_height)])
            outward_normals.append(normal)
    return faces, paths, np.asarray(outward_normals)


def camera_facing_mask(axis, faces, normals: np.ndarray) -> np.ndarray:
    """Select facade sides whose outward direction projects toward screen bottom."""
    bases = np.asarray([[(face[0][0] + face[1][0]) / 2,
                         (face[0][1] + face[1][1]) / 2, 0]
                        for face in faces])
    _, base_y, _ = proj3d.proj_transform(bases[:, 0], bases[:, 1], bases[:, 2],
                                         axis.get_proj())
    probe = bases.copy()
    probe[:, :2] += normals * .25
    _, probe_y, _ = proj3d.proj_transform(probe[:, 0], probe[:, 1], probe[:, 2],
                                          axis.get_proj())
    return probe_y < base_y - 1e-7


def room_facing_mask(faces, normals: np.ndarray, grid: np.ndarray,
                     x0: float, y0: float, resolution: float) -> np.ndarray:
    """Keep wall-band faces that border measured free room space."""
    keep = []
    for face, normal in zip(faces, normals):
        midpoint = np.asarray([(face[0][0] + face[1][0]) / 2,
                               (face[0][1] + face[1][1]) / 2])
        touches_free = False
        # Probe outward at several distances because occupied wall bands can be
        # a few cells thick and the RGB-D free-space boundary is not perfectly
        # uniform along furniture and door frames.
        for distance_cells in (1.5, 2.5, 4.0, 6.0):
            point = midpoint + normal * (distance_cells * resolution)
            col = int(np.floor((point[0] - x0) / resolution))
            row = int(np.floor((point[1] - y0) / resolution))
            r0, r1 = max(0, row - 1), min(grid.shape[0], row + 2)
            c0, c1 = max(0, col - 1), min(grid.shape[1], col + 2)
            if r0 < r1 and c0 < c1 and np.any(grid[r0:r1, c0:c1] == 0):
                touches_free = True
                break
        keep.append(touches_free)
    return np.asarray(keep, dtype=bool)


def screen_styled_wall_colors(axis, faces, alpha_far: float = .055,
                              alpha_near: float = .34) -> tuple[np.ndarray, np.ndarray]:
    """Style walls by actual screen position, independent of world-map axes."""
    centers = np.asarray([[sum(vertex[0] for vertex in face) / 4,
                           sum(vertex[1] for vertex in face) / 4,
                           sum(vertex[2] for vertex in face) / 4]
                          for face in faces])
    _, screen_y, _ = proj3d.proj_transform(centers[:, 0], centers[:, 1],
                                           centers[:, 2], axis.get_proj())
    span = float(np.ptp(screen_y))
    # In axes coordinates, smaller y is closer to the bottom of the finished
    # figure. This is the foreground facade the oblique top view must retain.
    near = np.ones_like(screen_y) if span < 1e-9 else (screen_y.max() - screen_y) / span
    # Keep the palette neutral and paper-friendly. The foreground gain is the
    # main depth cue; it does not alter the wall geometry or camera direction.
    far_rgb = np.asarray([0.78, 0.84, 0.88])
    near_rgb = np.asarray([0.56, 0.68, 0.75])
    rgb = far_rgb[None, :] * (1 - near[:, None]) + near_rgb[None, :] * near[:, None]
    alpha = alpha_far + (alpha_near - alpha_far) * near ** 1.7
    return np.column_stack((rgb, alpha)), near


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("grid_metadata", type=Path)
    parser.add_argument("habitat_floors", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--l2-crop", type=int, nargs=2, default=(1266, 2382))
    parser.add_argument("--wall-height", type=float, default=0.50)
    parser.add_argument("--minimum-component-cells", type=int, default=20)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=32.0)
    parser.add_argument("--z-exaggeration", type=float, default=2.6)
    parser.add_argument("--focal-length", type=float, default=2.2,
                        help="Perspective focal length; larger values are gentler.")
    parser.add_argument("--wall-alpha-far", type=float, default=.15)
    parser.add_argument("--wall-alpha-near", type=float, default=.62)
    parser.add_argument("--texture-width", type=int, default=700)
    parser.add_argument("--show-wall-base-outline", action="store_true",
                        help="Draw only the outermost official RGB floor outline as a dashed calibration line.")
    parser.add_argument("--outline-only", action="store_true",
                        help="Hide walls and shadows so the outer calibration outline is unobstructed.")
    args = parser.parse_args()

    metadata = json.loads(args.grid_metadata.read_text())
    grid = np.load(args.grid_metadata.with_name(args.grid_metadata.stem + ".npy"))
    broad_mask, mask = clean_wall_mask(grid, args.minimum_component_cells)
    resolution = float(metadata["resolution_m"])
    x0, y0 = map(float, metadata["origin_xy_m"])
    height, width = grid.shape
    x1, y1 = x0 + width * resolution, y0 + height * resolution
    x_edges = x0 + np.arange(width + 1) * resolution
    y_edges = y0 + np.arange(height + 1) * resolution
    xx, yy = np.meshgrid(x_edges, y_edges)
    texture_height = max(2, int(round(args.texture_width * (y1 - y0) / (x1 - x0))))
    floor_x_edges = np.linspace(x0, x1, args.texture_width + 1)
    floor_y_edges = np.linspace(y0, y1, texture_height + 1)
    floor_xx, floor_yy = np.meshgrid(floor_x_edges, floor_y_edges)
    xc, yc = np.meshgrid((floor_x_edges[:-1] + floor_x_edges[1:]) / 2,
                         (floor_y_edges[:-1] + floor_y_edges[1:]) / 2)

    full = Image.open(args.habitat_floors).convert("RGB")
    crop_top, crop_bottom = args.l2_crop
    l2 = np.asarray(full.crop((0, crop_top, full.width, crop_bottom)))
    foreground = l2.max(axis=2) > 25
    py, px = np.where(foreground)
    u0, u1, v0, v1 = float(px.min()), float(px.max()), float(py.min()), float(py.max())
    component_count, component_labels, component_stats, _ = (
        cv2.connectedComponentsWithStats(foreground.astype(np.uint8), connectivity=8))
    largest_label = 1 + int(np.argmax(component_stats[1:, cv2.CC_STAT_AREA]))
    outer_source_mask = (component_labels == largest_label).astype(np.uint8)
    outer_contours, _ = cv2.findContours(
        outer_source_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    outer_pixels = cv2.approxPolyDP(
        max(outer_contours, key=cv2.contourArea), epsilon=2.0, closed=True)[:, 0, :]
    outer_u = outer_pixels[:, 0].astype(float)
    outer_v = outer_pixels[:, 1].astype(float)
    outer_x = x1 - (outer_v - v0) / max(v1 - v0, 1e-9) * (x1 - x0)
    outer_y = y1 - (outer_u - u0) / max(u1 - u0, 1e-9) * (y1 - y0)
    uu = u0 + (y1 - yc) / (y1 - y0) * (u1 - u0)
    vv = v0 + (x1 - xc) / (x1 - x0) * (v1 - v0)
    ui = np.clip(np.rint(uu).astype(int), 0, l2.shape[1] - 1)
    vi = np.clip(np.rint(vv).astype(int), 0, l2.shape[0] - 1)
    texture = l2[vi, ui]
    alpha = np.where(texture.max(axis=2, keepdims=True) <= 25, 0, 255).astype(np.uint8)
    floor_rgba = np.concatenate((texture, alpha), axis=2) / 255.0

    figure = plt.figure(figsize=(11, 9), facecolor="white")
    # Matplotlib's collection-level automatic depth ordering can place an
    # entire foreground facade behind the textured floor. Explicit ordering is
    # essential here: otherwise walls are visible only over white holes and
    # appear to float in the air.
    axis = figure.add_subplot(111, projection="3d", computed_zorder=False)
    axis.set_xlim(x0, x1)
    axis.set_ylim(y0, y1)
    axis.set_zlim(-.03, args.wall_height + .08)
    axis.set_box_aspect((x1 - x0, y1 - y0,
                         args.wall_height * args.z_exaggeration))
    axis.set_proj_type("persp", focal_length=args.focal_length)
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.plot_surface(floor_xx, floor_yy, np.zeros_like(floor_xx), rstride=1, cstride=1,
                      facecolors=floor_rgba, shade=False, antialiased=False,
                      linewidth=0, rasterized=True, zorder=1)
    # The user-approved closed RGB outline is the wall base. Extruding this
    # exact polyline avoids the registration error of the independently built
    # wall grid while keeping every wall segment attached to the floor edge.
    outline_xy = np.column_stack((outer_x, outer_y))
    sides = []
    for index, a in enumerate(outline_xy):
        b = outline_xy[(index + 1) % len(outline_xy)]
        sides.append([(a[0], a[1], 0), (b[0], b[1], 0),
                      (b[0], b[1], args.wall_height),
                      (a[0], a[1], args.wall_height)])
    side_colors, _ = screen_styled_wall_colors(
        axis, sides, alpha_far=args.wall_alpha_far,
        alpha_near=args.wall_alpha_near)
    if not args.outline_only:
        axis.add_collection3d(Poly3DCollection(
            sides, facecolors=side_colors, edgecolors="none", zsort="average",
            antialiaseds=False, zorder=3))
    if args.show_wall_base_outline:
        axis.plot(np.r_[outer_x, outer_x[0]], np.r_[outer_y, outer_y[0]],
                  np.full(len(outer_x) + 1, .025), color="#0b4265",
                  linewidth=1.55, linestyle=(0, (3, 2)), alpha=1.0,
                  zorder=5, antialiased=True)

    # Do not infer an exterior facade from gaps in the RGB footprint: open
    # notches can otherwise become unsupported curtain-like walls. All visible
    # geometry above comes directly from the measured L2 wall grid.

    axis.set_axis_off()
    figure.subplots_adjust(left=0, right=1, bottom=0, top=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=240, bbox_inches="tight", pad_inches=.02,
                   facecolor="white")
    plt.close(figure)

    report = {
        "format": "pre_map_vln.habitat_l2_rgb_simplified_walls.v1",
        "floor": 2,
        "habitat_rgb_source": str(args.habitat_floors.resolve()),
        "habitat_l2_crop_rows": [crop_top, crop_bottom],
        "wall_grid_source": str(args.grid_metadata.resolve()),
        "wall_geometry": "zero-thickness vertical faces extruded from the approved closed outer RGB-floor outline",
        "wall_surface_rule": "use exactly the outermost official RGB connected-component contour",
        "foreground_facade_source": str(args.habitat_floors.resolve()),
        "foreground_facade_rule": "approved dashed outer outline extruded directly",
        "occlusion_rule": "explicit floor-shadow-wall draw order; foreground walls stay above the RGB floor",
        "original_wall_cells": int(np.count_nonzero(grid == 100)),
        "connected_wall_cells_before_thinning": int(broad_mask.sum()),
        "retained_wall_cells": int(mask.sum()),
        "wall_thickness": "zero; single vertical surface",
        "minimum_component_cells": args.minimum_component_cells,
        "wall_height_m": args.wall_height,
        "wall_style": {"side_color_far": "#c7d6e0",
                       "side_color_near": "#8fadbf",
                       "grid_side_alpha_far": args.wall_alpha_far,
                       "grid_side_alpha_near": args.wall_alpha_near,
                       "top_color": "#d6e3eb", "top_alpha": 0.24},
        "view": {"azimuth": args.azimuth, "elevation": args.elevation,
                 "projection": "perspective", "focal_length": args.focal_length,
                 "z_exaggeration": args.z_exaggeration,
                 "camera_direction_unchanged": True},
        "uses_pointcloud": False,
        "uses_image_generation": False,
        "shows_outermost_rgb_outline": args.show_wall_base_outline,
        "outline_only": args.outline_only,
        "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
