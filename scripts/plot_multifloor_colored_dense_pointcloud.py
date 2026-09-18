#!/usr/bin/env python3
"""Render a clean, stacked multi-floor RGB-D dense point cloud.

The geometry and colors come directly from the recorded RGB-D frames.  The
floor spacing is only a display transform; no walls, mesh, trajectory, labels,
or axes are added.  A binary PLY with the same display coordinates is written
alongside the PNG so it can be inspected interactively in CloudCompare/Open3D.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
import numpy as np

from plot_partial_exploration_pointcloud import rgbd_points, voxel_first


DEFAULT_FLOOR_Z = (-0.2, 2.6, 5.4)


def write_binary_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    """Write XYZ + uint8 RGB as a compact binary little-endian PLY."""
    points = np.asarray(points, dtype=np.float32)
    colors = np.clip(np.asarray(colors) * 255.0, 0, 255).astype(np.uint8)
    if len(points) != len(colors):
        raise ValueError("points and colors must have the same length")
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    record = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                          ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    record["x"], record["y"], record["z"] = points.T
    record["red"], record["green"], record["blue"] = colors.T
    path.write_bytes(header + record.tobytes())


def floor_extent(meta_path: Path) -> tuple[float, float, float, float]:
    meta = json.loads(meta_path.read_text())
    origin = np.asarray(meta["origin_xy_m"], dtype=np.float64)
    resolution = float(meta["resolution_m"])
    return (float(origin[0]), float(origin[0] + int(meta["width"]) * resolution),
            float(origin[1]), float(origin[1] + int(meta["height"]) * resolution))


def visible_surface(points: np.ndarray, colors: np.ndarray, azimuth: float,
                    elevation: float, pixel_m: float, depth_band_m: float):
    """Keep the camera-facing surface while retaining a shallow depth band.

    This is a lightweight z-buffer approximation for a paper figure. It is
    applied after voxel fusion and does not alter the underlying RGB-D data.
    """
    az, el = np.deg2rad([azimuth, elevation])
    view = np.asarray([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    right = np.asarray([-np.sin(az), np.cos(az), 0.0])
    up = np.cross(right, view)
    centered = points - np.mean(points, axis=0)
    screen = np.stack((centered @ right, centered @ up), axis=1)
    depth = centered @ view
    keys = np.floor((screen - screen.min(axis=0)) / pixel_m).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    nearest = np.full(len(np.unique(inverse)), -np.inf, dtype=np.float32)
    np.maximum.at(nearest, inverse, depth.astype(np.float32))
    keep = depth >= nearest[inverse] - depth_band_m
    return points[keep], colors[keep]


def voxel_random(points: np.ndarray, voxel: float, colors: np.ndarray, rng: np.random.Generator):
    """Choose a random representative in each voxel to avoid grid-like patterns."""
    keys = np.floor(points / voxel).astype(np.int32)
    shuffled = rng.permutation(len(points))
    _, local = np.unique(keys[shuffled], axis=0, return_index=True)
    indices = shuffled[local]
    indices.sort()
    return points[indices], colors[indices]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path, help="episode directory containing frame_*.npz")
    parser.add_argument("output", type=Path, help="clean PNG output")
    parser.add_argument("--wall-grid-dir", type=Path, default=None,
                        help="directory containing L1_wall_grid.json ... L3_wall_grid.json")
    parser.add_argument("--floor-z", type=float, nargs=3, default=list(DEFAULT_FLOOR_Z))
    parser.add_argument("--floors", type=int, nargs="+", choices=(1, 2, 3),
                        default=(1, 2, 3), help="floors to render")
    parser.add_argument("--stack-gap", type=float, default=4.2)
    parser.add_argument("--color-mode", choices=("rgb", "monochrome"), default="rgb",
                        help="use recorded RGB colors or the clean single-color style")
    parser.add_argument("--monochrome-color", default="#8ea3b3",
                        help="uniform point color when --color-mode monochrome")
    parser.add_argument("--visibility", choices=("none", "outer"), default="none",
                        help="optional camera-facing surface filter")
    parser.add_argument("--visibility-pixel", type=float, default=0.04,
                        help="screen-space cell size in metres for visibility filter")
    parser.add_argument("--visibility-depth-band", type=float, default=0.20,
                        help="retained depth behind the nearest surface in metres")
    parser.add_argument("--lower", type=float, default=0.10)
    parser.add_argument("--upper", type=float, default=2.30)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--maximum-depth", type=float, default=5.0)
    parser.add_argument("--voxel", type=float, default=0.025)
    parser.add_argument("--sampling", choices=("first", "random"), default="first",
                        help="representative point selection within each voxel")
    parser.add_argument("--seed", type=int, default=337,
                        help="random seed used by --sampling random")
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=26.0)
    parser.add_argument("--projection", choices=("ortho", "perspective"), default="ortho")
    parser.add_argument("--focal-length", type=float, default=1.0,
                        help="Matplotlib perspective focal length")
    parser.add_argument("--z-aspect", type=float, default=1.5)
    parser.add_argument("--point-size", type=float, default=0.62)
    parser.add_argument("--alpha", type=float, default=0.92)
    parser.add_argument("--facade", action="store_true", help="tight orthographic elevation figure")
    parser.add_argument("--saturation", type=float, default=1.0)
    parser.add_argument("--brightness", type=float, default=1.0)
    parser.add_argument("--skip-ply", action="store_true")
    parser.add_argument("--figsize", type=float, nargs=2, default=(10.0, 10.0),
                        metavar=("WIDTH", "HEIGHT"))
    args = parser.parse_args()
    selected_floors = list(dict.fromkeys(args.floors))
    selected_indices = [floor - 1 for floor in selected_floors]

    frames = sorted(args.episode.glob("frame_*.npz"))
    if not frames:
        raise FileNotFoundError(f"no frame_*.npz in {args.episode}")
    if args.wall_grid_dir is None:
        args.wall_grid_dir = args.episode.parent.parent.parent.parent / "wall_extraction" / \
            "00337_dormant_recovery_ceiling_clearance_L1-L3"

    extents = []
    for floor in range(1, 4):
        meta_path = args.wall_grid_dir / f"L{floor}_wall_grid.json"
        extents.append(floor_extent(meta_path) if meta_path.exists() else None)

    rng = np.random.default_rng(args.seed)
    floor_points: list[list[np.ndarray]] = [[], [], []]
    floor_colors: list[list[np.ndarray]] = [[], [], []]
    raw_counts = [0, 0, 0]
    for frame_path in frames:
        points, colors, _ = rgbd_points(frame_path, args.pixel_stride, args.maximum_depth)
        for index in selected_indices:
            floor_z = args.floor_z[index]
            keep = ((points[:, 2] >= floor_z + args.lower) &
                    (points[:, 2] <= floor_z + args.upper))
            extent = extents[index]
            if extent is not None:
                keep &= ((points[:, 0] >= extent[0]) & (points[:, 0] <= extent[1]) &
                         (points[:, 1] >= extent[2]) & (points[:, 1] <= extent[3]))
            if np.any(keep):
                floor_points[index].append(points[keep])
                floor_colors[index].append(colors[keep])
                raw_counts[index] += int(np.count_nonzero(keep))

    stacked_points, stacked_colors = [], []
    floor_reports = []
    for display_index, index in enumerate(selected_indices):
        floor_z = args.floor_z[index]
        if not floor_points[index]:
            raise RuntimeError(f"no RGB-D points remain for floor L{index + 1}")
        raw_points = np.concatenate(floor_points[index], axis=0)
        raw_colors = np.concatenate(floor_colors[index], axis=0)
        if args.sampling == "random":
            points, colors = voxel_random(raw_points, args.voxel, raw_colors, rng)
        else:
            points, colors = voxel_first(raw_points, args.voxel, raw_colors)
        if args.visibility == "outer":
            points, colors = visible_surface(
                points, colors, args.azimuth, args.elevation,
                args.visibility_pixel, args.visibility_depth_band)
        # Put each floor on its own display plane while retaining measured
        # within-floor height (walls/furniture remain volumetric).
        display = points.copy()
        display[:, 2] = (display_index * args.stack_gap) + (points[:, 2] - float(floor_z))
        order = np.argsort(display[:, 2])
        display, colors = display[order], colors[order]
        if args.color_mode == "monochrome":
            colors = np.empty((len(colors), 3), dtype=np.float32)
            colors[:] = to_rgb(args.monochrome_color)
        stacked_points.append(display)
        stacked_colors.append(colors)
        floor_reports.append({
            "floor": index + 1,
            "floor_z_m": float(floor_z),
            "z_band_m": [float(floor_z + args.lower), float(floor_z + args.upper)],
            "raw_input_points": raw_counts[index],
            "fused_voxel_points": int(len(display)),
            "xy_extent_m": None if extents[index] is None else list(extents[index]),
        })

    stacked_points = np.concatenate(stacked_points, axis=0)
    stacked_colors = np.concatenate(stacked_colors, axis=0)
    gray = stacked_colors @ np.array([0.2126, 0.7152, 0.0722])
    stacked_colors = np.clip((gray[:, None] + args.saturation *
                            (stacked_colors - gray[:, None])) * args.brightness, 0, 1)
    # A stable draw order gives distant points a clean, uncluttered appearance.
    order = np.argsort(stacked_points[:, 2])
    stacked_points, stacked_colors = stacked_points[order], stacked_colors[order]

    x_low, x_high = float(np.min(stacked_points[:, 0])), float(np.max(stacked_points[:, 0]))
    y_low, y_high = float(np.min(stacked_points[:, 1])), float(np.max(stacked_points[:, 1]))
    z_low, z_high = float(np.min(stacked_points[:, 2])), float(np.max(stacked_points[:, 2]))
    x_span, y_span, z_span = x_high - x_low, y_high - y_low, z_high - z_low
    pad = 0.35

    figure = plt.figure(figsize=tuple(args.figsize), dpi=220, facecolor="white")
    axis = figure.add_subplot(111, projection="3d")
    axis.set_facecolor("white")
    axis.scatter(stacked_points[:, 0], stacked_points[:, 1], stacked_points[:, 2],
                 c=np.clip(stacked_colors, 0, 1), s=args.point_size,
                 alpha=args.alpha, linewidths=0, depthshade=False, rasterized=True)
    axis.set_xlim(x_low - pad, x_high + pad)
    axis.set_ylim(y_low - pad, y_high + pad)
    axis.set_zlim(z_low - 0.1, z_high + 0.25)
    axis.set_box_aspect((x_span, y_span, max(0.8, z_span * args.z_aspect)))
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    try:
        if args.projection == "perspective":
            axis.set_proj_type("persp", focal_length=args.focal_length)
        else:
            axis.set_proj_type("ortho")
    except (AttributeError, TypeError):
        pass
    axis.set_axis_off()
    axis.grid(False)
    figure.subplots_adjust(left=0, right=1, bottom=0, top=1)

    if args.facade:
        plt.close(figure)
        az, el = np.deg2rad([args.azimuth, args.elevation])
        view = np.array([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)])
        right = np.array([-np.sin(az), np.cos(az), 0.0])
        up = np.cross(view, right)
        xy = np.column_stack((stacked_points @ right, stacked_points @ up))
        order = np.argsort(stacked_points @ view)
        span = np.ptp(xy, axis=0)
        figure, axis = plt.subplots(figsize=(12, 12*span[1]/span[0]), dpi=220)
        axis.scatter(xy[order, 0], xy[order, 1], c=stacked_colors[order],
                     s=args.point_size, alpha=args.alpha, linewidths=0, rasterized=True)
        axis.set_aspect('equal')
        axis.set_axis_off()
        axis.margins(0.015)
        figure.subplots_adjust(left=0, right=1, bottom=0, top=1)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=220, facecolor="white", bbox_inches="tight", pad_inches=0.02)
    pdf = args.output.with_suffix(".pdf")
    figure.savefig(pdf, facecolor="white", bbox_inches="tight", pad_inches=0.02)
    plt.close(figure)

    ply = args.output.with_suffix(".ply")
    if not args.skip_ply:
        write_binary_ply(ply, stacked_points, stacked_colors)
    metadata = {
        "format": "pre_map_vln.multifloor_colored_dense_pointcloud.v1",
        "source_episode": str(args.episode.resolve()),
        "source": "recorded RGB-D frames (depth back-projection + RGB colors)",
        "floors": selected_floors,
        "floor_z_m": [float(args.floor_z[index]) for index in selected_indices],
        "stack_gap_m": args.stack_gap, "z_band_offset_m": [args.lower, args.upper],
        "pixel_stride": args.pixel_stride, "maximum_depth_m": args.maximum_depth,
        "voxel_m": args.voxel, "color_mode": args.color_mode,
        "sampling": args.sampling, "random_seed": args.seed if args.sampling == "random" else None,
        "uses_rgb": args.color_mode == "rgb",
        "monochrome_color": args.monochrome_color if args.color_mode == "monochrome" else None,
        "visibility": args.visibility,
        "visibility_pixel_m": args.visibility_pixel,
        "visibility_depth_band_m": args.visibility_depth_band,
        "azimuth_deg": args.azimuth, "elevation_deg": args.elevation,
        "projection": args.projection, "focal_length": args.focal_length,
        "z_aspect": args.z_aspect, "point_size": args.point_size, "alpha": args.alpha,
        "floor_reports": floor_reports,
        "facade": args.facade, "saturation": args.saturation, "brightness": args.brightness,
        "figsize": args.figsize,
        "png": str(args.output.resolve()), "pdf": str(pdf.resolve()), "ply": None if args.skip_ply else str(ply.resolve()),
        "display_note": "PLY coordinates use the same separated-floor display transform as the PNG; no walls/mesh/trajectory are included.",
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(metadata, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
