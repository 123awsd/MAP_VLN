#!/usr/bin/env python3
"""Render a standalone L2 Habitat RGB plate with low measured RGB-D walls."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from plot_partial_exploration_pointcloud import rgbd_points, voxel_first


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("grid_metadata", type=Path)
    parser.add_argument("habitat_floors", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--l2-crop", type=int, nargs=2, default=(1266, 2382))
    parser.add_argument("--wall-center-above-floor", type=float, default=1.20)
    parser.add_argument("--wall-window", type=float, default=0.50)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--maximum-depth", type=float, default=5.0)
    parser.add_argument("--voxel", type=float, default=0.025)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=26.0)
    parser.add_argument("--z-exaggeration", type=float, default=4.0)
    args = parser.parse_args()

    metadata = json.loads(args.grid_metadata.read_text())
    wall_grid = np.load(args.grid_metadata.with_name(args.grid_metadata.stem + ".npy"))
    floor_z = float(metadata["floor_z_m"])
    origin = np.asarray(metadata["origin_xy_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    height, width = wall_grid.shape
    x0, y0 = map(float, origin)
    x1 = x0 + width * resolution
    y1 = y0 + height * resolution
    center_z = floor_z + args.wall_center_above_floor
    wall_z0 = center_z - args.wall_window / 2.0
    wall_z1 = center_z + args.wall_window / 2.0

    point_parts, color_parts = [], []
    frames = sorted(args.episode.glob("frame_*.npz"))
    for frame in frames:
        points, colors, _ = rgbd_points(frame, args.pixel_stride, args.maximum_depth)
        keep = ((points[:, 2] >= wall_z0) & (points[:, 2] <= wall_z1)
                & (points[:, 0] >= x0) & (points[:, 0] <= x1)
                & (points[:, 1] >= y0) & (points[:, 1] <= y1))
        if np.any(keep):
            point_parts.append(points[keep])
            color_parts.append(colors[keep])
    points, colors = voxel_first(np.concatenate(point_parts), args.voxel,
                                 np.concatenate(color_parts))
    gx = np.floor((points[:, 0] - x0) / resolution).astype(int)
    gy = np.floor((points[:, 1] - y0) / resolution).astype(int)
    valid = ((gx >= 0) & (gx < width) & (gy >= 0) & (gy < height))
    on_wall = np.zeros(len(points), dtype=bool)
    on_wall[valid] = wall_grid[gy[valid], gx[valid]] == 100
    wall_points = points[on_wall]
    wall_colors = colors[on_wall]

    # Extract the unmodified official L2 plate and sample it using its documented
    # u=-FalconY, v=-FalconX convention so it aligns with the measured wall points.
    full = Image.open(args.habitat_floors).convert("RGB")
    crop_top, crop_bottom = args.l2_crop
    l2 = np.asarray(full.crop((0, crop_top, full.width, crop_bottom)))
    foreground = l2.max(axis=2) > 25
    py, px = np.where(foreground)
    u0, u1, v0, v1 = float(px.min()), float(px.max()), float(py.min()), float(py.max())
    x_edges = x0 + np.arange(width + 1) * resolution
    y_edges = y0 + np.arange(height + 1) * resolution
    xx, yy = np.meshgrid(x_edges, y_edges)
    uu = u0 + (y1 - yy) / (y1 - y0) * (u1 - u0)
    vv = v0 + (x1 - xx) / (x1 - x0) * (v1 - v0)
    ui = np.clip(np.rint(uu).astype(int), 0, l2.shape[1] - 1)
    vi = np.clip(np.rint(vv).astype(int), 0, l2.shape[0] - 1)
    texture = l2[vi, ui]
    alpha = np.where(texture.max(axis=2, keepdims=True) <= 25, 0, 255).astype(np.uint8)
    rgba = np.concatenate((texture, alpha), axis=2) / 255.0

    figure = plt.figure(figsize=(11, 9), facecolor="white")
    axis = figure.add_subplot(111, projection="3d")
    axis.plot_surface(xx, yy, np.zeros_like(xx), rstride=1, cstride=1,
                      facecolors=rgba, shade=False, antialiased=False,
                      linewidth=0, rasterized=True)
    order = np.argsort(wall_points[:, 2])
    wall_points, wall_colors = wall_points[order], wall_colors[order]
    axis.scatter(wall_points[:, 0], wall_points[:, 1],
                 wall_points[:, 2] - wall_z0,
                 c=np.clip(wall_colors, 0, 1), s=.55, alpha=.92,
                 linewidths=0, depthshade=True, rasterized=True)
    axis.set_xlim(x0, x1)
    axis.set_ylim(y0, y1)
    axis.set_zlim(-.03, args.wall_window + .04)
    axis.set_box_aspect((x1 - x0, y1 - y0,
                         args.wall_window * args.z_exaggeration))
    axis.set_proj_type("ortho")
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.set_axis_off()
    figure.subplots_adjust(left=0, right=1, bottom=0, top=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=240, bbox_inches="tight", pad_inches=.02,
                   facecolor="white")
    plt.close(figure)

    report = {
        "format": "pre_map_vln.habitat_l2_low_wall_scene.v1",
        "floor": 2,
        "base": "official Habitat/HM3D L2 RGB top view",
        "habitat_rgb_source": str(args.habitat_floors.resolve()),
        "habitat_l2_crop_rows": [crop_top, crop_bottom],
        "wall_source": "recorded RGB-D points selected by L2 wall grid",
        "source_episode": str(args.episode.resolve()),
        "source_grid_metadata": str(args.grid_metadata.resolve()),
        "wall_window_m": args.wall_window,
        "wall_source_z_band_m": [wall_z0, wall_z1],
        "wall_point_count": int(len(wall_points)),
        "voxel_m": args.voxel,
        "view": {"azimuth": args.azimuth, "elevation": args.elevation,
                 "projection": "orthographic", "z_exaggeration": args.z_exaggeration},
        "uses_image_generation": False,
        "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
