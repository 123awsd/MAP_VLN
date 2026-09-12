#!/usr/bin/env python3
"""Stack the original Habitat L2 RGB plate above mapping, semantics and Boxer planes."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from plot_dense_rgbd_floor_pointcloud import depth_points
from plot_partial_exploration_pointcloud import voxel_first
from plot_multifloor_semantic_stack import semantic_raster
from plot_multifloor_box_stack import box_corners, draw_cuboid, dashed_outer_outline


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("episode", type=Path)
    p.add_argument("grid_dir", type=Path)
    p.add_argument("scene_dir", type=Path)
    p.add_argument("box_dir", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--habitat-rgb", type=Path,
                   default=Path("/home/uav/map_VLN/PRE_MAP_VLN/docs/assets/system_overview/official_scene/topdown_floors.png"))
    p.add_argument("--habitat-l2-crop", type=int, nargs=2, default=(1266, 2382),
                   metavar=("TOP", "BOTTOM"),
                   help="原始三层 Habitat 楼层图中 L2 的行裁剪范围")
    p.add_argument("--stack-gap", type=float, default=4.2)
    p.add_argument("--azimuth", type=float, default=-58.0)
    p.add_argument("--elevation", type=float, default=26.0)
    p.add_argument("--z-aspect", type=float, default=1.5)
    args = p.parse_args()
    floor = 2
    meta = json.loads((args.grid_dir / "L2_wall_grid.json").read_text())
    grid = np.load(args.grid_dir / "L2_wall_grid.npy")
    origin = np.asarray(meta["origin_xy_m"], dtype=float)
    resolution = float(meta["resolution_m"])
    h, w = grid.shape
    x_edges = origin[0] + np.arange(w + 1) * resolution
    y_edges = origin[1] + np.arange(h + 1) * resolution
    xx, yy = np.meshgrid(x_edges, y_edges)
    x0, x1, y0, y1 = x_edges[0], x_edges[-1], y_edges[0], y_edges[-1]
    floor_z = float(meta["floor_z_m"])

    # Original L2 dense depth cloud.
    parts = []
    for frame in sorted(args.episode.glob("frame_*.npz")):
        pts = depth_points(frame, 2, 5.0)
        keep = ((pts[:, 2] >= floor_z + .10) & (pts[:, 2] <= floor_z + 2.30) &
                (pts[:, 0] >= x0) & (pts[:, 0] <= x1) &
                (pts[:, 1] >= y0) & (pts[:, 1] <= y1))
        if np.any(keep): parts.append(pts[keep])
    cloud, _ = voxel_first(np.concatenate(parts), .025)

    graph = json.loads((args.scene_dir / "L2_scene_graph_qwen.json").read_text())
    semantic = semantic_raster(grid, origin, resolution, graph["rooms"], .22)
    # Official HM3D/Habitat RGB top view.  The source sheet contains L1/L2/L3
    # vertically; use only the original L2 plate and map it to the same metric
    # XY extent as the other three layers.  Black exterior canvas is transparent
    # so it does not become a large opaque rectangle in the stack.
    rgb_full = Image.open(args.habitat_rgb).convert("RGB")
    crop_top, crop_bottom = args.habitat_l2_crop
    rgb_l2 = np.asarray(rgb_full.crop((0, crop_top, rgb_full.width, crop_bottom)))
    # Match the official renderer's axis convention: u=-FalconY and v=-FalconX.
    # Sampling at the metric grid coordinates fixes the apparent 90/180-degree
    # mismatch without rotating or otherwise editing the original RGB pixels.
    foreground = rgb_l2.max(axis=2) > 25
    py, px = np.where(foreground)
    u0, u1, v0, v1 = float(px.min()), float(px.max()), float(py.min()), float(py.max())
    uu = u0 + (y1 - yy) / (y1 - y0) * (u1 - u0)
    vv = v0 + (x1 - xx) / (x1 - x0) * (v1 - v0)
    ui = np.clip(np.rint(uu).astype(int), 0, rgb_l2.shape[1] - 1)
    vi = np.clip(np.rint(vv).astype(int), 0, rgb_l2.shape[0] - 1)
    rgb_sampled = rgb_l2[vi, ui]
    rgb_rgba = np.concatenate([rgb_sampled, np.where(rgb_sampled.max(axis=2, keepdims=True) <= 25, 0, 255).astype(np.uint8)], axis=2) / 255.0
    with (args.box_dir / "L2_boxes.csv").open(newline="") as f:
        boxes = list(csv.DictReader(f))

    fig = plt.figure(figsize=(12, 10), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    zs = [3 * args.stack_gap, 2 * args.stack_gap, args.stack_gap, 0.0]
    # Top: original Habitat L2 RGB floor plate.
    ax.plot_surface(xx, yy, np.full_like(xx, zs[0]), rstride=1, cstride=1,
                    facecolors=rgb_rgba, shade=False, antialiased=False,
                    linewidth=0, rasterized=True)
    # Second: point cloud.
    ax.scatter(cloud[:, 0], cloud[:, 1], np.full(len(cloud), zs[1]),
               s=.22, c="#164e70", alpha=.82, linewidths=0, rasterized=True)
    # Middle: filled semantic raster.
    ax.plot_surface(xx, yy, np.full_like(xx, zs[2]), rstride=1, cstride=1,
                    facecolors=semantic, shade=False, antialiased=False,
                    linewidth=0, rasterized=True)
    # Bottom: true 3D oriented Boxer cuboids.
    ax.plot_surface(xx, yy, np.full_like(xx, zs[3] - .035), rstride=1, cstride=1,
                    facecolors=np.zeros((*grid.shape, 4)), shade=False,
                    antialiased=False, linewidth=0, rasterized=True)
    # Match the Boxer figure: one-cell, closed dashed outer floor outline only.
    outer = dashed_outer_outline(grid, 7, 5)
    oy, ox = np.where(outer)
    ax.scatter(origin[0] + (ox + .5) * resolution,
               origin[1] + (oy + .5) * resolution,
               np.full(len(ox), zs[3] + .015), s=1.6, c="#657f99",
               marker="s", alpha=.96, linewidths=0, depthshade=False,
               rasterized=True)
    for row in boxes:
        draw_cuboid(ax, row, zs[3], floor_z, .62)

    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)
    ax.set_zlim(-.5, zs[0] + 1.8)
    ax.set_box_aspect((x1 - x0, y1 - y0, args.stack_gap * 4 * args.z_aspect))
    ax.set_proj_type("ortho"); ax.view_init(elev=args.elevation, azim=args.azimuth)
    ax.set_axis_off(); fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=220, bbox_inches="tight", pad_inches=.02, facecolor="white")
    plt.close(fig)
    report = {
        "format": "pre_map_vln.l2_three_stage_stack.v1",
        "floor": 2, "stack_gap": args.stack_gap, "azimuth": args.azimuth,
        "elevation": args.elevation, "z_aspect": args.z_aspect,
        "stages_top_to_bottom": ["habitat_l2_rgb_original", "pointcloud_mapping", "room_segmentation", "boxer_detection"],
        "habitat_rgb_source": str(args.habitat_rgb.resolve()),
        "habitat_l2_crop_rows": [crop_top, crop_bottom],
        "box_outer_outline": {"mode": "outer-dashed", "dash_cells": [7, 5], "color": "#657f99"},
        "source_episode": str(args.episode.resolve()),
        "source_scene_dir": str(args.scene_dir.resolve()),
        "source_box_csv": str((args.box_dir / "L2_boxes.csv").resolve()),
        "point_count": int(len(cloud)), "room_count": len(graph.get("rooms", [])),
        "box_count": len(boxes), "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
