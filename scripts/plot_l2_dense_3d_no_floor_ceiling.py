#!/usr/bin/env python3
"""Render a flat L2 RGB-D overview with only measured wall points kept in 3D."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_dense_rgbd_floor_pointcloud import depth_points
from plot_partial_exploration_pointcloud import voxel_first


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("grid_metadata", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--maximum-depth", type=float, default=5.0)
    parser.add_argument("--voxel", type=float, default=0.03)
    parser.add_argument("--slice-center-above-floor", type=float, default=1.20)
    parser.add_argument("--slice-height", type=float, default=0.50)
    parser.add_argument("--overview-floor-clearance", type=float, default=0.18)
    parser.add_argument("--overview-ceiling-clearance", type=float, default=0.65)
    parser.add_argument("--storey-height", type=float, default=2.80)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=26.0)
    parser.add_argument("--z-exaggeration", type=float, default=1.55)
    args = parser.parse_args()

    metadata = json.loads(args.grid_metadata.read_text())
    floor_z = float(metadata["floor_z_m"])
    slice_center_z = floor_z + args.slice_center_above_floor
    z_min = slice_center_z - args.slice_height / 2.0
    z_max = slice_center_z + args.slice_height / 2.0
    origin = np.asarray(metadata["origin_xy_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    x_min = float(origin[0])
    x_max = x_min + int(metadata["width"]) * resolution
    y_min = float(origin[1])
    y_max = y_min + int(metadata["height"]) * resolution

    pieces = []
    frames = sorted(args.episode.glob("frame_*.npz"))
    overview_min = floor_z + args.overview_floor_clearance
    overview_max = floor_z + args.storey_height - args.overview_ceiling_clearance
    for frame in frames:
        points = depth_points(frame, args.pixel_stride, args.maximum_depth)
        keep = (
            (points[:, 2] >= overview_min) & (points[:, 2] <= overview_max)
            & (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
            & (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
        )
        if np.any(keep):
            pieces.append(points[keep])
    if not pieces:
        raise RuntimeError("No L2 points remain after floor/ceiling filtering")
    raw_points = np.concatenate(pieces)
    points, _ = voxel_first(raw_points, args.voxel)

    # Use the measured wall grid only as a selector. No wall geometry is
    # synthesized: selected points are still the original RGB-D samples.
    wall_grid = np.load(args.grid_metadata.with_name(args.grid_metadata.stem + ".npy"))
    gx = np.floor((points[:, 0] - x_min) / resolution).astype(int)
    gy = np.floor((points[:, 1] - y_min) / resolution).astype(int)
    valid_cell = ((gx >= 0) & (gx < wall_grid.shape[1])
                  & (gy >= 0) & (gy < wall_grid.shape[0]))
    wall_selector = np.zeros(len(points), dtype=bool)
    wall_selector[valid_cell] = wall_grid[gy[valid_cell], gx[valid_cell]] == 100
    wall_selector &= (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    wall_points = points[wall_selector]

    # The overview is a conventional top projection. Only points classified as
    # wall returns keep their measured height within the requested 0.3 m slice.
    points = points[np.argsort(points[:, 2])]
    figure = plt.figure(figsize=(10, 9), facecolor="white")
    axis = figure.add_subplot(111, projection="3d")
    base_z = z_min - 0.035
    axis.scatter(points[:, 0], points[:, 1], np.full(len(points), base_z),
                 s=.25, c="#527d94", alpha=.50, linewidths=0,
                 depthshade=False, rasterized=True)
    axis.scatter(wall_points[:, 0], wall_points[:, 1], wall_points[:, 2],
                 s=.42, c="#164e70", alpha=.90, linewidths=0,
                 depthshade=True, rasterized=True)
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    axis.set_zlim(base_z - .02, z_max)
    axis.set_box_aspect((x_max - x_min, y_max - y_min,
                         (z_max - z_min) * args.z_exaggeration))
    axis.set_proj_type("ortho")
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.set_axis_off()
    figure.subplots_adjust(left=0, right=1, bottom=0, top=1)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=240, bbox_inches="tight", pad_inches=.02,
                   facecolor="white")
    plt.close(figure)

    report = {
        "format": "pre_map_vln.l2_flat_overview_3d_walls.v1",
        "floor": int(metadata.get("floor", 2)),
        "source_episode": str(args.episode.resolve()),
        "source_grid_metadata": str(args.grid_metadata.resolve()),
        "recorded_frames": len(frames),
        "filter": {
            "floor_z_m": floor_z,
            "kept_z_band_m": [z_min, z_max],
            "slice_center_above_floor_m": args.slice_center_above_floor,
            "slice_height_m": args.slice_height,
            "overview_source_z_band_m": [overview_min, overview_max],
        },
        "raw_filtered_points": int(len(raw_points)),
        "voxelized_points": int(len(points)),
        "measured_wall_points_in_3d": int(len(wall_points)),
        "non_wall_rendering": "all overview points projected to one horizontal plane",
        "wall_rendering": "RGB-D points selected by wall grid retain measured Z",
        "voxel_m": args.voxel,
        "pixel_stride": args.pixel_stride,
        "uses_rgb": False,
        "geometry_source": "recorded RGB-D depth",
        "view": {"azimuth": args.azimuth, "elevation": args.elevation,
                 "projection": "orthographic", "z_exaggeration": args.z_exaggeration},
        "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
