#!/usr/bin/env python3
"""Render a parameterized, non-color dense depth point-cloud stack."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_dense_rgbd_floor_pointcloud import depth_points
from plot_partial_exploration_pointcloud import voxel_first


def add_soft_image_shadow(path: Path, strength: float, blur_px: float,
                          offset_px: tuple[int, int]) -> None:
    """Add a screen-space shadow behind rendered pixels without thickening them."""
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


def load_floor_clouds(episode: Path, grid_dir: Path, floors: list[int], stride: int,
                      maximum_depth: float, voxel: float, lower: float,
                      upper: float) -> tuple[dict[int, np.ndarray], dict[int, dict]]:
    frames = sorted(episode.glob("frame_*.npz"))
    clouds: dict[int, np.ndarray] = {}
    reports: dict[int, dict] = {}
    for floor in floors:
        metadata = json.loads((grid_dir / f"L{floor}_wall_grid.json").read_text())
        floor_z = float(metadata["floor_z_m"])
        origin = np.asarray(metadata["origin_xy_m"], dtype=np.float32)
        resolution = float(metadata["resolution_m"])
        extent = np.asarray([
            origin[0], origin[0] + int(metadata["width"]) * resolution,
            origin[1], origin[1] + int(metadata["height"]) * resolution,
        ])
        parts: list[np.ndarray] = []
        for frame in frames:
            points = depth_points(frame, stride, maximum_depth)
            keep = (
                (points[:, 2] >= floor_z + lower)
                & (points[:, 2] <= floor_z + upper)
                & (points[:, 0] >= extent[0]) & (points[:, 0] <= extent[1])
                & (points[:, 1] >= extent[2]) & (points[:, 1] <= extent[3])
            )
            if np.any(keep):
                parts.append(points[keep])
        if not parts:
            raise RuntimeError(f"No depth points remain for floor L{floor}")
        raw = np.concatenate(parts)
        fused, _ = voxel_first(raw, voxel)
        clouds[floor] = fused
        reports[floor] = {
            "floor_z_m": floor_z,
            "z_band_m": [floor_z + lower, floor_z + upper],
            "raw_points": int(len(raw)),
            "fused_points": int(len(fused)),
            "xy_extent_m": extent.tolist(),
        }
    return clouds, reports


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("grid_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--floors", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--maximum-depth", type=float, default=5.0)
    parser.add_argument("--voxel", type=float, default=0.025)
    parser.add_argument("--lower", type=float, default=0.10)
    parser.add_argument("--upper", type=float, default=2.30)
    parser.add_argument("--stack-gap", type=float, default=2.2,
                        help="Display gap between flat floor planes")
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=28.0)
    parser.add_argument("--point-size", type=float, default=0.45)
    parser.add_argument("--wall-size", type=float, default=1.6,
                        help="Size of shared wall-grid structure points")
    parser.add_argument("--no-wall-overlay", action="store_true",
                        help="Do not overlay the emphasized wall-grid edge")
    parser.add_argument("--floor-shadow", action="store_true",
                        help="Draw a soft footprint shadow below each flat cloud")
    parser.add_argument("--shadow-strength", type=float, default=.13)
    parser.add_argument("--shadow-blur", type=float, default=18.0,
                        help="Screen-space shadow blur in pixels")
    parser.add_argument("--shadow-offset", type=int, nargs=2, default=[12, 12],
                        metavar=("X", "Y"), help="Screen-space shadow offset in pixels")
    parser.add_argument("--z-aspect", type=float, default=1.35,
                        help="Visual emphasis of the vertical stack dimension")
    args = parser.parse_args()

    clouds, reports = load_floor_clouds(
        args.episode, args.grid_dir, args.floors, args.pixel_stride,
        args.maximum_depth, args.voxel, args.lower, args.upper,
    )
    all_points = np.concatenate(list(clouds.values()))
    xmin, ymin = all_points[:, :2].min(axis=0)
    xmax, ymax = all_points[:, :2].max(axis=0)
    padx, pady = (xmax - xmin) * 0.05, (ymax - ymin) * 0.05

    figure = plt.figure(figsize=(12, 10), facecolor="white")
    axis = figure.add_subplot(111, projection="3d")
    color = "#123f5a"
    for index, floor in enumerate(sorted(clouds)):
        metadata = json.loads((args.grid_dir / f"L{floor}_wall_grid.json").read_text())
        grid = np.load(args.grid_dir / f"L{floor}_wall_grid.npy")
        origin = np.asarray(metadata["origin_xy_m"], dtype=float)
        resolution = float(metadata["resolution_m"])
        height, width = grid.shape
        if not args.no_wall_overlay:
            xs = origin[0] + (np.arange(width) + 0.5) * resolution
            ys = origin[1] + (np.arange(height) + 0.5) * resolution
            xx, yy = np.meshgrid(xs, ys)
            wall = grid == 100
            wall_xy = np.column_stack([xx[wall], yy[wall]])
            wall_z = np.full(len(wall_xy), index * args.stack_gap, dtype=np.float32)
            axis.scatter(wall_xy[:, 0], wall_xy[:, 1], wall_z, s=args.wall_size,
                         c="#64748b", marker="s", alpha=.82, linewidths=0,
                         depthshade=False, rasterized=True)
        points = clouds[floor]
        z = np.full(len(points), index * args.stack_gap, dtype=np.float32)
        axis.scatter(points[:, 0], points[:, 1], z, s=args.point_size,
                     c=color, marker=".", linewidths=0, alpha=0.88,
                     depthshade=False, rasterized=True)

    axis.set_xlim(xmin - padx, xmax + padx)
    axis.set_ylim(ymin - pady, ymax + pady)
    axis.set_zlim(-args.stack_gap * 0.2, args.stack_gap * (len(clouds) - 1) + args.stack_gap * 0.3)
    axis.set_box_aspect((xmax - xmin, ymax - ymin,
                         args.stack_gap * len(clouds) * args.z_aspect))
    axis.set_proj_type("ortho")
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.set_axis_off()
    figure.subplots_adjust(left=0, right=1, bottom=0, top=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=220, facecolor="white",
                   bbox_inches="tight", pad_inches=0.02)
    plt.close(figure)
    if args.floor_shadow:
        add_soft_image_shadow(
            args.output, args.shadow_strength, args.shadow_blur,
            tuple(args.shadow_offset),
        )

    report = {
        "format": "pre_map_vln.multifloor_dense_depth_stack.v1",
        "source_episode": str(args.episode.resolve()),
        "source_grid_dir": str(args.grid_dir.resolve()),
        "floors": sorted(clouds),
        "stack_gap": args.stack_gap,
        "azimuth": args.azimuth,
        "elevation": args.elevation,
        "z_aspect": args.z_aspect,
        "voxel_m": args.voxel,
        "pixel_stride": args.pixel_stride,
        "color_mode": "depth-only",
        "wall_overlay": not args.no_wall_overlay,
        "style": {
            "floor_shadow": args.floor_shadow,
            "shadow_strength": args.shadow_strength,
            "shadow_blur": args.shadow_blur,
            "shadow_offset_px": args.shadow_offset,
            "shadow_mode": "screen_space_postprocess",
        },
        "floor_reports": reports,
        "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
