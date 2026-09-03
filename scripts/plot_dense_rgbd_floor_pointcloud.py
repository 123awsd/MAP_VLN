#!/usr/bin/env python3
"""Fuse recorded RGB-D frames into a dense, real-color floor point cloud."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from plot_partial_exploration_pointcloud import quaternion_matrix_xyzw, rgbd_points, voxel_first


def depth_points(frame_path, stride, maximum_depth):
    """Back-project geometry without loading or using the RGB image."""
    with np.load(frame_path) as frame:
        depth = frame["depth_m"][::stride, ::stride].astype(np.float64)
        position = frame["position"].astype(np.float64)
        rotation = quaternion_matrix_xyzw(frame["orientation_xyzw"])
    rows, columns = np.mgrid[0:480:stride, 0:640:stride]
    z = depth.reshape(-1)
    valid = np.isfinite(z) & (z > 0.15) & (z <= maximum_depth)
    optical = np.stack(((columns.reshape(-1) - 320.0) * z / 320.0,
                        (rows.reshape(-1) - 240.0) * z / 320.0, z), axis=1)
    return (optical[valid] @ rotation.T + position).astype(np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("grid_metadata", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--maximum-depth", type=float, default=5.0)
    parser.add_argument("--voxel", type=float, default=0.025)
    parser.add_argument("--lower", type=float, default=0.10)
    parser.add_argument("--upper", type=float, default=2.30)
    parser.add_argument("--color-mode", choices=("rgb", "depth-only"), default="rgb")
    args = parser.parse_args()

    metadata = json.loads(args.grid_metadata.read_text())
    floor_z = float(metadata["floor_z_m"])
    origin = np.asarray(metadata["origin_xy_m"], dtype=np.float32)
    resolution = float(metadata["resolution_m"])
    extent = np.asarray([
        origin[0], origin[0] + int(metadata["width"]) * resolution,
        origin[1], origin[1] + int(metadata["height"]) * resolution,
    ])
    point_parts, color_parts = [], []
    frames = sorted(args.episode.glob("frame_*.npz"))
    for frame in frames:
        if args.color_mode == "rgb":
            points, colors, _ = rgbd_points(frame, args.pixel_stride, args.maximum_depth)
        else:
            points = depth_points(frame, args.pixel_stride, args.maximum_depth)
            colors = None
        keep = (
            (points[:, 2] >= floor_z + args.lower)
            & (points[:, 2] <= floor_z + args.upper)
            & (points[:, 0] >= extent[0]) & (points[:, 0] <= extent[1])
            & (points[:, 1] >= extent[2]) & (points[:, 1] <= extent[3])
        )
        if np.any(keep):
            point_parts.append(points[keep])
            if colors is not None:
                color_parts.append(colors[keep])
    if not point_parts:
        raise RuntimeError("No RGB-D points remain in requested floor band")
    raw_points = np.concatenate(point_parts)
    if args.color_mode == "rgb":
        raw_colors = np.concatenate(color_parts)
        points, colors = voxel_first(raw_points, args.voxel, raw_colors)
    else:
        points, _ = voxel_first(raw_points, args.voxel)
        colors = None
    # Draw high returns last, matching an orthographic top-down point-cloud view.
    order = np.argsort(points[:, 2])
    points = points[order]
    if colors is not None:
        colors = colors[order]

    def draw(path, clean):
        figure, axis = plt.subplots(1, 1, figsize=(9, 9), facecolor="white")
        point_colors = np.clip(colors, 0, 1) if colors is not None else "#123f5a"
        axis.scatter(points[:, 0], points[:, 1], c=point_colors,
                     s=1.05, marker=".", linewidths=0, alpha=1.0,
                     rasterized=True)
        axis.set_xlim(extent[0], extent[1]); axis.set_ylim(extent[2], extent[3])
        axis.set_aspect("equal"); axis.grid(False)
        if clean:
            axis.axis("off"); figure.subplots_adjust(left=0, right=1, bottom=0, top=1)
        else:
            title_source = "RGB-D" if args.color_mode == "rgb" else "depth"
            axis.set_title(f"L2 | fused {title_source} point cloud (top view)")
            axis.set_xlabel("FALCON X (m)"); axis.set_ylabel("FALCON Y (m)")
            figure.tight_layout()
        figure.savefig(path, dpi=220, bbox_inches="tight", pad_inches=0 if clean else .1,
                       facecolor="white")
        figure.savefig(path.with_suffix(".pdf"), bbox_inches="tight",
                       pad_inches=0 if clean else .1, facecolor="white")
        plt.close(figure)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cloud_cache = args.output.with_name(args.output.stem + "_cloud.npz")
    np.savez_compressed(cloud_cache, points=points.astype(np.float32))
    draw(args.output, False)
    clean = args.output.with_name(args.output.stem + "_clean" + args.output.suffix)
    draw(clean, True)
    report = {
        "format": "pre_map_vln.dense_floor_pointcloud.v2",
        "source_episode": str(args.episode.resolve()), "recorded_frames": len(frames),
        "floor": int(metadata.get("floor", 2)), "floor_z_m": floor_z,
        "z_band_m": [floor_z + args.lower, floor_z + args.upper],
        "raw_input_points": int(len(raw_points)), "fused_voxel_points": int(len(points)),
        "voxel_m": args.voxel, "pixel_stride": args.pixel_stride,
        "color_mode": args.color_mode, "uses_rgb": args.color_mode == "rgb",
        "xy_extent_m": extent.tolist(), "png": str(args.output.resolve()),
        "clean_png": str(clean.resolve()), "cloud_cache": str(cloud_cache.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
