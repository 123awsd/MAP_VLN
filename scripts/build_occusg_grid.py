#!/usr/bin/env python3
"""Turn a recorded Habitat RGB-D episode into a ROS-compatible 2-D grid."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def quaternion_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion.astype(np.float64)
    norm = np.linalg.norm(quaternion)
    if norm == 0:
        raise ValueError("zero-norm camera quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def line_cells(start, end):
    """Integer Bresenham cells, including both endpoints."""
    x0, y0 = int(start[0]), int(start[1])
    x1, y1 = int(end[0]), int(end[1])
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def dilate(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    result = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            y0, y1 = max(0, dy), min(mask.shape[0], mask.shape[0] + dy)
            x0, x1 = max(0, dx), min(mask.shape[1], mask.shape[1] + dx)
            result[y0:y1, x0:x1] |= mask[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("output_prefix", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--max-depth", type=float, default=8.0)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--obstacle-min-z", type=float, default=0.15)
    parser.add_argument("--obstacle-max-z", type=float, default=1.8)
    args = parser.parse_args()

    manifest = json.loads((args.episode / "manifest.json").read_text(encoding="utf-8"))
    frames = sorted(args.episode.glob("frame_*.npz"))
    if not frames:
        raise FileNotFoundError(f"no recorded frames in {args.episode}")

    ray_sets = []
    all_xy = []
    fx, fy = float(manifest["fx"]), float(manifest["fy"])
    cx, cy = float(manifest["cx"]), float(manifest["cy"])
    for frame_path in frames:
        with np.load(frame_path) as frame:
            depth = frame["depth_m"]
            position = frame["position"].astype(np.float64)
            rotation = quaternion_matrix_xyzw(frame["orientation_xyzw"])
        rows = np.arange(0, depth.shape[0], args.stride)
        cols = np.arange(0, depth.shape[1], args.stride)
        vv, uu = np.meshgrid(rows, cols, indexing="ij")
        zz = depth[vv, uu]
        valid = np.isfinite(zz) & (zz > 0.15) & (zz <= args.max_depth)
        optical = np.stack(((uu - cx) * zz / fx, (vv - cy) * zz / fy, zz), axis=-1)
        endpoints = optical[valid] @ rotation.T + position
        if endpoints.size == 0:
            continue
        ray_sets.append((position, endpoints))
        all_xy.extend((position[:2], endpoints[:, :2].min(axis=0), endpoints[:, :2].max(axis=0)))
    if not ray_sets:
        raise RuntimeError("episode contains no usable depth rays")

    extent = np.asarray(all_xy)
    origin = extent.min(axis=0) - args.padding
    upper = extent.max(axis=0) + args.padding
    width, height = np.ceil((upper - origin) / args.resolution).astype(int) + 1
    free_votes = np.zeros((height, width), dtype=np.uint16)
    occupied_votes = np.zeros((height, width), dtype=np.uint16)

    for camera, endpoints in ray_sets:
        start = np.floor((camera[:2] - origin) / args.resolution).astype(int)
        for endpoint in endpoints:
            finish = np.floor((endpoint[:2] - origin) / args.resolution).astype(int)
            cells = list(line_cells(start, finish))
            for x, y in cells[:-1]:
                if 0 <= x < width and 0 <= y < height:
                    free_votes[y, x] = min(65535, int(free_votes[y, x]) + 1)
            x, y = cells[-1]
            if (args.obstacle_min_z <= endpoint[2] <= args.obstacle_max_z
                    and 0 <= x < width and 0 <= y < height):
                occupied_votes[y, x] = min(65535, int(occupied_votes[y, x]) + 1)

    occupied = dilate(occupied_votes > 0, radius=1)
    grid = np.full((height, width), -1, dtype=np.int8)
    grid[free_votes > 0] = 0
    grid[occupied] = 100

    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output_prefix.with_suffix(".npy"), grid)
    metadata = {
        "format": "pre_map_vln.occupancy_grid.v1",
        "source_episode": str(args.episode),
        "coordinate_frame": manifest["coordinate_frame"],
        "resolution_m": args.resolution,
        "origin_xy_m": origin.tolist(),
        "width": int(width),
        "height": int(height),
        "frame_count": len(frames),
        "free_cells": int(np.count_nonzero(grid == 0)),
        "occupied_cells": int(np.count_nonzero(grid == 100)),
        "unknown_cells": int(np.count_nonzero(grid == -1)),
    }
    args.output_prefix.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    preview = np.zeros((height, width, 3), dtype=np.uint8)
    preview[grid == -1] = (90, 90, 90)
    preview[grid == 0] = (245, 245, 245)
    preview[grid == 100] = (20, 20, 20)
    Image.fromarray(np.flipud(preview)).save(args.output_prefix.with_suffix(".png"))
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
