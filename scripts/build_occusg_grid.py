#!/usr/bin/env python3
"""Turn a recorded Habitat RGB-D episode into a ROS-compatible 2-D grid."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


NON_STRUCTURAL_LABELS = {
    "bed", "cabinet", "chair", "lamp", "plant", "shelf", "television", "tv",
}
FOOTPRINT_CLEAR_LABELS = {"bed"}


def non_structural_semantic_ids(path: Path | None) -> set[int]:
    if path is None:
        return set()
    result = set()
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if str(row.get("name", "")).strip().lower() not in NON_STRUCTURAL_LABELS:
                continue
            try:
                result.add(int(float(row["sem_id"])))
            except (KeyError, TypeError, ValueError):
                continue
    return result


def object_footprint_mask(path: Path | None, origin, resolution, width, height) -> np.ndarray:
    image = Image.new("1", (width, height), 0)
    if path is None:
        return np.zeros((height, width), dtype=bool)
    draw = ImageDraw.Draw(image)
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if str(row.get("name", "")).strip().lower() not in FOOTPRINT_CLEAR_LABELS:
                continue
            if float(row["scale_x"]) * float(row["scale_y"]) < 1.0:
                continue
            center = np.asarray([float(row["tx_world_object"]), float(row["ty_world_object"])])
            # Expand 10% beyond the fitted OBB so the original occupancy
            # dilation does not leave a furniture-shaped residual outline.
            half = 0.55 * np.asarray([float(row["scale_x"]), float(row["scale_y"])])
            w, x, y, z = [float(row[key]) for key in ("qw_world_object", "qx_world_object", "qy_world_object", "qz_world_object")]
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            corners = []
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                point = center + rotation @ (half * np.asarray([sx, sy]))
                pixel = np.floor((point - origin) / resolution).astype(int)
                corners.append((int(pixel[0]), int(pixel[1])))
            draw.polygon(corners, fill=1)
    return np.asarray(image, dtype=bool)


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
    parser.add_argument(
        "--object-boxes", type=Path, default=None,
        help="Boxer CSV whose semantic instance IDs are removed from structural occupancy",
    )
    args = parser.parse_args()

    manifest = json.loads((args.episode / "manifest.json").read_text(encoding="utf-8"))
    ignored_semantic_ids = non_structural_semantic_ids(args.object_boxes)
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
            semantic = frame["semantic"] if "semantic" in frame.files else np.full(depth.shape, -1, dtype=np.int32)
            position = frame["position"].astype(np.float64)
            rotation = quaternion_matrix_xyzw(frame["orientation_xyzw"])
        rows = np.arange(0, depth.shape[0], args.stride)
        cols = np.arange(0, depth.shape[1], args.stride)
        vv, uu = np.meshgrid(rows, cols, indexing="ij")
        zz = depth[vv, uu]
        valid = np.isfinite(zz) & (zz > 0.15) & (zz <= args.max_depth)
        optical = np.stack(((uu - cx) * zz / fx, (vv - cy) * zz / fy, zz), axis=-1)
        endpoints = optical[valid] @ rotation.T + position
        endpoint_semantic_ids = semantic[vv, uu][valid].astype(np.int64, copy=False)
        if endpoints.size == 0:
            continue
        ray_sets.append((position, endpoints, endpoint_semantic_ids))
        all_xy.extend((position[:2], endpoints[:, :2].min(axis=0), endpoints[:, :2].max(axis=0)))
    if not ray_sets:
        raise RuntimeError("episode contains no usable depth rays")

    extent = np.asarray(all_xy)
    origin = extent.min(axis=0) - args.padding
    upper = extent.max(axis=0) + args.padding
    width, height = np.ceil((upper - origin) / args.resolution).astype(int) + 1
    free_votes = np.zeros((height, width), dtype=np.uint16)
    occupied_votes = np.zeros((height, width), dtype=np.uint16)

    ignored_obstacle_endpoints = 0
    for camera, endpoints, endpoint_semantic_ids in ray_sets:
        start = np.floor((camera[:2] - origin) / args.resolution).astype(int)
        for endpoint, semantic_id in zip(endpoints, endpoint_semantic_ids):
            finish = np.floor((endpoint[:2] - origin) / args.resolution).astype(int)
            cells = list(line_cells(start, finish))
            for x, y in cells[:-1]:
                if 0 <= x < width and 0 <= y < height:
                    free_votes[y, x] = min(65535, int(free_votes[y, x]) + 1)
            x, y = cells[-1]
            if int(semantic_id) in ignored_semantic_ids:
                ignored_obstacle_endpoints += 1
            elif (args.obstacle_min_z <= endpoint[2] <= args.obstacle_max_z
                    and 0 <= x < width and 0 <= y < height):
                occupied_votes[y, x] = min(65535, int(occupied_votes[y, x]) + 1)

    occupied = dilate(occupied_votes > 0, radius=1)
    grid = np.full((height, width), -1, dtype=np.int8)
    grid[free_votes > 0] = 0
    grid[occupied] = 100
    footprint_mask = object_footprint_mask(
        args.object_boxes, origin, args.resolution, width, height
    )
    # Never clear near unknown space: this protects exterior and incompletely
    # observed walls even when an object OBB overlaps them.
    protected_boundary = dilate(grid == -1, radius=2)
    cleared_footprint = footprint_mask & (grid == 100) & ~protected_boundary
    grid[cleared_footprint] = 0

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
        "object_filter": {
            "source_boxes": None if args.object_boxes is None else str(args.object_boxes),
            "semantic_instance_count": len(ignored_semantic_ids),
            "ignored_endpoint_count": ignored_obstacle_endpoints,
            "cleared_footprint_cell_count": int(np.count_nonzero(cleared_footprint)),
            "labels": sorted(NON_STRUCTURAL_LABELS),
            "footprint_clear_labels": sorted(FOOTPRINT_CLEAR_LABELS),
        },
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
