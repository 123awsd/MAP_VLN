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
DEFAULT_EXCLUDED_NAVIGATION_LABELS = {
    "stair", "stairs", "staircase", "stairway", "stairs railing", "stair railing",
}


def semantic_ids_for_labels(path: Path | None, labels: set[str]) -> set[int]:
    """Read HM3D semantic.txt and return instance ids matching exact normalized labels."""
    if path is None:
        return set()
    normalized = {label.strip().lower() for label in labels}
    result = set()
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.reader(handle):
            if len(row) < 3 or row[2].strip().lower() not in normalized:
                continue
            try:
                result.add(int(row[0]))
            except ValueError:
                continue
    return result


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


def convex_hull(points: np.ndarray) -> np.ndarray:
    """Return a 2-D monotonic-chain hull without adding a geometry dependency."""
    unique = sorted({(int(point[0]), int(point[1])) for point in points})
    if len(unique) <= 2:
        return np.asarray(unique, dtype=np.int32)

    def cross(origin, a, b):
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1], dtype=np.int32)


def group_nearby_instances(instances, maximum_gap_m=0.50):
    """Group stair surfaces and their railings into physical stair assemblies."""
    groups = [{"semantic_ids": [semantic_id], "points": points} for semantic_id, points in instances]
    changed = True
    while changed:
        changed = False
        for first_index in range(len(groups)):
            first = groups[first_index]["points"]
            first_min, first_max = first.min(axis=0), first.max(axis=0)
            for second_index in range(first_index + 1, len(groups)):
                second = groups[second_index]["points"]
                second_min, second_max = second.min(axis=0), second.max(axis=0)
                gap = np.maximum(0.0, np.maximum(first_min - second_max, second_min - first_max))
                if float(np.linalg.norm(gap)) > maximum_gap_m:
                    continue
                groups[first_index] = {
                    "semantic_ids": groups[first_index]["semantic_ids"] + groups[second_index]["semantic_ids"],
                    "points": np.concatenate([first, second], axis=0),
                }
                groups.pop(second_index)
                changed = True
                break
            if changed:
                break
    return groups


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
        "--semantic-labels", type=Path, default=None,
        help="HM3D semantic.txt used to mask stairs from the single-floor map",
    )
    parser.add_argument(
        "--navigation-exclusion-radius", type=float, default=0.20,
        help="metric dilation around stairs and other excluded navigation semantics",
    )
    parser.add_argument(
        "--object-boxes", type=Path, default=None,
        help="Boxer CSV whose semantic instance IDs are removed from structural occupancy",
    )
    args = parser.parse_args()

    manifest = json.loads((args.episode / "manifest.json").read_text(encoding="utf-8"))
    ignored_semantic_ids = non_structural_semantic_ids(args.object_boxes)
    excluded_navigation_ids = semantic_ids_for_labels(
        args.semantic_labels, DEFAULT_EXCLUDED_NAVIGATION_LABELS
    )
    frames = sorted(args.episode.glob("frame_*.npz"))
    if not frames:
        raise FileNotFoundError(f"no recorded frames in {args.episode}")

    ray_sets = []
    excluded_navigation_xy: dict[int, list[np.ndarray]] = {
        semantic_id: [] for semantic_id in excluded_navigation_ids
    }
    all_xy = []
    fx, fy = float(manifest["fx"]), float(manifest["fy"])
    cx, cy = float(manifest["cx"]), float(manifest["cy"])
    for frame_path in frames:
        with np.load(frame_path) as frame:
            depth = frame["depth_m"]
            semantic = frame["semantic"] if "semantic" in frame.files else np.full(depth.shape, -1, dtype=np.int32)
            position = frame["position"].astype(np.float64)
            rotation = quaternion_matrix_xyzw(frame["orientation_xyzw"])
        # Navigation exclusions need a denser semantic projection than the
        # ordinary occupancy-ray stride, otherwise a thin stair edge can be
        # missed and leave a false room beside the stairwell.
        for semantic_id in excluded_navigation_ids:
            excluded_v, excluded_u = np.nonzero(semantic == semantic_id)
            if len(excluded_v) == 0:
                continue
            excluded_v = excluded_v[::2]
            excluded_u = excluded_u[::2]
            excluded_depth = depth[excluded_v, excluded_u]
            excluded_valid = (
                np.isfinite(excluded_depth) & (excluded_depth > 0.15)
                & (excluded_depth <= args.max_depth)
            )
            excluded_v = excluded_v[excluded_valid]
            excluded_u = excluded_u[excluded_valid]
            excluded_depth = excluded_depth[excluded_valid]
            if len(excluded_depth) == 0:
                continue
            excluded_optical = np.stack((
                (excluded_u - cx) * excluded_depth / fx,
                (excluded_v - cy) * excluded_depth / fy,
                excluded_depth,
            ), axis=-1)
            excluded_navigation_xy[semantic_id].append(
                (excluded_optical @ rotation.T + position)[:, :2]
            )
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
        excluded = np.isin(endpoint_semantic_ids, list(excluded_navigation_ids))
        # This project intentionally plans on one 2-D floor. Rays terminating on
        # floors, ceilings, or another stair level must not be flattened into free
        # space on the active level.
        floor_band = (
            (endpoints[:, 2] >= args.obstacle_min_z)
            & (endpoints[:, 2] <= args.obstacle_max_z)
            & ~excluded
        )
        endpoints = endpoints[floor_band]
        endpoint_semantic_ids = endpoint_semantic_ids[floor_band]
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
    navigation_exclusion = np.zeros((height, width), dtype=bool)
    exclusion_image = Image.new("1", (width, height), 0)
    exclusion_draw = ImageDraw.Draw(exclusion_image)
    excluded_navigation_regions = []
    instances = [
        (semantic_id, np.concatenate(chunks, axis=0))
        for semantic_id, chunks in excluded_navigation_xy.items() if chunks
    ]
    for group in group_nearby_instances(instances):
        points = group["points"]
        pixels = np.floor((points - origin) / args.resolution).astype(int)
        hull = convex_hull(pixels)
        if len(hull) >= 3:
            exclusion_draw.polygon([tuple(point) for point in hull], fill=1)
            excluded_navigation_regions.append({
                "semantic_ids": sorted(group["semantic_ids"]),
                "polygon_xy_m": (origin + hull.astype(np.float64) * args.resolution).tolist(),
            })
        else:
            for point in hull:
                exclusion_draw.point(tuple(point), fill=1)
    navigation_exclusion = np.asarray(exclusion_image, dtype=bool)
    if np.any(navigation_exclusion):
        radius = max(1, int(round(args.navigation_exclusion_radius / args.resolution)))
        navigation_exclusion = dilate(navigation_exclusion, radius=radius)
        occupied |= navigation_exclusion
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
        "single_floor_policy": {
            "endpoint_height_band_m": [args.obstacle_min_z, args.obstacle_max_z],
            "semantic_labels": None if args.semantic_labels is None else str(args.semantic_labels),
            "excluded_navigation_labels": sorted(DEFAULT_EXCLUDED_NAVIGATION_LABELS),
            "excluded_navigation_semantic_ids": sorted(excluded_navigation_ids),
            "excluded_navigation_regions": excluded_navigation_regions,
            "exclusion_radius_m": args.navigation_exclusion_radius,
            "excluded_cell_count": int(np.count_nonzero(navigation_exclusion)),
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
