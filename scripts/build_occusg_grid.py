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


DEFAULT_EXCLUDED_NAVIGATION_LABELS = {
    "stair", "stairs", "staircase", "stairway", "stairs railing", "stair railing",
}
DEFAULT_STRUCTURAL_BOUNDARY_LABELS = {
    "wall", "recessed wall", "door frame", "column", "pillar",
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


def policy_boxes(box_path: Path | None, policy_path: Path | None) -> list[dict]:
    """Join Qwen decisions to Boxer rows without confusing detector class IDs with scene IDs."""
    if box_path is None and policy_path is None:
        return []
    if box_path is None or policy_path is None:
        raise ValueError("--object-boxes and --structure-policy must be provided together")
    with box_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    decisions = {str(item["instance_id"]): item for item in policy.get("instances", [])}
    result = []
    for index, row in enumerate(rows):
        instance_id = f"boxer_{index}"
        decision = decisions.get(instance_id)
        if not decision:
            continue
        if str(decision.get("label", "")).strip().lower() != str(row.get("name", "")).strip().lower():
            raise ValueError(f"structure policy no longer matches {instance_id}")
        result.append({"instance_id": instance_id, "decision": decision, "row": row})
    return result


def removable_boxes_from_policy(box_path: Path | None, policy_path: Path | None) -> list[dict]:
    return [
        box for box in policy_boxes(box_path, policy_path)
        if bool(box["decision"].get("remove_from_structure_map", False))
    ]


def quaternion_matrix_wxyz(row: dict) -> np.ndarray:
    w, x, y, z = [float(row[key]) for key in (
        "qw_world_object", "qx_world_object", "qy_world_object", "qz_world_object"
    )]
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm == 0:
        return np.eye(3)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def points_in_removable_boxes(points: np.ndarray, boxes: list[dict], expansion=1.05) -> np.ndarray:
    selected = np.zeros(len(points), dtype=bool)
    for box in boxes:
        row = box["row"]
        center = np.asarray([float(row[key]) for key in (
            "tx_world_object", "ty_world_object", "tz_world_object"
        )])
        half = 0.5 * expansion * np.asarray([float(row[key]) for key in (
            "scale_x", "scale_y", "scale_z"
        )])
        local = (points - center) @ quaternion_matrix_wxyz(row)
        selected |= np.all(np.abs(local) <= half, axis=1)
    return selected


def object_footprint_mask(
    boxes: list[dict], origin, resolution, width, height, expansion: float,
) -> np.ndarray:
    image = Image.new("1", (width, height), 0)
    if not boxes:
        return np.zeros((height, width), dtype=bool)
    draw = ImageDraw.Draw(image)
    for box in boxes:
        row = box["row"]
        center = np.asarray([float(row["tx_world_object"]), float(row["ty_world_object"])])
        # Detector boxes are approximate; a shared uncertainty margin prevents
        # every furniture category from needing its own residual-shell rule.
        half = 0.5 * expansion * np.asarray([float(row["scale_x"]), float(row["scale_y"])])
        rotation = quaternion_matrix_wxyz(row)[:2, :2]
        corners = []
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            point = center + rotation @ (half * np.asarray([sx, sy]))
            pixel = np.floor((point - origin) / resolution).astype(int)
            corners.append((int(pixel[0]), int(pixel[1])))
        draw.polygon(corners, fill=1)
    return np.asarray(image, dtype=bool)


def clear_non_boundary_components(
    grid: np.ndarray, protected_mask: np.ndarray,
) -> tuple[np.ndarray, int, int]:
    """Remove observed obstacle islands that cannot enclose a physical room.

    Enclosure walls are connected to unknown/exterior space. Explicit door or
    structural boxes are protected separately. Remaining isolated components
    are interior obstacles, regardless of their object category.
    """
    occupied = grid == 100
    visited = np.zeros(grid.shape, dtype=bool)
    removed_components = 0
    removed_cells = 0
    height, width = grid.shape
    for start_y, start_x in zip(*np.nonzero(occupied & ~visited)):
        if visited[start_y, start_x]:
            continue
        stack = [(int(start_y), int(start_x))]
        visited[start_y, start_x] = True
        component = []
        keep = False
        while stack:
            y, x = stack.pop()
            component.append((y, x))
            keep |= bool(protected_mask[y, x])
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < height and 0 <= xx < width and occupied[yy, xx] and not visited[yy, xx]:
                        visited[yy, xx] = True
                        stack.append((yy, xx))
        if keep:
            continue
        removed_components += 1
        removed_cells += len(component)
        yy, xx = zip(*component)
        grid[np.asarray(yy), np.asarray(xx)] = 0
    return grid, removed_components, removed_cells


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


def horizontal_endpoint_mask(
    endpoints: np.ndarray,
    camera: np.ndarray,
    max_vertical_delta: float | None,
) -> np.ndarray:
    """Keep depth returns that can represent vertical obstacles on a 2-D floor.

    A depth ray ending on a floor or ceiling has a large vertical displacement
    from the camera, while a wall/column return is comparatively horizontal.
    Rejecting the former prevents those horizontal surfaces from becoming large
    occupied patches when several storeys are collapsed into one XY grid.
    """
    if max_vertical_delta is None:
        return np.ones(len(endpoints), dtype=bool)
    if max_vertical_delta < 0:
        raise ValueError("max_vertical_delta must be non-negative")
    return np.abs(endpoints[:, 2] - float(camera[2])) <= max_vertical_delta


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


def minimum_area_rectangle(points: np.ndarray) -> np.ndarray:
    """Return the smallest oriented rectangle enclosing a 2-D point set.

    A convex hull alone follows the visible stair/railing surfaces and can leave
    the empty projection below a suspended flight unmasked.  The rectangle is
    the conservative footprint of the whole stair assembly on our single floor.
    """
    hull = convex_hull(points).astype(np.float64)
    if len(hull) < 3:
        return hull.astype(np.int32)

    edges = np.roll(hull, -1, axis=0) - hull
    angles = np.unique(np.mod(np.arctan2(edges[:, 1], edges[:, 0]), np.pi / 2.0))
    best_corners = None
    best_area = np.inf
    for angle in angles:
        axis_long = np.asarray([np.cos(angle), np.sin(angle)])
        axis_lat = np.asarray([-np.sin(angle), np.cos(angle)])
        longitudinal = hull @ axis_long
        lateral = hull @ axis_lat
        long_min, long_max = longitudinal.min(), longitudinal.max()
        lat_min, lat_max = lateral.min(), lateral.max()
        area = (long_max - long_min) * (lat_max - lat_min)
        if area >= best_area:
            continue
        best_area = area
        best_corners = np.asarray([
            long_min * axis_long + lat_min * axis_lat,
            long_max * axis_long + lat_min * axis_lat,
            long_max * axis_long + lat_max * axis_lat,
            long_min * axis_long + lat_max * axis_lat,
        ])
    return np.rint(best_corners).astype(np.int32)


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
    parser.add_argument(
        "--camera-height-min", type=float, default=None,
        help="optional lower Falcon-Z bound for frames used in a per-floor grid",
    )
    parser.add_argument(
        "--camera-height-max", type=float, default=None,
        help="optional upper Falcon-Z bound for frames used in a per-floor grid",
    )
    parser.add_argument(
        "--max-endpoint-vertical-delta", type=float, default=None,
        help=(
            "reject depth endpoints whose world-Z differs from the camera by "
            "more than this many metres (useful for floor/ceiling returns)"
        ),
    )
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument(
        "--min-occupied-observations", type=int, default=1,
        help="minimum endpoint observations required to mark an occupied seed",
    )
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
        help="Boxer CSV paired with --structure-policy for room-structure filtering",
    )
    parser.add_argument(
        "--structure-policy", type=Path, default=None,
        help="audited per-instance decisions produced by classify_structure_objects.py",
    )
    parser.add_argument(
        "--object-box-expansion", type=float, default=1.15,
        help="shared uncertainty margin for removable 3-D boxes",
    )
    args = parser.parse_args()
    if args.min_occupied_observations < 1:
        parser.error("--min-occupied-observations must be positive")
    if (args.camera_height_min is None) != (args.camera_height_max is None):
        parser.error("--camera-height-min and --camera-height-max must be provided together")
    if (args.camera_height_min is not None
            and args.camera_height_min >= args.camera_height_max):
        parser.error("camera height minimum must be below maximum")
    if (args.max_endpoint_vertical_delta is not None
            and args.max_endpoint_vertical_delta < 0):
        parser.error("--max-endpoint-vertical-delta cannot be negative")

    manifest = json.loads((args.episode / "manifest.json").read_text(encoding="utf-8"))
    all_policy_boxes = policy_boxes(args.object_boxes, args.structure_policy)
    removable_boxes = [
        box for box in all_policy_boxes
        if bool(box["decision"].get("remove_from_structure_map", False))
    ]
    explicit_boundary_boxes = [
        box for box in all_policy_boxes
        if box["decision"].get("role") in {"structural_boundary", "opening_boundary"}
    ]
    excluded_navigation_ids = semantic_ids_for_labels(
        args.semantic_labels, DEFAULT_EXCLUDED_NAVIGATION_LABELS
    )
    structural_boundary_ids = (
        semantic_ids_for_labels(args.semantic_labels, DEFAULT_STRUCTURAL_BOUNDARY_LABELS)
        if args.structure_policy is not None else set()
    )
    frames = sorted(args.episode.glob("frame_*.npz"))
    if not frames:
        raise FileNotFoundError(f"no recorded frames in {args.episode}")

    ray_sets = []
    rejected_vertical_endpoint_count = 0
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
        if args.camera_height_min is not None and position[2] < args.camera_height_min:
            continue
        if args.camera_height_max is not None and position[2] > args.camera_height_max:
            continue
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
        horizontal = horizontal_endpoint_mask(
            endpoints, position, args.max_endpoint_vertical_delta
        )
        rejected_vertical_endpoint_count += int(np.count_nonzero(~horizontal))
        floor_band = (
            (endpoints[:, 2] >= args.obstacle_min_z)
            & (endpoints[:, 2] <= args.obstacle_max_z)
            & ~excluded
            & horizontal
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
    structural_boundary_votes = np.zeros((height, width), dtype=np.uint16)

    ignored_obstacle_endpoints = 0
    for camera, endpoints, endpoint_semantic_ids in ray_sets:
        start = np.floor((camera[:2] - origin) / args.resolution).astype(int)
        removable_endpoint_mask = points_in_removable_boxes(
            endpoints, removable_boxes, expansion=args.object_box_expansion
        )
        # In simulation this is exact HM3D evidence. The same interface can be
        # supplied by a wall/column segmenter on the real robot.
        removable_endpoint_mask &= ~np.isin(endpoint_semantic_ids, list(structural_boundary_ids))
        for endpoint, semantic_id, removable in zip(
            endpoints, endpoint_semantic_ids, removable_endpoint_mask
        ):
            finish = np.floor((endpoint[:2] - origin) / args.resolution).astype(int)
            cells = list(line_cells(start, finish))
            for x, y in cells[:-1]:
                if 0 <= x < width and 0 <= y < height:
                    free_votes[y, x] = min(65535, int(free_votes[y, x]) + 1)
            x, y = cells[-1]
            if removable:
                ignored_obstacle_endpoints += 1
            elif (args.obstacle_min_z <= endpoint[2] <= args.obstacle_max_z
                    and 0 <= x < width and 0 <= y < height):
                occupied_votes[y, x] = min(65535, int(occupied_votes[y, x]) + 1)
                if int(semantic_id) in structural_boundary_ids:
                    structural_boundary_votes[y, x] = min(
                        65535, int(structural_boundary_votes[y, x]) + 1
                    )

    structural_seeds = occupied_votes >= args.min_occupied_observations
    occupied = dilate(structural_seeds, radius=1)
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
        footprint = minimum_area_rectangle(pixels)
        if len(footprint) >= 3:
            exclusion_draw.polygon([tuple(point) for point in footprint], fill=1)
            excluded_navigation_regions.append({
                "semantic_ids": sorted(group["semantic_ids"]),
                "footprint_method": "minimum_area_rectangle",
                "polygon_xy_m": (
                    origin + footprint.astype(np.float64) * args.resolution
                ).tolist(),
            })
        else:
            for point in footprint:
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
        removable_boxes, origin, args.resolution, width, height, args.object_box_expansion
    )
    # Infer the floor beneath a removed object only inside its fitted footprint.
    # Retained depth endpoints, stairs, and unknown exterior boundaries always win.
    protected_boundary = dilate(grid == -1, radius=2)
    protected_structure = dilate(structural_seeds, radius=1)
    filled_footprint = (
        footprint_mask & ~protected_boundary & ~protected_structure & ~navigation_exclusion
    )
    filled_unknown = filled_footprint & (grid == -1)
    cleared_occupied = filled_footprint & (grid == 100)
    grid[filled_footprint] = 0
    removed_component_count = 0
    removed_component_cell_count = 0
    if args.structure_policy is not None:
        explicit_boundary_mask = object_footprint_mask(
            explicit_boundary_boxes, origin, args.resolution, width, height, 1.10
        )
        component_protection = (
            protected_boundary | navigation_exclusion | explicit_boundary_mask
            | dilate(structural_boundary_votes > 0, radius=1)
        )
        grid, removed_component_count, removed_component_cell_count = clear_non_boundary_components(
            grid, component_protection
        )

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
        "map_role": "room_structure" if args.structure_policy is not None else "navigation_occupancy",
        "min_occupied_observations": args.min_occupied_observations,
        "object_filter": {
            "source_boxes": None if args.object_boxes is None else str(args.object_boxes),
            "structure_policy": None if args.structure_policy is None else str(args.structure_policy),
            "method": (
                "oriented_box_endpoint_filter_and_guarded_floor_completion"
                if args.structure_policy is not None else "none"
            ),
            "box_expansion": args.object_box_expansion,
            "removable_instance_count": len(removable_boxes),
            "removable_instance_ids": [box["instance_id"] for box in removable_boxes],
            "ignored_endpoint_count": ignored_obstacle_endpoints,
            "cleared_occupied_cell_count": int(np.count_nonzero(cleared_occupied)),
            "filled_unknown_cell_count": int(np.count_nonzero(filled_unknown)),
            "completed_footprint_cell_count": int(np.count_nonzero(filled_footprint)),
            "removed_non_boundary_component_count": removed_component_count,
            "removed_non_boundary_component_cell_count": removed_component_cell_count,
            "protected_structural_labels": sorted(DEFAULT_STRUCTURAL_BOUNDARY_LABELS),
            "protected_structural_semantic_ids": sorted(structural_boundary_ids),
            "protected_structural_endpoint_cell_count": int(np.count_nonzero(structural_boundary_votes)),
        },
        "single_floor_policy": {
            "endpoint_height_band_m": [args.obstacle_min_z, args.obstacle_max_z],
            "max_endpoint_vertical_delta_m": args.max_endpoint_vertical_delta,
            "rejected_vertical_endpoint_count": rejected_vertical_endpoint_count,
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
