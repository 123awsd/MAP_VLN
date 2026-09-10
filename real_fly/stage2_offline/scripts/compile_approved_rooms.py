#!/usr/bin/env python3
"""Validate a manually reviewed room map and compile a planning scene graph."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import cv2
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
import numpy as np


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def area(vertices) -> float:
    points = np.asarray(vertices, dtype=float)
    return 0.5 * abs(float(np.dot(points[:, 0], np.roll(points[:, 1], -1))
                           - np.dot(points[:, 1], np.roll(points[:, 0], -1))))


def centroid(vertices) -> list[float]:
    points = np.asarray(vertices, dtype=float)
    cross = points[:, 0] * np.roll(points[:, 1], -1) - np.roll(points[:, 0], -1) * points[:, 1]
    signed = 0.5 * float(cross.sum())
    if abs(signed) < 1e-9:
        return [float(value) for value in points.mean(axis=0)]
    return [float(np.sum((points[:, axis] + np.roll(points[:, axis], -1)) * cross) / (6 * signed))
            for axis in (0, 1)]


def orientation(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a, b, point) -> bool:
    return (min(a[0], b[0]) - 1e-9 <= point[0] <= max(a[0], b[0]) + 1e-9
            and min(a[1], b[1]) - 1e-9 <= point[1] <= max(a[1], b[1]) + 1e-9)


def segments_intersect(a, b, c, d) -> bool:
    values = (orientation(a, b, c), orientation(a, b, d), orientation(c, d, a), orientation(c, d, b))
    if values[0] * values[1] < 0 and values[2] * values[3] < 0:
        return True
    return any(abs(value) < 1e-9 and on_segment(*segment) for value, segment in (
        (values[0], (a, b, c)), (values[1], (a, b, d)),
        (values[2], (c, d, a)), (values[3], (c, d, b))))


def self_intersects(vertices) -> bool:
    count = len(vertices)
    for first in range(count):
        for second in range(first + 1, count):
            if second in {first, (first + 1) % count} or first == (second + 1) % count:
                continue
            if segments_intersect(vertices[first], vertices[(first + 1) % count],
                                  vertices[second], vertices[(second + 1) % count]):
                return True
    return False


def point_segment_distance(point, first, second) -> float:
    point, first, second = (np.asarray(value, dtype=float) for value in (point, first, second))
    delta = second - first
    ratio = 0.0 if np.dot(delta, delta) <= 1e-12 else float(np.clip(np.dot(point - first, delta) / np.dot(delta, delta), 0, 1))
    return float(np.linalg.norm(point - first - ratio * delta))


def point_polygon_distance(point, vertices) -> float:
    return min(point_segment_distance(point, vertices[index], vertices[(index + 1) % len(vertices)])
               for index in range(len(vertices)))


def polygon_mask(vertices, origin, resolution, shape_yx) -> np.ndarray:
    pixels = np.rint((np.asarray(vertices, dtype=float) - origin[:2]) / resolution).astype(np.int32)
    mask = np.zeros(shape_yx, dtype=np.uint8)
    cv2.fillPoly(mask, [pixels], 1)
    return mask.astype(bool)


def flood_reachable(safe: np.ndarray, start_zyx) -> np.ndarray:
    reachable = np.zeros(safe.shape, dtype=bool)
    if start_zyx is None:
        return reachable
    queue = deque([start_zyx])
    reachable[start_zyx] = True
    depth, height, width = safe.shape
    while queue:
        z, y, x = queue.popleft()
        for dz, dy, dx in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            zz, yy, xx = z + dz, y + dy, x + dx
            if 0 <= zz < depth and 0 <= yy < height and 0 <= xx < width and safe[zz, yy, xx] and not reachable[zz, yy, xx]:
                reachable[zz, yy, xx] = True
                queue.append((zz, yy, xx))
    return reachable


def nearest_safe_index(safe: np.ndarray, requested_xyz, origin, resolution):
    xyz = np.rint((np.asarray(requested_xyz) - origin) / resolution - 0.5).astype(int)
    candidate = (int(xyz[2]), int(xyz[1]), int(xyz[0]))
    if all(0 <= candidate[index] < safe.shape[index] for index in range(3)) and safe[candidate]:
        return candidate
    cells = np.argwhere(safe)
    if not len(cells):
        return None
    requested_zyx = xyz[::-1]
    return tuple(int(value) for value in cells[np.argmin(np.sum((cells - requested_zyx) ** 2, axis=1))])


def flatten_objects(scene_graph: dict) -> list[dict]:
    return [item for room in scene_graph.get("rooms", []) for item in room.get("objects", [])]


def match_objects(clusters, base_objects, threshold, maximum_distance):
    selected, warnings = [], []
    for cluster in clusters:
        observations = int(cluster.get("observations", 0))
        if observations < threshold:
            continue
        position = np.asarray([cluster["world_x_m"], cluster["world_y_m"], cluster["world_z_m"]], dtype=float)
        matches = [item for item in base_objects if str(item.get("label", "")).strip().lower()
                   == str(cluster.get("label", "")).strip().lower()]
        if not matches:
            warnings.append(f'cluster {cluster.get("cluster_id")} ({cluster.get("label")}): no 3-D Box geometry')
            continue
        distances = [float(np.linalg.norm(position - np.asarray(item["center_xyz_m"], dtype=float))) for item in matches]
        match = matches[int(np.argmin(distances))]
        distance = min(distances)
        if distance > maximum_distance:
            warnings.append(f'cluster {cluster.get("cluster_id")} ({cluster.get("label")}): nearest Box is {distance:.2f} m away')
            continue
        selected.append({
            "label": cluster["label"], "center_xyz_m": position.tolist(),
            "size_xyz_m": match["size_xyz_m"], "orientation_wxyz": match["orientation_wxyz"],
            "probability": float(cluster.get("confidence_max", match.get("probability", 0.0))),
            "instance": str(cluster.get("cluster_id")), "observation_count": observations,
            "source": "boxer_localization_cluster_reviewed", "geometry_match_distance_m": distance,
        })
    return selected, warnings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--rooms", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--clusters", type=Path, required=True)
    parser.add_argument("--voxel-snapshot", type=Path, required=True)
    parser.add_argument("--planning-start", type=Path, required=True)
    parser.add_argument("--min-observations", type=int, required=True)
    parser.add_argument("--min-room-area", type=float, default=5.0)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--preview", type=Path, required=True)
    parser.add_argument("--maximum-geometry-match-m", type=float, default=2.0)
    parser.add_argument("--maximum-unassigned-gap-m", type=float, default=0.75)
    args = parser.parse_args()

    draft = json.loads(args.rooms.read_text(encoding="utf-8"))
    base = json.loads(args.scene_graph.read_text(encoding="utf-8"))
    clusters = json.loads(args.clusters.read_text(encoding="utf-8"))
    metadata = json.loads((args.voxel_snapshot / "metadata.json").read_text(encoding="utf-8"))
    arrays = np.load(args.voxel_snapshot / "voxel_map.npz")
    raw, esdf = arrays["raw_occupancy_zyx"], arrays["esdf_zyx_m"]
    origin = np.asarray(metadata["origin_xyz_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    upper = origin + resolution * np.asarray(metadata["shape_xyz"], dtype=float)
    minimum_esdf = float(metadata["minimum_esdf_distance_m"])
    safe = (raw == 0) & (esdf >= minimum_esdf)
    start = json.loads(args.planning_start.read_text(encoding="utf-8"))["start_xyz_yaw"][:3]
    start_index = nearest_safe_index(safe, start, origin, resolution)
    reachable = flood_reachable(safe, start_index)

    errors, warnings = [], []
    if draft.get("run_id") != args.run_id:
        errors.append(f'draft run_id {draft.get("run_id")!r} does not match {args.run_id!r}')
    try:
        draft_threshold = int(draft.get("min_object_observations", -1))
    except (TypeError, ValueError):
        draft_threshold = -1
    if draft_threshold != args.min_observations:
        errors.append("draft Box threshold does not match this review command")
    try:
        draft_min_area = float(draft.get("min_room_area_m2", -1))
    except (TypeError, ValueError):
        draft_min_area = -1
    if not math.isclose(draft_min_area, args.min_room_area, abs_tol=1e-9):
        errors.append("draft minimum room area does not match this review command")
    rooms = draft.get("rooms", [])
    if not rooms:
        errors.append("no rooms were defined")
    ids = [str(room.get("id", "")) for room in rooms]
    names = [str(room.get("name", "")).strip() for room in rooms]
    if len(set(ids)) != len(ids) or any(not value for value in ids):
        errors.append("room IDs must be non-empty and unique")
    if len({value.casefold() for value in names}) != len(names) or any(not value for value in names):
        errors.append("room names must be non-empty and unique")

    room_masks, room_safe = {}, {}
    overlap_resolution = min(0.05, resolution / 5.0)
    overlap_shape = tuple(np.ceil((upper[1::-1] - origin[1::-1]) / overlap_resolution).astype(int))
    occupied_by_interiors = np.zeros(overlap_shape, dtype=np.uint16)
    for room in rooms:
        room_id = str(room.get("id", "?"))
        vertices = room.get("polygon_xy_m", [])
        try:
            points = np.asarray(vertices, dtype=float)
            valid_polygon = len(points) >= 3 and points.shape[1:] == (2,) and np.all(np.isfinite(points))
        except (TypeError, ValueError):
            valid_polygon = False
        if not valid_polygon:
            errors.append(f"{room_id}: polygon requires at least three finite XY vertices")
            continue
        if area(vertices) < 0.25:
            errors.append(f"{room_id}: polygon area is below 0.25 m^2")
        if room.get("space_role") == "room" and area(vertices) < args.min_room_area:
            errors.append(
                f"{room_id}: room area {area(vertices):.2f} m^2 is below "
                f"the required {args.min_room_area:.2f} m^2")
        if self_intersects(vertices):
            errors.append(f"{room_id}: polygon self-intersects")
        if np.any(points < origin[:2] - 0.5) or np.any(points > upper[:2] + 0.5):
            errors.append(f"{room_id}: polygon extends outside the voxel-map XY bounds")
        if room.get("space_role") not in {"room", "transition_space"}:
            errors.append(f"{room_id}: space_role must be room or transition_space")
        try:
            z_min, z_max = float(room["z_min_m"]), float(room["z_max_m"])
            if not math.isfinite(z_min) or not math.isfinite(z_max) or z_min >= z_max:
                raise ValueError
        except (KeyError, TypeError, ValueError):
            errors.append(f"{room_id}: invalid z range")
            continue
        mask = polygon_mask(vertices, origin, resolution, raw.shape[1:])
        room_masks[room_id] = mask
        fine_mask = polygon_mask(vertices, origin, overlap_resolution, overlap_shape).astype(np.uint8)
        fine_interior = cv2.erode(fine_mask, np.ones((3, 3), dtype=np.uint8), iterations=1)
        occupied_by_interiors += fine_interior.astype(np.uint16)
        z_centers = origin[2] + (np.arange(raw.shape[0]) + 0.5) * resolution
        z_mask = (z_centers >= z_min) & (z_centers <= z_max)
        feasible = safe & z_mask[:, None, None] & mask[None, :, :]
        reachable_feasible = feasible & reachable
        room_safe[room_id] = reachable_feasible
        if not np.any(feasible):
            errors.append(f"{room_id}: contains no collision-safe voxel")
        elif not np.any(reachable_feasible):
            errors.append(f"{room_id}: safe voxels are unreachable from planning_start")

    overlap_cells = int(np.count_nonzero(occupied_by_interiors > 1))
    overlap_area = overlap_cells * overlap_resolution * overlap_resolution
    if overlap_area > 0.25:
        errors.append(f"room polygons overlap by {overlap_area:.2f} m^2 (maximum 0.25 m^2)")

    id_set = set(ids)
    adjacency = {room["id"]: set(room.get("adjacent_room_ids", [])) for room in rooms if room.get("id")}
    for room_id, neighbors in adjacency.items():
        for neighbor in neighbors:
            if neighbor not in id_set:
                errors.append(f"{room_id}: adjacent room {neighbor!r} does not exist")
            elif room_id not in adjacency.get(neighbor, set()):
                errors.append(f"{room_id} <-> {neighbor}: adjacency is not symmetric")
    if len(ids) > 1 and ids:
        seen, queue = {ids[0]}, deque([ids[0]])
        while queue:
            for neighbor in adjacency.get(queue.popleft(), set()):
                if neighbor in id_set and neighbor not in seen:
                    seen.add(neighbor); queue.append(neighbor)
        if seen != id_set:
            errors.append("room adjacency graph is disconnected; check doors/transition spaces")

    objects, object_warnings = match_objects(
        clusters, flatten_objects(base), args.min_observations, args.maximum_geometry_match_m)
    warnings.extend(object_warnings)
    output_rooms = []
    for room in rooms:
        room_id = room["id"]
        feasible = room_safe.get(room_id)
        z_indices = np.nonzero(np.any(feasible, axis=(1, 2)))[0] if feasible is not None else np.asarray([])
        output_rooms.append({
            "id": room_id, "name": room["name"], "aliases": room.get("aliases", []),
            "floor_id": int(room.get("floor_id", draft.get("floor_id", 1))),
            "semantic_type": room["name"], "space_role": room["space_role"],
            "adjacent_room_ids": sorted(set(room.get("adjacent_room_ids", []))),
            "centroid_xy_m": centroid(room["polygon_xy_m"]),
            "polygon_xy_m": room["polygon_xy_m"],
            "z_min_m": float(room["z_min_m"]), "z_max_m": float(room["z_max_m"]),
            "camera_height_band_m": ([float(origin[2] + (z_indices[0] + 0.5) * resolution),
                                       float(origin[2] + (z_indices[-1] + 0.5) * resolution)]
                                      if len(z_indices) else []),
            "reachable_safe_voxel_count": int(np.count_nonzero(feasible)) if feasible is not None else 0,
            "objects": [],
        })

    unassigned = []
    room_lookup = {room["id"]: room for room in output_rooms}
    for index, item in enumerate(objects):
        point = item["center_xyz_m"]
        candidates = [room for room in output_rooms
                      if MplPath(room["polygon_xy_m"]).contains_point(point[:2], radius=1e-8)]
        if not candidates:
            nearby = [(point_polygon_distance(point[:2], room["polygon_xy_m"]), room)
                      for room in output_rooms]
            nearest = min(nearby, key=lambda value: value[0]) if nearby else None
            candidates = [nearest[1]] if nearest and nearest[0] <= args.maximum_unassigned_gap_m else []
        item["id"] = f'L{draft.get("floor_id", 1)}_boxer_reviewed_{index:03d}'
        if candidates:
            assigned = min(candidates, key=lambda room: area(room["polygon_xy_m"]))
            item["room_id"] = assigned["id"]
            room_lookup[assigned["id"]]["objects"].append(item)
        else:
            unassigned.append(item)
    if unassigned:
        warnings.append(f"{len(unassigned)} reviewed Box clusters are outside all room ranges")

    candidate = {
        "format": "pre_map_vln.approved_real_scene_graph.v1", "run_id": args.run_id,
        "frame_id": base.get("frame_id", "world"),
        "map_epoch_uuid": metadata.get("identity", {}).get("map_epoch_uuid"),
        "min_object_observations": args.min_observations,
        "min_room_area_m2": args.min_room_area,
        "source_room_draft": str(args.rooms.resolve()), "source_scene_graph": str(args.scene_graph.resolve()),
        "rooms": output_rooms, "unassigned_objects": unassigned,
    }
    report = {
        "format": "pre_map_vln.real_room_validation.v1", "run_id": args.run_id,
        "valid": not errors, "errors": errors, "warnings": warnings,
        "counts": {"rooms": len(output_rooms), "objects": len(objects),
                   "unassigned_objects": len(unassigned), "overlap_cells": overlap_cells},
        "minimum_room_area_m2": args.min_room_area,
        "safety": {"voxel_map_unchanged": True, "minimum_esdf_distance_m": minimum_esdf,
                   "planning_start_safe_index_zyx": list(start_index) if start_index else None},
    }
    atomic_json(args.candidate, candidate)
    atomic_json(args.report, report)

    fig, ax = plt.subplots(figsize=(12, 8))
    projection = np.max(raw == 100, axis=0)
    extent = [origin[0], upper[0], origin[1], upper[1]]
    ax.imshow(projection, origin="lower", extent=extent, cmap="gray_r", alpha=0.55)
    for room in output_rooms:
        points = np.asarray(room["polygon_xy_m"])
        ax.fill(points[:, 0], points[:, 1], alpha=0.18)
        ax.plot(np.r_[points[:, 0], points[0, 0]], np.r_[points[:, 1], points[0, 1]], linewidth=2)
        ax.text(*room["centroid_xy_m"], f'{room["id"]}\n{room["name"]}', ha="center", va="center")
    for item in objects:
        ax.scatter(item["center_xyz_m"][0], item["center_xyz_m"][1], c="red", marker="x")
        ax.text(item["center_xyz_m"][0], item["center_xyz_m"][1], item["label"], fontsize=7)
    ax.set_aspect("equal"); ax.set_title(f'{args.run_id}: room review validation ({"PASS" if not errors else "FAIL"})')
    ax.set_xlabel("world x [m]"); ax.set_ylabel("world y [m]")
    args.preview.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.preview, dpi=180, bbox_inches="tight")
    plt.close(fig)

    print(json.dumps(report, ensure_ascii=False, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
