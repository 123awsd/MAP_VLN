"""Generate collision-free, target-facing terminal pose candidates from 3-D boxes."""

from __future__ import annotations

import math
from typing import Any

from .grid_map import OccupancyGrid


def _yaw_from_wxyz(quaternion: list[float]) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _relation_ok(relation: str | None, angle: float, object_yaw: float) -> bool:
    if relation not in {"front", "behind", "left", "right"}:
        return True
    desired = {
        "front": object_yaw,
        "behind": object_yaw + math.pi,
        "left": object_yaw + math.pi / 2.0,
        "right": object_yaw - math.pi / 2.0,
    }[relation]
    difference = math.atan2(math.sin(angle - desired), math.cos(angle - desired))
    return abs(difference) <= math.radians(55.0)


def matching_objects(scene_graph: dict[str, Any], task: dict[str, Any]) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    label = task["target"]["label"].lower()
    room_name = task["target"].get("room")
    aliases = {
        "tv": {"tv", "television"}, "television": {"tv", "television"},
        "couch": {"couch", "sofa"}, "sofa": {"couch", "sofa"},
        "desk": {"desk", "table"}, "cup": {"cup", "mug"}, "mug": {"cup", "mug"},
    }
    accepted = aliases.get(label, {label})
    all_matches = []
    room_matches = []
    for room in scene_graph.get("rooms", []):
        for obj in room.get("objects", []):
            if str(obj.get("label", "")).lower() not in accepted:
                continue
            item = (room, obj)
            all_matches.append(item)
            if room_name and str(room.get("semantic_type", "")).lower() == room_name:
                room_matches.append(item)
    reference = task["target"].get("reference")
    if reference:
        reference_centers = [
            obj["center_xyz_m"]
            for room in scene_graph.get("rooms", []) for obj in room.get("objects", [])
            if str(obj.get("label", "")).lower() == reference
        ]
        if reference_centers:
            def reference_key(item):
                center = item[1]["center_xyz_m"]
                distance = min(math.dist(center[:2], other[:2]) for other in reference_centers)
                return distance, -float(item[1].get("probability", 0.0))
            return sorted(all_matches, key=reference_key)[:12]
    matches = room_matches or all_matches
    return sorted(matches, key=lambda item: float(item[1].get("probability", 0.0)), reverse=True)[:12]


def generate_candidates(
    grid: OccupancyGrid,
    scene_graph: dict[str, Any],
    task: dict[str, Any],
    max_candidates: int = 8,
) -> list[dict[str, Any]]:
    constraints = task["spatial_constraints"]
    minimum, maximum = constraints["distance_m"]
    nominal = 0.5 * (minimum + maximum)
    radii = sorted(set([minimum, nominal, maximum]))
    candidates = []
    reference_label = task["target"].get("reference")
    reference_centers = [
        other["center_xyz_m"]
        for other_room in scene_graph.get("rooms", []) for other in other_room.get("objects", [])
        if reference_label and str(other.get("label", "")).lower() == reference_label
    ]
    for room, obj in matching_objects(scene_graph, task):
        center = [float(value) for value in obj["center_xyz_m"]]
        size = [float(value) for value in obj["size_xyz_m"]]
        object_yaw = _yaw_from_wxyz([float(value) for value in obj["orientation_wxyz"]])
        object_radius = 0.5 * math.hypot(size[0], size[1])
        for radius in radii:
            radius_from_center = object_radius + radius
            for sample in range(24):
                angle = 2.0 * math.pi * sample / 24.0
                if not _relation_ok(constraints.get("relation"), angle, object_yaw):
                    continue
                xy = [center[0] + radius_from_center * math.cos(angle), center[1] + radius_from_center * math.sin(angle)]
                if not grid.is_free(xy):
                    continue
                target_boundary = [
                    center[0] + object_radius * math.cos(angle),
                    center[1] + object_radius * math.sin(angle),
                ]
                visible = grid.line_is_free(xy, target_boundary, allow_endpoint_cells=2)
                if constraints.get("visibility_required", True) and not visible:
                    continue
                z = constraints.get("height_m")
                if z is None:
                    z = min(1.8, max(0.65, center[2]))
                yaw = math.atan2(center[1] - xy[1], center[0] - xy[0])
                reference_distance = min(
                    (math.dist(center[:2], other[:2]) for other in reference_centers),
                    default=0.0,
                )
                terminal_score = (
                    abs(radius - nominal)
                    + (0.0 if visible else 2.0)
                    + 0.1 * abs(float(z) - center[2])
                    + 0.15 * reference_distance
                )
                candidates.append({
                    "id": f"{task['id']}__{obj['id']}__{len(candidates):03d}",
                    "task_id": task["id"],
                    "object_id": obj["id"],
                    "object_label": obj["label"],
                    "room_id": room["id"],
                    "room_type": room.get("semantic_type", "unknown"),
                    "pose": {"x": xy[0], "y": xy[1], "z": float(z), "yaw": yaw},
                    "target_xyz_m": center,
                    "distance_to_box_surface_m": radius,
                    "line_of_sight": visible,
                    "reference_distance_m": reference_distance if reference_centers else None,
                    "terminal_cost": terminal_score,
                })
    candidates.sort(key=lambda item: (item["terminal_cost"], -float(item["line_of_sight"]), item["id"]))
    # Spatial diversity prevents several nearly identical samples around one object.
    selected = []
    for candidate in candidates:
        position = candidate["pose"]
        if any(math.hypot(position["x"] - old["pose"]["x"], position["y"] - old["pose"]["y"]) < 0.35 for old in selected):
            continue
        selected.append(candidate)
        if len(selected) >= max_candidates:
            break
    return selected


def generate_all_candidates(grid: OccupancyGrid, scene_graph: dict[str, Any], task_graph: dict[str, Any], max_candidates: int = 8) -> dict[str, list[dict[str, Any]]]:
    return {
        task["id"]: generate_candidates(grid, scene_graph, task, max_candidates=max_candidates)
        for task in task_graph["tasks"]
    }
