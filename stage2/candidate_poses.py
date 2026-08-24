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


def matching_objects(
    scene_graph: dict[str, Any], task: dict[str, Any], prefer_room: bool = True,
) -> list[tuple[dict[str, Any], dict[str, Any]]]:
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
            preferred = room_matches if prefer_room and room_matches else all_matches
            return sorted(preferred, key=reference_key)[:12]
    matches = room_matches if prefer_room and room_matches else all_matches
    return sorted(matches, key=lambda item: float(item[1].get("probability", 0.0)), reverse=True)[:12]


def generate_candidates(
    grid: OccupancyGrid,
    scene_graph: dict[str, Any],
    task: dict[str, Any],
    max_candidates: int = 8,
    allowed_object_ids: set[str] | None = None,
    prefer_room: bool = True,
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
    for room, obj in matching_objects(scene_graph, task, prefer_room=prefer_room):
        if allowed_object_ids is not None and obj["id"] not in allowed_object_ids:
            continue
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


def select_spread_target_objects(scene_graph: dict[str, Any], task_graph: dict[str, Any]) -> dict[str, str]:
    """Choose language-compatible instances in distinct, distant rooms.

    This selects the mission anchors only. The downstream joint planner still
    computes the geometrically shortest feasible tour between those anchors.
    """
    selected: dict[str, str] = {}
    used_rooms: set[int] = set()
    centers: list[list[float]] = []
    def selection_order(task):
        choices = matching_objects(scene_graph, task)
        destination_groups = {
            int(room.get("parent_room_id", room["id"]))
            for room, _ in choices if room.get("space_role") == "room"
        }
        # Allocate scarce labels first (for example, chairs), leaving flexible
        # labels such as doors available to fill another distant room.
        return not task.get("active_initially", True), len(destination_groups) or len(choices)

    tasks = sorted(task_graph["tasks"], key=selection_order)
    for task in tasks:
        choices = matching_objects(scene_graph, task)
        if not choices:
            continue
        # Prefer true destination rooms. Objects assigned to corridor geometry
        # are retained in the graph, but should not anchor a showcase mission
        # when a compatible instance exists in a major room.
        destination_choices = [item for item in choices if item[0].get("space_role") == "room"]
        if destination_choices:
            choices = destination_choices

        def room_group(room):
            return int(room.get("parent_room_id", room["id"]))

        if not centers or not task.get("active_initially", True):
            room, obj = choices[0]
        else:
            def spread_key(item):
                room, obj = item
                center = [float(value) for value in obj["center_xyz_m"][:2]]
                separation = min(math.dist(center, old) for old in centers)
                distinct_room = int(room_group(room) not in used_rooms)
                return distinct_room, separation, float(obj.get("probability", 0.0))
            room, obj = max(choices, key=spread_key)
        selected[task["id"]] = obj["id"]
        if task.get("active_initially", True):
            used_rooms.add(room_group(room))
            centers.append([float(value) for value in obj["center_xyz_m"][:2]])
    return selected


def generate_all_candidates(
    grid: OccupancyGrid,
    scene_graph: dict[str, Any],
    task_graph: dict[str, Any],
    max_candidates: int = 8,
    selected_objects: dict[str, str] | None = None,
) -> dict[str, list[dict[str, Any]]]:
    result = {}
    for task in task_graph["tasks"]:
        object_id = None if selected_objects is None else selected_objects.get(task["id"])
        values = generate_candidates(
            grid, scene_graph, task, max_candidates=max_candidates,
            allowed_object_ids=None if object_id is None else {object_id},
        )
        if not values and object_id is not None:
            values = generate_candidates(
                grid, scene_graph, task, max_candidates=max_candidates, prefer_room=False,
            )
        result[task["id"]] = values
    return result
