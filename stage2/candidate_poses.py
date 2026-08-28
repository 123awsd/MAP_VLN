"""Generate collision-free, target-facing terminal pose candidates from 3-D boxes."""

from __future__ import annotations

import math
from typing import Any

from .grid_map import OccupancyGrid
from .vocabulary import load_semantic_aliases


ALIASES = load_semantic_aliases()

DEFAULT_VERTICAL_FOV_DEG = 70.0
DEFAULT_HORIZONTAL_FOV_DEG = 90.0
DEFAULT_MIN_SENSOR_Z = 0.65
DEFAULT_MAX_SENSOR_Z = 1.80
REGION_TYPES = {
    "support_surface", "below_region", "surrounding_region",
    "instance_region", "between_region",
}


def _yaw_from_wxyz(quaternion: list[float]) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _relation_ok(
    relation: str | None,
    angle: float,
    object_yaw: float,
    tolerance_deg: float = 55.0,
) -> bool:
    # ``facing`` constrains the terminal yaw, which is generated toward the
    # target below; it does not require the UAV to stand on the target's front
    # side.  The four directional relations constrain the position angle.
    if relation == "facing":
        return True
    if relation not in {"front", "behind", "left", "right"}:
        return True
    desired = {
        "front": object_yaw,
        "behind": object_yaw + math.pi,
        "left": object_yaw + math.pi / 2.0,
        "right": object_yaw - math.pi / 2.0,
    }[relation]
    return abs(_angle_difference(angle, desired)) <= math.radians(tolerance_deg)


def _region_type(constraints: dict[str, Any]) -> str:
    explicit = str(constraints.get("region_type", "auto") or "auto").strip().lower()
    if explicit in REGION_TYPES:
        return explicit
    relation = constraints.get("relation")
    return {
        "on": "support_surface",
        "below": "below_region",
        "between": "between_region",
        "near": "surrounding_region",
    }.get(relation, "instance_region")


def _reference_objects(
    scene_graph: dict[str, Any], task: dict[str, Any]
) -> list[dict[str, Any]]:
    target = task.get("target", {})
    raw_references = target.get("references") or []
    labels = [
        str(value).strip().lower()
        for value in raw_references
        if str(value).strip()
    ]
    if not labels:
        labels = [
            str(value).strip().lower()
            for value in (target.get("reference"), target.get("reference_secondary"))
            if str(value or "").strip()
        ]
    result = []
    for label in labels:
        accepted = ALIASES.get(label, {label})
        matches = [
            obj
            for room in scene_graph.get("rooms", [])
            for obj in room.get("objects", [])
            if str(obj.get("label", "")).strip().lower() in accepted
        ]
        if matches:
            result.append(max(matches, key=lambda item: float(item.get("probability", 0.0))))
    return result


def _oriented_xy(center: list[float], yaw: float, local_x: float, local_y: float) -> list[float]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        center[0] + cosine * local_x - sine * local_y,
        center[1] + sine * local_x + cosine * local_y,
    ]


def _region_samples(
    center: list[float], size: list[float], object_yaw: float, region_type: str,
) -> list[list[float]]:
    """Return 3-D samples for the task-relevant part of the target box."""
    half_x, half_y, half_z = [max(0.06, value * 0.5) for value in size]
    if region_type == "support_surface":
        z = center[2] + half_z
        return [
            _oriented_xy(center, object_yaw, ix * half_x * 0.8, iy * half_y * 0.8) + [z]
            for ix in (-1.0, 0.0, 1.0) for iy in (-1.0, 0.0, 1.0)
        ]
    if region_type == "below_region":
        z = max(0.08, center[2] - half_z + 0.05)
        return [
            _oriented_xy(center, object_yaw, ix * half_x * 0.8, iy * half_y * 0.8) + [z]
            for ix in (-1.0, 0.0, 1.0) for iy in (-1.0, 0.0, 1.0)
        ]
    if region_type == "surrounding_region":
        return [
            _oriented_xy(center, object_yaw, (half_x + 0.02) * math.cos(index * math.pi / 4.0),
                         (half_y + 0.02) * math.sin(index * math.pi / 4.0)) + [center[2]]
            for index in range(8)
        ]
    # An instance is observed as a small horizontal surface/box cross.  The
    # vertical coordinates make the FoV test meaningful without changing the
    # 2-D navigation grid used by the current implementation.
    return [
        _oriented_xy(center, object_yaw, ix * half_x * 0.8, iy * half_y * 0.8)
        + [center[2] + z * half_z * 0.8]
        for ix, iy, z in (
            (-1.0, -1.0, -1.0), (-1.0, 0.0, 0.0), (-1.0, 1.0, 1.0),
            (0.0, -1.0, -0.5), (0.0, 0.0, 0.0), (0.0, 1.0, 0.5),
            (1.0, -1.0, -1.0), (1.0, 0.0, 0.0), (1.0, 1.0, 1.0),
        )
    ]


def _between_geometry(
    xy: list[float], references: list[dict[str, Any]], tolerance_m: float,
) -> tuple[bool, float]:
    if len(references) < 2:
        return False, math.inf
    first = [float(value) for value in references[0]["center_xyz_m"][:2]]
    second = [float(value) for value in references[1]["center_xyz_m"][:2]]
    dx, dy = second[0] - first[0], second[1] - first[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1e-9:
        return False, math.inf
    t = ((xy[0] - first[0]) * dx + (xy[1] - first[1]) * dy) / denominator
    clamped = max(0.0, min(1.0, t))
    closest = [first[0] + clamped * dx, first[1] + clamped * dy]
    distance = math.dist(xy, closest)
    return -0.05 <= t <= 1.05 and distance <= tolerance_m, distance


def _preferred_observation_distance(
    minimum: float, maximum: float, size: list[float], region_type: str,
    observation_detail: str,
) -> float:
    """Choose the preferred radius while keeping the requested range hard."""
    span = maximum - minimum
    preferred = minimum + 0.5 * span
    if region_type == "below_region":
        preferred = minimum + 0.25 * span
    elif region_type == "surrounding_region":
        preferred = minimum + 0.75 * span
    elif region_type == "instance_region":
        box_radius = 0.5 * math.hypot(size[0], size[1])
        preferred = max(minimum, min(maximum, 0.75 + box_radius))
    if observation_detail == "fine":
        # Fine inspection moves the preferred radius toward the near end of
        # the caller-provided interval, without violating that interval.
        preferred = minimum + 0.35 * (preferred - minimum)
    return min(maximum, max(minimum, preferred))


def _height_candidates(
    center: list[float], size: list[float], constraints: dict[str, Any],
    region_type: str,
) -> list[float]:
    explicit = constraints.get("height_m")
    if explicit is not None:
        return [float(explicit)]
    height_range = constraints.get("height_range_m")
    if height_range is not None:
        lower, upper = [float(value) for value in height_range]
        return sorted(set(round(value, 6) for value in (lower, (lower + upper) * 0.5, upper)))

    relation = constraints.get("relation")
    if region_type == "support_surface" or relation == "above":
        nominal = center[2] + 0.5 * max(0.0, size[2]) + 0.20
    elif region_type == "below_region" or relation == "below":
        nominal = center[2] - 0.25 * max(0.0, size[2])
    else:
        nominal = center[2]
    nominal = min(DEFAULT_MAX_SENSOR_Z, max(DEFAULT_MIN_SENSOR_Z, nominal))
    result = [nominal]
    if str(constraints.get("observation_detail", "normal")) == "fine":
        result.extend((nominal - 0.25, nominal + 0.25))
    return sorted(set(round(min(DEFAULT_MAX_SENSOR_Z, max(DEFAULT_MIN_SENSOR_Z, value)), 6) for value in result))


def _height_relation_ok(
    z: float, center_z: float, relation: str | None, tolerance_m: float = 0.05,
) -> bool:
    if relation == "above":
        return z >= center_z + tolerance_m
    if relation == "below":
        return z <= center_z - tolerance_m
    return True


def _visible_region_samples(
    grid: OccupancyGrid, xy: list[float], z: float, yaw: float,
    samples: list[list[float]], horizontal_fov_deg: float, vertical_fov_deg: float,
) -> list[int]:
    visible = []
    horizontal_half = math.radians(horizontal_fov_deg) * 0.5
    vertical_half = math.radians(vertical_fov_deg) * 0.5
    for index, sample in enumerate(samples):
        horizontal_distance = math.dist(xy, sample[:2])
        bearing = math.atan2(sample[1] - xy[1], sample[0] - xy[0])
        vertical_angle = math.atan2(sample[2] - z, max(1e-6, horizontal_distance))
        if abs(_angle_difference(bearing, yaw)) > horizontal_half:
            continue
        if abs(vertical_angle) > vertical_half:
            continue
        if grid.line_is_free(xy, sample[:2], allow_endpoint_cells=2):
            visible.append(index)
    return visible


def matching_objects(
    scene_graph: dict[str, Any], task: dict[str, Any], prefer_room: bool = True,
) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    label = task["target"]["label"].lower()
    room_name = task["target"].get("room")
    accepted = ALIASES.get(label, {label})
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
    references = _reference_objects(scene_graph, task)
    if references:
        reference_centers = [obj["center_xyz_m"] for obj in references]
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
    candidates = []
    references = _reference_objects(scene_graph, task)
    reference_centers = [other["center_xyz_m"] for other in references]
    constraints_region = _region_type(constraints)
    vertical_fov = float(constraints.get("vertical_fov_deg", DEFAULT_VERTICAL_FOV_DEG))
    horizontal_fov = float(constraints.get("horizontal_fov_deg", DEFAULT_HORIZONTAL_FOV_DEG))
    yaw_tolerance = float(constraints.get("yaw_tolerance_deg", 55.0))
    for room, obj in matching_objects(scene_graph, task, prefer_room=prefer_room):
        if allowed_object_ids is not None and obj["id"] not in allowed_object_ids:
            continue
        center = [float(value) for value in obj["center_xyz_m"]]
        size = [float(value) for value in obj["size_xyz_m"]]
        object_yaw = _yaw_from_wxyz([float(value) for value in obj["orientation_wxyz"]])
        object_radius = 0.5 * math.hypot(size[0], size[1])
        preferred_distance = _preferred_observation_distance(
            minimum, maximum, size, constraints_region,
            str(constraints.get("observation_detail", "normal")),
        )
        radii = sorted(set([minimum, preferred_distance, maximum]))
        region_samples = _region_samples(center, size, object_yaw, constraints_region)
        height_values = _height_candidates(center, size, constraints, constraints_region)
        for radius in radii:
            radius_from_center = object_radius + radius
            for sample in range(24):
                angle = 2.0 * math.pi * sample / 24.0
                if not _relation_ok(constraints.get("relation"), angle, object_yaw, yaw_tolerance):
                    continue
                xy = [center[0] + radius_from_center * math.cos(angle), center[1] + radius_from_center * math.sin(angle)]
                if not grid.is_free(xy):
                    continue
                between_ok, between_distance = _between_geometry(
                    xy, references, tolerance_m=max(0.75, 0.5 * maximum)
                ) if constraints.get("relation") == "between" else (True, 0.0)
                if not between_ok:
                    continue
                yaw = math.atan2(center[1] - xy[1], center[0] - xy[0])
                for z in height_values:
                    if not _height_relation_ok(z, center[2], constraints.get("relation")):
                        continue
                    visible_ids = _visible_region_samples(
                        grid, xy, z, yaw, region_samples, horizontal_fov, vertical_fov
                    )
                    visible = bool(visible_ids)
                    if constraints.get("visibility_required", True) and not visible:
                        continue
                    view_quality = len(visible_ids) / max(1, len(region_samples))
                    reference_distance = min(
                        (math.dist(center[:2], other[:2]) for other in reference_centers),
                        default=0.0,
                    )
                    terminal_score = (
                        abs(radius - preferred_distance)
                        + (1.0 - view_quality)
                        + 0.1 * abs(float(z) - center[2])
                        + 0.15 * reference_distance
                        + 0.15 * between_distance
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
                        "preferred_observation_distance_m": preferred_distance,
                        "line_of_sight": visible,
                        "candidate_angle_rad": angle % (2.0 * math.pi),
                        "region_type": constraints_region,
                        "region_sample_count": len(region_samples),
                        "visible_region_sample_ids": visible_ids,
                        "view_quality": view_quality,
                        "vertical_fov_deg": vertical_fov,
                        "horizontal_fov_deg": horizontal_fov,
                        "height_fov_validated": True,
                        "height_collision_validated": False,
                        "collision_validation": "2d_occupancy_only",
                        "reference_distance_m": reference_distance if reference_centers else None,
                        "between_distance_m": between_distance if constraints.get("relation") == "between" else None,
                        "terminal_cost": terminal_score,
                    })
    candidates.sort(key=lambda item: (item["terminal_cost"], -float(item["line_of_sight"]), item["id"]))
    # Preserve at least one candidate per feasible height before filling the
    # remaining budget.  Otherwise the cheaper middle height would hide the
    # documented multi-height observation option.
    selected = []
    selected_ids = set()

    if max_candidates <= 0:
        return selected

    def spatially_distinct(candidate: dict[str, Any]) -> bool:
        position = candidate["pose"]
        return not any(
            math.hypot(position["x"] - old["pose"]["x"], position["y"] - old["pose"]["y"]) < 0.35
            and abs(position["z"] - old["pose"]["z"]) < 0.20
            for old in selected
        )

    best_by_height = []
    for height in sorted({item["pose"]["z"] for item in candidates}):
        candidate = next(
            (item for item in candidates
             if item["pose"]["z"] == height and spatially_distinct(item)),
            None,
        )
        if candidate is not None:
            best_by_height.append(candidate)

    # Keep the documented height diversity when the budget permits it.  For a
    # smaller caller-provided budget, retain globally better candidates instead
    # of systematically preferring the lowest height.
    if len(best_by_height) <= max_candidates:
        for candidate in best_by_height:
            selected.append(candidate)
            selected_ids.add(candidate["id"])
    else:
        for candidate in sorted(best_by_height, key=lambda item: (item["terminal_cost"], item["id"]))[:max_candidates]:
            selected.append(candidate)
            selected_ids.add(candidate["id"])

    for candidate in candidates:
        if candidate["id"] in selected_ids or not spatially_distinct(candidate):
            continue
        selected.append(candidate)
        selected_ids.add(candidate["id"])
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
