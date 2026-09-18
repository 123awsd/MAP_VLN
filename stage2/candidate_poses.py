"""Generate collision-free, target-facing terminal pose candidates from 3-D boxes."""

from __future__ import annotations

import copy
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
    scene_graph: dict[str, Any], task: dict[str, Any], target_room: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    target = task.get("target", {})
    requested_floor = target.get("floor_id")
    requested_room_id = target.get("room_id")
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
        matches = []
        for room in scene_graph.get("rooms", []):
            if (
                target_room is None
                and requested_room_id is not None
                and str(room.get("id")) != str(requested_room_id)
            ):
                continue
            if (
                target_room is None
                and requested_floor is not None
                and int(room.get("floor_id", -1)) != int(requested_floor)
            ):
                continue
            if target_room is not None:
                same_room = room.get("id") == target_room.get("id")
                same_floor = room.get("floor_id") == target_room.get("floor_id")
                if not same_room and not same_floor:
                    continue
            for obj in room.get("objects", []):
                if str(obj.get("label", "")).strip().lower() in accepted:
                    matches.append((
                        0 if target_room is not None and room.get("id") == target_room.get("id") else 1,
                        obj,
                    ))
        if matches:
            _, selected = min(
                matches,
                key=lambda item: (item[0], -float(item[1].get("probability", 0.0))),
            )
            result.append(selected)
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


def _automatic_observation_profile(
    size: list[float], region_type: str, observation_detail: str,
    horizontal_fov_deg: float,
) -> tuple[float, float, float]:
    """Derive a soft surface-distance profile from geometry, not language defaults."""
    box_radius = max(0.08, 0.5 * math.hypot(size[0], size[1]))
    half_fov = max(math.radians(10.0), math.radians(horizontal_fov_deg) * 0.5)
    # A circumscribed horizontal circle fits in the image when the camera
    # center is at least r/sin(FoV/2) from the region center.
    fit_surface_distance = box_radius * (1.0 / max(0.15, math.sin(half_fov)) - 1.0)
    preferred_floor = {
        "support_surface": 0.75,
        "below_region": 0.60,
        "surrounding_region": 1.05,
        "instance_region": 0.70,
        "between_region": 0.85,
    }.get(region_type, 0.80)
    minimum = max(0.35, fit_surface_distance)
    preferred = max(minimum, preferred_floor)
    if observation_detail == "fine":
        # Fine inspection prefers a larger image footprint, but this remains a
        # soft preference. Farther collision-free/FoV-valid views are retained.
        preferred = max(minimum, 0.70 * preferred)
    maximum = min(3.5, max(preferred + 0.8, minimum + 1.0))
    return minimum, maximum, preferred


def _height_candidates_for_view(
    xy: list[float], region_samples: list[list[float]], constraints: dict[str, Any],
    vertical_fov_deg: float, height_bounds: list[float] | None = None,
) -> list[float]:
    """Intersect fixed-camera FoV and flight bands for one horizontal view."""
    explicit = constraints.get("height_m")
    if explicit is not None:
        return [float(explicit)]
    height_range = constraints.get("height_range_m")
    if height_range is not None:
        lower, upper = [float(value) for value in height_range]
        return sorted(set(round(value, 6) for value in (lower, (lower + upper) * 0.5, upper)))

    lower, upper = (
        (DEFAULT_MIN_SENSOR_Z, DEFAULT_MAX_SENSOR_Z)
        if height_bounds is None else (float(height_bounds[0]), float(height_bounds[1]))
    )
    tangent = math.tan(math.radians(vertical_fov_deg) * 0.5)
    for sample in region_samples:
        horizontal_distance = max(1e-6, math.dist(xy, sample[:2]))
        reach = horizontal_distance * tangent
        lower = max(lower, float(sample[2]) - reach)
        upper = min(upper, float(sample[2]) + reach)
    if lower > upper + 1e-9:
        return []
    region_center_z = sum(float(sample[2]) for sample in region_samples) / max(1, len(region_samples))
    nominal = min(upper, max(lower, region_center_z))
    values = [nominal]
    if str(constraints.get("observation_detail", "normal")) == "fine" and upper - lower >= 0.08:
        values.extend((lower + 0.25 * (upper - lower), lower + 0.75 * (upper - lower)))
    return sorted(set(round(value, 6) for value in values))


def _observation_region(
    target: dict[str, Any], references: list[dict[str, Any]], constraints: dict[str, Any],
    target_is_search_anchor: bool = False,
) -> tuple[dict[str, Any], str]:
    """Resolve target, anchor and search region without changing target identity."""
    region_type = _region_type(constraints)
    relation = constraints.get("relation")
    explicit = str(constraints.get("region_type", "auto") or "auto").strip().lower()
    if target_is_search_anchor:
        # The mapped target is a surrogate anchor for an unmapped verification
        # object (for example: search for a pillow on the bed near a curtain).
        # References only disambiguate which anchor instance to use; they must
        # not replace the bed itself as the observed region.
        return target, region_type
    if references and relation in {"on", "above", "below", "near"} and explicit != "instance_region":
        region_type = {
            "on": "support_surface",
            "above": "support_surface",
            "below": "below_region",
            "near": "surrounding_region",
        }[relation] if explicit == "auto" else explicit
        return references[0], region_type
    return target, region_type


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
        if hasattr(grid, "line_of_sight"):
            line_free = grid.line_of_sight([xy[0], xy[1], z], sample)
        else:
            line_free = grid.line_is_free(xy, sample[:2], allow_endpoint_cells=2)
        if line_free:
            visible.append(index)
    return visible


def _point_visible_from_pose(
    grid, xyz: list[float], yaw: float, point: list[float],
    horizontal_fov_deg: float, vertical_fov_deg: float,
) -> bool:
    delta = [float(point[i]) - float(xyz[i]) for i in range(3)]
    horizontal_distance = math.hypot(delta[0], delta[1])
    if horizontal_distance <= 1e-6:
        return False
    bearing = math.atan2(delta[1], delta[0])
    if abs(_angle_difference(bearing, yaw)) > math.radians(horizontal_fov_deg) * 0.5:
        return False
    if abs(math.atan2(delta[2], horizontal_distance)) > math.radians(vertical_fov_deg) * 0.5:
        return False
    if hasattr(grid, "line_of_sight"):
        return bool(grid.line_of_sight(xyz, point))
    return bool(grid.line_is_free(xyz[:2], point[:2], allow_endpoint_cells=2))


def matching_objects(
    scene_graph: dict[str, Any], task: dict[str, Any], prefer_room: bool = True,
) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    label = task["target"]["label"].lower()
    verification_label = str(task.get("verification_label", label)).strip().lower()
    target_is_search_anchor = verification_label not in ALIASES.get(label, {label})
    room_name = task["target"].get("room")
    requested_room_id = task["target"].get("room_id")
    requested_object_id = task["target"].get("object_id")
    floor_id = task["target"].get("floor_id")
    accepted = ALIASES.get(label, {label})
    all_matches = []
    room_matches = []
    for room in scene_graph.get("rooms", []):
        if requested_room_id is not None and str(room.get("id")) != str(requested_room_id):
            continue
        if floor_id is not None and int(room.get("floor_id", -1)) != int(floor_id):
            continue
        for obj in room.get("objects", []):
            if requested_object_id is not None and str(obj.get("id")) != str(requested_object_id):
                continue
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
            ranked = sorted(preferred, key=reference_key)
            # When the mapped target is a surrogate search anchor, references
            # identify which physical anchor the user meant (e.g. the bed near
            # the curtain while looking for an unmapped pillow). Do not admit
            # every same-label instance into the initial location pool.
            return ranked[:1] if target_is_search_anchor else ranked[:12]
    matches = room_matches if prefer_room and room_matches else all_matches
    return sorted(matches, key=lambda item: float(item[1].get("probability", 0.0)), reverse=True)[:12]


def generate_candidates(
    grid: OccupancyGrid,
    scene_graph: dict[str, Any],
    task: dict[str, Any],
    max_candidates: int = 8,
    allowed_object_ids: set[str] | None = None,
    prefer_room: bool = True,
    distance_scale: float = 1.0,
) -> list[dict[str, Any]]:
    constraints = task["spatial_constraints"]
    candidates = []
    distance_scale = max(1.0, float(distance_scale))
    vertical_fov = float(constraints.get("vertical_fov_deg", DEFAULT_VERTICAL_FOV_DEG))
    horizontal_fov = float(constraints.get("horizontal_fov_deg", DEFAULT_HORIZONTAL_FOV_DEG))
    yaw_tolerance = float(constraints.get("yaw_tolerance_deg", 55.0))
    for room, obj in matching_objects(scene_graph, task, prefer_room=prefer_room):
        if allowed_object_ids is not None and obj["id"] not in allowed_object_ids:
            continue
        references = _reference_objects(scene_graph, task, room)
        reference_centers = [other["center_xyz_m"] for other in references]
        center = [float(value) for value in obj["center_xyz_m"]]
        size = [float(value) for value in obj["size_xyz_m"]]
        target_label = str(task.get("target", {}).get("label", "")).strip().lower()
        verification_label = str(task.get("verification_label", target_label)).strip().lower()
        target_is_search_anchor = verification_label not in ALIASES.get(
            target_label, {target_label}
        )
        region_object, constraints_region = _observation_region(
            obj, references, constraints,
            target_is_search_anchor=target_is_search_anchor,
        )
        region_center = [float(value) for value in region_object["center_xyz_m"]]
        region_size = [float(value) for value in region_object["size_xyz_m"]]
        region_yaw = _yaw_from_wxyz([
            float(value) for value in region_object.get("orientation_wxyz", [1.0, 0.0, 0.0, 0.0])
        ])
        object_yaw = _yaw_from_wxyz([
            float(value) for value in obj.get("orientation_wxyz", [1.0, 0.0, 0.0, 0.0])
        ])
        object_radius = 0.5 * math.hypot(region_size[0], region_size[1])
        explicit_distance = constraints.get("distance_m")
        if explicit_distance is None:
            minimum, maximum, preferred_distance = _automatic_observation_profile(
                region_size, constraints_region,
                str(constraints.get("observation_detail", "normal")), horizontal_fov,
            )
            distance_source = "geometry_fov_auto"
        else:
            minimum, maximum = [float(value) for value in explicit_distance]
            preferred_distance = _preferred_observation_distance(
                minimum, maximum, region_size, constraints_region,
                str(constraints.get("observation_detail", "normal")),
            )
            distance_source = "user_constraint"
        radii = sorted(set([minimum, preferred_distance, maximum]))
        if distance_scale != 1.0:
            radii = [value * distance_scale for value in radii]
        region_samples = _region_samples(
            region_center, region_size, region_yaw, constraints_region
        )
        for radius in radii:
            radius_from_center = object_radius + radius
            for sample in range(24):
                angle = 2.0 * math.pi * sample / 24.0
                if not _relation_ok(constraints.get("relation"), angle, object_yaw, yaw_tolerance):
                    continue
                xy = [
                    region_center[0] + radius_from_center * math.cos(angle),
                    region_center[1] + radius_from_center * math.sin(angle),
                ]
                if not hasattr(grid, "is_state_valid") and not grid.is_free(xy):
                    continue
                between_ok, between_distance = _between_geometry(
                    xy, references, tolerance_m=max(0.75, 0.5 * maximum)
                ) if constraints.get("relation") == "between" else (True, 0.0)
                if not between_ok:
                    continue
                yaw = math.atan2(region_center[1] - xy[1], region_center[0] - xy[0])
                height_values = _height_candidates_for_view(
                    xy, region_samples, constraints, vertical_fov,
                    room.get("camera_height_band_m") if hasattr(grid, "is_state_valid") else None,
                )
                for z in height_values:
                    xyz = [xy[0], xy[1], float(z)]
                    if hasattr(grid, "is_state_valid") and not grid.is_state_valid(xyz):
                        continue
                    visible_ids = _visible_region_samples(
                        grid, xy, z, yaw, region_samples, horizontal_fov, vertical_fov
                    )
                    visible = bool(visible_ids)
                    if constraints.get("visibility_required", True) and not visible:
                        continue
                    reference_visible = [
                        _point_visible_from_pose(
                            grid, xyz, yaw, [float(value) for value in other["center_xyz_m"]],
                            horizontal_fov, vertical_fov,
                        )
                        for other in references
                    ]
                    # The anchor defines the region to inspect; its occupied
                    # center need not be visible from the same frame.  Region
                    # samples already enforce FoV/LOS, while the detected
                    # target-to-anchor relation is verified after observation.
                    view_quality = len(visible_ids) / max(1, len(region_samples))
                    reference_distance = min(
                        (math.dist(center[:2], other[:2]) for other in reference_centers),
                        default=0.0,
                    )
                    terminal_score = (
                        abs(radius - preferred_distance)
                        + (1.0 - view_quality)
                        + 0.1 * abs(float(z) - sum(point[2] for point in region_samples) / len(region_samples))
                        + 0.15 * reference_distance
                        + 0.15 * between_distance
                    )
                    candidates.append({
                        "id": f"{task['id']}__{obj['id']}__r{distance_scale:.2f}__{len(candidates):03d}",
                        "task_id": task["id"],
                        "object_id": obj["id"],
                        "object_label": obj["label"],
                        "room_id": room["id"],
                        "room_type": room.get("semantic_type", "unknown"),
                        "floor_id": room.get("floor_id"),
                        "pose": {"x": xy[0], "y": xy[1], "z": float(z), "yaw": yaw},
                        "target_xyz_m": center,
                        "search_region_center_xyz_m": region_center,
                        "search_region_size_xyz_m": region_size,
                        "anchor_object_id": (
                            region_object["id"] if region_object["id"] != obj["id"] else None
                        ),
                        "location_hypothesis_id": region_object["id"],
                        "distance_to_box_surface_m": radius,
                        "preferred_observation_distance_m": preferred_distance,
                        "observation_distance_source": (
                            f"{distance_source}_fallback_ring_{distance_scale:.2f}"
                            if distance_scale != 1.0 else distance_source
                        ),
                        "line_of_sight": visible,
                        "candidate_angle_rad": angle % (2.0 * math.pi),
                        "region_type": constraints_region,
                        "region_sample_count": len(region_samples),
                        "visible_region_sample_ids": visible_ids,
                        "view_quality": view_quality,
                        "vertical_fov_deg": vertical_fov,
                        "horizontal_fov_deg": horizontal_fov,
                        "height_fov_validated": True,
                        "height_generation": "region_fov_interval",
                        "height_collision_validated": bool(hasattr(grid, "is_state_valid")),
                        "collision_validation": (
                            "falcon_raw_free_3d" if hasattr(grid, "is_state_valid")
                            else "2d_occupancy_only"
                        ),
                        "reference_distance_m": reference_distance if reference_centers else None,
                        "reference_object_ids": [item["id"] for item in references],
                        "reference_visible": reference_visible,
                        "target_reference_relation": (
                            constraints.get("relation") if references else None
                        ),
                        "between_distance_m": between_distance if constraints.get("relation") == "between" else None,
                        "terminal_cost": terminal_score,
                    })
    candidates.sort(key=lambda item: (item["terminal_cost"], -float(item["line_of_sight"]), item["id"]))
    if max_candidates <= 0:
        return []

    # A task may match several physical instances (for example, four beds on
    # the requested floor).  Each instance is an independent location
    # hypothesis and therefore owns an independent viewpoint budget.  A global
    # truncation made the cheapest bed consume nearly every candidate and left
    # the other beds with zero or one observation pose.
    by_location: dict[str, list[dict[str, Any]]] = {}
    for candidate in candidates:
        location_id = str(
            candidate.get("location_hypothesis_id")
            or candidate.get("object_id")
            or "default"
        )
        by_location.setdefault(location_id, []).append(candidate)

    def select_location(values: list[dict[str, Any]]) -> list[dict[str, Any]]:
        selected: list[dict[str, Any]] = []
        selected_ids: set[str] = set()
        occlusion_sensitive = any(
            item.get("region_type") == "below_region" for item in values
        )

        if occlusion_sensitive:
            # Under-furniture searches are dominated by bed/table edges and
            # legs. Two azimuths can expose very different RGB evidence even
            # when the coarse geometric region samples visible from them are
            # identical. Keep the best pose at every actually feasible sampled
            # azimuth, and only apply the per-location cap after that.
            best_by_angle: dict[float, dict[str, Any]] = {}
            for candidate in values:
                angle_key = round(float(candidate.get("candidate_angle_rad", 0.0)), 6)
                previous = best_by_angle.get(angle_key)
                candidate_key = (
                    -len(candidate.get("visible_region_sample_ids", [])),
                    float(candidate["terminal_cost"]),
                    candidate["id"],
                )
                if previous is None:
                    best_by_angle[angle_key] = candidate
                    continue
                previous_key = (
                    -len(previous.get("visible_region_sample_ids", [])),
                    float(previous["terminal_cost"]),
                    previous["id"],
                )
                if candidate_key < previous_key:
                    best_by_angle[angle_key] = candidate

            angular_candidates = list(best_by_angle.values())
            if len(angular_candidates) <= max_candidates:
                return sorted(
                    angular_candidates,
                    key=lambda item: (float(item["terminal_cost"]), item["id"]),
                )

            # More than eight valid azimuths are possible in an open room. Keep
            # the best first view, then maximize angular separation so the cap
            # covers the whole reachable perimeter rather than one side.
            first = min(
                angular_candidates,
                key=lambda item: (float(item["terminal_cost"]), item["id"]),
            )
            selected = [first]
            remaining = [item for item in angular_candidates if item["id"] != first["id"]]
            while remaining and len(selected) < max_candidates:
                candidate = max(
                    remaining,
                    key=lambda item: (
                        min(
                            abs(_angle_difference(
                                float(item.get("candidate_angle_rad", 0.0)),
                                float(old.get("candidate_angle_rad", 0.0)),
                            ))
                            for old in selected
                        ),
                        len(item.get("visible_region_sample_ids", [])),
                        -float(item["terminal_cost"]),
                    ),
                )
                selected.append(candidate)
                remaining.remove(candidate)
            return sorted(
                selected,
                key=lambda item: (float(item["terminal_cost"]), item["id"]),
            )

        def spatially_distinct(candidate: dict[str, Any]) -> bool:
            position = candidate["pose"]
            return not any(
                math.hypot(
                    position["x"] - old["pose"]["x"],
                    position["y"] - old["pose"]["y"],
                ) < 0.35
                and abs(position["z"] - old["pose"]["z"]) < 0.20
                for old in selected
            )

        # Preserve feasible height diversity first.  This matters for support
        # surfaces and under-furniture searches with a fixed-pitch camera.
        best_by_height = []
        for height in sorted({item["pose"]["z"] for item in values}):
            candidate = next(
                (item for item in values
                 if item["pose"]["z"] == height and spatially_distinct(item)),
                None,
            )
            if candidate is not None:
                best_by_height.append(candidate)
        if len(best_by_height) > max_candidates:
            best_by_height = sorted(
                best_by_height, key=lambda item: (item["terminal_cost"], item["id"])
            )[:max_candidates]
        for candidate in best_by_height:
            selected.append(candidate)
            selected_ids.add(candidate["id"])

        # Fill the remaining slots with complementary views.  Prefer a view
        # that exposes previously unseen search-region samples, then one with a
        # different azimuth.  This avoids spending all visits on nearly
        # identical poses merely because their path cost is a little cheaper.
        covered = {
            sample_id
            for candidate in selected
            for sample_id in candidate.get("visible_region_sample_ids", [])
        }
        while len(selected) < max_candidates:
            options = []
            for candidate in values:
                if candidate["id"] in selected_ids or not spatially_distinct(candidate):
                    continue
                visible_ids = set(candidate.get("visible_region_sample_ids", []))
                new_coverage = len(visible_ids - covered)
                if selected:
                    angle = float(candidate.get("candidate_angle_rad", 0.0))
                    angular_separation = min(
                        abs(_angle_difference(
                            angle, float(old.get("candidate_angle_rad", 0.0))
                        ))
                        for old in selected
                    )
                else:
                    angular_separation = math.pi
                options.append((
                    new_coverage,
                    angular_separation,
                    -float(candidate["terminal_cost"]),
                    candidate["id"],
                    candidate,
                ))
            if not options:
                break
            _, angular_separation, _, _, candidate = max(options, key=lambda item: item[:4])
            # Once no new region is exposed, retain only genuinely different
            # azimuths (30 degrees or more) instead of redundant jitter views.
            visible_ids = set(candidate.get("visible_region_sample_ids", []))
            if selected and not (visible_ids - covered) and angular_separation < math.radians(30.0):
                break
            selected.append(candidate)
            selected_ids.add(candidate["id"])
            covered.update(visible_ids)
        return selected

    selected = []
    ordered_locations = sorted(
        by_location.items(),
        key=lambda item: (
            min(float(value["terminal_cost"]) for value in item[1]),
            item[0],
        ),
    )
    for _, values in ordered_locations:
        selected.extend(select_location(values))
    return sorted(
        selected,
        key=lambda item: (float(item["terminal_cost"]), str(item["location_hypothesis_id"]), item["id"]),
    )


def select_spread_target_objects(scene_graph: dict[str, Any], task_graph: dict[str, Any]) -> dict[str, str]:
    """Choose language-compatible instances in distinct, distant rooms.

    This selects the mission anchors only. The downstream joint planner still
    computes the geometrically shortest feasible tour between those anchors.
    """
    selected: dict[str, str] = {}
    used_rooms: set[Any] = set()
    centers: list[list[float]] = []
    def selection_order(task):
        choices = matching_objects(scene_graph, task)
        destination_groups = {
            room.get("parent_room_id", room["id"])
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
            return room.get("parent_room_id", room["id"])

        if not centers or not task.get("active_initially", True):
            room, obj = choices[0]
        else:
            def spread_key(item):
                room, obj = item
                center = [float(value) for value in obj["center_xyz_m"]]
                separation = min(math.dist(center, old) for old in centers)
                distinct_room = int(room_group(room) not in used_rooms)
                return distinct_room, separation, float(obj.get("probability", 0.0))
            room, obj = max(choices, key=spread_key)
        selected[task["id"]] = obj["id"]
        if task.get("active_initially", True):
            used_rooms.add(room_group(room))
            centers.append([float(value) for value in obj["center_xyz_m"]])
    return selected


def generate_all_candidates(
    grid: OccupancyGrid,
    scene_graph: dict[str, Any],
    task_graph: dict[str, Any],
    max_candidates: int = 8,
    selected_objects: dict[str, str] | None = None,
    distance_scale: float = 1.0,
) -> dict[str, list[dict[str, Any]]]:
    result = {}
    for task in task_graph["tasks"]:
        object_id = None if selected_objects is None else selected_objects.get(task["id"])
        values = generate_candidates(
            grid, scene_graph, task, max_candidates=max_candidates,
            allowed_object_ids=None if object_id is None else {object_id},
            distance_scale=distance_scale,
        )
        if not values and object_id is not None:
            values = generate_candidates(
                grid, scene_graph, task, max_candidates=max_candidates, prefer_room=False,
                distance_scale=distance_scale,
            )
        # Language models identify task semantics, not scene-specific camera
        # tuning. If the requested semantic target is grounded but the nominal
        # viewing profile has no valid 3-D pose, try a small deterministic set
        # of observation profiles. Every result still passes the same FREE,
        # line-of-sight, FoV, A* and later B-spline checks.
        if not values and hasattr(grid, "is_state_valid"):
            for vertical_fov in (80.0, 90.0):
                relaxed = copy.deepcopy(task)
                relaxed["spatial_constraints"]["vertical_fov_deg"] = vertical_fov
                values = generate_candidates(
                    grid, scene_graph, relaxed, max_candidates=max_candidates,
                    allowed_object_ids=None if object_id is None else {object_id},
                    distance_scale=distance_scale,
                )
                for candidate in values:
                    candidate["observation_profile_fallback"] = {
                        "reason": "nominal_profile_has_no_valid_3d_candidate",
                        "distance_m": relaxed["spatial_constraints"].get("distance_m"),
                        "vertical_fov_deg": vertical_fov,
                    }
                if values:
                    break
        result[task["id"]] = values
    return result
