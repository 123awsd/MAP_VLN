"""Semantic-region geometry, view coverage, and negative-observation belief updates."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Any

from .candidate_poses import generate_candidates


SEMANTIC_PRIOR = {"high": 1.0, "medium": 0.6, "low": 0.3}
SUPPORT_LABELS = {
    "table", "dining table", "coffee table", "desk", "counter", "countertop",
    "nightstand", "bedside table", "shelf", "bookshelf", "cabinet", "dresser",
}
LOW_SEARCH_TARGETS = {"ball", "toy", "shoe", "slipper", "bucket", "trash can", "backpack", "bag"}
FIXED_TARGETS = {"television", "tv", "refrigerator", "fridge", "lamp", "oven", "microwave"}
SEMANTIC_BELIEF_PENALTY_WEIGHT = 1.5
COVER_COST_WEIGHT = 0.6
VIEW_QUALITY_PENALTY_WEIGHT = 0.8
REGION_TYPE_ALIASES = {
    "under_furniture": "below_region",
    "floor_near_anchor": "surrounding_region",
    "furniture_neighborhood": "surrounding_region",
    "fixed_instance": "instance_region",
}


def canonical_region_type(value: str) -> str:
    normalized = str(value).strip().lower()
    return REGION_TYPE_ALIASES.get(normalized, normalized)


def infer_region_type(target_label: str, anchor_label: str, relation: str = "near") -> str:
    """Convert an LLM relation/anchor pair into a geometric region family."""
    target = target_label.strip().lower()
    anchor = anchor_label.strip().lower()
    relation = relation.strip().lower()
    if relation in {"under", "below"}:
        return "below_region"
    if target == anchor or (target in FIXED_TARGETS and target in anchor):
        return "instance_region"
    if target in LOW_SEARCH_TARGETS and relation not in {"on", "above", "inside"}:
        return "surrounding_region"
    if relation in {"on", "above", "inside"} or anchor in SUPPORT_LABELS:
        return "support_surface"
    return "surrounding_region"


def _yaw_from_wxyz(value: list[float]) -> float:
    w, _, _, z = value
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def sample_region_points(
    obj: dict[str, Any], region_type: str, target_label: str | None = None,
) -> list[list[float]]:
    """Discretize the task-relevant part of an oriented object region."""
    region_type = canonical_region_type(region_type)
    center = [float(value) for value in obj["center_xyz_m"]]
    size = [max(0.12, float(value)) for value in obj["size_xyz_m"]]
    yaw = _yaw_from_wxyz([float(value) for value in obj.get("orientation_wxyz", [1, 0, 0, 0])])
    cosine, sine = math.cos(yaw), math.sin(yaw)

    def world(local_x: float, local_y: float, z: float) -> list[float]:
        return [
            center[0] + cosine * local_x - sine * local_y,
            center[1] + sine * local_x + cosine * local_y,
            z,
        ]

    if region_type == "support_surface":
        z = center[2] + 0.5 * size[2]
        return [
            world(ix * 0.4 * size[0], iy * 0.4 * size[1], z)
            for ix in (-1, 0, 1) for iy in (-1, 0, 1)
        ]
    if region_type == "instance_region":
        return [
            world(ix * 0.4 * size[0], iy * 0.4 * size[1], center[2] + iz * 0.4 * size[2])
            for ix, iy, iz in (
                (-1, -1, -1), (-1, 0, 0), (-1, 1, 1),
                (0, -1, -1), (0, 0, 0), (0, 1, 1),
                (1, -1, -1), (1, 0, 0), (1, 1, 1),
            )
        ]
    if region_type == "below_region":
        z = max(0.08, center[2] - 0.45 * size[2])
        return [
            world(ix * 0.35 * size[0], iy * 0.35 * size[1], z)
            for ix in (-1, 0, 1) for iy in (-1, 0, 1)
        ]

    radius_x, radius_y = 0.5 * size[0] + 0.45, 0.5 * size[1] + 0.45
    z = (
        0.12 if region_type == "surrounding_region"
        and str(target_label or "").strip().lower() in LOW_SEARCH_TARGETS
        else max(0.25, center[2])
    )
    return [
        world(radius_x * math.cos(angle), radius_y * math.sin(angle), z)
        for angle in (index * math.pi / 4.0 for index in range(8))
    ]


def _visible_samples(
    grid, pose: dict[str, float], samples: list[list[float]], endpoint_clearance_m: float = 0.10,
) -> list[int]:
    visible = []
    for index, sample in enumerate(samples):
        if hasattr(grid, "line_of_sight"):
            line_free = grid.line_of_sight(
                [pose["x"], pose["y"], pose["z"]], sample
            )
        else:
            endpoint_cells = max(2, int(math.ceil(endpoint_clearance_m / grid.resolution)))
            line_free = grid.line_is_free(
                [pose["x"], pose["y"]], sample[:2], allow_endpoint_cells=endpoint_cells
            )
        if line_free:
            visible.append(index)
    return visible


def _coverage_route_cost(grid, candidates: list[dict[str, Any]], sample_count: int) -> float:
    """Greedy set-cover tour estimate used as intrinsic region search cost."""
    uncovered = set(range(sample_count))
    selected: list[dict[str, Any]] = []
    remaining = list(candidates)
    while uncovered and remaining and len(selected) < 4:
        best = max(remaining, key=lambda item: len(uncovered & set(item["visible_region_sample_ids"])))
        gain = uncovered & set(best["visible_region_sample_ids"])
        if not gain:
            break
        selected.append(best)
        uncovered -= gain
        remaining.remove(best)
    cost = 0.0
    for first, second in zip(selected, selected[1:]):
        if hasattr(grid, "path"):
            path = grid.path(
                [first["pose"][axis] for axis in ("x", "y", "z")],
                [second["pose"][axis] for axis in ("x", "y", "z")],
            )
        else:
            path = grid.astar(
                [first["pose"]["x"], first["pose"]["y"]],
                [second["pose"]["x"], second["pose"]["y"]],
            )
        cost += math.inf if path is None else path.length_m
    if not math.isfinite(cost):
        return 1e3
    uncovered_fraction = len(uncovered) / max(1, sample_count)
    return cost + 2.0 * uncovered_fraction


def materialize_semantic_regions(
    grid, scene_graph: dict[str, Any], task: dict[str, Any], search_plan: dict[str, Any],
    max_candidates_per_region: int = 4,
) -> list[dict[str, Any]]:
    """Instantiate VLM suggestions as geometrically scored search-region views."""
    objects = {
        obj["id"]: (room, obj)
        for room in scene_graph.get("rooms", []) for obj in room.get("objects", [])
    }
    regions: list[tuple[dict[str, Any], list[dict[str, Any]], float]] = []
    target_label = str(task.get("verification_label") or task["target"]["label"]).lower()
    for index, hypothesis in enumerate(search_plan.get("hypotheses", []), start=1):
        pair = objects.get(hypothesis["anchor_object_id"])
        if pair is None:
            continue
        room, obj = pair
        region_type = canonical_region_type(hypothesis.get("semantic_region") or infer_region_type(
            target_label, obj["label"], hypothesis.get("relation", "near")
        ))
        region_id = f"region_{room['id']}__{obj['id']}__{region_type}"
        samples = sample_region_points(obj, region_type, target_label)
        endpoint_clearance = (
            0.6 * math.hypot(float(obj["size_xyz_m"][0]), float(obj["size_xyz_m"][1]))
            if region_type in {"support_surface", "below_region", "instance_region"}
            else 0.10
        )
        branch_task = copy.deepcopy(task)
        branch_task["target"] = {
            "label": obj["label"], "room": None, "floor_id": room.get("floor_id"),
            "reference": None, "reference_secondary": None, "references": [],
        }
        branch_task["spatial_constraints"]["relation"] = None
        branch_task["spatial_constraints"]["region_type"] = region_type
        generated = generate_candidates(
            grid, scene_graph, branch_task, max_candidates=max_candidates_per_region * 3,
            allowed_object_ids={obj["id"]}, prefer_room=False,
        )
        enriched = []
        for candidate in generated:
            visible = _visible_samples(
                grid, candidate["pose"], samples, endpoint_clearance_m=endpoint_clearance
            )
            if not visible:
                continue
            fraction = len(visible) / max(1, len(samples))
            historical_confidence = float(hypothesis.get("historical_confidence", 0.0))
            semantic_prior = SEMANTIC_PRIOR[hypothesis.get("relevance", "medium")]
            belief_score = max(semantic_prior, historical_confidence)
            candidate.update({
                "id": f"{task['id']}__{region_id}__v{len(enriched):02d}",
                "candidate_source": "vlm_semantic_region",
                "location_hypothesis_id": region_id,
                "semantic_region_id": region_id,
                "semantic_region_type": region_type,
                "semantic_prior_level": hypothesis.get("relevance", "medium"),
                "semantic_prior_score": semantic_prior,
                "historical_confidence": historical_confidence,
                "belief_score": belief_score,
                "region_sample_count": len(samples),
                "visible_region_sample_ids": visible,
                "view_quality": fraction,
                "recovery_search": copy.deepcopy(hypothesis),
            })
            enriched.append(candidate)
            if len(enriched) >= max_candidates_per_region:
                break
        if enriched:
            regions.append((hypothesis, enriched, _coverage_route_cost(grid, enriched, len(samples))))

    finite_costs = [cost for _, _, cost in regions if math.isfinite(cost)]
    low, high = (min(finite_costs), max(finite_costs)) if finite_costs else (0.0, 0.0)
    result = []
    for _, candidates, cover_cost in regions:
        cover_norm = 0.0 if high <= low + 1e-9 else (cover_cost - low) / (high - low)
        for candidate in candidates:
            candidate["region_cover_cost_m"] = cover_cost
            candidate["normalized_cover_cost"] = cover_norm
            candidate["base_geometry_cost"] = (
                COVER_COST_WEIGHT * cover_norm
                + VIEW_QUALITY_PENALTY_WEIGHT * (1.0 - candidate["view_quality"])
            )
            candidate["terminal_cost"] = (
                candidate["base_geometry_cost"]
                + SEMANTIC_BELIEF_PENALTY_WEIGHT * (1.0 - candidate["belief_score"])
            )
            candidate["search_cost_components"] = {
                "semantic_belief": candidate["belief_score"],
                "normalized_region_cover_cost": cover_norm,
                "view_quality": candidate["view_quality"],
                "weights": {
                    "semantic_belief_penalty": SEMANTIC_BELIEF_PENALTY_WEIGHT,
                    "region_cover_cost": COVER_COST_WEIGHT,
                    "view_quality_penalty": VIEW_QUALITY_PENALTY_WEIGHT,
                },
            }
            result.append(candidate)
    return sorted(result, key=lambda item: (item["terminal_cost"], item["id"]))


def materialize_room_frontier_fallback(
    grid, scene_graph: dict[str, Any], task: dict[str, Any], search_plan: dict[str, Any],
    max_candidates: int = 4,
) -> list[dict[str, Any]]:
    """Create low-priority task-visibility views after semantic regions are exhausted.

    This is deliberately held out of the main candidate set.  The executor only
    activates it after every task-relevant region has failed, making generic
    room coverage a fallback rather than the default search behavior.
    """
    room_ids = []
    for hypothesis in search_plan.get("hypotheses", []):
        room_id = hypothesis["room_id"]
        if room_id not in room_ids:
            room_ids.append(room_id)
    rooms = {room["id"]: room for room in scene_graph.get("rooms", [])}
    result = []
    for room_id in room_ids:
        room = rooms.get(room_id)
        if room is None:
            continue
        center = [float(value) for value in room.get("centroid_xy_m", [0.0, 0.0])]
        polygon = room.get("polygon_xy_m") or [center]
        stride = max(1, len(polygon) // 12)
        samples_xy = [[float(value) for value in point] for point in polygon[::stride]][:12]
        if center not in samples_xy:
            samples_xy.append(center)
        region_id = f"room_{room_id}__task_frontier_fallback"
        pose_seeds = []
        if len(samples_xy) == 1:
            pose_seeds = [center]
        else:
            for point in samples_xy[::max(1, len(samples_xy) // 6)]:
                pose_seeds.append([
                    center[0] + 0.55 * (point[0] - center[0]),
                    center[1] + 0.55 * (point[1] - center[1]),
                ])
        candidates = []
        height_band = room.get("camera_height_band_m", [0.65, 1.8])
        nominal_z = 0.5 * (float(height_band[0]) + float(height_band[1]))
        for seed in pose_seeds:
            if hasattr(grid, "voxel_map"):
                free_xyz = grid.voxel_map.nearest_valid([seed[0], seed[1], nominal_z], radius_m=1.0)
                free = None if free_xyz is None else free_xyz[:2]
                free_z = None if free_xyz is None else free_xyz[2]
            else:
                free = grid.nearest_free(seed, max_radius_m=1.0)
                free_z = nominal_z
            if free is None or any(math.dist(free, old["pose_xy"]) < 0.5 for old in candidates):
                continue
            if hasattr(grid, "line_of_sight"):
                visible = [
                    index for index, sample in enumerate(samples_xy)
                    if grid.line_of_sight([free[0], free[1], free_z], [sample[0], sample[1], free_z])
                ]
            else:
                visible = [
                    index for index, sample in enumerate(samples_xy)
                    if grid.line_is_free(free, sample, allow_endpoint_cells=2)
                ]
            if not visible:
                continue
            yaw = math.atan2(center[1] - free[1], center[0] - free[0])
            candidates.append({"pose_xy": free, "z": free_z, "visible": visible, "yaw": yaw})
        candidates.sort(key=lambda value: (-len(value["visible"]), value["pose_xy"]))
        for index, value in enumerate(candidates[:max_candidates]):
            quality = len(value["visible"]) / max(1, len(samples_xy))
            base_geometry = 1.2 + VIEW_QUALITY_PENALTY_WEIGHT * (1.0 - quality)
            result.append({
                "id": f"{task['id']}__{region_id}__v{index:02d}",
                "task_id": task["id"], "object_id": f"room_{room_id}_fallback",
                "object_label": "task_frontier", "room_id": room_id,
                "room_type": room.get("semantic_type", "unknown"),
                "floor_id": room.get("floor_id"),
                "pose": {"x": value["pose_xy"][0], "y": value["pose_xy"][1], "z": value["z"], "yaw": value["yaw"]},
                "candidate_angle_rad": math.atan2(
                    value["pose_xy"][1] - center[1], value["pose_xy"][0] - center[0]
                ) % (2.0 * math.pi),
                "target_xyz_m": [center[0], center[1], value["z"]],
                "line_of_sight": True,
                "terminal_cost": base_geometry + SEMANTIC_BELIEF_PENALTY_WEIGHT * 0.85,
                "candidate_source": "room_frontier_fallback",
                "location_hypothesis_id": region_id, "semantic_region_id": region_id,
                "semantic_region_type": "room_frontier_fallback",
                "semantic_prior_level": "fallback", "semantic_prior_score": 0.15,
                "belief_score": 0.15, "region_sample_count": len(samples_xy),
                "visible_region_sample_ids": value["visible"], "view_quality": quality,
                "region_cover_cost_m": 0.0, "normalized_cover_cost": 1.0,
                "base_geometry_cost": base_geometry,
            })
    return sorted(result, key=lambda item: (item["terminal_cost"], item["id"]))


@dataclass
class SemanticRegionBelief:
    """Task-local belief over semantic regions, updated from negative views."""

    detector_reliability: float = 0.80
    beliefs: dict[str, float] = field(default_factory=dict)
    observed_samples: dict[str, set[int]] = field(default_factory=dict)

    def register(self, candidates: list[dict[str, Any]]) -> None:
        for candidate in candidates:
            region_id = candidate.get("semantic_region_id")
            if not region_id:
                continue
            self.beliefs.setdefault(region_id, float(candidate["semantic_prior_score"]))
            self.observed_samples.setdefault(region_id, set())
        self.apply(candidates)

    def observe(self, candidate: dict[str, Any], found: bool) -> dict[str, Any] | None:
        region_id = candidate.get("semantic_region_id")
        if not region_id:
            return None
        old = self.beliefs[region_id]
        samples = set(map(int, candidate.get("visible_region_sample_ids", [])))
        observed = self.observed_samples[region_id]
        novel = samples - observed
        observed.update(samples)
        total = max(1, int(candidate.get("region_sample_count", 1)))
        incremental_coverage = len(novel) / total
        cumulative_coverage = min(1.0, len(observed) / total)
        quality = max(0.0, min(1.0, float(candidate.get("view_quality", 0.0))))
        if found:
            new = 1.0
        else:
            new = old * (1.0 - self.detector_reliability * incremental_coverage * quality)
        self.beliefs[region_id] = max(0.0, min(1.0, new))
        return {
            "type": "semantic_region_belief",
            "semantic_region_id": region_id,
            "old_belief": old,
            "new_belief": self.beliefs[region_id],
            "incremental_coverage": incremental_coverage,
            "cumulative_coverage": cumulative_coverage,
            "view_quality": quality,
            "negative_observation": not found,
        }

    def apply(self, candidates: list[dict[str, Any]]) -> None:
        for candidate in candidates:
            region_id = candidate.get("semantic_region_id")
            if not region_id:
                continue
            belief = self.beliefs.get(region_id, float(candidate["semantic_prior_score"]))
            candidate["belief_score"] = belief
            candidate["terminal_cost"] = (
                float(candidate["base_geometry_cost"])
                + SEMANTIC_BELIEF_PENALTY_WEIGHT * (1.0 - belief)
            )
            if "search_cost_components" in candidate:
                candidate["search_cost_components"]["semantic_belief"] = belief
