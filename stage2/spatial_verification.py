"""Geometric verification of target-to-reference relations in the 3-D scene graph."""

from __future__ import annotations

import math
from typing import Any

from .vocabulary import load_semantic_aliases


ALIASES = load_semantic_aliases()


def _accepted(label: str) -> set[str]:
    normalized = str(label).strip().lower()
    return set(ALIASES.get(normalized, {normalized}))


def scoped_reference_objects(
    scene_graph: dict[str, Any], target_object_id: str, reference_labels: list[str],
) -> list[dict[str, Any]]:
    target_room = None
    for room in scene_graph.get("rooms", []):
        if any(obj.get("id") == target_object_id for obj in room.get("objects", [])):
            target_room = room
            break
    if target_room is None:
        return []
    result = []
    for label in reference_labels:
        accepted = _accepted(label)
        matches = [
            obj for obj in target_room.get("objects", [])
            if str(obj.get("label", "")).lower() in accepted
        ]
        if matches:
            result.append(max(matches, key=lambda obj: float(obj.get("probability", 0.0))))
    return result


def verify_target_reference_relation(
    relation: str | None, target: dict[str, Any], references: list[dict[str, Any]],
    *, near_distance_m: float = 2.0, support_vertical_tolerance_m: float = 0.45,
) -> dict[str, Any]:
    relation = None if relation is None else str(relation).lower()
    if relation is None:
        return {"required": False, "valid": True, "relation": None}
    if not references:
        return {"required": True, "valid": False, "relation": relation, "reason": "reference_missing"}
    tc = [float(v) for v in target["center_xyz_m"]]
    ts = [float(v) for v in target["size_xyz_m"]]
    distances = [math.dist(tc, [float(v) for v in ref["center_xyz_m"]]) for ref in references]
    valid = True
    metrics: dict[str, Any] = {"center_distances_m": distances}
    if relation == "near":
        valid = min(distances) <= near_distance_m
    elif relation == "on":
        ref = references[0]
        rc = [float(v) for v in ref["center_xyz_m"]]
        rs = [float(v) for v in ref["size_xyz_m"]]
        horizontal_tolerance = 0.5 * math.hypot(rs[0] + ts[0], rs[1] + ts[1])
        horizontal_distance = math.dist(tc[:2], rc[:2])
        target_bottom = tc[2] - 0.5 * ts[2]
        reference_top = rc[2] + 0.5 * rs[2]
        vertical_gap = target_bottom - reference_top
        metrics.update({"horizontal_distance_m": horizontal_distance, "vertical_gap_m": vertical_gap})
        valid = horizontal_distance <= horizontal_tolerance and abs(vertical_gap) <= support_vertical_tolerance_m
    elif relation in {"above", "below"}:
        ref = references[0]
        rc = [float(v) for v in ref["center_xyz_m"]]
        rs = [float(v) for v in ref["size_xyz_m"]]
        horizontal_tolerance = 0.5 * math.hypot(rs[0] + ts[0], rs[1] + ts[1])
        horizontal_distance = math.dist(tc[:2], rc[:2])
        if relation == "above":
            signed_clearance = (tc[2] - 0.5 * ts[2]) - (rc[2] + 0.5 * rs[2])
            vertically_ordered = tc[2] > rc[2]
        else:
            signed_clearance = (rc[2] - 0.5 * rs[2]) - (tc[2] + 0.5 * ts[2])
            vertically_ordered = tc[2] < rc[2]
        metrics.update({
            "horizontal_distance_m": horizontal_distance,
            "signed_clearance_m": signed_clearance,
        })
        valid = (
            vertically_ordered
            and horizontal_distance <= horizontal_tolerance
            and signed_clearance >= -support_vertical_tolerance_m
        )
    elif relation == "between":
        valid = len(references) >= 2
    return {
        "required": True, "valid": bool(valid), "relation": relation,
        "target_object_id": target.get("id"),
        "reference_object_ids": [item.get("id") for item in references],
        "metrics": metrics,
    }


def effective_relation_context(
    task: dict[str, Any], candidate: dict[str, Any], mapped_objects: dict[str, dict[str, Any]],
    projected_targets: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Resolve relation scope for initial and semantic-recovery observations.

    A task may navigate using a known anchor (for example a vanity) while the
    live detector searches for another category (toilet paper).  Recovery may
    then move to a different anchor or an exact historical target box.  The
    original location relation must not leak into that new hypothesis.
    """
    projected_targets = projected_targets or []
    recovery = candidate.get("recovery_search") or {}
    source = str(recovery.get("source", ""))
    if source == "historical_target_box":
        return {
            "relation": None,
            "target": mapped_objects.get(candidate.get("object_id")),
            "references": [],
            "scope": "historical_target_box_visual_recheck",
        }

    task_target_label = str(task.get("target", {}).get("label", "")).strip().lower()
    verification_label = str(task.get("verification_label", task_target_label)).strip().lower()
    candidate_anchor_id = candidate.get("anchor_object_id")
    anchor_search = (
        bool(recovery)
        or task_target_label != verification_label
        or candidate_anchor_id is not None
    )
    if anchor_search:
        anchor_id = str(
            recovery.get("anchor_object_id")
            or candidate_anchor_id
            or candidate.get("object_id")
            or ""
        )
        anchor = mapped_objects.get(anchor_id)
        relation = recovery.get("relation") if recovery else task.get("spatial_constraints", {}).get("relation")
        confirmed = [item for item in projected_targets if item.get("confirmed")]
        projected = max(confirmed, key=lambda item: float(item.get("score", 0.0))) if confirmed else None
        target = None if projected is None else {
            "id": projected.get("associated_object_id") or f"online_{verification_label}",
            "center_xyz_m": projected["center"],
            "size_xyz_m": projected["size"],
        }
        return {
            "relation": relation,
            "target": target,
            "references": [] if anchor is None else [anchor],
            "scope": "recovery_anchor" if recovery else "explicit_location_anchor",
        }

    target_object = mapped_objects.get(candidate.get("object_id"))
    reference_labels = list(task.get("target", {}).get("references") or [])
    return {
        "relation": task.get("spatial_constraints", {}).get("relation"),
        "target": target_object,
        "reference_labels": reference_labels,
        "references": None,
        "scope": "original_task_relation",
    }
