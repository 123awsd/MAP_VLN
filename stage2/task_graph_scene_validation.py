"""Deterministic scene-grounding checks for a Qwen-generated task graph."""

from __future__ import annotations

from typing import Any

from .vocabulary import load_semantic_aliases


ALIASES = load_semantic_aliases()


class TaskSceneValidationError(ValueError):
    pass


def _accepted(label: str) -> set[str]:
    normalized = str(label).strip().lower()
    return set(ALIASES.get(normalized, {normalized}))


def validate_task_graph_against_scene(
    graph: dict[str, Any], scene_graph: dict[str, Any], *, require_candidates: bool = False,
) -> dict[str, Any]:
    errors = []
    grounding = {}
    for task in graph.get("tasks", []):
        target = task["target"]
        floor_id = target.get("floor_id")
        accepted = _accepted(target["label"])
        matches = []
        for room in scene_graph.get("rooms", []):
            if floor_id is not None and int(room.get("floor_id", -1)) != int(floor_id):
                continue
            for obj in room.get("objects", []):
                if str(obj.get("label", "")).lower() in accepted:
                    matches.append((room, obj))
        if not matches:
            errors.append(f"{task['id']}: target {target['label']} is absent on requested floor")
            continue
        reference_ids = []
        for reference in target.get("references", []):
            reference_accepted = _accepted(reference)
            scoped = [
                ref
                for room, obj in matches
                for ref in room.get("objects", [])
                if str(ref.get("label", "")).lower() in reference_accepted
            ]
            if not scoped:
                errors.append(
                    f"{task['id']}: reference {reference} is absent from every matching target room"
                )
            else:
                reference_ids.extend(ref["id"] for ref in scoped)
        grounding[task["id"]] = {
            "target_object_ids": [obj["id"] for _, obj in matches],
            "reference_object_ids": sorted(set(reference_ids)),
        }
        if (
            task.get("search_policy", {}).get("mode") == "semantic_recovery"
            and (target.get("floor_id") is not None or target.get("room") or target.get("references"))
        ):
            errors.append(f"{task['id']}: explicit location must use fixed search policy")
    instruction = str(graph.get("instruction", ""))
    if "最后" in instruction:
        tasks = graph.get("tasks", [])
        conditional_targets = {
            task_id for rule in graph.get("conditional_rules", [])
            for task_id in rule.get("activate_task_ids", [])
        }
        final_candidates = [
            task for task in tasks
            if task["id"] not in conditional_targets and task.get("prerequisites")
        ]
        if final_candidates:
            final_task = final_candidates[-1]
            required = {
                task["id"] for task in tasks
                if task.get("active_initially", True) and task["id"] != final_task["id"]
            }
            missing = required - set(final_task.get("prerequisites", []))
            if missing:
                errors.append(
                    f"{final_task['id']}: final task misses prerequisites {sorted(missing)}"
                )
    if errors:
        raise TaskSceneValidationError("; ".join(errors))
    return {"valid": True, "tasks": grounding}
