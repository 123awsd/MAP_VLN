"""Canonical task-graph schema, normalization, and semantic validation."""

from __future__ import annotations

import copy
import re
from collections import defaultdict, deque
from typing import Any


FORMAT = "pre_map_vln.task_graph.v1"
ALLOWED_ACTIONS = {"inspect", "find", "observe", "deliver", "approach"}
ALLOWED_RELATIONS = {
    "front", "behind", "left", "right", "above", "below", "on", "between", "near", "facing"
}
ALLOWED_SEARCH_MODES = {"fixed", "semantic_recovery"}
ALLOWED_EXHAUSTION_POLICIES = {"finish", "qwen_semantic_recovery"}
ALLOWED_GOAL_TYPES = {"verify_presence", "locate_target", "execute_action"}
ALLOWED_NOT_FOUND_POLICIES = {"report_absent", "semantic_recovery", "explicit_branch"}
ALLOWED_REGION_TYPES = {
    "auto", "support_surface", "below_region", "surrounding_region",
    "instance_region", "between_region",
}
ALLOWED_OBSERVATION_DETAILS = {"normal", "fine"}
IDENTIFIER = re.compile(r"^[a-z][a-z0-9_]{0,63}$")


class TaskGraphError(ValueError):
    """Raised when a VLM result cannot represent a safe executable task graph."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise TaskGraphError(message)


def _optional_text(value: Any) -> str | None:
    text = str(value or "").strip().lower()
    return None if text in {"", "none", "null", "无", "没有", "不指定"} else text


def _detect_cycle(tasks: list[dict[str, Any]]) -> None:
    task_ids = {task["id"] for task in tasks}
    outgoing: dict[str, list[str]] = defaultdict(list)
    indegree = {task_id: 0 for task_id in task_ids}
    for task in tasks:
        for predecessor in task["prerequisites"]:
            outgoing[predecessor].append(task["id"])
            indegree[task["id"]] += 1
    queue = deque(sorted(task_id for task_id, degree in indegree.items() if degree == 0))
    visited = 0
    while queue:
        current = queue.popleft()
        visited += 1
        for successor in outgoing[current]:
            indegree[successor] -= 1
            if indegree[successor] == 0:
                queue.append(successor)
    _require(visited == len(task_ids), "task prerequisites contain a cycle")


def _collapse_generic_same_target_recovery(graph: dict[str, Any]) -> None:
    """Fold a model-invented global fallback task into scoped semantic recovery.

    A vague request such as "if it is not there, continue looking elsewhere"
    is one locate-target task: exhaust the user-specified location, then invoke
    semantic recovery.  It is not an explicit branch to a second, unscoped task.
    Concrete fallback rooms/floors/references remain ordinary conditional tasks.
    """
    tasks = graph.get("tasks")
    rules = graph.get("conditional_rules")
    if not isinstance(tasks, list) or not isinstance(rules, list):
        return
    tasks_by_id = {
        str(task.get("id", "")): task for task in tasks if isinstance(task, dict)
    }
    prerequisite_references = {
        str(predecessor)
        for task in tasks if isinstance(task, dict)
        for predecessor in (task.get("prerequisites") or [])
    }
    rule_references: dict[str, int] = defaultdict(int)
    for rule in rules:
        if not isinstance(rule, dict):
            continue
        for task_id in rule.get("activate_task_ids", []) or []:
            rule_references[str(task_id)] += 1
        for task_id in rule.get("skip_task_ids", []) or []:
            rule_references[str(task_id)] += 1

    removed_tasks: set[str] = set()
    removed_rule_indices: set[int] = set()
    for index, rule in enumerate(rules):
        if not isinstance(rule, dict):
            continue
        if str(rule.get("if_outcome", "not_found")).strip().lower() != "not_found":
            continue
        activate = [str(item) for item in (rule.get("activate_task_ids") or [])]
        if len(activate) != 1 or (rule.get("skip_task_ids") or []):
            continue
        source_id = str(rule.get("source_task_id", ""))
        fallback_id = activate[0]
        source = tasks_by_id.get(source_id)
        fallback = tasks_by_id.get(fallback_id)
        if source is None or fallback is None or fallback_id in prerequisite_references:
            continue
        if rule_references.get(fallback_id) != 1:
            continue
        source_label = str(
            source.get("verification_label")
            or (source.get("target") or {}).get("label", "")
        ).strip().lower()
        fallback_label = str(
            fallback.get("verification_label")
            or (fallback.get("target") or {}).get("label", "")
        ).strip().lower()
        if not source_label or source_label != fallback_label:
            continue
        fallback_target = fallback.get("target") or {}
        has_concrete_scope = any(
            _optional_text(fallback_target.get(key)) is not None
            for key in ("room", "room_id", "floor_id", "reference", "reference_secondary")
        ) or any(
            _optional_text(item) is not None
            for item in (fallback_target.get("references") or [])
        )
        if has_concrete_scope:
            continue
        fallback_intent = fallback.get("intent") or {}
        fallback_search = fallback.get("search_policy") or {}
        is_semantic_recovery = (
            str(fallback_intent.get("not_found_policy", "")).strip().lower()
            == "semantic_recovery"
            or str(fallback_search.get("mode", "")).strip().lower()
            == "semantic_recovery"
            or str(fallback_search.get("on_exhaustion", "")).strip().lower()
            == "qwen_semantic_recovery"
        )
        if not is_semantic_recovery:
            continue

        source["action"] = "find"
        source["intent"] = {
            "goal_type": "locate_target", "not_found_policy": "semantic_recovery",
        }
        source["search_policy"] = {
            "mode": "fixed", "on_exhaustion": "qwen_semantic_recovery",
            "maximum_location_hypotheses": int(
                fallback_search.get("maximum_location_hypotheses", 3)
            ),
        }
        removed_tasks.add(fallback_id)
        removed_rule_indices.add(index)

    if removed_tasks:
        graph["tasks"] = [
            task for task in tasks if str(task.get("id", "")) not in removed_tasks
        ]
        graph["conditional_rules"] = [
            rule for index, rule in enumerate(rules) if index not in removed_rule_indices
        ]


def normalize_and_validate_task_graph(value: dict[str, Any], instruction: str = "") -> dict[str, Any]:
    """Return a normalized deep copy, rejecting unsafe or ambiguous VLM output."""
    _require(isinstance(value, dict), "task graph must be an object")
    graph = copy.deepcopy(value)
    _collapse_generic_same_target_recovery(graph)
    graph["format"] = FORMAT
    graph["instruction"] = str(graph.get("instruction") or instruction).strip()
    tasks = graph.get("tasks")
    _require(isinstance(tasks, list) and tasks, "task graph must contain non-empty tasks")
    declared_rules = graph.get("conditional_rules")
    declared_negative_branch_sources: set[str] = set()
    if isinstance(declared_rules, list):
        declared_negative_branch_sources = {
            str(rule.get("source_task_id", ""))
            for rule in declared_rules
            if isinstance(rule, dict)
            and str(rule.get("if_outcome", "not_found")).strip().lower() == "not_found"
        }

    seen: set[str] = set()
    explicit_intent_tasks: set[str] = set()
    normalized_tasks = []
    for index, source in enumerate(tasks):
        _require(isinstance(source, dict), f"task {index} must be an object")
        task = copy.deepcopy(source)
        task_id = str(task.get("id", "")).strip()
        _require(bool(IDENTIFIER.fullmatch(task_id)), f"invalid task id: {task_id!r}")
        _require(task_id not in seen, f"duplicate task id: {task_id}")
        seen.add(task_id)
        action = str(task.get("action", "inspect")).lower().strip()
        _require(action in ALLOWED_ACTIONS, f"unsupported action {action!r} in {task_id}")
        target = task.get("target")
        _require(isinstance(target, dict), f"task {task_id} target must be an object")
        label = str(target.get("label", "")).strip().lower()
        _require(bool(label), f"task {task_id} target label is empty")
        reference = _optional_text(target.get("reference"))
        reference_secondary = _optional_text(target.get("reference_secondary"))
        raw_references = target.get("references")
        if raw_references is not None:
            _require(
                isinstance(raw_references, list),
                f"task {task_id} target references must be a list",
            )
            references = [
                value for value in (_optional_text(item) for item in raw_references)
                if value is not None
            ]
            if reference is None and references:
                reference = references[0]
            if reference_secondary is None and len(references) >= 2:
                reference_secondary = references[1]
            _require(
                len(references) <= 2,
                f"task {task_id} target references supports at most two reference objects",
            )
        references = [value for value in (reference, reference_secondary) if value is not None]
        floor_id = target.get("floor_id")
        if floor_id is not None:
            _require(
                isinstance(floor_id, int) and not isinstance(floor_id, bool),
                f"task {task_id} target floor_id must be an integer or null",
            )
        normalized_target = {
            "label": label,
            "room": _optional_text(target.get("room")),
            "room_id": (
                str(target.get("room_id")).strip()
                if str(target.get("room_id") or "").strip().lower()
                not in {"", "none", "null", "无", "没有", "不指定"}
                else None
            ),
            "floor_id": floor_id,
            "reference": reference,
            "reference_secondary": reference_secondary,
            "references": references,
        }
        if normalized_target["floor_id"] is not None:
            _require(
                normalized_target["floor_id"] >= 1,
                f"task {task_id} target floor_id must be a positive integer",
            )
        constraints = task.get("spatial_constraints") or {}
        _require(isinstance(constraints, dict), f"task {task_id} spatial_constraints must be an object")
        relation = constraints.get("relation")
        if relation is not None:
            relation = str(relation).lower().strip()
            _require(relation in ALLOWED_RELATIONS, f"unsupported relation {relation!r}")
        if relation == "between":
            _require(
                len(references) >= 2,
                f"task {task_id} relation 'between' requires two reference objects",
            )
        distance = constraints.get("distance_m")
        if distance is not None:
            _require(
                isinstance(distance, list) and len(distance) == 2,
                f"task {task_id} distance_m must be null or have two values",
            )
            distance = [float(distance[0]), float(distance[1])]
            _require(
                0.2 <= distance[0] <= distance[1] <= 8.0,
                f"task {task_id} has invalid distance_m",
            )
        region_type = str(constraints.get("region_type", "auto")).strip().lower()
        _require(
            region_type in ALLOWED_REGION_TYPES,
            f"unsupported region_type {region_type!r} in {task_id}",
        )
        observation_detail = str(constraints.get("observation_detail", "normal")).strip().lower()
        _require(
            observation_detail in ALLOWED_OBSERVATION_DETAILS,
            f"unsupported observation_detail {observation_detail!r} in {task_id}",
        )
        height_range = constraints.get("height_range_m")
        if height_range is not None:
            _require(
                isinstance(height_range, list) and len(height_range) == 2,
                f"task {task_id} height_range_m must have two values",
            )
            height_range = [float(height_range[0]), float(height_range[1])]
            _require(
                0.1 <= height_range[0] <= height_range[1] <= 10.0,
                f"task {task_id} has invalid height_range_m",
            )
        height_m = constraints.get("height_m")
        if height_m is not None:
            height_m = float(height_m)
            _require(0.1 <= height_m <= 10.0, f"task {task_id} has invalid height_m")
        vertical_fov = float(constraints.get("vertical_fov_deg", 70.0))
        _require(10.0 <= vertical_fov <= 170.0, f"task {task_id} has invalid vertical_fov_deg")
        horizontal_fov = float(constraints.get("horizontal_fov_deg", 90.0))
        _require(10.0 <= horizontal_fov <= 170.0, f"task {task_id} has invalid horizontal_fov_deg")
        yaw_tolerance = float(constraints.get("yaw_tolerance_deg", 55.0))
        _require(1.0 <= yaw_tolerance <= 180.0, f"task {task_id} has invalid yaw_tolerance_deg")
        prerequisites = sorted(set(str(item) for item in (task.get("prerequisites") or [])))
        task["id"] = task_id
        task["action"] = action
        task["target"] = normalized_target
        task["verification_label"] = str(task.get("verification_label") or label).strip().lower()
        task["spatial_constraints"] = {
            "relation": relation,
            "distance_m": distance,
            "height_m": height_m,
            "height_range_m": height_range,
            "region_type": region_type,
            "observation_detail": observation_detail,
            "vertical_fov_deg": vertical_fov,
            "horizontal_fov_deg": horizontal_fov,
            "yaw_tolerance_deg": yaw_tolerance,
            "face_target": bool(constraints.get("face_target", True)),
            "visibility_required": bool(constraints.get("visibility_required", True)),
        }
        task["prerequisites"] = prerequisites
        task["active_initially"] = bool(task.get("active_initially", True))
        task["success_outcome"] = str(task.get("success_outcome", "found" if action in {"find", "inspect"} else "done"))
        search_policy = task.get("search_policy") or {}
        _require(isinstance(search_policy, dict), f"task {task_id} search_policy must be an object")
        search_mode = str(search_policy.get("mode", "fixed")).strip().lower()
        _require(search_mode in ALLOWED_SEARCH_MODES, f"unsupported search mode {search_mode!r}")
        on_exhaustion = str(search_policy.get(
            "on_exhaustion",
            "qwen_semantic_recovery" if search_mode == "semantic_recovery" else "finish",
        )).strip().lower()
        _require(
            on_exhaustion in ALLOWED_EXHAUSTION_POLICIES,
            f"unsupported exhaustion policy {on_exhaustion!r} in {task_id}",
        )
        intent = task.get("intent") or {}
        _require(isinstance(intent, dict), f"task {task_id} intent must be an object")
        if task.get("intent") is not None:
            explicit_intent_tasks.add(task_id)
        goal_type = str(intent.get(
            "goal_type",
            "locate_target" if (
                action == "find"
                or search_mode == "semantic_recovery"
                or on_exhaustion == "qwen_semantic_recovery"
            ) else "verify_presence" if action in {"inspect", "observe"} else "execute_action",
        )).strip().lower()
        _require(goal_type in ALLOWED_GOAL_TYPES, f"unsupported goal type {goal_type!r} in {task_id}")
        not_found_policy = str(intent.get(
            "not_found_policy",
            "semantic_recovery" if on_exhaustion == "qwen_semantic_recovery" else "report_absent",
        )).strip().lower()
        _require(
            not_found_policy in ALLOWED_NOT_FOUND_POLICIES,
            f"unsupported not-found policy {not_found_policy!r} in {task_id}",
        )
        has_declared_negative_branch = task_id in declared_negative_branch_sources
        requested_semantic_recovery = (
            search_mode == "semantic_recovery"
            or not_found_policy == "semantic_recovery"
            or on_exhaustion == "qwen_semantic_recovery"
        )
        if has_declared_negative_branch:
            _require(
                not requested_semantic_recovery,
                f"task {task_id} cannot combine autonomous recovery with an explicit branch",
            )
            # The conditional rule is the authoritative representation of the
            # user's explicit fallback. Normalize redundant model fields instead
            # of rejecting a valid verify-presence branch as report_absent.
            not_found_policy = "explicit_branch"
            on_exhaustion = "finish"
        expected_exhaustion = (
            "qwen_semantic_recovery" if not_found_policy == "semantic_recovery" else "finish"
        )
        if "on_exhaustion" in search_policy:
            _require(
                on_exhaustion == expected_exhaustion,
                f"task {task_id} intent.not_found_policy conflicts with search_policy.on_exhaustion",
            )
        else:
            on_exhaustion = expected_exhaustion
        _require(
            not (goal_type == "verify_presence" and not_found_policy == "semantic_recovery"),
            f"task {task_id} presence verification cannot start autonomous semantic recovery",
        )
        task["intent"] = {
            "goal_type": goal_type,
            "not_found_policy": not_found_policy,
        }
        task["search_policy"] = {
            "mode": search_mode,
            "completion_policy": "first_success" if search_mode == "semantic_recovery" else "fixed_target",
            "on_exhaustion": on_exhaustion,
            "maximum_location_hypotheses": max(1, min(5, int(search_policy.get("maximum_location_hypotheses", 3)))),
        }
        normalized_tasks.append(task)

    for task in normalized_tasks:
        for predecessor in task["prerequisites"]:
            _require(predecessor in seen, f"task {task['id']} references unknown prerequisite {predecessor}")
            _require(predecessor != task["id"], f"task {task['id']} depends on itself")
    _detect_cycle(normalized_tasks)

    rules = graph.get("conditional_rules") or []
    _require(isinstance(rules, list), "conditional_rules must be a list")
    normalized_rules = []
    activated_by_rule: set[str] = set()
    for index, source in enumerate(rules):
        _require(isinstance(source, dict), f"conditional rule {index} must be an object")
        trigger = str(source.get("source_task_id", ""))
        _require(trigger in seen, f"conditional rule references unknown source task {trigger}")
        activate = sorted(set(str(item) for item in source.get("activate_task_ids", [])))
        skip = sorted(set(str(item) for item in source.get("skip_task_ids", [])))
        _require(activate or skip, f"conditional rule {index} has no effect")
        for task_id in activate + skip:
            _require(task_id in seen, f"conditional rule references unknown task {task_id}")
            _require(task_id != trigger, f"conditional rule cannot target its source task {trigger}")
        outcome = str(source.get("if_outcome", "not_found")).strip().lower()
        _require(outcome in {"found", "not_found", "success", "failure", "true", "false"}, f"invalid conditional outcome {outcome}")
        activated_by_rule.update(activate)
        normalized_rules.append({
            "source_task_id": trigger,
            "if_outcome": outcome,
            "activate_task_ids": activate,
            "skip_task_ids": skip,
        })
    for task in normalized_tasks:
        task["active_initially"] = task["id"] not in activated_by_rule

    negative_branch_sources = {
        rule["source_task_id"] for rule in normalized_rules
        if rule["if_outcome"] == "not_found"
    }
    tasks_by_id = {task["id"]: task for task in normalized_tasks}
    for task_id in negative_branch_sources:
        task = tasks_by_id[task_id]
        # Legacy graphs without an explicit intent retain their historical
        # locate-target interpretation. Explicit Qwen/user intent may instead
        # be a presence check whose negative result activates another task.
        if task_id not in explicit_intent_tasks:
            task["intent"]["goal_type"] = "locate_target"
        task["intent"]["not_found_policy"] = "explicit_branch"
        task["search_policy"]["on_exhaustion"] = "finish"
        _require(
            task["intent"]["goal_type"] in {"locate_target", "verify_presence"},
            f"task {task_id} with a not_found branch must be observable",
        )
    for task in normalized_tasks:
        if task["intent"]["not_found_policy"] == "explicit_branch":
            _require(
                task["id"] in negative_branch_sources,
                f"task {task['id']} selects explicit_branch but has no not_found conditional rule",
            )
        if task["intent"]["goal_type"] == "verify_presence":
            task["acceptable_outcomes"] = ["found", "not_found"]
        else:
            task["acceptable_outcomes"] = [task["success_outcome"]]

    graph["tasks"] = normalized_tasks
    graph["conditional_rules"] = normalized_rules
    graph["summary"] = {
        "task_count": len(normalized_tasks),
        "precedence_edge_count": sum(len(task["prerequisites"]) for task in normalized_tasks),
        "conditional_rule_count": len(normalized_rules),
    }
    return graph
