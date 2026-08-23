"""Reproducible task suites, planner baselines, and paper-level metrics."""

from __future__ import annotations

import copy
import math
import time
from typing import Any, Callable

from .grid_map import GridPath, OccupancyGrid
from .joint_planner import (
    DEFAULT_TERMINAL_WEIGHT,
    DEFAULT_TIME_WEIGHT,
    PlanningError,
    _motion_time,
    plan_joint_mission,
)
from .mission_executor import MissionState
from .task_graph import normalize_and_validate_task_graph


Planner = Callable[
    [OccupancyGrid, dict[str, Any], dict[str, list[dict[str, Any]]], list[float], set[str], set[str]],
    dict[str, Any],
]


def _task(
    task_id: str,
    label: str,
    *,
    prerequisites: list[str] | None = None,
    relation: str | None = None,
    active: bool = True,
) -> dict[str, Any]:
    return {
        "id": task_id,
        "action": "inspect",
        "target": {"label": label, "room": None, "reference": None},
        "verification_label": label,
        "spatial_constraints": {
            "relation": relation,
            "distance_m": [0.8, 1.8],
            "height_m": None,
            "face_target": True,
            "visibility_required": True,
        },
        "prerequisites": prerequisites or [],
        "active_initially": active,
        "success_outcome": "found",
    }


def build_task_suites() -> dict[str, dict[str, Any]]:
    """Return controlled task graphs covering the main language constraints."""
    suites = {
        "independent": {
            "instruction": "检查灯、电视、柜子、门和椅子，顺序不限。",
            "tasks": [
                _task("inspect_lamp", "lamp"),
                _task("inspect_tv", "television"),
                _task("inspect_cabinet", "cabinet"),
                _task("inspect_door", "door"),
                _task("inspect_chair", "chair"),
            ],
            "conditional_rules": [],
        },
        "precedence": {
            "instruction": "先检查灯，再看电视，之后检查柜子；门和椅子顺序不限。",
            "tasks": [
                _task("inspect_lamp", "lamp"),
                _task("inspect_tv", "television", prerequisites=["inspect_lamp"]),
                _task("inspect_cabinet", "cabinet", prerequisites=["inspect_tv"]),
                _task("inspect_door", "door"),
                _task("inspect_chair", "chair"),
            ],
            "conditional_rules": [],
        },
        "conditional": {
            "instruction": "检查灯和电视；如果没有电视则检查柜子，门和椅子顺序不限。",
            "tasks": [
                _task("inspect_lamp", "lamp"),
                _task("inspect_tv", "television", prerequisites=["inspect_lamp"]),
                _task("inspect_cabinet", "cabinet", active=False),
                _task("inspect_door", "door"),
                _task("inspect_chair", "chair"),
            ],
            "conditional_rules": [{
                "source_task_id": "inspect_tv",
                "if_outcome": "not_found",
                "activate_task_ids": ["inspect_cabinet"],
                "skip_task_ids": [],
            }],
        },
        "spatial": {
            "instruction": "从正面观察灯、从左侧观察电视、从右侧观察柜子、从后方检查门。",
            "tasks": [
                _task("front_lamp", "lamp", relation="front"),
                _task("left_tv", "television", relation="left"),
                _task("right_cabinet", "cabinet", relation="right"),
                _task("behind_door", "door", relation="behind"),
            ],
            "conditional_rules": [],
        },
    }
    return {
        name: normalize_and_validate_task_graph({"format": "pre_map_vln.task_graph.v1", **graph})
        for name, graph in suites.items()
    }


def _active_graph(
    task_graph: dict[str, Any], active_task_ids: set[str], completed_task_ids: set[str]
) -> list[dict[str, Any]]:
    tasks = [
        task for task in task_graph["tasks"]
        if task["id"] in active_task_ids and task["id"] not in completed_task_ids
    ]
    missing = {
        task["id"]: [
            predecessor for predecessor in task["prerequisites"]
            if predecessor not in active_task_ids and predecessor not in completed_task_ids
        ]
        for task in tasks
    }
    blocked = {key: value for key, value in missing.items() if value}
    if blocked:
        raise PlanningError(f"active tasks have unsatisfied inactive prerequisites: {blocked}")
    return tasks


def _plan_in_language_order(
    grid: OccupancyGrid,
    task_graph: dict[str, Any],
    candidates_by_task: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
    active_task_ids: set[str],
    completed_task_ids: set[str],
    terminal_weight: float,
) -> dict[str, Any]:
    tasks = _active_graph(task_graph, active_task_ids, completed_task_ids)
    current = start_xyz_yaw[:2]
    locally_completed = set(completed_task_ids)
    pending = list(tasks)
    visits, segments = [], []
    total_length = total_terminal = 0.0
    while pending:
        task = next(
            (item for item in pending if set(item["prerequisites"]) <= locally_completed), None
        )
        if task is None:
            raise PlanningError("language-order planner cannot satisfy prerequisites")
        options = []
        for candidate in candidates_by_task.get(task["id"], []):
            pose = candidate["pose"]
            path = grid.astar(current, [pose["x"], pose["y"]])
            if path is None:
                continue
            score = path.length_m + terminal_weight * float(candidate["terminal_cost"])
            options.append((score, candidate["id"], candidate, path))
        if not options:
            raise PlanningError(f"no reachable candidate for {task['id']}")
        _, _, candidate, path = min(options, key=lambda item: (item[0], item[1]))
        visits.append({
            "sequence": len(visits), "task_id": task["id"], "candidate_id": candidate["id"],
            "object_id": candidate["object_id"], "pose": candidate["pose"],
            "terminal_cost": candidate["terminal_cost"],
        })
        segments.append({
            "from_xy_m": list(current), "to_task_id": task["id"],
            "length_m": path.length_m, "points_xy_m": path.points_xy_m,
        })
        total_length += path.length_m
        total_terminal += float(candidate["terminal_cost"])
        current = [candidate["pose"]["x"], candidate["pose"]["y"]]
        locally_completed.add(task["id"])
        pending.remove(task)
    return {
        "format": "pre_map_vln.mission_plan.v1", "planner": "fixed_order",
        "visits": visits, "segments": segments, "total_path_length_m": total_length,
        "total_terminal_cost": total_terminal,
        "objective_cost": total_length + terminal_weight * total_terminal,
        "estimated_time_s": total_length,
        "weights": {"path_length": 1.0, "terminal_quality": terminal_weight},
    }


class _EuclideanGrid:
    """Distance oracle used only to select an ablation route; execution stays A*."""

    def astar(self, start_xy, goal_xy):
        start, goal = list(start_xy), list(goal_xy)
        return GridPath([start, goal], math.dist(start, goal))


def _realize_with_astar(
    grid: OccupancyGrid, plan: dict[str, Any], start_xyz_yaw: list[float]
) -> dict[str, Any]:
    result = copy.deepcopy(plan)
    current = start_xyz_yaw[:2]
    current_pose = list(start_xyz_yaw)
    segments, total, total_time = [], 0.0, 0.0
    for visit in result["visits"]:
        goal = [visit["pose"]["x"], visit["pose"]["y"]]
        path = grid.astar(current, goal)
        if path is None:
            raise PlanningError(f"euclidean plan selected unreachable task {visit['task_id']}")
        segments.append({
            "from_xy_m": list(current), "to_task_id": visit["task_id"],
            "length_m": path.length_m, "points_xy_m": path.points_xy_m,
        })
        total += path.length_m
        total_time += _motion_time(current_pose, visit["pose"], path.length_m, 1.0, 0.5, 1.0)
        current = goal
        current_pose = [visit["pose"][key] for key in ("x", "y", "z", "yaw")]
    result["segments"] = segments
    result["proxy_path_length_m"] = result["total_path_length_m"]
    result["total_path_length_m"] = total
    result["estimated_time_s"] = total_time
    result["objective_cost"] = (
        total + DEFAULT_TIME_WEIGHT * total_time
        + result["weights"]["terminal_quality"] * result["total_terminal_cost"]
    )
    result["planner"] = "subset_dp_euclidean_selection_astar_execution"
    return result


def planner_registry() -> dict[str, tuple[str, Planner]]:
    """Return method name -> (group, standardized planner callback)."""
    def fixed_nearest(grid, graph, candidates, start, active, completed):
        return _plan_in_language_order(grid, graph, candidates, start, active, completed, 0.0)

    def fixed_viewpoint(grid, graph, candidates, start, active, completed):
        return _plan_in_language_order(grid, graph, candidates, start, active, completed, DEFAULT_TERMINAL_WEIGHT)

    def single_pose(grid, graph, candidates, start, active, completed):
        reduced = {
            task_id: [min(values, key=lambda value: (value["terminal_cost"], value["id"]))]
            for task_id, values in candidates.items()
        }
        return plan_joint_mission(grid, graph, reduced, start, active, completed)

    def euclidean(grid, graph, candidates, start, active, completed):
        proxy = plan_joint_mission(_EuclideanGrid(), graph, candidates, start, active, completed)
        return _realize_with_astar(grid, proxy, start)

    def no_terminal(grid, graph, candidates, start, active, completed):
        return plan_joint_mission(grid, graph, candidates, start, active, completed, terminal_weight=0.0)

    def full(grid, graph, candidates, start, active, completed):
        return plan_joint_mission(grid, graph, candidates, start, active, completed)

    return {
        "fixed_order_nearest": ("baseline", fixed_nearest),
        "fixed_order_viewpoint": ("baseline", fixed_viewpoint),
        "task_graph_single_pose": ("baseline", single_pose),
        "euclidean_cost": ("ablation", euclidean),
        "no_terminal_quality": ("ablation", no_terminal),
        "joint_astar": ("proposed", full),
    }


def evaluate_method(
    method_name: str,
    method_group: str,
    planner: Planner,
    grid: OccupancyGrid,
    task_graph: dict[str, Any],
    candidates: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
    outcomes: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Dynamically execute one planned task at a time and compute common metrics."""
    outcomes = outcomes or {}
    state = MissionState(task_graph)
    pose = list(start_xyz_yaw)
    visits, segments = [], []
    planning_ms = 0.0
    flight_time_s = 0.0
    yaw_rotation_rad = 0.0
    while not state.is_finished():
        if not state.active:
            break
        started = time.perf_counter()
        plan = planner(grid, task_graph, candidates, pose, state.active, state.completed)
        planning_ms += (time.perf_counter() - started) * 1000.0
        if not plan["visits"]:
            break
        visit, segment = plan["visits"][0], plan["segments"][0]
        visits.append(visit)
        segments.append(segment)
        flight_time_s += _motion_time(pose, visit["pose"], float(segment["length_m"]), 1.0, 0.5, 1.0)
        yaw_rotation_rad += abs(math.atan2(
            math.sin(float(visit["pose"]["yaw"]) - float(pose[3])),
            math.cos(float(visit["pose"]["yaw"]) - float(pose[3])),
        ))
        pose = [visit["pose"][key] for key in ("x", "y", "z", "yaw")]
        task = next(item for item in task_graph["tasks"] if item["id"] == visit["task_id"])
        state.finish_task(visit["task_id"], outcomes.get(visit["task_id"], task["success_outcome"]))

    sequence = {visit["task_id"]: index for index, visit in enumerate(visits)}
    checked_edges = satisfied_edges = 0
    for task in task_graph["tasks"]:
        if task["id"] not in sequence:
            continue
        for predecessor in task["prerequisites"]:
            if predecessor in sequence:
                checked_edges += 1
                satisfied_edges += int(sequence[predecessor] < sequence[task["id"]])
    completed = sum(value == "completed" for value in state.status.values())
    required = sum(value != "skipped" for value in state.status.values())
    terminal_costs = [float(visit.get("terminal_cost", 0.0)) for visit in visits]
    condition_targets = {
        target
        for rule in task_graph.get("conditional_rules", [])
        for target in rule["activate_task_ids"]
    }
    expected_condition_visits = {
        target
        for rule in task_graph.get("conditional_rules", [])
        if outcomes.get(rule["source_task_id"]) == rule["if_outcome"]
        for target in rule["activate_task_ids"]
    }
    visited_ids = set(sequence)
    condition_ok = (
        expected_condition_visits <= visited_ids
        and not ((condition_targets - expected_condition_visits) & visited_ids)
    )
    path_length = sum(float(segment["length_m"]) for segment in segments)
    return {
        "method": method_name,
        "method_group": method_group,
        "status": "completed" if state.is_finished() else "incomplete",
        "mission_success": state.is_finished(),
        "task_completion_rate": completed / max(1, required),
        "constraint_satisfaction_rate": 1.0 if checked_edges == 0 else satisfied_edges / checked_edges,
        "condition_satisfaction": condition_ok,
        "path_length_m": path_length,
        "estimated_time_s": flight_time_s,
        "yaw_rotation_rad": yaw_rotation_rad,
        "planning_time_ms": planning_ms,
        "replan_count": len(visits),
        "visit_count": len(visits),
        "mean_terminal_cost": sum(terminal_costs) / max(1, len(terminal_costs)),
        "viewpoint_quality": sum(1.0 / (1.0 + value) for value in terminal_costs) / max(1, len(terminal_costs)),
        "visibility_rate": sum(
            bool(next(candidate for candidate in candidates[visit["task_id"]] if candidate["id"] == visit["candidate_id"])["line_of_sight"])
            for visit in visits
        ) / max(1, len(visits)),
        "task_status": state.status,
        "visit_order": [visit["task_id"] for visit in visits],
    }
