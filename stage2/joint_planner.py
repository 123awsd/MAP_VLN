"""Subset-DP joint optimization of task order, terminal pose, and A* cost."""

from __future__ import annotations

import math
from typing import Any

from .grid_map import OccupancyGrid


class PlanningError(RuntimeError):
    pass


DEFAULT_TERMINAL_WEIGHT = 2.0
DEFAULT_TIME_WEIGHT = 0.25


def _xy(candidate: dict[str, Any]) -> list[float]:
    return [candidate["pose"]["x"], candidate["pose"]["y"]]


def _xyz(candidate: dict[str, Any]) -> list[float]:
    return [candidate["pose"]["x"], candidate["pose"]["y"], candidate["pose"]["z"]]


def _motion_path(planner, start_xyz_yaw, candidate):
    if hasattr(planner, "path"):
        return planner.path(start_xyz_yaw[:3], _xyz(candidate))
    return planner.astar(start_xyz_yaw[:2], _xy(candidate))


def _segment_payload(path, start_xyz_yaw, task_id):
    result = {"to_task_id": task_id, "length_m": path.length_m}
    if hasattr(path, "points_xyz_m"):
        result.update({
            "from_xyz_m": list(start_xyz_yaw[:3]),
            "points_xyz_m": path.points_xyz_m,
            "vertical_distance_m": path.vertical_distance_m,
            "minimum_clearance_m": path.minimum_clearance_m,
            "geometry_map_version": path.map_identity.geometry_map_version,
            "map_epoch_uuid": path.map_identity.map_epoch_uuid,
            "planner_profile_hash": path.map_identity.planner_profile_hash,
        })
    else:
        result.update({
            "from_xy_m": list(start_xyz_yaw[:2]),
            "points_xy_m": path.points_xy_m,
        })
    return result


def _motion_time(
    start_pose: list[float], end_pose: dict[str, float], horizontal_length_m: float,
    speed_mps: float, climb_speed_mps: float, yaw_rate_rps: float,
) -> float:
    yaw_delta = abs(math.atan2(
        math.sin(float(end_pose["yaw"]) - float(start_pose[3])),
        math.cos(float(end_pose["yaw"]) - float(start_pose[3])),
    ))
    horizontal_time = horizontal_length_m / max(0.05, speed_mps)
    vertical_time = abs(float(end_pose["z"]) - float(start_pose[2])) / max(0.05, climb_speed_mps)
    yaw_time = yaw_delta / max(0.05, yaw_rate_rps)
    return max(horizontal_time, vertical_time, yaw_time)


def plan_joint_mission(
    grid: Any,
    task_graph: dict[str, Any],
    candidates_by_task: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
    active_task_ids: set[str] | None = None,
    completed_task_ids: set[str] | None = None,
    terminal_weight: float = DEFAULT_TERMINAL_WEIGHT,
    time_weight: float = DEFAULT_TIME_WEIGHT,
    speed_mps: float = 1.0,
    climb_speed_mps: float = 0.5,
    yaw_rate_rps: float = 1.0,
) -> dict[str, Any]:
    completed_task_ids = completed_task_ids or set()
    if active_task_ids is None:
        active_task_ids = {task["id"] for task in task_graph["tasks"] if task["active_initially"]}
    tasks = [task for task in task_graph["tasks"] if task["id"] in active_task_ids and task["id"] not in completed_task_ids]
    if not tasks:
        return {"format": "pre_map_vln.mission_plan.v1", "visits": [], "segments": [], "total_path_length_m": 0.0, "total_terminal_cost": 0.0, "objective_cost": 0.0, "estimated_time_s": 0.0}
    index = {task["id"]: offset for offset, task in enumerate(tasks)}
    external_missing = {
        task["id"]: [pre for pre in task["prerequisites"] if pre not in index and pre not in completed_task_ids]
        for task in tasks
    }
    blocked = {task_id: missing for task_id, missing in external_missing.items() if missing}
    if blocked:
        raise PlanningError(f"active tasks have unsatisfied inactive prerequisites: {blocked}")
    for task in tasks:
        if not candidates_by_task.get(task["id"]):
            raise PlanningError(f"no feasible terminal pose for task {task['id']}")

    prerequisite_masks = []
    for task in tasks:
        mask = 0
        for predecessor in task["prerequisites"]:
            if predecessor in index:
                mask |= 1 << index[predecessor]
        prerequisite_masks.append(mask)

    states: dict[tuple[int, int, int], tuple[float, float, float, tuple[int, int, int] | None, Any]] = {}
    for task_index, task in enumerate(tasks):
        if prerequisite_masks[task_index]:
            continue
        for candidate_index, candidate in enumerate(candidates_by_task[task["id"]]):
            path = _motion_path(grid, start_xyz_yaw, candidate)
            if path is None:
                continue
            terminal = float(candidate["terminal_cost"])
            motion_time = _motion_time(start_xyz_yaw, candidate["pose"], path.length_m, speed_mps, climb_speed_mps, yaw_rate_rps)
            states[(1 << task_index, task_index, candidate_index)] = (
                path.length_m + time_weight * motion_time + terminal_weight * terminal,
                terminal,
                motion_time,
                None,
                path,
            )
    full_mask = (1 << len(tasks)) - 1
    for visited_count in range(1, len(tasks)):
        snapshot = list(states.items())
        for state, (cost, terminal_sum, time_sum, _, _) in snapshot:
            mask, last_task_index, last_candidate_index = state
            if bin(mask).count("1") != visited_count:
                continue
            last_candidate = candidates_by_task[tasks[last_task_index]["id"]][last_candidate_index]
            for next_task_index, next_task in enumerate(tasks):
                bit = 1 << next_task_index
                if mask & bit or prerequisite_masks[next_task_index] & ~mask:
                    continue
                for next_candidate_index, candidate in enumerate(candidates_by_task[next_task["id"]]):
                    last_pose = last_candidate["pose"]
                    start_pose = [last_pose["x"], last_pose["y"], last_pose["z"], last_pose["yaw"]]
                    path = _motion_path(grid, start_pose, candidate)
                    if path is None:
                        continue
                    terminal = float(candidate["terminal_cost"])
                    motion_time = _motion_time(start_pose, candidate["pose"], path.length_m, speed_mps, climb_speed_mps, yaw_rate_rps)
                    next_state = (mask | bit, next_task_index, next_candidate_index)
                    next_cost = cost + path.length_m + time_weight * motion_time + terminal_weight * terminal
                    old = states.get(next_state)
                    if old is None or next_cost < old[0]:
                        states[next_state] = (next_cost, terminal_sum + terminal, time_sum + motion_time, state, path)
    finals = [(state, value) for state, value in states.items() if state[0] == full_mask]
    if not finals:
        raise PlanningError("no collision-free route can cover all active tasks")
    final_state, final_value = min(finals, key=lambda item: item[1][0])

    chain = []
    state = final_state
    while state is not None:
        value = states[state]
        chain.append((state, value))
        state = value[3]
    chain.reverse()
    visits, segments = [], []
    previous_pose = list(start_xyz_yaw)
    path_length = 0.0
    for sequence, (state, value) in enumerate(chain):
        _, task_index, candidate_index = state
        task = tasks[task_index]
        candidate = candidates_by_task[task["id"]][candidate_index]
        path = value[4]
        path_length += path.length_m
        visits.append({
            "sequence": sequence,
            "task_id": task["id"],
            "candidate_id": candidate["id"],
            "object_id": candidate["object_id"],
            "pose": candidate["pose"],
            "terminal_cost": candidate["terminal_cost"],
        })
        segments.append(_segment_payload(path, previous_pose, task["id"]))
        pose = candidate["pose"]
        previous_pose = [pose["x"], pose["y"], pose["z"], pose["yaw"]]
    result = {
        "format": "pre_map_vln.mission_plan.v1",
        "planner": "subset_dp_astar_3d" if hasattr(grid, "path") else "subset_dp_astar",
        "visits": visits,
        "segments": segments,
        "total_path_length_m": path_length,
        "total_terminal_cost": final_value[1],
        "objective_cost": final_value[0],
        "estimated_time_s": final_value[2],
        "weights": {
            "path_length": 1.0, "flight_time": time_weight,
            "terminal_quality": terminal_weight,
        },
        "motion_limits": {
            "horizontal_speed_mps": speed_mps,
            "climb_speed_mps": climb_speed_mps,
            "yaw_rate_rps": yaw_rate_rps,
        },
    }
    if segments and "map_epoch_uuid" in segments[0]:
        result.update({
            "start_xyz_yaw": list(start_xyz_yaw),
            "map_epoch_uuid": segments[0]["map_epoch_uuid"],
            "geometry_map_version": segments[0]["geometry_map_version"],
            "planner_profile_hash": segments[0]["planner_profile_hash"],
        })
    return result


def plan_fixed_order_baseline(
    grid: Any,
    task_graph: dict[str, Any],
    candidates_by_task: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
) -> dict[str, Any]:
    current = list(start_xyz_yaw)
    visits, segments, total = [], [], 0.0
    completed: set[str] = set()
    pending = [task for task in task_graph["tasks"] if task["active_initially"]]
    while pending:
        eligible = next((task for task in pending if set(task["prerequisites"]) <= completed), None)
        if eligible is None:
            raise PlanningError("fixed-order baseline cannot satisfy prerequisites")
        options = []
        for candidate in candidates_by_task.get(eligible["id"], []):
            path = _motion_path(grid, current, candidate)
            if path is not None:
                options.append((path.length_m, candidate, path))
        if not options:
            raise PlanningError(f"baseline cannot reach task {eligible['id']}")
        _, candidate, path = min(options, key=lambda item: item[0])
        visits.append({"sequence": len(visits), "task_id": eligible["id"], "candidate_id": candidate["id"], "object_id": candidate["object_id"], "pose": candidate["pose"]})
        segments.append(_segment_payload(path, current, eligible["id"]))
        total += path.length_m
        pose = candidate["pose"]
        current = [pose["x"], pose["y"], pose["z"], pose["yaw"]]
        completed.add(eligible["id"])
        pending.remove(eligible)
    return {"format": "pre_map_vln.mission_plan.v1", "planner": "fixed_instruction_order", "visits": visits, "segments": segments, "total_path_length_m": total, "objective_cost": total, "estimated_time_s": total}


def plan_all_candidates_in_task_order(
    grid: Any,
    task_graph: dict[str, Any],
    candidates_by_task: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
) -> dict[str, Any]:
    """Exhaust every candidate of each task before moving to the next task."""
    completed: set[str] = set()
    pending = [task for task in task_graph["tasks"] if task["active_initially"]]
    ordered_tasks = []
    while pending:
        eligible = next(
            (task for task in pending if set(task["prerequisites"]) <= completed), None
        )
        if eligible is None:
            raise PlanningError("all-candidate replay cannot satisfy prerequisites")
        ordered_tasks.append(eligible)
        completed.add(eligible["id"])
        pending.remove(eligible)

    current = list(start_xyz_yaw)
    visits, segments = [], []
    total_length = 0.0
    total_terminal = 0.0
    total_time = 0.0
    for task in ordered_tasks:
        remaining = list(candidates_by_task.get(task["id"], []))
        if not remaining:
            raise PlanningError(f"no feasible terminal pose for task {task['id']}")
        visit_policy = str(task.get("candidate_visit_policy", "all")).strip().lower()
        if visit_policy not in {"all", "first"}:
            raise PlanningError(
                f"unsupported candidate_visit_policy for task {task['id']}: {visit_policy}"
            )
        visit_limit = len(remaining) if visit_policy == "all" else 1
        task_visit_count = 0
        while remaining and task_visit_count < visit_limit:
            options = []
            for candidate in remaining:
                path = _motion_path(grid, current, candidate)
                if path is not None:
                    options.append((
                        path.length_m, float(candidate["terminal_cost"]),
                        candidate["id"], candidate, path,
                    ))
            if not options:
                raise PlanningError(
                    f"cannot visit every candidate for task {task['id']}"
                )
            _, terminal, _, candidate, path = min(options, key=lambda item: item[:3])
            visits.append({
                "sequence": len(visits), "task_id": task["id"],
                "candidate_id": candidate["id"], "object_id": candidate["object_id"],
                "pose": candidate["pose"], "terminal_cost": terminal,
            })
            segments.append(_segment_payload(path, current, task["id"]))
            total_length += path.length_m
            total_terminal += terminal
            total_time += _motion_time(
                current, candidate["pose"], path.length_m, 1.0, 0.5, 1.0,
            )
            pose = candidate["pose"]
            current = [pose["x"], pose["y"], pose["z"], pose["yaw"]]
            remaining.remove(candidate)
            task_visit_count += 1

    result = {
        "format": "pre_map_vln.mission_plan.v1",
        "planner": "all_candidates_task_order_astar_3d",
        "visits": visits,
        "segments": segments,
        "total_path_length_m": total_length,
        "total_terminal_cost": total_terminal,
        "objective_cost": total_length + total_terminal,
        "estimated_time_s": total_time,
        "weights": {"path_length": 1.0, "flight_time": 0.0, "terminal_quality": 1.0},
        "motion_limits": {
            "horizontal_speed_mps": 1.0, "climb_speed_mps": 0.5,
            "yaw_rate_rps": 1.0,
        },
        "candidate_execution_policy": "task_order_with_per_task_visit_policy",
    }
    if segments and "map_epoch_uuid" in segments[0]:
        result.update({
            "start_xyz_yaw": list(start_xyz_yaw),
            "map_epoch_uuid": segments[0]["map_epoch_uuid"],
            "geometry_map_version": segments[0]["geometry_map_version"],
            "planner_profile_hash": segments[0]["planner_profile_hash"],
        })
    return result
