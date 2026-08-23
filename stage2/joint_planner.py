"""Subset-DP joint optimization of task order, terminal pose, and A* cost."""

from __future__ import annotations

import math
from typing import Any

from .grid_map import OccupancyGrid


class PlanningError(RuntimeError):
    pass


def _xy(candidate: dict[str, Any]) -> list[float]:
    return [candidate["pose"]["x"], candidate["pose"]["y"]]


def plan_joint_mission(
    grid: OccupancyGrid,
    task_graph: dict[str, Any],
    candidates_by_task: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
    active_task_ids: set[str] | None = None,
    completed_task_ids: set[str] | None = None,
    terminal_weight: float = 0.35,
    speed_mps: float = 1.0,
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

    start_xy = start_xyz_yaw[:2]
    states: dict[tuple[int, int, int], tuple[float, float, tuple[int, int, int] | None, Any]] = {}
    for task_index, task in enumerate(tasks):
        if prerequisite_masks[task_index]:
            continue
        for candidate_index, candidate in enumerate(candidates_by_task[task["id"]]):
            path = grid.astar(start_xy, _xy(candidate))
            if path is None:
                continue
            terminal = float(candidate["terminal_cost"])
            states[(1 << task_index, task_index, candidate_index)] = (
                path.length_m + terminal_weight * terminal,
                terminal,
                None,
                path,
            )
    full_mask = (1 << len(tasks)) - 1
    for visited_count in range(1, len(tasks)):
        snapshot = list(states.items())
        for state, (cost, terminal_sum, _, _) in snapshot:
            mask, last_task_index, last_candidate_index = state
            if bin(mask).count("1") != visited_count:
                continue
            last_candidate = candidates_by_task[tasks[last_task_index]["id"]][last_candidate_index]
            for next_task_index, next_task in enumerate(tasks):
                bit = 1 << next_task_index
                if mask & bit or prerequisite_masks[next_task_index] & ~mask:
                    continue
                for next_candidate_index, candidate in enumerate(candidates_by_task[next_task["id"]]):
                    path = grid.astar(_xy(last_candidate), _xy(candidate))
                    if path is None:
                        continue
                    terminal = float(candidate["terminal_cost"])
                    next_state = (mask | bit, next_task_index, next_candidate_index)
                    next_cost = cost + path.length_m + terminal_weight * terminal
                    old = states.get(next_state)
                    if old is None or next_cost < old[0]:
                        states[next_state] = (next_cost, terminal_sum + terminal, state, path)
    finals = [(state, value) for state, value in states.items() if state[0] == full_mask]
    if not finals:
        raise PlanningError("no collision-free route can cover all active tasks")
    final_state, final_value = min(finals, key=lambda item: item[1][0])

    chain = []
    state = final_state
    while state is not None:
        value = states[state]
        chain.append((state, value))
        state = value[2]
    chain.reverse()
    visits, segments = [], []
    previous_xy = start_xy
    path_length = 0.0
    for sequence, (state, value) in enumerate(chain):
        _, task_index, candidate_index = state
        task = tasks[task_index]
        candidate = candidates_by_task[task["id"]][candidate_index]
        path = value[3]
        path_length += path.length_m
        visits.append({
            "sequence": sequence,
            "task_id": task["id"],
            "candidate_id": candidate["id"],
            "object_id": candidate["object_id"],
            "pose": candidate["pose"],
            "terminal_cost": candidate["terminal_cost"],
        })
        segments.append({
            "from_xy_m": list(previous_xy),
            "to_task_id": task["id"],
            "length_m": path.length_m,
            "points_xy_m": path.points_xy_m,
        })
        previous_xy = _xy(candidate)
    return {
        "format": "pre_map_vln.mission_plan.v1",
        "planner": "subset_dp_astar",
        "visits": visits,
        "segments": segments,
        "total_path_length_m": path_length,
        "total_terminal_cost": final_value[1],
        "objective_cost": final_value[0],
        "estimated_time_s": path_length / max(0.05, speed_mps),
        "weights": {"path_length": 1.0, "terminal_quality": terminal_weight},
    }


def plan_fixed_order_baseline(
    grid: OccupancyGrid,
    task_graph: dict[str, Any],
    candidates_by_task: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
) -> dict[str, Any]:
    current = start_xyz_yaw[:2]
    visits, segments, total = [], [], 0.0
    completed: set[str] = set()
    pending = [task for task in task_graph["tasks"] if task["active_initially"]]
    while pending:
        eligible = next((task for task in pending if set(task["prerequisites"]) <= completed), None)
        if eligible is None:
            raise PlanningError("fixed-order baseline cannot satisfy prerequisites")
        options = []
        for candidate in candidates_by_task.get(eligible["id"], []):
            path = grid.astar(current, _xy(candidate))
            if path is not None:
                options.append((path.length_m, candidate, path))
        if not options:
            raise PlanningError(f"baseline cannot reach task {eligible['id']}")
        _, candidate, path = min(options, key=lambda item: item[0])
        visits.append({"sequence": len(visits), "task_id": eligible["id"], "candidate_id": candidate["id"], "object_id": candidate["object_id"], "pose": candidate["pose"]})
        segments.append({"from_xy_m": list(current), "to_task_id": eligible["id"], "length_m": path.length_m, "points_xy_m": path.points_xy_m})
        total += path.length_m
        current = _xy(candidate)
        completed.add(eligible["id"])
        pending.remove(eligible)
    return {"format": "pre_map_vln.mission_plan.v1", "planner": "fixed_instruction_order", "visits": visits, "segments": segments, "total_path_length_m": total, "objective_cost": total, "estimated_time_s": total}
