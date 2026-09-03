"""Lazy A* evaluation for one-representative-per-task mission planning.

The ordinary stage-two planner expands the full task/candidate state space and
asks the motion oracle for every transition encountered by subset DP.  This
module provides the paper-oriented alternative discussed for the second stage:
one fixed representative viewpoint is used for each currently active task,
and only edges on a tentatively optimal route are evaluated with A*.

The lazy loop is exact for the representative graph.  Unknown edges use the
Euclidean/kinematic lower bound; after the tentative route is evaluated, the
DP is repeated until every edge on that route is exact.  At that point the
returned route is certified against the lower bounds of all other routes.
Candidate recovery is intentionally separate: after a failed observation the
caller supplies the remaining viewpoints for that task and this class plans a
single local move to the next reachable one.
"""

from __future__ import annotations

import math
from typing import Any, Iterable

from .joint_planner import (
    DEFAULT_TERMINAL_WEIGHT,
    DEFAULT_TIME_WEIGHT,
    PlanningError,
    _motion_path,
    _motion_time,
    _segment_payload,
)


class LazyRepresentativePlanner:
    """Plan over fixed task representatives while evaluating motion lazily."""

    def __init__(
        self,
        grid: Any,
        *,
        terminal_weight: float = DEFAULT_TERMINAL_WEIGHT,
        time_weight: float = DEFAULT_TIME_WEIGHT,
        speed_mps: float = 1.0,
        climb_speed_mps: float = 0.5,
        yaw_rate_rps: float = 1.0,
    ) -> None:
        self.grid = grid
        self.terminal_weight = float(terminal_weight)
        self.time_weight = float(time_weight)
        self.speed_mps = float(speed_mps)
        self.climb_speed_mps = float(climb_speed_mps)
        self.yaw_rate_rps = float(yaw_rate_rps)
        # The key includes the source pose and destination candidate.  Keeping
        # yaw in the source key is necessary because it contributes to the
        # flight-time term even though the geometric A* path is yaw-invariant.
        self._exact_edges: dict[tuple[Any, ...], Any | None] = {}
        self._edge_evaluations = 0
        self._global_plan_calls = 0
        self._local_plan_calls = 0
        self._global_iterations = 0
        self._last_plan: dict[str, Any] | None = None

    @staticmethod
    def _pose_key(pose: Iterable[float]) -> tuple[str, tuple[float, ...]]:
        return (
            "pose",
            tuple(round(float(value), 7) for value in pose),
        )

    @classmethod
    def _edge_key(
        cls, start_pose: list[float], candidate: dict[str, Any]
    ) -> tuple[Any, ...]:
        goal = candidate["pose"]
        return (
            cls._pose_key(start_pose),
            "candidate",
            str(candidate["id"]),
            tuple(round(float(goal[key]), 7) for key in ("x", "y", "z", "yaw")),
        )

    @staticmethod
    def _goal_xyz(candidate: dict[str, Any]) -> list[float]:
        return [float(candidate["pose"][key]) for key in ("x", "y", "z")]

    @staticmethod
    def _candidate_pose(candidate: dict[str, Any]) -> list[float]:
        pose = candidate["pose"]
        return [float(pose[key]) for key in ("x", "y", "z", "yaw")]

    def _lower_motion_cost(
        self, start_pose: list[float], candidate: dict[str, Any]
    ) -> float:
        goal = self._goal_xyz(candidate)
        horizontal = math.dist(start_pose[:2], goal[:2])
        euclidean = math.dist(start_pose[:3], goal)
        # OccupancyGrid.astar is 2-D and reports horizontal length only.  The
        # 3-D oracle reports the length of its xyz polyline.
        distance_lower_bound = euclidean if hasattr(self.grid, "path") else horizontal
        vertical_time = abs(goal[2] - float(start_pose[2])) / max(
            0.05, self.climb_speed_mps
        )
        yaw_delta = abs(math.atan2(
            math.sin(float(candidate["pose"]["yaw"]) - float(start_pose[3])),
            math.cos(float(candidate["pose"]["yaw"]) - float(start_pose[3])),
        ))
        time_lower_bound = max(
            horizontal / max(0.05, self.speed_mps),
            vertical_time,
            yaw_delta / max(0.05, self.yaw_rate_rps),
        )
        return distance_lower_bound + self.time_weight * time_lower_bound

    def _exact_motion_cost(
        self,
        start_pose: list[float],
        candidate: dict[str, Any],
        path: Any,
    ) -> float:
        return float(path.length_m) + self.time_weight * _motion_time(
            start_pose,
            candidate["pose"],
            float(path.length_m),
            self.speed_mps,
            self.climb_speed_mps,
            self.yaw_rate_rps,
        )

    def _evaluate_edge(
        self, start_pose: list[float], candidate: dict[str, Any]
    ) -> Any | None:
        key = self._edge_key(start_pose, candidate)
        if key in self._exact_edges:
            return self._exact_edges[key]
        self._edge_evaluations += 1
        path = _motion_path(self.grid, start_pose, candidate)
        self._exact_edges[key] = path
        return path

    def _edge_cost(
        self,
        start_pose: list[float],
        candidate: dict[str, Any],
        *,
        evaluate_unknown: bool,
    ) -> tuple[float, Any | None, bool]:
        key = self._edge_key(start_pose, candidate)
        if key in self._exact_edges:
            path = self._exact_edges[key]
            if path is None:
                return math.inf, None, True
            return self._exact_motion_cost(start_pose, candidate, path), path, True
        if evaluate_unknown:
            path = self._evaluate_edge(start_pose, candidate)
            if path is None:
                return math.inf, None, True
            return self._exact_motion_cost(start_pose, candidate, path), path, True
        return self._lower_motion_cost(start_pose, candidate), None, False

    @staticmethod
    def _task_by_id(task_graph: dict[str, Any]) -> dict[str, dict[str, Any]]:
        return {str(task["id"]): task for task in task_graph["tasks"]}

    def _representatives(
        self,
        candidates_by_task: dict[str, list[dict[str, Any]]],
        task_ids: set[str],
    ) -> dict[str, dict[str, Any]]:
        representatives: dict[str, dict[str, Any]] = {}
        for task_id in sorted(task_ids):
            values = candidates_by_task.get(task_id, [])
            if not values:
                raise PlanningError(f"no feasible terminal pose for task {task_id}")
            # Candidate generation sorts by terminal quality and retains the
            # complementary viewpoints in recovery order.  The first one is
            # deliberately frozen as the representative for this plan.
            representatives[task_id] = dict(values[0])
            representatives[task_id].setdefault("task_id", task_id)
        return representatives

    def _build_dp(
        self,
        tasks: list[dict[str, Any]],
        representatives: dict[str, dict[str, Any]],
        start_xyz_yaw: list[float],
        completed_task_ids: set[str] | None = None,
    ) -> tuple[list[int], float]:
        """Return the lowest lower-bound route and its lower-bound cost."""
        completed_task_ids = completed_task_ids or set()
        index = {task["id"]: offset for offset, task in enumerate(tasks)}
        external_missing = {
            task["id"]: [
                predecessor for predecessor in task.get("prerequisites", [])
                if predecessor not in index and predecessor not in completed_task_ids
            ]
            for task in tasks
        }
        blocked = {
            task_id: missing for task_id, missing in external_missing.items() if missing
        }
        if blocked:
            raise PlanningError(
                f"active tasks have unsatisfied inactive prerequisites: {blocked}"
            )

        prerequisite_masks = []
        for task in tasks:
            mask = 0
            for predecessor in task.get("prerequisites", []):
                if predecessor in index:
                    mask |= 1 << index[predecessor]
            prerequisite_masks.append(mask)

        # state -> (lower-bound objective, previous state)
        states: dict[tuple[int, int], tuple[float, tuple[int, int] | None]] = {}
        for task_index, task in enumerate(tasks):
            if prerequisite_masks[task_index]:
                continue
            candidate = representatives[task["id"]]
            edge_cost, _, _ = self._edge_cost(
                start_xyz_yaw, candidate, evaluate_unknown=False
            )
            cost = edge_cost + self.terminal_weight * float(
                candidate.get("terminal_cost", 0.0)
            )
            if math.isfinite(cost):
                states[(1 << task_index, task_index)] = (cost, None)

        for visited_count in range(1, len(tasks)):
            snapshot = list(states.items())
            for state, (cost, _) in snapshot:
                mask, last_index = state
                if bin(mask).count("1") != visited_count:
                    continue
                last_candidate = representatives[tasks[last_index]["id"]]
                last_pose = self._candidate_pose(last_candidate)
                for next_index, task in enumerate(tasks):
                    bit = 1 << next_index
                    if mask & bit or prerequisite_masks[next_index] & ~mask:
                        continue
                    candidate = representatives[task["id"]]
                    edge_cost, _, _ = self._edge_cost(
                        last_pose, candidate, evaluate_unknown=False
                    )
                    next_cost = cost + edge_cost + self.terminal_weight * float(
                        candidate.get("terminal_cost", 0.0)
                    )
                    if not math.isfinite(next_cost):
                        continue
                    next_state = (mask | bit, next_index)
                    previous = states.get(next_state)
                    if previous is None or next_cost < previous[0] - 1e-9:
                        states[next_state] = (next_cost, state)

        full_mask = (1 << len(tasks)) - 1
        finals = [
            (state, value) for state, value in states.items()
            if state[0] == full_mask
        ]
        if not finals:
            raise PlanningError("no collision-free route can cover all active task representatives")
        final_state, final_value = min(
            finals, key=lambda item: (item[1][0], item[0][1])
        )
        route_indices: list[int] = []
        state: tuple[int, int] | None = final_state
        while state is not None:
            route_indices.append(state[1])
            state = states[state][1]
        route_indices.reverse()
        return route_indices, final_value[0]

    def _make_plan(
        self,
        start_xyz_yaw: list[float],
        visits_data: list[tuple[dict[str, Any], Any]],
        *,
        planner_name: str,
        lazy_metadata: dict[str, Any],
    ) -> dict[str, Any]:
        visits: list[dict[str, Any]] = []
        segments: list[dict[str, Any]] = []
        previous_pose = list(start_xyz_yaw)
        path_length = 0.0
        terminal_sum = 0.0
        time_sum = 0.0
        for sequence, (candidate, path) in enumerate(visits_data):
            task_id = str(candidate["task_id"])
            path_length += float(path.length_m)
            terminal_sum += float(candidate.get("terminal_cost", 0.0))
            time_sum += _motion_time(
                previous_pose,
                candidate["pose"],
                float(path.length_m),
                self.speed_mps,
                self.climb_speed_mps,
                self.yaw_rate_rps,
            )
            visits.append({
                "sequence": sequence,
                "task_id": task_id,
                "candidate_id": candidate["id"],
                "object_id": candidate.get("object_id", candidate["id"]),
                "pose": candidate["pose"],
                "terminal_cost": float(candidate.get("terminal_cost", 0.0)),
            })
            segments.append(_segment_payload(path, previous_pose, task_id))
            previous_pose = self._candidate_pose(candidate)

        result = {
            "format": "pre_map_vln.mission_plan.v1",
            "planner": planner_name,
            "visits": visits,
            "segments": segments,
            "total_path_length_m": path_length,
            "total_terminal_cost": terminal_sum,
            "objective_cost": path_length + self.time_weight * time_sum + self.terminal_weight * terminal_sum,
            "estimated_time_s": time_sum,
            "weights": {
                "path_length": 1.0,
                "flight_time": self.time_weight,
                "terminal_quality": self.terminal_weight,
            },
            "motion_limits": {
                "horizontal_speed_mps": self.speed_mps,
                "climb_speed_mps": self.climb_speed_mps,
                "yaw_rate_rps": self.yaw_rate_rps,
            },
            "lazy": lazy_metadata,
        }
        if segments and "map_epoch_uuid" in segments[0]:
            result.update({
                "start_xyz_yaw": list(start_xyz_yaw),
                "map_epoch_uuid": segments[0]["map_epoch_uuid"],
                "geometry_map_version": segments[0]["geometry_map_version"],
                "planner_profile_hash": segments[0]["planner_profile_hash"],
            })
        return result

    def plan_global(
        self,
        task_graph: dict[str, Any],
        candidates_by_task: dict[str, list[dict[str, Any]]],
        start_xyz_yaw: list[float],
        *,
        active_task_ids: set[str] | None = None,
        completed_task_ids: set[str] | None = None,
    ) -> dict[str, Any]:
        """Plan all currently available tasks with one representative each."""
        completed = completed_task_ids or set()
        if active_task_ids is None:
            task_ids = {
                task["id"] for task in task_graph["tasks"]
                if task.get("active_initially", True)
            }
        else:
            task_ids = set(active_task_ids)
        task_ids -= set(completed)
        tasks_by_id = self._task_by_id(task_graph)
        tasks = [
            tasks_by_id[task["id"]] for task in task_graph["tasks"]
            if task["id"] in task_ids
        ]
        self._global_plan_calls += 1
        motion_misses_before = getattr(self.grid, "cache_misses", None)
        if not tasks:
            metadata = {
                "mode": "global_representatives",
                "representative_candidate_ids": {},
                "candidate_pool_sizes": {},
                "iterations": 0,
                "edge_evaluations": 0,
                "astar_cache_misses": None,
                "unique_exact_edges": len(self._exact_edges),
                "certified": True,
            }
            return self._make_plan(
                start_xyz_yaw, [],
                planner_name="lazy_representative_subset_dp_astar_3d" if hasattr(self.grid, "path") else "lazy_representative_subset_dp_astar",
                lazy_metadata=metadata,
            )

        representatives = self._representatives(candidates_by_task, task_ids)
        before_evaluations = self._edge_evaluations
        iterations = 0
        while True:
            iterations += 1
            self._global_iterations += 1
            route_indices, _ = self._build_dp(
                tasks, representatives, start_xyz_yaw, completed_task_ids=completed
            )
            route_candidates = [representatives[tasks[index]["id"]] for index in route_indices]
            previous_pose = list(start_xyz_yaw)
            unknown_edges = []
            for candidate in route_candidates:
                key = self._edge_key(previous_pose, candidate)
                if key not in self._exact_edges:
                    unknown_edges.append((list(previous_pose), candidate))
                previous_pose = self._candidate_pose(candidate)
            if not unknown_edges:
                break
            # Evaluating every unknown edge on the current route usually gives
            # the 4-edge first pass discussed in the paper and avoids repeated
            # one-edge DP iterations. Failed edges are retained as exact
            # negative cache entries, so the next loop chooses an alternative.
            for edge_start, candidate in unknown_edges:
                self._evaluate_edge(edge_start, candidate)

        visits_data = []
        previous_pose = list(start_xyz_yaw)
        for index in route_indices:
            candidate = representatives[tasks[index]["id"]]
            _, path, exact = self._edge_cost(
                previous_pose, candidate, evaluate_unknown=True
            )
            if not exact or path is None:
                raise PlanningError("lazy route contains an unevaluated or unreachable edge")
            visits_data.append((candidate, path))
            previous_pose = self._candidate_pose(candidate)
        metadata = {
            "mode": "global_representatives",
            "representative_candidate_ids": {
                task_id: candidate["id"]
                for task_id, candidate in representatives.items()
            },
            "candidate_pool_sizes": {
                task_id: len(candidates_by_task.get(task_id, []))
                for task_id in sorted(task_ids)
            },
            "iterations": iterations,
            "edge_evaluations": self._edge_evaluations - before_evaluations,
            "astar_cache_misses": (
                None if motion_misses_before is None
                else int(getattr(self.grid, "cache_misses", 0)) - int(motion_misses_before)
            ),
            "unique_exact_edges": len(self._exact_edges),
            "certified": True,
        }
        result = self._make_plan(
            start_xyz_yaw,
            visits_data,
            planner_name="lazy_representative_subset_dp_astar_3d" if hasattr(self.grid, "path") else "lazy_representative_subset_dp_astar",
            lazy_metadata=metadata,
        )
        self._last_plan = result
        return result

    def plan_local_recovery(
        self,
        task_id: str,
        candidates: list[dict[str, Any]],
        start_xyz_yaw: list[float],
    ) -> dict[str, Any]:
        """Move to the first reachable remaining viewpoint in recovery order."""
        self._local_plan_calls += 1
        if not candidates:
            raise PlanningError(f"no remaining recovery viewpoint for task {task_id}")
        before_evaluations = self._edge_evaluations
        motion_misses_before = getattr(self.grid, "cache_misses", None)
        selected = None
        selected_path = None
        for candidate in candidates:
            candidate = dict(candidate)
            candidate["task_id"] = task_id
            _, path, exact = self._edge_cost(
                start_xyz_yaw, candidate, evaluate_unknown=True
            )
            if exact and path is not None:
                selected, selected_path = candidate, path
                break
        if selected is None or selected_path is None:
            raise PlanningError(f"no reachable recovery viewpoint for task {task_id}")
        result = self._make_plan(
            start_xyz_yaw,
            [(selected, selected_path)],
            planner_name="lazy_representative_local_recovery_3d" if hasattr(self.grid, "path") else "lazy_representative_local_recovery",
            lazy_metadata={
                "mode": "local_viewpoint_recovery",
                "task_id": task_id,
                "candidate_pool_size": len(candidates),
                "selected_candidate_id": selected["id"],
                "edge_evaluations": self._edge_evaluations - before_evaluations,
                "astar_cache_misses": (
                    None if motion_misses_before is None
                    else int(getattr(self.grid, "cache_misses", 0)) - int(motion_misses_before)
                ),
                "unique_exact_edges": len(self._exact_edges),
                "certified": True,
            },
        )
        self._last_plan = result
        return result

    def statistics(self) -> dict[str, Any]:
        """Return cumulative planner-level measurements for the execution trace."""
        return {
            "global_plan_calls": self._global_plan_calls,
            "local_plan_calls": self._local_plan_calls,
            "global_dp_iterations": self._global_iterations,
            "edge_evaluations": self._edge_evaluations,
            "unique_exact_edges": len(self._exact_edges),
            "last_plan": None if self._last_plan is None else self._last_plan.get("lazy"),
        }
