"""Mission state machine with conditional activation and online replanning."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from .grid_map import OccupancyGrid
from .joint_planner import plan_joint_mission


TERMINAL_STATES = {"completed", "skipped", "failed"}


@dataclass
class MissionState:
    task_graph: dict[str, Any]
    status: dict[str, str] = field(init=False)
    outcomes: dict[str, str] = field(default_factory=dict)
    events: list[dict[str, Any]] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.status = {
            task["id"]: "pending" if task["active_initially"] else "inactive"
            for task in self.task_graph["tasks"]
        }

    @property
    def completed(self) -> set[str]:
        return {task_id for task_id, value in self.status.items() if value == "completed"}

    @property
    def active(self) -> set[str]:
        return {task_id for task_id, value in self.status.items() if value == "pending"}

    def available(self) -> set[str]:
        prerequisites = {task["id"]: set(task["prerequisites"]) for task in self.task_graph["tasks"]}
        return {task_id for task_id in self.active if prerequisites[task_id] <= self.completed}

    def finish_task(self, task_id: str, outcome: str) -> None:
        if self.status.get(task_id) != "pending":
            raise ValueError(f"task {task_id} is not pending")
        outcome = outcome.lower().strip()
        self.status[task_id] = "completed"
        self.outcomes[task_id] = outcome
        changes = []
        for rule in self.task_graph.get("conditional_rules", []):
            if rule["source_task_id"] != task_id:
                continue
            matched = rule["if_outcome"] == outcome
            for target in rule["activate_task_ids"]:
                if matched and self.status[target] == "inactive":
                    self.status[target] = "pending"
                    changes.append({"task_id": target, "status": "pending", "reason": "condition_matched"})
                elif not matched and self.status[target] == "inactive":
                    self.status[target] = "skipped"
                    changes.append({"task_id": target, "status": "skipped", "reason": "condition_not_matched"})
            if matched:
                for target in rule["skip_task_ids"]:
                    if self.status[target] not in TERMINAL_STATES:
                        self.status[target] = "skipped"
                        changes.append({"task_id": target, "status": "skipped", "reason": "condition_matched"})
        self.events.append({"event": "task_finished", "task_id": task_id, "outcome": outcome, "state_changes": changes})

    def is_finished(self) -> bool:
        return all(value in TERMINAL_STATES for value in self.status.values())

    def unresolved_task_ids(self) -> list[str]:
        """Return terminal tasks whose requested success was never established."""
        tasks = {task["id"]: task for task in self.task_graph["tasks"]}
        resolved_negative_sources = set()
        for rule in self.task_graph.get("conditional_rules", []):
            source = rule["source_task_id"]
            if self.outcomes.get(source) != rule["if_outcome"]:
                continue
            if all(
                self.outcomes.get(target) == tasks[target]["success_outcome"]
                for target in rule["activate_task_ids"]
            ):
                resolved_negative_sources.add(source)
        unresolved = []
        for task_id, status in self.status.items():
            if status == "skipped":
                continue
            if status != "completed":
                unresolved.append(task_id)
                continue
            acceptable = set(tasks[task_id].get(
                "acceptable_outcomes", [tasks[task_id]["success_outcome"]]
            ))
            if self.outcomes.get(task_id) not in acceptable and task_id not in resolved_negative_sources:
                unresolved.append(task_id)
        return sorted(unresolved)

    def result_status(self) -> str:
        if not self.is_finished():
            return "incomplete"
        return "complete" if not self.unresolved_task_ids() else "completed_with_unresolved_tasks"


def simulate_dynamic_execution(
    grid: OccupancyGrid,
    task_graph: dict[str, Any],
    candidates_by_task: dict[str, list[dict[str, Any]]],
    start_xyz_yaw: list[float],
    outcomes: dict[str, str] | None = None,
    outcome_provider: Callable[[str], str] | None = None,
) -> dict[str, Any]:
    """Execute one optimized visit at a time and replan after every observation."""
    outcomes = outcomes or {}
    state = MissionState(task_graph)
    pose = list(start_xyz_yaw)
    replans, executed_visits, executed_segments = [], [], []
    while not state.is_finished():
        active = state.active
        if not active:
            break
        plan = plan_joint_mission(
            grid,
            task_graph,
            candidates_by_task,
            pose,
            active_task_ids=active,
            completed_task_ids=state.completed,
        )
        replans.append({"index": len(replans), "active_task_ids": sorted(active), "plan": plan})
        if not plan["visits"]:
            break
        visit = plan["visits"][0]
        segment = plan["segments"][0]
        task_id = visit["task_id"]
        executed_visits.append(visit)
        executed_segments.append(segment)
        pose = [visit["pose"][key] for key in ("x", "y", "z", "yaw")]
        if outcome_provider is not None:
            outcome = outcome_provider(task_id)
        else:
            task = next(item for item in task_graph["tasks"] if item["id"] == task_id)
            outcome = outcomes.get(task_id, task["success_outcome"])
        state.finish_task(task_id, outcome)
    return {
        "format": "pre_map_vln.execution_trace.v1",
        "status": state.result_status(),
        "unresolved_task_ids": state.unresolved_task_ids(),
        "task_status": state.status,
        "outcomes": state.outcomes,
        "events": state.events,
        "replans": replans,
        "executed_visits": executed_visits,
        "executed_segments": executed_segments,
        "total_path_length_m": sum(segment["length_m"] for segment in executed_segments),
        "final_pose": pose,
    }
