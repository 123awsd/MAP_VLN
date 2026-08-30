"""Dynamic per-location entry viewpoints for bounded global joint planning."""

from __future__ import annotations

import math
from typing import Any


def location_id(candidate: dict[str, Any]) -> str:
    return str(
        candidate.get("location_hypothesis_id")
        or candidate.get("object_id")
        or "default"
    )


def representative_score(
    candidate: dict[str, Any], current_xyz_yaw: list[float], yaw_rate_rps: float = 1.0,
) -> float:
    """Cheap lower-bound analogue of the joint planner's first-edge objective."""
    pose = candidate["pose"]
    goal = [float(pose[key]) for key in ("x", "y", "z")]
    euclidean = math.dist(current_xyz_yaw[:3], goal)
    horizontal = math.dist(current_xyz_yaw[:2], goal[:2])
    vertical_time = abs(goal[2] - float(current_xyz_yaw[2])) / 0.5
    yaw_delta = abs(math.atan2(
        math.sin(float(pose["yaw"]) - float(current_xyz_yaw[3])),
        math.cos(float(pose["yaw"]) - float(current_xyz_yaw[3])),
    ))
    estimated_time = max(
        horizontal,
        vertical_time,
        yaw_delta / max(0.05, float(yaw_rate_rps)),
    )
    return (
        euclidean
        + 0.25 * estimated_time
        + 2.0 * float(candidate.get("terminal_cost", 0.0))
    )


def select_location_representatives(
    candidates_by_task: dict[str, list[dict[str, Any]]],
    current_xyz_yaw: list[float],
    active_task_ids: set[str],
    maximum_per_location: int = 1,
    yaw_rate_rps: float = 1.0,
) -> dict[str, list[dict[str, Any]]]:
    """Keep a small ranked entry set per physical location hypothesis."""
    limit = max(1, int(maximum_per_location))
    result: dict[str, list[dict[str, Any]]] = {}
    for task_id, values in candidates_by_task.items():
        if task_id not in active_task_ids:
            result[task_id] = list(values)
            continue
        groups: dict[str, list[dict[str, Any]]] = {}
        for candidate in values:
            groups.setdefault(location_id(candidate), []).append(candidate)
        selected = []
        for _, group in sorted(groups.items()):
            ranked = sorted(
                group,
                key=lambda item: (
                    representative_score(item, current_xyz_yaw, yaw_rate_rps),
                    item["id"],
                ),
            )
            selected.extend(ranked[:limit])
        result[task_id] = sorted(
            selected,
            key=lambda item: (
                representative_score(item, current_xyz_yaw, yaw_rate_rps),
                location_id(item),
                item["id"],
            ),
        )
    return result

