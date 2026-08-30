"""Bounded alternative-viewpoint recovery for failed visual verification."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ViewpointRecovery:
    maximum_attempts: int | None = None
    maximum_attempts_per_location: int | None = None
    maximum_locations: int | None = None
    attempted: dict[str, set[str]] = field(default_factory=dict)
    exhausted_locations: dict[str, set[str]] = field(default_factory=dict)
    active_locations: dict[str, str] = field(default_factory=dict)
    directions: dict[str, int] = field(default_factory=dict)
    last_angles: dict[str, float] = field(default_factory=dict)

    @staticmethod
    def _location(candidate: dict[str, Any]) -> str:
        return str(
            candidate.get("location_hypothesis_id")
            or candidate.get("object_id")
            or "default"
        )

    @staticmethod
    def _angle(candidate: dict[str, Any]) -> float | None:
        value = candidate.get("candidate_angle_rad")
        return None if value is None else float(value) % (2.0 * math.pi)

    @staticmethod
    def _forward_delta(start: float, end: float, direction: int) -> float:
        delta = (end - start) % (2.0 * math.pi)
        return delta if direction > 0 else (-delta) % (2.0 * math.pi)

    def _choose_direction(
        self, current_angle: float, candidates: list[dict[str, Any]],
    ) -> int | None:
        scored = []
        for direction in (1, -1):
            options = []
            for candidate in candidates:
                angle = self._angle(candidate)
                if angle is None:
                    continue
                delta = self._forward_delta(current_angle, angle, direction)
                if delta <= 1e-6:
                    continue
                options.append((delta, float(candidate.get("terminal_cost", 0.0)), candidate["id"]))
            if options:
                scored.append((min(options), direction))
        if not scored:
            return None
        return min(scored, key=lambda item: (item[0], -item[1]))[1]

    def decide(
        self,
        task_id: str,
        candidate_id: str,
        found: bool,
        candidates: list[dict[str, Any]],
    ) -> dict[str, Any]:
        tried = self.attempted.setdefault(task_id, set())
        tried.add(candidate_id)
        current = next((item for item in candidates if item["id"] == candidate_id), {})
        location_id = self._location(current)
        current_angle = self._angle(current)
        attempted_at_location = [
            item["id"] for item in candidates
            if item["id"] in tried
            and self._location(item) == location_id
        ]
        remaining = [item["id"] for item in candidates if item["id"] not in tried]
        same_location = [
            item["id"] for item in candidates if item["id"] not in tried
            and self._location(item) == location_id
        ]
        location_budget = self.maximum_attempts_per_location
        already_exhausted = self.exhausted_locations.get(task_id, set())
        visited_locations = {
            self._location(item) for item in candidates if item["id"] in tried
        }
        alternative_locations = [
            item["id"] for item in candidates
            if item["id"] in remaining
            and item["id"] not in same_location
            and self._location(item) not in already_exhausted
            and (
                self.maximum_locations is None
                or self._location(item) in visited_locations
                or len(visited_locations) < self.maximum_locations
            )
        ]
        retry_same = bool(same_location) and (
            location_budget is None or len(attempted_at_location) < location_budget
        )
        if not found and retry_same:
            self.active_locations[task_id] = location_id
            if current_angle is not None:
                self.last_angles[task_id] = current_angle
                if task_id not in self.directions:
                    remaining_candidates = [
                        item for item in candidates
                        if item["id"] in same_location
                    ]
                    direction = self._choose_direction(current_angle, remaining_candidates)
                    if direction is not None:
                        self.directions[task_id] = direction
        # The ordinary path exhausts the finite candidate pool.  An explicit
        # positive global cap remains available for compatibility experiments;
        # location-budgeted semantic recovery is governed independently.
        within_budget = (
            self.maximum_attempts is None
            or self.maximum_attempts <= 0
            or len(tried) < self.maximum_attempts
            or self.maximum_locations is not None
        )
        retry = not found and within_budget and (retry_same or bool(alternative_locations))
        location_exhausted = not found and not retry_same
        if location_exhausted:
            self.exhausted_locations.setdefault(task_id, set()).add(str(location_id))
            if self.active_locations.get(task_id) == location_id:
                self.active_locations.pop(task_id, None)
                self.directions.pop(task_id, None)
                self.last_angles.pop(task_id, None)
        return {
            "retry": retry,
            "attempt_index": len(tried),
            "maximum_attempts": self.maximum_attempts,
            "viewpoint_attempt_index": len(tried),
            "location_attempt_index": len(visited_locations),
            "maximum_locations": self.maximum_locations,
            "attempted_candidate_ids": sorted(tried),
            "remaining_candidate_ids": remaining,
            "location_hypothesis_id": location_id,
            "direction": self.directions.get(task_id),
            "location_exhausted": location_exhausted,
            "reason": ("alternative_viewpoint_available" if retry_same else "alternative_location_available") if retry else (
                "target_found" if found else (
                    "viewpoint_budget_exhausted"
                    if not within_budget else "candidate_pool_exhausted"
                )
            ),
        }

    def filtered_candidates(
        self, candidates_by_task: dict[str, list[dict[str, Any]]]
    ) -> dict[str, list[dict[str, Any]]]:
        result = {}
        for task_id, values in candidates_by_task.items():
            exhausted = self.exhausted_locations.get(task_id, set())
            available = [
                item for item in values
                if item["id"] not in self.attempted.get(task_id, set())
                and self._location(item) not in exhausted
            ]
            active_location = self.active_locations.get(task_id)
            focused = [item for item in available if self._location(item) == active_location]
            if focused:
                direction = self.directions.get(task_id)
                start_angle = self.last_angles.get(task_id)
                if direction is not None and start_angle is not None:
                    ordered_values = []
                    for item in focused:
                        angle = self._angle(item)
                        if angle is None:
                            continue
                        delta = self._forward_delta(start_angle, angle, direction)
                        if delta > 1e-6:
                            ordered_values.append((delta, item))
                    ordered = sorted(
                        ordered_values,
                        key=lambda value: (value[0], float(value[1].get("terminal_cost", 0.0)), value[1]["id"]),
                    )
                    result[task_id] = [ordered[0][1]] if ordered else focused
                else:
                    result[task_id] = focused
            else:
                result[task_id] = available
        return result

    def focused_task_ids(
        self, active_task_ids: set[str], candidates_by_task: dict[str, list[dict[str, Any]]]
    ) -> set[str]:
        """Return tasks whose failed location must be continued before global replanning."""
        focused = set()
        for task_id in active_task_ids:
            active_location = self.active_locations.get(task_id)
            if active_location is None:
                continue
            if any(
                self._location(item) == active_location
                for item in candidates_by_task.get(task_id, [])
                if item["id"] not in self.attempted.get(task_id, set())
                and active_location not in self.exhausted_locations.get(task_id, set())
            ):
                focused.add(task_id)
        return focused
