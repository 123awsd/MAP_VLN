"""Bounded alternative-viewpoint recovery for failed visual verification."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ViewpointRecovery:
    maximum_attempts: int = 3
    maximum_attempts_per_location: int | None = None
    attempted: dict[str, set[str]] = field(default_factory=dict)
    exhausted_locations: dict[str, set[str]] = field(default_factory=dict)

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
        location_id = current.get("location_hypothesis_id") or current.get("object_id") or "default"
        attempted_at_location = [
            item["id"] for item in candidates
            if item["id"] in tried
            and (item.get("location_hypothesis_id") or item.get("object_id") or "default") == location_id
        ]
        remaining = [item["id"] for item in candidates if item["id"] not in tried]
        same_location = [
            item["id"] for item in candidates if item["id"] not in tried
            and (item.get("location_hypothesis_id") or item.get("object_id") or "default") == location_id
        ]
        location_budget = self.maximum_attempts_per_location or self.maximum_attempts
        already_exhausted = self.exhausted_locations.get(task_id, set())
        alternative_locations = [
            item["id"] for item in candidates
            if item["id"] in remaining
            and item["id"] not in same_location
            and str(item.get("location_hypothesis_id") or item.get("object_id") or "default")
            not in already_exhausted
        ]
        retry_same = len(attempted_at_location) < location_budget and bool(same_location)
        retry = not found and len(tried) < self.maximum_attempts and (retry_same or bool(alternative_locations))
        location_exhausted = not found and not retry_same
        if location_exhausted:
            self.exhausted_locations.setdefault(task_id, set()).add(str(location_id))
        return {
            "retry": retry,
            "attempt_index": len(tried),
            "maximum_attempts": self.maximum_attempts,
            "attempted_candidate_ids": sorted(tried),
            "remaining_candidate_ids": remaining,
            "location_hypothesis_id": location_id,
            "location_exhausted": location_exhausted,
            "reason": ("alternative_viewpoint_available" if retry_same else "alternative_location_available") if retry else (
                "target_found" if found else "viewpoint_budget_exhausted"
            ),
        }

    def filtered_candidates(
        self, candidates_by_task: dict[str, list[dict[str, Any]]]
    ) -> dict[str, list[dict[str, Any]]]:
        result = {}
        for task_id, values in candidates_by_task.items():
            exhausted = self.exhausted_locations.get(task_id, set())
            result[task_id] = [
                item for item in values
                if item["id"] not in self.attempted.get(task_id, set())
                and str(item.get("location_hypothesis_id") or item.get("object_id") or "default")
                not in exhausted
            ]
        return result
