"""Bounded alternative-viewpoint recovery for failed visual verification."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ViewpointRecovery:
    maximum_attempts: int = 3
    attempted: dict[str, set[str]] = field(default_factory=dict)

    def decide(
        self,
        task_id: str,
        candidate_id: str,
        found: bool,
        candidates: list[dict[str, Any]],
    ) -> dict[str, Any]:
        tried = self.attempted.setdefault(task_id, set())
        tried.add(candidate_id)
        remaining = [item["id"] for item in candidates if item["id"] not in tried]
        retry = not found and len(tried) < self.maximum_attempts and bool(remaining)
        return {
            "retry": retry,
            "attempt_index": len(tried),
            "maximum_attempts": self.maximum_attempts,
            "attempted_candidate_ids": sorted(tried),
            "remaining_candidate_ids": remaining,
            "reason": "alternative_viewpoint_available" if retry else (
                "target_found" if found else "viewpoint_budget_exhausted"
            ),
        }

    def filtered_candidates(
        self, candidates_by_task: dict[str, list[dict[str, Any]]]
    ) -> dict[str, list[dict[str, Any]]]:
        return {
            task_id: [item for item in values if item["id"] not in self.attempted.get(task_id, set())]
            for task_id, values in candidates_by_task.items()
        }
