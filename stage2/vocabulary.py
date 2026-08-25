"""Shared canonical semantic aliases for task-time perception and planning."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_semantic_aliases(path: Path | None = None) -> dict[str, set[str]]:
    source = path or ROOT / "config/semantic_aliases.json"
    data = json.loads(source.read_text(encoding="utf-8"))
    result: dict[str, set[str]] = {}
    for canonical, values in data["aliases"].items():
        group = {str(canonical).strip().lower()}
        group.update(str(value).strip().lower() for value in values)
        group.discard("")
        for label in group:
            result[label] = set(group)
    return result
