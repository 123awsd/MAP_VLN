"""Resolve the Habitat origin used when comparing a recorded FALCON map.

The recorder anchors the FALCON frame at the selected initial Habitat point.
Comparisons must use that same point; sampling a fresh random navmesh point
silently translates the official geometry and produces an apparent drift.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]


def _derived_manifest_path(bag: Path) -> Path:
    stem = bag.stem
    episode_name = stem[:-4] if stem.endswith("_raw") else stem
    return ROOT / "data" / "episodes" / episode_name / "manifest.json"


def load_recording_origin(
    bag: Path,
    recording_manifest: Path | None = None,
    explicit_xyz: Iterable[float] | None = None,
) -> tuple[list[float] | None, str, Path | None]:
    """Return ``(xyz, source, manifest_path)`` for a recorded map origin.

    An explicit CLI value wins.  Otherwise a manifest supplied by the caller,
    or the conventional ``data/episodes/<bag-stem>/manifest.json`` sidecar, is
    inspected.  ``None`` means the caller should retain its legacy random
    navmesh fallback.
    """
    if explicit_xyz is not None:
        values = [float(value) for value in explicit_xyz]
        if len(values) != 3:
            raise ValueError("initial Habitat origin must contain exactly three values")
        return values, "cli", None

    manifest_path = recording_manifest.resolve() if recording_manifest else _derived_manifest_path(bag)
    if not manifest_path.is_file():
        return None, "random_navmesh_fallback", None
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        values = payload["initial_start"]["habitat_xyz"]
        values = [float(value) for value in values]
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
        raise ValueError(
            f"invalid recording manifest or missing initial_start.habitat_xyz: {manifest_path}"
        ) from error
    if len(values) != 3:
        raise ValueError(f"initial_start.habitat_xyz must contain three values: {manifest_path}")
    return values, "recording_manifest", manifest_path
