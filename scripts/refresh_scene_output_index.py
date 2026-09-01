#!/usr/bin/env python3
"""Build a scene-centric index over existing Stage1 outputs.

The project historically stores runs, Bags, and evaluation figures by artifact
type.  This script adds a browseable ``outputs/scenes/<scene-id>/stage1`` view
using relative symbolic links.  Source data is never moved, copied, or deleted.
"""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path


SCENE_RE = re.compile(r"^[0-9]{5}-[A-Za-z0-9]+$")


def relative_link(source: Path, target: Path) -> str:
    """Create or verify one generated relative link and return its target."""
    source = source.resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    relative = os.path.relpath(source, target.parent.resolve())
    if target.is_symlink():
        if target.resolve() == source:
            return relative
        target.unlink()
    elif target.exists():
        raise RuntimeError(f"refusing to replace non-link index entry: {target}")
    target.symlink_to(relative)
    return relative


def prune_generated_links(directory: Path, expected_names: set[str]) -> None:
    """Remove stale links from an index directory, never regular files."""
    if not directory.is_dir():
        return
    for path in directory.iterdir():
        if path.is_symlink() and path.name not in expected_names:
            path.unlink()


def load_run_result(run_dir: Path) -> dict:
    result_path = run_dir / "run_result.json"
    if not result_path.is_file():
        return {"termination": "missing"}
    try:
        result = json.loads(result_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as error:
        return {"termination": "invalid", "error": str(error)}
    return {
        key: result.get(key)
        for key in ("termination", "elapsed_seconds", "frames")
        if key in result
    }


def scene_ids(outputs_root: Path, requested: list[str]) -> list[str]:
    stage1_root = outputs_root / "stage1_3d"
    discovered = sorted(
        path.name
        for path in stage1_root.iterdir()
        if path.is_dir() and SCENE_RE.fullmatch(path.name)
    ) if stage1_root.is_dir() else []
    if not requested:
        return discovered
    invalid = [value for value in requested if not SCENE_RE.fullmatch(value)]
    if invalid:
        raise ValueError(f"invalid scene ID(s): {', '.join(invalid)}")
    missing = [value for value in requested if value not in discovered]
    if missing:
        raise FileNotFoundError(
            "no Stage1 output directory for: " + ", ".join(missing)
        )
    return sorted(set(requested))


def matching_bags(outputs_root: Path, scene_id: str) -> list[tuple[Path, str]]:
    number = scene_id.split("-", 1)[0]
    pattern = re.compile(
        rf"^hm3d_stage1_3d_{re.escape(number)}_(.+)_(compact|full)\.bag$"
    )
    matches = []
    bag_root = outputs_root / "bags"
    if not bag_root.is_dir():
        return matches
    for path in sorted(bag_root.iterdir()):
        if not path.is_file():
            continue
        match = pattern.fullmatch(path.name)
        if match:
            matches.append((path, f"{match.group(1)}_{match.group(2)}.bag"))
    return matches


def matching_comparisons(outputs_root: Path, scene_id: str) -> list[Path]:
    number = scene_id.split("-", 1)[0]
    matches = []
    for path in sorted(outputs_root.iterdir()):
        if not path.is_dir() or path.name in {"scenes", "stage1_3d"}:
            continue
        lowered = path.name.lower()
        if number in path.name and (
            "comparison" in lowered or "benchmark" in lowered
        ):
            matches.append(path)
    return matches


def refresh_scene(outputs_root: Path, scene_id: str) -> dict:
    source_runs = outputs_root / "stage1_3d" / scene_id
    scene_stage1 = outputs_root / "scenes" / scene_id / "stage1"
    run_index = scene_stage1 / "runs"
    bag_index = scene_stage1 / "bags"
    comparison_index = scene_stage1 / "comparisons"

    runs = []
    source_run_dirs = sorted(path for path in source_runs.iterdir() if path.is_dir())
    prune_generated_links(run_index, {path.name for path in source_run_dirs})
    for run_dir in source_run_dirs:
        link = run_index / run_dir.name
        relative_link(run_dir, link)
        runs.append({
            "name": run_dir.name,
            "source": str(run_dir.relative_to(outputs_root)),
            "index": str(link.relative_to(outputs_root)),
            "result": load_run_result(run_dir),
            "has_bag_export": (run_dir / "bag_export" / "map_occupied.pcd").is_file(),
        })

    bags = []
    source_bags = matching_bags(outputs_root, scene_id)
    prune_generated_links(bag_index, {name for _, name in source_bags})
    for bag_path, display_name in source_bags:
        link = bag_index / display_name
        relative_link(bag_path, link)
        bags.append({
            "name": display_name,
            "bytes": bag_path.stat().st_size,
            "source": str(bag_path.relative_to(outputs_root)),
            "index": str(link.relative_to(outputs_root)),
        })

    comparisons = []
    source_comparisons = matching_comparisons(outputs_root, scene_id)
    prune_generated_links(
        comparison_index, {path.name for path in source_comparisons}
    )
    for comparison_dir in source_comparisons:
        link = comparison_index / comparison_dir.name
        relative_link(comparison_dir, link)
        comparisons.append({
            "name": comparison_dir.name,
            "source": str(comparison_dir.relative_to(outputs_root)),
            "index": str(link.relative_to(outputs_root)),
        })

    # Benchmark runs may render directly into the scene-centric directory.
    # Include those regular directories in the manifest as canonical outputs.
    if comparison_index.is_dir():
        known = {item["name"] for item in comparisons}
        for path in sorted(comparison_index.iterdir()):
            if path.name in known or not path.is_dir() or path.is_symlink():
                continue
            comparisons.append({
                "name": path.name,
                "source": str(path.relative_to(outputs_root)),
                "index": str(path.relative_to(outputs_root)),
            })

    manifest = {
        "format": "pre_map_vln.scene_stage1_index.v1",
        "scene_id": scene_id,
        "note": (
            "Scene-centric relative links only; source artifacts remain in their "
            "legacy locations and are not duplicated."
        ),
        "runs": runs,
        "bags": bags,
        "comparisons": comparisons,
    }
    scene_stage1.mkdir(parents=True, exist_ok=True)
    (scene_stage1 / "index.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--outputs-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "outputs",
    )
    parser.add_argument("--scene-id", action="append", default=[])
    args = parser.parse_args()

    outputs_root = args.outputs_root.resolve()
    selected = scene_ids(outputs_root, args.scene_id)
    if not selected:
        raise SystemExit(f"no Stage1 scenes found below {outputs_root / 'stage1_3d'}")
    for scene_id in selected:
        manifest = refresh_scene(outputs_root, scene_id)
        print(
            f"indexed {scene_id}: {len(manifest['runs'])} runs, "
            f"{len(manifest['bags'])} bags, "
            f"{len(manifest['comparisons'])} comparison groups"
        )
    print(f"scene-centric index: {outputs_root / 'scenes'}")


if __name__ == "__main__":
    main()
