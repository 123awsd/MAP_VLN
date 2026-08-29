#!/usr/bin/env python3
"""Generate one navmesh/map comparison per automatically detected floor."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import habitat_sim
import numpy as np

from comparison_origin import load_recording_origin


ROOT = Path(__file__).resolve().parents[1]
OUTPUTS = ROOT / "outputs"
HABITAT_PYTHON = ROOT / ".envs/habitat/bin/python"


def within_outputs(path: Path) -> Path:
    resolved = path.resolve()
    try:
        return resolved.relative_to(OUTPUTS.resolve())
    except ValueError as error:
        raise ValueError(f"bag must be below {OUTPUTS}: {resolved}") from error


def detect_floor_levels(
    scene: Path,
    scene_config: Path,
    seed: int,
    floor_count: int,
    min_separation: float,
    recording_origin_xyz: list[float] | None = None,
) -> list[float]:
    """Detect actual floor planes from the scene navmesh, not a fixed count.

    Random navigable samples form dense horizontal bands on each floor and a
    sparse tail on stairs/ramps.  A smoothed histogram plus a prominence
    threshold rejects those transition tails while retaining small floors.
    Returned heights are FALCON world-Z values for the recording seed.
    """
    simulator = habitat_sim.SimulatorConfiguration()
    simulator.scene_id = str(scene)
    simulator.scene_dataset_config_file = str(scene_config)
    simulator.create_renderer = False
    agent = habitat_sim.agent.AgentConfiguration()
    with habitat_sim.Simulator(habitat_sim.Configuration(simulator, [agent])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("Habitat navmesh did not load")
        sim.pathfinder.seed(seed)
        origin_h = np.asarray(
            recording_origin_xyz
            if recording_origin_xyz is not None
            else sim.pathfinder.get_random_navigable_point(),
            dtype=np.float64,
        )
        samples = np.asarray(
            [sim.pathfinder.get_random_navigable_point()[1] for _ in range(100000)],
            dtype=np.float64,
        )
    samples = samples[np.isfinite(samples)]
    if len(samples) < 100:
        raise RuntimeError("not enough navigable samples to detect floor levels")
    bin_size = 0.10
    lower = math.floor(float(samples.min()) / bin_size) * bin_size - bin_size
    upper = math.ceil(float(samples.max()) / bin_size) * bin_size + bin_size
    edges = np.arange(lower, upper + bin_size * 0.5, bin_size)
    histogram, _ = np.histogram(samples, bins=edges)
    window = max(3, int(round(0.6 / bin_size)))
    if window % 2 == 0:
        window += 1
    smoothed = np.convolve(histogram.astype(np.float64), np.ones(window), mode="same")
    threshold = max(20.0, float(smoothed.max()) * 0.08)
    candidates = [
        index for index in range(1, len(smoothed) - 1)
        if smoothed[index] >= smoothed[index - 1]
        and smoothed[index] >= smoothed[index + 1]
        and smoothed[index] >= threshold
    ]
    candidates.sort(key=lambda index: smoothed[index], reverse=True)
    min_bins = max(1, int(round(min_separation / bin_size)))
    selected = []
    for index in candidates:
        if all(abs(index - other) >= min_bins for other in selected):
            selected.append(index)
        if floor_count > 0 and len(selected) >= floor_count:
            break
    habitat_levels = sorted(float((edges[index] + edges[index + 1]) * 0.5) for index in selected)
    # FALCON Z is anchored at 1 m at the recording seed's Habitat height.
    return [level - float(origin_h[1]) + 1.0 for level in habitat_levels]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--floor-count", type=int, default=0,
        help="optional maximum number of layers; zero detects every supported floor",
    )
    parser.add_argument("--floor-band", type=float, default=1.15)
    parser.add_argument(
        "--endpoint-band", type=float, default=0.45,
        help="half-width of the obstacle endpoint height band around each floor",
    )
    parser.add_argument(
        "--max-endpoint-vertical-delta", type=float, default=0.75,
        help="reject floor/ceiling returns whose endpoint Z differs too far from the camera",
    )
    parser.add_argument("--min-separation", type=float, default=1.0)
    parser.add_argument(
        "--levels", type=float, nargs="+", default=None,
        help="explicit Falcon world-Z levels; omit to detect them from the scene navmesh",
    )
    parser.add_argument("--keyframe-step", type=int, default=5)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--initial-habitat-xyz",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="recorded initial Habitat point; otherwise read the conventional episode manifest",
    )
    parser.add_argument(
        "--recording-manifest",
        type=Path,
        default=None,
        help="manifest containing initial_start.habitat_xyz (overrides automatic sidecar lookup)",
    )
    args = parser.parse_args()
    args.bag = args.bag.resolve()
    args.scene = args.scene.resolve()
    args.scene_config = args.scene_config.resolve()
    if args.recording_manifest is not None:
        args.recording_manifest = args.recording_manifest.resolve()
    if not args.bag.is_file():
        parser.error(f"bag not found: {args.bag}")
    if not args.scene.is_file():
        parser.error(f"scene not found: {args.scene}")
    if not args.scene_config.is_file():
        parser.error(f"scene config not found: {args.scene_config}")
    if args.recording_manifest is not None and not args.recording_manifest.is_file():
        parser.error(f"recording manifest not found: {args.recording_manifest}")
    if args.floor_count < 0:
        parser.error("--floor-count cannot be negative")
    if args.floor_band <= 0:
        parser.error("--floor-band must be positive")
    if args.endpoint_band <= 0:
        parser.error("--endpoint-band must be positive")
    if args.max_endpoint_vertical_delta < 0:
        parser.error("--max-endpoint-vertical-delta cannot be negative")
    if args.min_separation <= 0:
        parser.error("--min-separation must be positive")
    try:
        (
            recording_origin_xyz,
            recording_origin_source,
            recording_origin_manifest,
        ) = load_recording_origin(
            args.bag,
            args.recording_manifest,
            args.initial_habitat_xyz,
        )
    except ValueError as error:
        parser.error(str(error))
    if recording_origin_xyz is None:
        print(
            "warning: no recording initial Habitat origin found; using a random "
            "navmesh point, so official geometry may be translated",
            file=sys.stderr,
        )
    levels = (
        [float(level) for level in args.levels]
        if args.levels is not None
        else detect_floor_levels(
            args.scene,
            args.scene_config,
            args.seed,
            args.floor_count,
            args.min_separation,
            recording_origin_xyz,
        )
    )
    levels = sorted(levels)
    if not levels:
        raise RuntimeError("no floor levels detected")
    if args.floor_count and len(levels) > args.floor_count:
        levels = levels[: args.floor_count]
    output_dir = (args.output_dir or args.bag.parent).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.bag.stem[:-4] if args.bag.stem.endswith("_raw") else args.bag.stem
    reports = []
    for index, level in enumerate(levels):
        floor_number = index + 1
        output = output_dir / f"{stem}_floor_{floor_number}_navmesh_vs_map.png"
        command = [
            str(HABITAT_PYTHON), str(ROOT / "scripts/compare_navmesh_map.py"),
            "--bag", str(args.bag), "--scene", str(args.scene),
            "--scene-config", str(args.scene_config), "--output", str(output),
            "--resolution", str(args.resolution), "--keyframe-step", str(args.keyframe_step),
            "--height-min", str(max(0.05, level - args.endpoint_band)),
            "--height-max", str(level + args.endpoint_band),
            "--camera-height-min", str(max(0.05, level - args.floor_band)),
            "--camera-height-max", str(level + args.floor_band),
            "--floor-height", str(level), "--floor-index", str(floor_number),
            "--max-endpoint-vertical-delta", str(args.max_endpoint_vertical_delta),
            "--seed", str(args.seed),
        ]
        if recording_origin_manifest is not None:
            command.extend(["--recording-manifest", str(recording_origin_manifest)])
        elif recording_origin_xyz is not None:
            command.extend(["--initial-habitat-xyz", *[str(value) for value in recording_origin_xyz]])
        subprocess.run(command, cwd=ROOT, check=True)
        reports.append(json.loads(output.with_suffix(".json").read_text(encoding="utf-8")))
    print(json.dumps({
        "format": "pre_map_vln.multi_floor_comparison.v1",
        "levels_m": levels,
        "reports": reports,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
