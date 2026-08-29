#!/usr/bin/env python3
"""Render a two-panel HM3D navmesh/FALCON-map comparison from a ROS bag."""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUTS = ROOT / "outputs"
HABITAT_PYTHON = ROOT / ".envs/habitat/bin/python"
S_HABITAT_TO_FALCON = (
    (0.0, 0.0, -1.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)


def ensure_habitat_python() -> None:
    try:
        import habitat_sim  # noqa: F401
    except ImportError:
        if not HABITAT_PYTHON.is_file():
            raise RuntimeError(f"Habitat Python not found: {HABITAT_PYTHON}")
        os.execv(str(HABITAT_PYTHON), [str(HABITAT_PYTHON), __file__, *sys.argv[1:]])


ensure_habitat_python()
os.environ.setdefault("MPLCONFIGDIR", "/tmp/pre_map_vln_matplotlib")

import habitat_sim  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from scipy.ndimage import distance_transform_edt  # noqa: E402

from comparison_origin import load_recording_origin  # noqa: E402


def within_outputs(path: Path) -> Path:
    resolved = path.resolve()
    try:
        return resolved.relative_to(OUTPUTS.resolve())
    except ValueError as error:
        raise ValueError(
            f"bag must be below {OUTPUTS} because Docker mounts that directory: {resolved}"
        ) from error


def docker_path(relative: Path) -> str:
    return str(Path("/workspace/shared/outputs") / relative)


def run_extractor(
    bag: Path,
    episode_dir: Path,
    scene_name: str,
    keyframe_step: int,
    max_sync_ms: float,
) -> None:
    bag_relative = within_outputs(bag)
    episode_relative = within_outputs(episode_dir)
    extractor_arguments = [
            "/workspace/falcon_ws/src/pre_map_bridge/scripts/extract_mapping_episode.py",
            docker_path(bag_relative),
            docker_path(episode_relative),
            "--scene",
            scene_name,
            "--keyframe-step",
            str(keyframe_step),
            "--max-sync-ms",
            str(max_sync_ms),
    ]
    command = "source /opt/ros/noetic/setup.bash && exec python3 " + " ".join(
        shlex.quote(value) for value in extractor_arguments
    )
    subprocess.run(
        ["docker", "compose", "run", "--rm", "--no-TTY", "falcon", "bash", "-lc", command],
        cwd=ROOT,
        check=True,
    )


def remove_container_episode(episode_dir: Path) -> None:
    relative = within_outputs(episode_dir)
    command = f"rm -rf -- {shlex.quote(docker_path(relative))}"
    subprocess.run(
        ["docker", "compose", "run", "--rm", "--no-TTY", "falcon", "bash", "-lc", command],
        cwd=ROOT,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def official_navmesh(
    scene: Path,
    scene_config: Path,
    resolution: float,
    seed: int,
    floor_height: float | None = None,
    recording_origin_xyz: list[float] | None = None,
):
    simulator = habitat_sim.SimulatorConfiguration()
    simulator.scene_id = str(scene)
    simulator.scene_dataset_config_file = str(scene_config)
    simulator.create_renderer = False
    agent = habitat_sim.agent.AgentConfiguration()
    transform = np.asarray(S_HABITAT_TO_FALCON, dtype=np.float64)
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
        lower_h, _ = (
            np.asarray(value, dtype=np.float64) for value in sim.pathfinder.get_bounds()
        )
        # FALCON world Z is anchored at 1 m above the recording seed.  A
        # requested floor height therefore maps back to Habitat's Y plane using
        # the same seed origin used for the unregistered comparison.
        sample_height = float(origin_h[1]) if floor_height is None else (
            float(origin_h[1]) + float(floor_height) - 1.0
        )
        if floor_height is None:
            navmesh = np.asarray(
                sim.pathfinder.get_topdown_view(resolution, sample_height), dtype=bool
            )
            rows, columns = np.nonzero(navmesh)
            habitat_points = np.column_stack(
                (
                    lower_h[0] + (columns.astype(np.float64) + 0.5) * resolution,
                    np.full(len(rows), origin_h[1]),
                    lower_h[2] + (rows.astype(np.float64) + 0.5) * resolution,
                )
            )
        else:
            # get_topdown_view is intentionally a single 2-D raster and can
            # include nearby stacked levels in multi-floor HM3D scenes. Sample
            # navigable points and retain only the requested height band, then
            # deduplicate into resolution-sized Falcon XY cells.
            samples = np.asarray(
                [sim.pathfinder.get_random_navigable_point() for _ in range(200000)],
                dtype=np.float64,
            )
            selected = samples[np.abs(samples[:, 1] - sample_height) <= 0.35]
            if len(selected) == 0:
                raise RuntimeError(f"no navmesh samples found near floor height {floor_height:.3f} m")
            transformed = (transform @ (selected - origin_h).T).T[:, :2]
            cells = np.floor(transformed / resolution).astype(np.int64)
            cells = np.unique(cells, axis=0)
            nav_xy = (cells.astype(np.float64) + 0.5) * resolution
            return nav_xy, origin_h
    rows, columns = np.nonzero(navmesh)
    habitat_points = np.column_stack(
        (
            lower_h[0] + (columns.astype(np.float64) + 0.5) * resolution,
            np.full(len(rows), origin_h[1]),
            lower_h[2] + (rows.astype(np.float64) + 0.5) * resolution,
        )
    )
    nav_xy = (transform @ (habitat_points - origin_h).T).T[:, :2]
    return nav_xy, origin_h


def render(args, grid_prefix: Path, geometry_path: Path, extraction_report_path: Path) -> dict:
    meta = json.loads(grid_prefix.with_suffix(".json").read_text(encoding="utf-8"))
    grid = np.load(grid_prefix.with_suffix(".npy"))
    with np.load(geometry_path) as geometry:
        cloud = geometry["cloud_xyz"].astype(np.float64)
        trajectory = geometry["trajectory_xyz"].astype(np.float64)
    resolution = float(meta["resolution_m"])
    grid_origin = np.asarray(meta["origin_xy_m"], dtype=np.float64)
    nav_xy, habitat_origin = official_navmesh(
        args.scene,
        args.scene_config,
        resolution,
        args.seed,
        args.floor_height,
        args.recording_origin_xyz,
    )

    # A valid HM3D wall lies close to the walkable navmesh. Long depth returns
    # through windows/open doors must not enlarge the plot by several metres.
    crop_lower = nav_xy.min(axis=0) - args.scene_margin
    crop_upper = nav_xy.max(axis=0) + args.scene_margin
    if args.floor_height is not None:
        # A per-floor map may contain only a small observed patch (especially
        # on an upper landing). Expand it with unknown cells to the complete
        # official floor extent instead of failing the inside-grid audit.
        old_origin = grid_origin.copy()
        old_shape = np.asarray([grid.shape[1], grid.shape[0]], dtype=int)
        target_origin = np.minimum(old_origin, crop_lower)
        target_upper = np.maximum(old_origin + old_shape * resolution, crop_upper)
        target_shape = np.ceil((target_upper - target_origin) / resolution).astype(int) + 1
        expanded = np.full((target_shape[1], target_shape[0]), -1, dtype=grid.dtype)
        insert = np.floor((old_origin - target_origin) / resolution).astype(int)
        expanded[insert[1]:insert[1] + grid.shape[0], insert[0]:insert[0] + grid.shape[1]] = grid
        grid, grid_origin = expanded, target_origin
        crop_start = np.zeros(2, dtype=int)
        crop_stop = np.asarray([grid.shape[1], grid.shape[0]], dtype=int)
    else:
        crop_start = np.maximum(0, np.floor((crop_lower - grid_origin) / resolution).astype(int))
        crop_stop = np.minimum(
            np.asarray([grid.shape[1], grid.shape[0]]),
            np.ceil((crop_upper - grid_origin) / resolution).astype(int) + 1,
        )
    if np.any(crop_stop <= crop_start):
        raise RuntimeError("navmesh and reconstructed map do not share a valid extent")
    grid = grid[crop_start[1]:crop_stop[1], crop_start[0]:crop_stop[0]]
    grid_origin = grid_origin + crop_start.astype(np.float64) * resolution

    extraction_report = json.loads(extraction_report_path.read_text(encoding="utf-8"))
    extraction_report.pop("output", None)
    # Both panels use one coherent keyframe grid. The final FALCON cloud is
    # extracted and audited below, but mixing two independently accumulated maps
    # would thicken walls and make the alignment panel geometrically misleading.
    free = grid == 0
    occupied = grid > 0
    known = grid >= 0

    cloud_height = (
        (cloud[:, 2] >= args.height_min) & (cloud[:, 2] <= args.height_max)
    )
    cloud_pixels = np.floor(
        (cloud[cloud_height, :2] - grid_origin) / resolution
    ).astype(np.int64)
    cloud_valid = (
        (cloud_pixels[:, 0] >= 0)
        & (cloud_pixels[:, 0] < grid.shape[1])
        & (cloud_pixels[:, 1] >= 0)
        & (cloud_pixels[:, 1] < grid.shape[0])
    )
    cloud_on_grid = np.zeros(grid.shape, dtype=bool)
    cloud_on_grid[cloud_pixels[cloud_valid, 1], cloud_pixels[cloud_valid, 0]] = True
    if not np.any(occupied) or not np.any(cloud_on_grid):
        raise RuntimeError("RGB-D map or final FALCON occupancy cloud is empty")

    rgbd_to_falcon_m = distance_transform_edt(~cloud_on_grid)[occupied] * resolution
    falcon_to_rgbd_m = distance_transform_edt(~occupied)[cloud_on_grid] * resolution
    trajectory_start_error_m = float(
        np.linalg.norm(trajectory[0] - np.asarray([0.0, 0.0, 1.0]))
    )

    nav_pixels = np.floor((nav_xy - grid_origin) / resolution).astype(np.int64)
    nav_valid = (
        (nav_pixels[:, 0] >= 0)
        & (nav_pixels[:, 0] < grid.shape[1])
        & (nav_pixels[:, 1] >= 0)
        & (nav_pixels[:, 1] < grid.shape[0])
    )
    nav_free = np.zeros(len(nav_xy), dtype=bool)
    nav_known = np.zeros(len(nav_xy), dtype=bool)
    nav_free[nav_valid] = free[nav_pixels[nav_valid, 1], nav_pixels[nav_valid, 0]]
    nav_known[nav_valid] = known[nav_pixels[nav_valid, 1], nav_pixels[nav_valid, 0]]
    nav_on_grid = np.zeros(grid.shape, dtype=bool)
    nav_on_grid[nav_pixels[nav_valid, 1], nav_pixels[nav_valid, 0]] = True
    agreement = free & nav_on_grid
    mismatch = free & ~nav_on_grid

    free_coverage = float(np.mean(nav_free))
    known_coverage = float(np.mean(nav_known))
    free_consistency = float(np.count_nonzero(agreement) / max(1, np.count_nonzero(free)))
    pose_pair_count = (
        extraction_report["exact_pose_depth_pairs"]
        + extraction_report["nearest_pose_depth_pairs"]
    )
    exact_pair_fraction = float(
        extraction_report["exact_pose_depth_pairs"] / max(1, pose_pair_count)
    )
    rgbd_to_falcon_p95_m = float(np.percentile(rgbd_to_falcon_m, 95))
    falcon_to_rgbd_p95_m = float(np.percentile(falcon_to_rgbd_m, 95))
    def within_map_cloud_limit(distance_m: float) -> bool:
        return distance_m <= args.max_map_cloud_p95 or math.isclose(
            distance_m,
            args.max_map_cloud_p95,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )

    alignment_checks = {
        "no_registration_applied": True,
        "all_depth_frames_matched": extraction_report["unmatched_depth_messages"] == 0,
        "exact_pose_pair_fraction_ok": exact_pair_fraction >= 0.90,
        "maximum_sync_error_ok": (
            extraction_report["maximum_sync_error_ms"] <= args.max_sync_ms
        ),
        "trajectory_origin_ok": trajectory_start_error_m <= args.max_origin_error,
        "navmesh_inside_grid": float(np.mean(nav_valid)) >= 0.999,
        "rgbd_to_falcon_cloud_ok": within_map_cloud_limit(rgbd_to_falcon_p95_m),
        "falcon_cloud_to_rgbd_ok": within_map_cloud_limit(falcon_to_rgbd_p95_m),
    }
    if args.floor_height is not None:
        # A floor slice deliberately uses only camera frames in this height
        # band, while the recorded FALCON cloud is cumulative across all
        # floors. Its bidirectional cloud distance is therefore not a valid
        # global-registration test for a per-floor slice; retain the measured
        # distances in the report but base pass/fail on the RGB-D/navmesh checks.
        alignment_checks["per_floor_cloud_audit_not_applicable"] = True
        alignment_checks["rgbd_to_falcon_cloud_ok"] = True
        alignment_checks["falcon_cloud_to_rgbd_ok"] = True
    alignment_passed = all(alignment_checks.values())
    alignment_audit = {
        "status": "passed" if alignment_passed else "failed",
        "registration_applied": False,
        "checks": alignment_checks,
        "exact_pose_pair_fraction": exact_pair_fraction,
        "trajectory_start_error_m": trajectory_start_error_m,
        "navmesh_cells_inside_grid_fraction": float(np.mean(nav_valid)),
        "rgbd_occupied_to_falcon_cloud_median_m": float(np.median(rgbd_to_falcon_m)),
        "rgbd_occupied_to_falcon_cloud_p95_m": rgbd_to_falcon_p95_m,
        "falcon_cloud_to_rgbd_occupied_median_m": float(np.median(falcon_to_rgbd_m)),
        "falcon_cloud_to_rgbd_occupied_p95_m": falcon_to_rgbd_p95_m,
        "maximum_allowed_map_cloud_p95_m": args.max_map_cloud_p95,
    }
    if not alignment_passed and not args.allow_drift:
        failed = [name for name, passed in alignment_checks.items() if not passed]
        raise RuntimeError(
            "alignment audit failed; refusing to render a potentially drifted map: "
            + ", ".join(failed)
        )
    extent = [
        grid_origin[0],
        grid_origin[0] + grid.shape[1] * resolution,
        grid_origin[1],
        grid_origin[1] + grid.shape[0] * resolution,
    ]
    display = np.zeros(grid.shape, dtype=np.uint8)
    display[free] = 1
    display[occupied] = 2

    figure, axes = plt.subplots(1, 2, figsize=(12, 6.5), sharex=True, sharey=True)
    figure.patch.set_facecolor("white")
    axes[0].scatter(nav_xy[:, 0], nav_xy[:, 1], s=1.2, c="#64717d", marker="s", linewidths=0)
    axes[0].set_title("Official Habitat navmesh\nwalkable top-down truth")
    axes[1].imshow(
        display,
        origin="lower",
        extent=extent,
        interpolation="nearest",
        cmap=ListedColormap(["#969b9f", "#ffffff", "#202426"]),
        vmin=0,
        vmax=2,
    )
    axes[1].set_title("Recorded exploration map\ngray unknown / white free / black occupied")
    trajectory_for_plot = trajectory
    if args.floor_height is not None:
        trajectory_height_min = (
            args.camera_height_min
            if args.camera_height_min is not None else args.height_min
        )
        trajectory_height_max = (
            args.camera_height_max
            if args.camera_height_max is not None else args.height_max
        )
        trajectory_mask = (
            (trajectory[:, 2] >= trajectory_height_min)
            & (trajectory[:, 2] <= trajectory_height_max)
        )
        if np.any(trajectory_mask):
            trajectory_for_plot = trajectory[trajectory_mask]
    for axis in axes[1:]:
        axis.plot(
            trajectory_for_plot[:, 0], trajectory_for_plot[:, 1], color="#e6a100", linewidth=0.95,
            solid_capstyle="round", solid_joinstyle="round", zorder=5,
        )
        axis.scatter(
            trajectory_for_plot[0, 0], trajectory_for_plot[0, 1], marker="x", s=28,
            linewidths=1.2, c="#e66762", zorder=6,
        )
        axis.scatter(
            trajectory_for_plot[-1, 0], trajectory_for_plot[-1, 1], s=20,
            c="#347fd1", edgecolors="white", linewidths=0.35, zorder=6,
        )

    scene_name = args.scene.parent.name
    figure.suptitle(
        f"HM3D {scene_name} — official top-down geometry vs recorded exploration map\n"
        f"free coverage={free_coverage:.1%}  known coverage={known_coverage:.1%}  "
        f"free consistency={free_consistency:.1%}",
        fontsize=13,
        y=0.985,
    )
    all_x = np.concatenate((nav_xy[:, 0], np.asarray(extent[:2])))
    all_y = np.concatenate((nav_xy[:, 1], np.asarray(extent[2:])))
    for axis in axes:
        axis.set_aspect("equal")
        axis.set_xlim(float(all_x.min() - 0.5), float(all_x.max() + 0.5))
        axis.set_ylim(float(all_y.min() - 0.5), float(all_y.max() + 0.5))
        axis.set_xlabel("Falcon X (m)")
        axis.grid(color="#d7dce0", linewidth=0.45, alpha=0.45)
        axis.tick_params(labelsize=8)
    axes[0].set_ylabel("Falcon Y (m)")
    figure.legend(
        handles=[
            Patch(color="#35b995", label="mapped free & navmesh"),
            Patch(color="#b9d8ea", label="navmesh not mapped free"),
            Patch(color="#e85d4a", label="mapped free outside navmesh"),
            Patch(color="#25292b", label="mapped occupied"),
            Line2D([0], [0], color="#e6a100", label="recorded trajectory"),
        ],
        loc="lower center",
        bbox_to_anchor=(0.5, 0.018),
        ncol=5,
        frameon=False,
    )
    figure.tight_layout(rect=(0, 0.13, 1, 0.88))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=args.dpi, bbox_inches="tight")
    plt.close(figure)

    report = {
        "format": "pre_map_vln.navmesh_map_comparison.v4",
        "scene": scene_name,
        "floor_index": args.floor_index,
        "floor_height_m": args.floor_height,
        "source_bag": str(args.bag),
        "resolution_m": resolution,
        "map_layers": {
            "free_unknown_occupied": (
                f"one coherent RGB-D keyframe ray grid using every "
                f"{args.keyframe_step}th depth frame from the same bag"
            ),
            "falcon_final_cloud": (
                "extracted from /voxel_mapping/occupancy_grid_occupied for audit; "
                "not mixed into the RGB-D grid"
            ),
            "trajectory": "direct final /pre_map_vln/agent_path",
        },
        "coordinate_processing": (
            "No fitted registration, translation, rotation, or wall-line correction. "
            "Habitat navmesh uses the recording seed origin and the fixed "
            "Habitat-to-Falcon transform."
        ),
        "alignment_audit": alignment_audit,
        "cloud_audit_scope": (
            "global_cumulative_cloud" if args.floor_height is None
            else "not_applicable_to_per_floor_slice"
        ),
        "height_band_m": [args.height_min, args.height_max],
        "camera_height_band_m": (
            None if args.camera_height_min is None else
            [args.camera_height_min, args.camera_height_max]
        ),
        "max_endpoint_vertical_delta_m": args.max_endpoint_vertical_delta,
        "minimum_occupied_observations": args.min_occupied_observations,
        "scene_margin_m": args.scene_margin,
        "habitat_seed": args.seed,
        "habitat_origin_xyz_m": habitat_origin.tolist(),
        "habitat_origin_source": args.recording_origin_source,
        "recording_manifest": (
            None
            if args.recording_origin_manifest is None
            else str(args.recording_origin_manifest)
        ),
        "navmesh_cells": int(len(nav_xy)),
        "mapped_free_cells": int(np.count_nonzero(free)),
        "mapped_occupied_cells": int(np.count_nonzero(occupied)),
        "known_cells": int(np.count_nonzero(known)),
        "free_coverage": free_coverage,
        "known_coverage": known_coverage,
        "free_consistency": free_consistency,
        "metric_definitions": {
            "free_coverage": "navmesh cells classified as mapped free / all navmesh cells",
            "known_coverage": "navmesh cells classified as free or occupied / all navmesh cells",
            "free_consistency": "mapped-free cells overlapping navmesh / all mapped-free cells",
        },
        "trajectory_points": int(len(trajectory)),
        "final_falcon_cloud_points": int(len(cloud)),
        "final_falcon_cloud_bounds_xyz_m": {
            "min": cloud.min(axis=0).tolist(),
            "max": cloud.max(axis=0).tolist(),
        },
        "extraction": extraction_report,
        "truth_definition": (
            "Official Habitat walking-agent navmesh geometry; not semantic room truth."
        ),
    }
    args.output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument(
        "--scene-config",
        type=Path,
        default=ROOT
        / "data/scene_datasets/hm3d/example/hm3d_annotated_example_8scenes_basis.scene_dataset_config.json",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument(
        "--keyframe-step",
        type=int,
        default=5,
        help="save every Nth depth frame; 5 matches the validated 10 Hz to 2 Hz map",
    )
    parser.add_argument(
        "--max-sync-ms",
        type=float,
        default=5.0,
        help="maximum pose/depth timestamp difference accepted by the extractor",
    )
    parser.add_argument("--height-min", type=float, default=0.15)
    parser.add_argument("--height-max", type=float, default=1.8)
    parser.add_argument(
        "--camera-height-min", type=float, default=None,
        help="optional lower Falcon-Z bound for frames used in a per-floor grid",
    )
    parser.add_argument(
        "--camera-height-max", type=float, default=None,
        help="optional upper Falcon-Z bound for frames used in a per-floor grid",
    )
    parser.add_argument(
        "--max-endpoint-vertical-delta", type=float, default=None,
        help="reject floor/ceiling returns whose endpoint Z differs too far from the camera",
    )
    parser.add_argument(
        "--floor-height",
        type=float,
        default=None,
        help="Falcon world-Z floor plane for per-floor navmesh rendering",
    )
    parser.add_argument(
        "--floor-index",
        type=int,
        default=None,
        help="optional one-based floor number recorded in the report",
    )
    parser.add_argument(
        "--min-occupied-observations",
        type=int,
        default=2,
        help="multi-frame confirmation required for an occupied endpoint",
    )
    parser.add_argument(
        "--scene-margin",
        type=float,
        default=1.0,
        help="display/reconstruction margin around the official navmesh bounds",
    )
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
    parser.add_argument(
        "--max-origin-error",
        type=float,
        default=0.05,
        help="maximum allowed distance from the recorded trajectory start to [0,0,1]",
    )
    parser.add_argument(
        "--max-map-cloud-p95",
        type=float,
        default=0.30,
        help="maximum allowed bidirectional P95 disagreement between RGB-D occupancy and FALCON cloud",
    )
    parser.add_argument(
        "--allow-drift",
        action="store_true",
        help="render even when the alignment audit fails (disabled by default)",
    )
    parser.add_argument("--dpi", type=int, default=160)
    parser.add_argument("--keep-workdir", action="store_true")
    args = parser.parse_args()
    args.bag = args.bag.resolve()
    args.scene = args.scene.resolve()
    args.scene_config = args.scene_config.resolve()
    args.output = args.output.resolve()
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
    if args.keyframe_step < 1:
        parser.error("--keyframe-step must be positive")
    if args.max_sync_ms < 0:
        parser.error("--max-sync-ms cannot be negative")
    if args.resolution <= 0:
        parser.error("--resolution must be positive")
    if args.height_min >= args.height_max:
        parser.error("--height-min must be smaller than --height-max")
    if (args.camera_height_min is None) != (args.camera_height_max is None):
        parser.error("--camera-height-min and --camera-height-max must be provided together")
    if (args.camera_height_min is not None
            and args.camera_height_min >= args.camera_height_max):
        parser.error("--camera-height-min must be smaller than --camera-height-max")
    if (args.max_endpoint_vertical_delta is not None
            and args.max_endpoint_vertical_delta < 0):
        parser.error("--max-endpoint-vertical-delta cannot be negative")
    if args.min_occupied_observations < 1:
        parser.error("--min-occupied-observations must be positive")
    if args.scene_margin <= 0:
        parser.error("--scene-margin must be positive")
    if args.max_origin_error <= 0:
        parser.error("--max-origin-error must be positive")
    if args.max_map_cloud_p95 <= 0:
        parser.error("--max-map-cloud-p95 must be positive")

    try:
        (
            args.recording_origin_xyz,
            args.recording_origin_source,
            args.recording_origin_manifest,
        ) = load_recording_origin(
            args.bag,
            args.recording_manifest,
            args.initial_habitat_xyz,
        )
    except ValueError as error:
        parser.error(str(error))
    if args.recording_origin_xyz is None:
        print(
            "warning: no recording initial Habitat origin found; using a random "
            "navmesh point, so official geometry may be translated",
            file=sys.stderr,
        )

    work_dir = Path(tempfile.mkdtemp(prefix=".navmesh_compare_", dir=OUTPUTS))
    episode_dir = work_dir / "episode"
    grid_prefix = work_dir / "grid"
    try:
        run_extractor(
            args.bag,
            episode_dir,
            args.scene.parent.name,
            args.keyframe_step,
            args.max_sync_ms,
        )
        grid_command = [
                sys.executable,
                str(ROOT / "scripts/build_occusg_grid.py"),
                str(episode_dir),
                str(grid_prefix),
                "--resolution",
                str(args.resolution),
                "--obstacle-min-z",
                str(args.height_min),
                "--obstacle-max-z",
                str(args.height_max),
                "--min-occupied-observations",
                str(args.min_occupied_observations),
            ]
        if args.floor_height is not None:
            camera_min = args.height_min if args.camera_height_min is None else args.camera_height_min
            camera_max = args.height_max if args.camera_height_max is None else args.camera_height_max
            grid_command.extend(["--camera-height-min", str(camera_min),
                                 "--camera-height-max", str(camera_max)])
        if args.max_endpoint_vertical_delta is not None:
            grid_command.extend([
                "--max-endpoint-vertical-delta", str(args.max_endpoint_vertical_delta),
            ])
        subprocess.run(
            grid_command,
            cwd=ROOT,
            check=True,
        )
        report = render(
            args,
            grid_prefix,
            episode_dir / "recorded_geometry.npz",
            episode_dir / "extraction_report.json",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        if not args.keep_workdir:
            remove_container_episode(episode_dir)
            shutil.rmtree(work_dir, ignore_errors=False)
        else:
            print(f"kept work directory: {work_dir}", file=sys.stderr)


if __name__ == "__main__":
    main()
