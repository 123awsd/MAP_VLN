#!/usr/bin/env python3
"""Feed synchronized HM3D RGB-D/semantic observations to the FALCON file bridge."""

import argparse
import json
import os
import time
from pathlib import Path
from typing import Optional

import habitat_sim
import numpy as np
from habitat_sim.utils.common import quat_from_angle_axis


ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = ROOT / "data/scene_datasets/hm3d/example"
DEFAULT_SCENE_DIR = SCENE_ROOT / "00861-GLAQ4DNUx5U"
DEFAULT_SCENE = DEFAULT_SCENE_DIR / "GLAQ4DNUx5U.basis.glb"
DEFAULT_SCENE_CONFIG = SCENE_ROOT / "hm3d_annotated_example_basis.scene_dataset_config.json"
BRIDGE_DIR = ROOT / "runtime/bridge"

# Habitat world (right, up, back) -> FALCON world (forward, left, up).
S_HABITAT_TO_FALCON = np.asarray([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
# Optical camera (right, down, forward) -> Habitat camera (right, up, back).
C_OPTICAL_TO_HABITAT_CAMERA = np.diag([1.0, -1.0, -1.0])


def atomic_bytes(path: Path, data: bytes) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_bytes(data)
    os.replace(temp, path)


def atomic_json(path: Path, value: dict) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")
    os.replace(temp, path)


def rotation_matrix(rotation) -> np.ndarray:
    # numpy-quaternion exposes scalar real and xyz imaginary components.
    w = float(rotation.real)
    x, y, z = np.asarray(rotation.imag, dtype=np.float64)
    return np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def matrix_to_xyzw(matrix: np.ndarray) -> list:
    # Stable branch-based conversion for a proper 3x3 rotation matrix.
    trace = float(np.trace(matrix))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w, x, y, z = 0.25 * s, (matrix[2, 1] - matrix[1, 2]) / s, (matrix[0, 2] - matrix[2, 0]) / s, (matrix[1, 0] - matrix[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(matrix)))
        if i == 0:
            s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            w, x, y, z = (matrix[2, 1] - matrix[1, 2]) / s, 0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            w, x, y, z = (matrix[0, 2] - matrix[2, 0]) / s, (matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            w, x, y, z = (matrix[1, 0] - matrix[0, 1]) / s, (matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s
    quat = np.asarray([x, y, z, w], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat.tolist()


def sensor(uuid: str, kind: habitat_sim.SensorType) -> habitat_sim.CameraSensorSpec:
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid, spec.sensor_type = uuid, kind
    spec.resolution, spec.position, spec.hfov = [480, 640], [0.0, 1.0, 0.0], 90.0
    return spec


def read_command() -> Optional[dict]:
    try:
        return json.loads((BRIDGE_DIR / "command.json").read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None


def read_completion_status(path: Path) -> Optional[str]:
    """Return a validated terminal status from the atomic bridge sentinel."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    status = payload.get("status")
    return status if status in {"complete", "collision_stall"} else None


def detect_navmesh_floor_levels(pathfinder, sample_count: int = 10000) -> list[float]:
    """Return one-based-floor candidates as Habitat Y heights, lowest first.

    Navigable samples form dense horizontal bands on each floor.  A smoothed
    histogram suppresses stairs and ramps without assuming a fixed number of
    floors.
    """
    sample_count = max(100, int(sample_count))
    samples = np.asarray(
        [pathfinder.get_random_navigable_point()[1] for _ in range(sample_count)],
        dtype=np.float64,
    )
    samples = samples[np.isfinite(samples)]
    if len(samples) < 100:
        return []
    bin_size = 0.10
    lower = np.floor(float(samples.min()) / bin_size) * bin_size - bin_size
    upper = np.ceil(float(samples.max()) / bin_size) * bin_size + bin_size
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
    selected = []
    min_bins = max(1, int(round(1.5 / bin_size)))
    for index in candidates:
        if all(abs(index - other) >= min_bins for other in selected):
            selected.append(index)
    return sorted(float((edges[index] + edges[index + 1]) * 0.5) for index in selected)


def choose_initial_point(pathfinder, floor_number: int, sample_count: int) -> tuple[np.ndarray, dict]:
    """Choose a reproducible, open navmesh point on a one-based floor."""
    sample_count = max(100, int(sample_count))
    points = np.asarray(
        [pathfinder.get_random_navigable_point() for _ in range(sample_count)],
        dtype=np.float64,
    )
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) == 0:
        raise RuntimeError("HM3D navmesh returned no finite navigable points")
    levels = detect_navmesh_floor_levels(pathfinder, sample_count)
    requested = int(floor_number)
    if requested < 1:
        requested = 1
    level_index = min(requested - 1, len(levels) - 1) if levels else None
    target_level = levels[level_index] if level_index is not None else float(points[:, 1].min())
    band = 0.35
    floor_points = points[np.abs(points[:, 1] - target_level) <= band]
    if len(floor_points) == 0:
        nearest = np.argsort(np.abs(points[:, 1] - target_level))[: max(20, min(200, len(points)))]
        floor_points = points[nearest]

    # Prefer the largest connected island on the requested floor, then the
    # point with maximum obstacle clearance within that island.
    island_ids = np.asarray([pathfinder.get_island(point) for point in floor_points], dtype=np.int64)
    unique_islands = np.unique(island_ids)
    island_areas = {int(island): float(pathfinder.island_area(int(island))) for island in unique_islands}
    largest_island = max(unique_islands, key=lambda island: island_areas[int(island)])
    island_points = floor_points[island_ids == largest_island]
    clearances = np.asarray(
        [pathfinder.distance_to_closest_obstacle(point) for point in island_points],
        dtype=np.float64,
    )
    best = int(np.nanargmax(np.where(np.isfinite(clearances), clearances, -np.inf)))
    selected = island_points[best]
    return selected, {
        "requested_floor": int(floor_number),
        "selected_floor": int(level_index + 1) if level_index is not None else 1,
        "detected_floor_levels_habitat_y": levels,
        "habitat_xyz": selected.tolist(),
        "obstacle_clearance_m": float(clearances[best]),
        "navmesh_island": int(largest_island),
        "navmesh_island_area_m2": island_areas[int(largest_island)],
        "candidate_count": int(len(island_points)),
    }


def follow_command(
    sim,
    origin_h: np.ndarray,
    command: dict,
    dt: float,
    use_planner_yaw: bool = False,
    use_planner_z: bool = False,
    navmesh_constrained: bool = False,
) -> bool:
    agent = sim.get_agent(0)
    state = agent.get_state()
    current = np.asarray(state.position, dtype=np.float64)
    target_f = np.asarray(command["position"], dtype=np.float64) - np.asarray([0.0, 0.0, 1.0])
    target_h = origin_h + S_HABITAT_TO_FALCON.T @ target_f
    direction = target_h - current
    if not use_planner_z:
        direction[1] = 0.0
    distance = float(np.linalg.norm(direction))
    moved = False
    if distance > 1e-4:
        if navmesh_constrained:
            desired = current + direction / distance * min(distance, 1.0 * dt)
            next_position = np.asarray(
                sim.pathfinder.try_step(
                    current.astype(np.float32), desired.astype(np.float32)
                ),
                dtype=np.float64,
            )
        else:
            # PositionCommand is the time-sampled desired UAV pose. Track it
            # directly: Habitat's navmesh is for a walking cylinder and blocks
            # valid FALCON flight across railings, stairs and open voids.
            next_position = target_h
        if not use_planner_z:
            # Compatibility mode for ground/single-floor experiments.
            next_position[1] = current[1]
        state.position = next_position
        moved = bool(np.linalg.norm(next_position - current) > 1e-4)
        if not use_planner_yaw:
            # Keep the optical camera facing the actual horizontal movement.  FALCON's
            # independent yaw command can lag the position trajectory and made Habitat
            # appear to fly backwards in first-person recordings.
            movement_f = S_HABITAT_TO_FALCON @ (next_position - current)
            if np.linalg.norm(movement_f[:2]) > 1e-4:
                heading = float(np.arctan2(movement_f[1], movement_f[0]))
                state.rotation = quat_from_angle_axis(
                    heading, np.asarray([0.0, 1.0, 0.0])
                )
    if use_planner_yaw:
        # FALCON yaw 0 faces +X; Habitat yaw 0 faces -Z, which maps to +X.
        state.rotation = quat_from_angle_axis(
            float(command.get("yaw", 0.0)), np.asarray([0.0, 1.0, 0.0])
        )
    agent.set_state(state)
    return moved


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--scene-config", type=Path, default=DEFAULT_SCENE_CONFIG)
    parser.add_argument(
        "--start-floor", type=int, default=1,
        help="one-based navmesh floor for the initial pose; floor 1 is the lowest",
    )
    parser.add_argument(
        "--start-samples", type=int, default=10000,
        help="navmesh samples used to find an open initial pose",
    )
    parser.add_argument("--follow-falcon", action="store_true")
    parser.add_argument(
        "--navmesh-constrained",
        action="store_true",
        help="use Habitat ground-agent navmesh instead of exact UAV command tracking",
    )
    parser.add_argument(
        "--use-planner-yaw",
        action="store_true",
        help="use FALCON yaw instead of facing the actual movement direction",
    )
    parser.add_argument(
        "--use-planner-z",
        action="store_true",
        help="track the vertical component of FALCON PositionCommand",
    )
    parser.add_argument(
        "--idle-scan-rate",
        type=float,
        default=0.0,
        help="camera yaw scan rate in degrees/s while the agent is stationary",
    )
    parser.add_argument(
        "--idle-scan-after",
        type=float,
        default=0.5,
        help="stationary delay in seconds before the camera starts scanning",
    )
    parser.add_argument("--panorama", action="store_true", help="rotate in place for a 360-degree scan")
    parser.add_argument("--record-dir", type=Path, default=None)
    parser.add_argument("--record-every", type=int, default=5)
    parser.add_argument(
        "--completion-file",
        type=Path,
        default=None,
        help="stop after this FALCON-completion sentinel appears",
    )
    parser.add_argument(
        "--finish-hold",
        type=float,
        default=3.0,
        help="seconds to keep publishing frames after FALCON enters FINISH",
    )
    parser.add_argument("--result-file", type=Path, default=None)
    args = parser.parse_args()
    if args.navmesh_constrained and args.use_planner_z:
        parser.error("--use-planner-z cannot be combined with --navmesh-constrained")
    if args.start_floor < 1:
        parser.error("--start-floor must be at least 1")
    if args.start_samples < 100:
        parser.error("--start-samples must be at least 100")
    scene = args.scene.resolve()
    scene_config = args.scene_config.resolve()
    if not scene.exists():
        raise FileNotFoundError(scene)
    if not scene_config.exists():
        raise FileNotFoundError(scene_config)
    scene_name = scene.parent.name
    BRIDGE_DIR.mkdir(parents=True, exist_ok=True)
    if args.record_dir is not None:
        args.record_dir.mkdir(parents=True, exist_ok=True)
        (args.record_dir / "manifest.json").write_text(json.dumps({
            "format": "pre_map_vln.habitat_episode.v1", "scene": scene_name,
            "scene_path": str(scene), "scene_config": str(scene_config),
            "width": 640, "height": 480, "fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
            "coordinate_frame": "falcon_world_z_up_camera_optical",
        }, indent=2), encoding="utf-8")

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id, sim_cfg.scene_dataset_config_file = str(scene), str(scene_config)
    sim_cfg.enable_physics = True
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor("rgb", habitat_sim.SensorType.COLOR), sensor("depth", habitat_sim.SensorType.DEPTH), sensor("semantic", habitat_sim.SensorType.SEMANTIC)]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("HM3D navmesh did not load")
        sim.pathfinder.seed(7)
        initial, initial_meta = choose_initial_point(
            sim.pathfinder, args.start_floor, args.start_samples
        )
        print(
            "initial_start floor=%d habitat_xyz=%s clearance=%.3f m candidates=%d"
            % (
                initial_meta["selected_floor"],
                np.round(initial, 3).tolist(),
                initial_meta["obstacle_clearance_m"],
                initial_meta["candidate_count"],
            ),
            flush=True,
        )
        state = habitat_sim.AgentState()
        state.position = initial
        state.rotation = quat_from_angle_axis(0.0, np.asarray([0.0, 1.0, 0.0]))
        agent = sim.initialize_agent(0, state)
        initial_sensor_h = np.asarray(agent.get_state().sensor_states["depth"].position, dtype=np.float64)
        if args.record_dir is not None:
            manifest_path = args.record_dir / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["initial_start"] = initial_meta
            manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

        period, started, sequence, idle_time = 1.0 / args.hz, time.monotonic(), 0, 0.0
        completion_seen = None
        completion_status = None
        while time.monotonic() - started < args.duration:
            tick = time.monotonic()
            if args.completion_file is not None and args.completion_file.exists():
                if completion_seen is None:
                    completion_status = read_completion_status(args.completion_file)
                    if completion_status is None:
                        # The bridge writes atomically, but tolerate an invalid
                        # or future sentinel without falsely declaring success.
                        time.sleep(max(0.0, period - (time.monotonic() - tick)))
                        continue
                    completion_seen = tick
                    print(
                        f"FALCON terminal status={completion_status}; "
                        f"holding {args.finish_hold:.1f}s for final frames",
                        flush=True,
                    )
                elif tick - completion_seen >= args.finish_hold:
                    break
            if args.follow_falcon:
                command = read_command()
                moved = False
                if command is not None:
                    moved = follow_command(
                        sim,
                        np.asarray(initial, dtype=np.float64),
                        command,
                        period,
                        use_planner_yaw=args.use_planner_yaw,
                        use_planner_z=args.use_planner_z,
                        navmesh_constrained=args.navmesh_constrained,
                    )
                idle_time = 0.0 if moved else idle_time + period
                if (
                    not moved
                    and not args.use_planner_yaw
                    and args.idle_scan_rate != 0.0
                    and idle_time >= args.idle_scan_after
                ):
                    # During a planning hover (including the initial wait for a
                    # command), keep collecting real Habitat observations by
                    # slowly scanning in place. Translation still faces actual
                    # motion, so this does not reintroduce the backwards view.
                    state = agent.get_state()
                    delta_yaw = np.deg2rad(args.idle_scan_rate) * period
                    state.rotation = quat_from_angle_axis(
                        delta_yaw, np.asarray([0.0, 1.0, 0.0])
                    ) * state.rotation
                    agent.set_state(state)
            elif args.panorama:
                state = agent.get_state()
                yaw = 2.0 * np.pi * min(1.0, (time.monotonic() - started) / args.duration)
                state.rotation = quat_from_angle_axis(yaw, np.asarray([0.0, 1.0, 0.0]))
                agent.set_state(state)
            obs = sim.get_sensor_observations()
            sensor_state = agent.get_state().sensor_states["depth"]
            position_h = np.asarray(sensor_state.position, dtype=np.float64)
            position_f = S_HABITAT_TO_FALCON @ (position_h - initial_sensor_h) + np.asarray([0.0, 0.0, 1.0])
            rotation_f_opt = S_HABITAT_TO_FALCON @ rotation_matrix(sensor_state.rotation) @ C_OPTICAL_TO_HABITAT_CAMERA
            rotation_f_body = S_HABITAT_TO_FALCON @ rotation_matrix(sensor_state.rotation) @ S_HABITAT_TO_FALCON.T

            depth = np.nan_to_num(np.asarray(obs["depth"], dtype=np.float32), nan=0.0, posinf=0.0)
            depth_u16 = np.clip(np.rint(depth * 1000.0), 0, 65535).astype("<u2")
            rgb = np.asarray(obs["rgb"])[..., :3].astype(np.uint8)
            semantic = np.asarray(obs["semantic"], dtype="<i4")
            atomic_bytes(BRIDGE_DIR / "depth_u16.raw", depth_u16.tobytes())
            atomic_bytes(BRIDGE_DIR / "rgb_u8.raw", rgb.tobytes())
            atomic_bytes(BRIDGE_DIR / "semantic_i32.raw", semantic.tobytes())
            atomic_json(BRIDGE_DIR / "state.json", {
                "sequence": sequence, "width": 640, "height": 480,
                "position": position_f.tolist(), "orientation_xyzw": matrix_to_xyzw(rotation_f_opt),
                "body_orientation_xyzw": matrix_to_xyzw(rotation_f_body),
                "depth_unit": "millimeter", "rgb_encoding": "rgb8", "semantic_dtype": "int32",
            })
            if args.record_dir is not None and sequence % args.record_every == 0:
                np.savez_compressed(
                    args.record_dir / f"frame_{sequence:06d}.npz",
                    rgb=rgb, depth_m=depth, semantic=semantic,
                    position=position_f.astype(np.float32),
                    orientation_xyzw=np.asarray(matrix_to_xyzw(rotation_f_opt), dtype=np.float32),
                    time_ns=np.asarray(time.time_ns(), dtype=np.int64),
                )
            sequence += 1
            if sequence % max(1, int(args.hz * 2)) == 0:
                print(f"frame={sequence} position_falcon={position_f.round(3).tolist()}", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
        elapsed = time.monotonic() - started
        termination = completion_status if completion_seen is not None else "timeout"
        result = {
            "termination": termination,
            "scene": scene_name,
            "elapsed_seconds": elapsed,
            "frames": sequence,
            "max_duration_seconds": args.duration,
            "finish_hold_seconds": args.finish_hold,
            "motion_mode": {
                "follow_falcon": args.follow_falcon,
                "planner_z": args.use_planner_z,
                "planner_yaw": args.use_planner_yaw,
                "navmesh_constrained": args.navmesh_constrained,
            },
            "initial_start": initial_meta,
        }
        if args.result_file is not None:
            args.result_file.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(args.result_file, result)
        print(
            f"terminated={termination} elapsed={elapsed:.1f}s frames={sequence} bridge={BRIDGE_DIR}",
            flush=True,
        )


if __name__ == "__main__":
    main()
