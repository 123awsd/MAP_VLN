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
SCENE_DIR = SCENE_ROOT / "00861-GLAQ4DNUx5U"
SCENE = SCENE_DIR / "GLAQ4DNUx5U.basis.glb"
SCENE_CONFIG = SCENE_ROOT / "hm3d_annotated_example_basis.scene_dataset_config.json"
DEFAULT_BRIDGE_DIR = ROOT / "runtime/bridge"

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


def read_command(bridge_dir: Path) -> Optional[dict]:
    try:
        return json.loads((bridge_dir / "command.json").read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None


def navmesh_path_step(pathfinder, current: np.ndarray, target: np.ndarray, max_step: float):
    """Take one short step along Habitat's ground shortest path.

    FALCON still chooses the target viewpoint. Habitat's navmesh is used only
    as the execution-side ground motion primitive, so a target on the other
    side of a wall cannot be reached by teleporting through the mesh.
    """
    snapped = np.asarray(pathfinder.snap_point(target.astype(np.float32)), dtype=np.float64)
    if snapped.shape != (3,) or not np.all(np.isfinite(snapped)):
        return None
    shortest = habitat_sim.ShortestPath()
    shortest.requested_start = current.astype(np.float32)
    shortest.requested_end = snapped.astype(np.float32)
    if not pathfinder.find_path(shortest) or not shortest.points:
        return None
    points = np.asarray(shortest.points, dtype=np.float64)
    if len(points) == 1:
        return points[0]
    remaining = max(0.0, float(max_step))
    anchor = current
    for point in points[1:]:
        segment = point - anchor
        length = float(np.linalg.norm(segment))
        if length <= 1e-6:
            anchor = point
            continue
        if length >= remaining:
            return anchor + segment * (remaining / length)
        remaining -= length
        anchor = point
    return points[-1]


def follow_command(
    sim,
    origin_h: np.ndarray,
    command: dict,
    dt: float,
    use_planner_yaw: bool = False,
    navmesh_constrained: bool = False,
    navmesh_path_follow: bool = False,
    allow_vertical_motion: bool = False,
) -> bool:
    agent = sim.get_agent(0)
    state = agent.get_state()
    current = np.asarray(state.position, dtype=np.float64)
    target_f = np.asarray(command["position"], dtype=np.float64) - np.asarray([0.0, 0.0, 1.0])
    target_h = origin_h + S_HABITAT_TO_FALCON.T @ target_f
    direction = target_h - current
    if not allow_vertical_motion:
        direction[1] = 0.0
    distance = float(np.linalg.norm(direction))
    moved = False
    if distance > 1e-4:
        if navmesh_path_follow:
            path_step = navmesh_path_step(sim.pathfinder, current, target_h, 1.0 * dt)
            next_position = current if path_step is None else path_step
        elif navmesh_constrained:
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
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--scene-config", type=Path, default=SCENE_CONFIG)
    parser.add_argument("--bridge-dir", type=Path, default=DEFAULT_BRIDGE_DIR)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--initial-position",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="explicit Habitat agent start generated from the same navmesh bounds",
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--follow-falcon", action="store_true")
    parser.add_argument(
        "--navmesh-constrained",
        action="store_true",
        help="use Habitat ground-agent navmesh instead of exact UAV command tracking",
    )
    parser.add_argument(
        "--navmesh-path-follow",
        action="store_true",
        help="follow each FALCON target along Habitat's ground shortest path",
    )
    parser.add_argument(
        "--allow-vertical-motion",
        action="store_true",
        help="track the full 3-D PositionCommand, including Habitat vertical motion",
    )
    parser.add_argument(
        "--use-planner-yaw",
        action="store_true",
        help="use FALCON yaw instead of facing the actual movement direction",
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
    if not args.scene.exists():
        raise FileNotFoundError(args.scene)
    if not args.scene_config.exists():
        raise FileNotFoundError(args.scene_config)
    bridge_dir = args.bridge_dir
    bridge_dir.mkdir(parents=True, exist_ok=True)
    if args.record_dir is not None:
        args.record_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = args.record_dir / "manifest.json"
        manifest_path.write_text(json.dumps({
            "format": "pre_map_vln.habitat_episode.v1", "scene": args.scene.stem,
            "scene_path": str(args.scene), "scene_config": str(args.scene_config),
            "seed": args.seed,
            "width": 640, "height": 480, "fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
            "coordinate_frame": "falcon_world_z_up_camera_optical",
        }, indent=2), encoding="utf-8")
    else:
        manifest_path = None

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id, sim_cfg.scene_dataset_config_file = str(args.scene), str(args.scene_config)
    sim_cfg.enable_physics = True
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor("rgb", habitat_sim.SensorType.COLOR), sensor("depth", habitat_sim.SensorType.DEPTH), sensor("semantic", habitat_sim.SensorType.SEMANTIC)]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("HM3D navmesh did not load")
        sim.pathfinder.seed(args.seed)
        initial = (
            sim.pathfinder.get_random_navigable_point()
            if args.initial_position is None
            else np.asarray(args.initial_position, dtype=np.float32)
        )
        if not sim.pathfinder.is_navigable(initial):
            raise RuntimeError(f"initial Habitat position is not navigable: {initial.tolist()}")
        state = habitat_sim.AgentState()
        state.position = initial
        state.rotation = quat_from_angle_axis(0.0, np.asarray([0.0, 1.0, 0.0]))
        agent = sim.initialize_agent(0, state)
        initial_sensor_h = np.asarray(agent.get_state().sensor_states["depth"].position, dtype=np.float64)
        if manifest_path is not None:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest.update({
                "initial_agent_habitat_xyz": np.asarray(initial, dtype=np.float64).tolist(),
                "initial_sensor_habitat_xyz": initial_sensor_h.tolist(),
                "main_floor_mapping": {
                    "habitat_y": float(initial[1]),
                    "falcon_sensor_z": 1.0,
                    "transform": S_HABITAT_TO_FALCON.tolist(),
                },
                "execution_mode": (
                    "habitat_navmesh_shortest_path"
                    if args.navmesh_path_follow
                    else "habitat_navmesh_try_step"
                    if args.navmesh_constrained
                    else "exact_falcon_command_tracking"
                ),
            })
            manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

        period, started, sequence, idle_time = 1.0 / args.hz, time.monotonic(), 0, 0.0
        completion_seen = None
        while time.monotonic() - started < args.duration:
            tick = time.monotonic()
            if args.completion_file is not None and args.completion_file.exists():
                if completion_seen is None:
                    completion_seen = tick
                    print(
                        f"FALCON completion detected; holding {args.finish_hold:.1f}s for final frames",
                        flush=True,
                    )
                elif tick - completion_seen >= args.finish_hold:
                    break
            if args.follow_falcon:
                command = read_command(bridge_dir)
                moved = False
                if command is not None:
                    moved = follow_command(
                        sim,
                        np.asarray(initial, dtype=np.float64),
                        command,
                        period,
                        use_planner_yaw=args.use_planner_yaw,
                        navmesh_constrained=args.navmesh_constrained,
                        navmesh_path_follow=args.navmesh_path_follow,
                        allow_vertical_motion=args.allow_vertical_motion,
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
            atomic_bytes(bridge_dir / "depth_u16.raw", depth_u16.tobytes())
            atomic_bytes(bridge_dir / "rgb_u8.raw", rgb.tobytes())
            atomic_bytes(bridge_dir / "semantic_i32.raw", semantic.tobytes())
            atomic_json(bridge_dir / "state.json", {
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
        termination = "complete" if completion_seen is not None else "timeout"
        result = {
            "termination": termination,
            "elapsed_seconds": elapsed,
            "frames": sequence,
            "max_duration_seconds": args.duration,
            "finish_hold_seconds": args.finish_hold,
        }
        if args.result_file is not None:
            args.result_file.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(args.result_file, result)
        print(
            f"terminated={termination} elapsed={elapsed:.1f}s frames={sequence} bridge={bridge_dir}",
            flush=True,
        )


if __name__ == "__main__":
    main()
