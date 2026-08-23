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


def follow_command(sim, origin_h: np.ndarray, command: dict, dt: float) -> None:
    agent = sim.get_agent(0)
    state = agent.get_state()
    current = np.asarray(state.position, dtype=np.float64)
    target_f = np.asarray(command["position"], dtype=np.float64) - np.asarray([0.0, 0.0, 1.0])
    target_h = origin_h + S_HABITAT_TO_FALCON.T @ target_f
    direction = target_h - current
    direction[1] = 0.0
    distance = float(np.linalg.norm(direction))
    if distance > 1e-4:
        desired = current + direction / distance * min(distance, 1.0 * dt)
        state.position = sim.pathfinder.try_step(current.astype(np.float32), desired.astype(np.float32))
    # FALCON yaw 0 faces +X; Habitat yaw 0 faces -Z, which maps to +X.
    state.rotation = quat_from_angle_axis(float(command.get("yaw", 0.0)), np.asarray([0.0, 1.0, 0.0]))
    agent.set_state(state)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--follow-falcon", action="store_true")
    parser.add_argument("--panorama", action="store_true", help="rotate in place for a 360-degree scan")
    parser.add_argument("--record-dir", type=Path, default=None)
    parser.add_argument("--record-every", type=int, default=5)
    args = parser.parse_args()
    if not SCENE.exists():
        raise FileNotFoundError(SCENE)
    BRIDGE_DIR.mkdir(parents=True, exist_ok=True)
    if args.record_dir is not None:
        args.record_dir.mkdir(parents=True, exist_ok=True)
        (args.record_dir / "manifest.json").write_text(json.dumps({
            "format": "pre_map_vln.habitat_episode.v1", "scene": "00861-GLAQ4DNUx5U",
            "width": 640, "height": 480, "fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
            "coordinate_frame": "falcon_world_z_up_camera_optical",
        }, indent=2), encoding="utf-8")

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id, sim_cfg.scene_dataset_config_file = str(SCENE), str(SCENE_CONFIG)
    sim_cfg.enable_physics = True
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor("rgb", habitat_sim.SensorType.COLOR), sensor("depth", habitat_sim.SensorType.DEPTH), sensor("semantic", habitat_sim.SensorType.SEMANTIC)]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("HM3D navmesh did not load")
        sim.pathfinder.seed(7)
        initial = sim.pathfinder.get_random_navigable_point()
        state = habitat_sim.AgentState()
        state.position = initial
        state.rotation = quat_from_angle_axis(0.0, np.asarray([0.0, 1.0, 0.0]))
        agent = sim.initialize_agent(0, state)
        initial_sensor_h = np.asarray(agent.get_state().sensor_states["depth"].position, dtype=np.float64)

        period, started, sequence = 1.0 / args.hz, time.monotonic(), 0
        while time.monotonic() - started < args.duration:
            tick = time.monotonic()
            if args.follow_falcon:
                command = read_command()
                if command is not None:
                    follow_command(sim, np.asarray(initial, dtype=np.float64), command, period)
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
        print(f"completed frames={sequence} bridge={BRIDGE_DIR}")


if __name__ == "__main__":
    main()
