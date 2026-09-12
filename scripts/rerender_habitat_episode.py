#!/usr/bin/env python3
"""Re-render a recorded Habitat episode at a different native resolution.

The camera poses and timestamps are copied from the source episode. RGB and
depth are rendered again from the HM3D mesh, allowing a controlled resolution
ablation without changing the exploration trajectory.
"""

import argparse
import json
from pathlib import Path

import habitat_sim
import magnum as mn
import numpy as np
from habitat_sim.utils.common import quat_from_coeffs
from habitat_sim.utils.common import quat_from_angle_axis


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
C_OPTICAL_TO_HABITAT_CAMERA = np.diag([1.0, -1.0, -1.0])


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=np.float64)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_xyzw(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w, x, y, z = (
            0.25 * s,
            (matrix[2, 1] - matrix[1, 2]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
            (matrix[1, 0] - matrix[0, 1]) / s,
        )
    else:
        i = int(np.argmax(np.diag(matrix)))
        if i == 0:
            s = np.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            w, x, y, z = (
                (matrix[2, 1] - matrix[1, 2]) / s,
                0.25 * s,
                (matrix[0, 1] + matrix[1, 0]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
            )
        elif i == 1:
            s = np.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            w, x, y, z = (
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[0, 1] + matrix[1, 0]) / s,
                0.25 * s,
                (matrix[1, 2] + matrix[2, 1]) / s,
            )
        else:
            s = np.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            w, x, y, z = (
                (matrix[1, 0] - matrix[0, 1]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
                (matrix[1, 2] + matrix[2, 1]) / s,
                0.25 * s,
            )
    result = np.asarray([x, y, z, w], dtype=np.float64)
    return result / np.linalg.norm(result)


def camera_sensor(uuid: str, sensor_type, width: int, height: int, hfov: float):
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = uuid
    spec.sensor_type = sensor_type
    spec.resolution = [height, width]
    spec.position = [0.0, 1.0, 0.0]
    spec.hfov = hfov
    return spec


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    parser.add_argument("--hfov", type=float, default=90.0)
    parser.add_argument("--skip", type=int, default=1)
    parser.add_argument("--start-index", type=int, default=0,
                        help="Zero-based index into sorted source frames.")
    parser.add_argument("--max-frames", type=int, default=99999)
    parser.add_argument("--gpu-device-id", type=int, default=0)
    parser.add_argument("--scene", type=Path, default=None,
                        help="Optional replacement scene GLB (keeps source poses/config).")
    parser.add_argument("--object-asset", type=Path, default=None)
    parser.add_argument("--object-position-f", type=float, nargs=3, default=None)
    parser.add_argument("--object-scale", type=float, default=1.0)
    parser.add_argument("--camera-position-f", type=float, nargs=3, default=None)
    parser.add_argument("--camera-target-f", type=float, nargs=3, default=None)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.skip <= 0 or args.max_frames <= 0:
        raise ValueError("width, height, skip and max-frames must be positive")

    source_manifest = json.loads((args.input / "manifest.json").read_text(encoding="utf-8"))
    all_frames = sorted(args.input.glob("frame_*.npz"))
    frames = all_frames[args.start_index:: args.skip][: args.max_frames]
    if not frames:
        raise FileNotFoundError(f"no frame_*.npz files found in {args.input}")
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(f"output directory is not empty: {args.output}")
    args.output.mkdir(parents=True, exist_ok=True)

    scene_path = (args.scene if args.scene is not None else Path(source_manifest["scene_path"])).resolve()
    scene_config = Path(source_manifest["scene_config"]).resolve()
    initial_sensor_h = np.asarray(source_manifest["initial_sensor_habitat_xyz"], dtype=np.float64)
    focal = 0.5 * args.width / np.tan(np.deg2rad(args.hfov) * 0.5)
    output_manifest = dict(source_manifest)
    output_manifest.update(
        {
            "format": "pre_map_vln.habitat_episode.rerender.v1",
            "source_episode": str(args.input.resolve()),
            "width": args.width,
            "height": args.height,
            "fx": focal,
            "fy": focal,
            "cx": args.width / 2.0,
            "cy": args.height / 2.0,
            "hfov": args.hfov,
            "source_frame_skip": args.skip,
            "frame_count": len(frames),
        }
    )
    (args.output / "manifest.json").write_text(
        json.dumps(output_manifest, indent=2), encoding="utf-8"
    )

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(scene_path)
    sim_cfg.scene_dataset_config_file = str(scene_config)
    sim_cfg.gpu_device_id = args.gpu_device_id
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [
        camera_sensor("rgb", habitat_sim.SensorType.COLOR, args.width, args.height, args.hfov),
        camera_sensor("depth", habitat_sim.SensorType.DEPTH, args.width, args.height, args.hfov),
    ]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        agent = sim.initialize_agent(0)
        if args.object_asset is not None:
            if args.object_position_f is None:
                raise ValueError("--object-position-f is required with --object-asset")
            attrs = habitat_sim.attributes.ObjectAttributes()
            attrs.render_asset_handle = str(args.object_asset.resolve())
            attrs.collision_asset_handle = str(args.object_asset.resolve())
            attrs.is_collidable = False
            attrs.scale = mn.Vector3(args.object_scale, args.object_scale, args.object_scale)
            tid = sim.get_object_template_manager().register_template(attrs, "preview_object")
            obj = sim.get_rigid_object_manager().add_object_by_template_id(tid)
            obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            p_f = np.asarray(args.object_position_f, dtype=np.float64)
            p_h = initial_sensor_h + S_HABITAT_TO_FALCON.T @ (p_f - np.asarray([0., 0., 1.]))
            obj.translation = mn.Vector3(*p_h)
        for output_index, source_path in enumerate(frames):
            with np.load(source_path) as source:
                position_f = np.asarray(source["position"], dtype=np.float64)
                rotation_f_opt = quat_xyzw_to_matrix(source["orientation_xyzw"])
                time_ns = int(source["time_ns"])
            state = habitat_sim.AgentState()
            if args.camera_position_f is not None:
                if args.camera_target_f is None:
                    raise ValueError("--camera-target-f is required with --camera-position-f")
                camera_f = np.asarray(args.camera_position_f, dtype=np.float64)
                target_f = np.asarray(args.camera_target_f, dtype=np.float64)
                sensor_position_h = initial_sensor_h + S_HABITAT_TO_FALCON.T @ (
                    camera_f - np.asarray([0.0, 0.0, 1.0]))
                target_h = initial_sensor_h + S_HABITAT_TO_FALCON.T @ (
                    target_f - np.asarray([0.0, 0.0, 1.0]))
                direction = target_h - sensor_position_h
                yaw = np.arctan2(-direction[0], -direction[2])
                state.rotation = quat_from_angle_axis(float(yaw), np.asarray([0., 1., 0.]))
            else:
                sensor_position_h = initial_sensor_h + S_HABITAT_TO_FALCON.T @ (
                    position_f - np.asarray([0.0, 0.0, 1.0])
                )
                rotation_h = S_HABITAT_TO_FALCON.T @ rotation_f_opt @ C_OPTICAL_TO_HABITAT_CAMERA.T
                state.rotation = quat_from_coeffs(matrix_to_xyzw(rotation_h))
            state.position = sensor_position_h - np.asarray([0.0, 1.0, 0.0])
            agent.set_state(state)
            observations = sim.get_sensor_observations()
            rgb = np.asarray(observations["rgb"])[..., :3].astype(np.uint8)
            depth = np.nan_to_num(np.asarray(observations["depth"], dtype=np.float32))
            np.savez_compressed(
                args.output / f"frame_{output_index:06d}.npz",
                rgb=rgb,
                depth_m=depth,
                position=position_f.astype(np.float32),
                orientation_xyzw=np.asarray(matrix_to_xyzw(rotation_f_opt), dtype=np.float32),
                time_ns=np.asarray(time_ns, dtype=np.int64),
            )
            print(f"[{output_index + 1}/{len(frames)}] {source_path.name}", flush=True)


if __name__ == "__main__":
    main()
