#!/usr/bin/env python3
"""Render a Stage 2 flight as a third-person video inside Habitat."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import habitat_sim
import imageio.v2 as imageio
import magnum as mn
import numpy as np
import cv2
from PIL import Image


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)
FALCON_INITIAL_POSITION = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a chase-camera video of a UAV flying in an HM3D scene."
    )
    parser.add_argument("--execution", required=True, type=Path)
    parser.add_argument("--generated-config", required=True, type=Path)
    parser.add_argument(
        "--drone-model",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "assets" / "drone" / "drone.glb",
        help="Drone GLB; defaults to the copy kept inside this project.",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--preview", type=Path)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--source-hz", type=float, default=5.0)
    parser.add_argument("--playback-rate", type=float, default=4.0)
    parser.add_argument("--hfov", type=float, default=76.0)
    parser.add_argument("--chase-distance", type=float, default=2.00)
    parser.add_argument("--chase-height", type=float, default=0.58)
    parser.add_argument(
        "--camera-smoothing",
        type=float,
        default=0.10,
        help="Per-frame follow fraction; smaller values make position/yaw smoother.",
    )
    parser.add_argument("--look-ahead", type=float, default=0.0)
    parser.add_argument("--look-height", type=float, default=0.02)
    parser.add_argument("--drone-scale", type=float, default=0.13)
    parser.add_argument(
        "--found-box-hold-s", type=float, default=3.0,
        help="Seconds to show a green 3D box after a target is found.",
    )
    parser.add_argument(
        "--show-found-boxes",
        action="store_true",
        help="Overlay green 3D boxes after target discoveries (disabled by default).",
    )
    parser.add_argument("--gpu-device-id", type=int, default=0)
    parser.add_argument(
        "--start-frame",
        type=int,
        default=0,
        help="Start at this source trajectory frame (useful for camera tests).",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Render only the first N frames for quick camera tests (0 renders all).",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def trajectory_in_habitat(execution: dict, generated: dict) -> tuple[np.ndarray, np.ndarray]:
    trajectory = np.asarray(execution["trajectory_xyz_yaw"], dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] < 4 or len(trajectory) < 1:
        raise ValueError("execution trajectory_xyz_yaw must be an Nx4 array")
    # The original first-person RGB is mounted 1 m above Habitat's ground agent.
    # Place the visual UAV at that sensor origin, not at the agent's floor position.
    initial_h = np.asarray(
        generated.get(
            "initial_sensor_habitat_xyz", generated["initial_agent_habitat_xyz"]
        ),
        dtype=np.float64,
    )
    positions_h = initial_h + (
        S_HABITAT_TO_FALCON.T
        @ (trajectory[:, :3] - FALCON_INITIAL_POSITION).T
    ).T
    return positions_h, trajectory[:, 3]


def falcon_points_in_habitat(points_f: np.ndarray, generated: dict) -> np.ndarray:
    points_f = np.asarray(points_f, dtype=np.float64).reshape(-1, 3)
    initial_h = np.asarray(
        generated.get("initial_sensor_habitat_xyz", generated["initial_agent_habitat_xyz"]),
        dtype=np.float64,
    )
    return initial_h + (
        S_HABITAT_TO_FALCON.T @ (points_f - FALCON_INITIAL_POSITION).T
    ).T


def forward_in_habitat(yaw: float) -> np.ndarray:
    forward_f = np.asarray([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
    return S_HABITAT_TO_FALCON.T @ forward_f


def limit_direction_change(previous, desired, max_angle):
    previous = np.asarray(previous, dtype=np.float64)
    desired = np.asarray(desired, dtype=np.float64)
    previous /= max(np.linalg.norm(previous), 1e-9)
    desired /= max(np.linalg.norm(desired), 1e-9)
    cosine = float(np.clip(np.dot(previous, desired), -1.0, 1.0))
    angle = math.acos(cosine)
    if angle <= max_angle:
        return desired
    axis = np.cross(previous, desired)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8:
        return previous
    axis /= axis_norm
    # Rodrigues rotation by the permitted step.
    step = max_angle
    return (
        previous * math.cos(step)
        + np.cross(axis, previous) * math.sin(step)
        + axis * np.dot(axis, previous) * (1.0 - math.cos(step))
    )


BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def found_boxes(execution, generated):
    boxes = []
    for observation in execution.get("observations", []):
        if observation.get("outcome") != "found":
            continue
        verification = observation.get("verification", {})
        projected = verification.get("owlv2", {}).get("projected_3d", [])
        confirmed = [item for item in projected if item.get("confirmed", False)]
        if not confirmed:
            confirmed = projected[:1]
        for item in confirmed:
            center_f = np.asarray(item.get("center", []), dtype=np.float64)
            size_f = np.asarray(item.get("size", []), dtype=np.float64)
            if center_f.shape != (3,) or size_f.shape != (3,):
                continue
            corners_f = np.asarray([
                center_f + np.asarray([sx, sy, sz]) * size_f * 0.5
                for sx, sy, sz in (
                    (-1, -1, -1), (-1, -1, 1), (-1, 1, 1), (-1, 1, -1),
                    (1, -1, -1), (1, -1, 1), (1, 1, 1), (1, 1, -1),
                )
            ])
            boxes.append({
                "frame": int(observation.get("frame_index", 0)),
                "label": str(item.get("label", verification.get("target_label", "target"))),
                "corners": falcon_points_in_habitat(corners_f, generated),
            })
    return boxes


def draw_found_boxes(rgb, boxes, camera_position, view_direction, hfov):
    height, width = rgb.shape[:2]
    world_up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    right = np.cross(view_direction, world_up)
    right /= max(np.linalg.norm(right), 1e-9)
    up = np.cross(right, view_direction)
    focal = 0.5 * width / math.tan(math.radians(hfov) * 0.5)
    output = rgb.copy()
    for box in boxes:
        projected = []
        for corner in box["corners"]:
            delta = corner - camera_position
            depth = float(np.dot(delta, view_direction))
            if depth <= 0.05:
                projected.append(None)
                continue
            px = focal * float(np.dot(delta, right)) / depth + width * 0.5
            py = -focal * float(np.dot(delta, up)) / depth + height * 0.5
            projected.append((int(round(px)), int(round(py))))
        for start, end in BOX_EDGES:
            if projected[start] is None or projected[end] is None:
                continue
            cv2.line(output, projected[start], projected[end], (45, 235, 80), 3, cv2.LINE_AA)
        visible = [point for point in projected if point is not None]
        if visible:
            x0 = max(0, min(point[0] for point in visible))
            y0 = max(22, min(point[1] for point in visible))
            cv2.putText(output, f"{box['label']}  FOUND", (x0, y0 - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (45, 235, 80), 2, cv2.LINE_AA)
    return output


def yaw_quaternion(yaw: float) -> mn.Quaternion:
    return mn.Quaternion.rotation(mn.Rad(yaw), mn.Vector3.y_axis())


def collision_aware_camera(
    sim: habitat_sim.Simulator,
    position: np.ndarray,
    forward: np.ndarray,
    chase_distance: float,
    chase_height: float,
) -> tuple[np.ndarray, float, float]:
    """Keep a fixed rear chase camera; only shorten distance when obstructed."""
    up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    ray_origin = position + 0.10 * up
    desired = position - chase_distance * forward + chase_height * up
    ray_vector = desired - ray_origin
    desired_distance = float(np.linalg.norm(ray_vector))
    ray_direction = ray_vector / desired_distance
    result = sim.cast_ray(
        habitat_sim.geo.Ray(
            ray_origin.astype(np.float32), ray_direction.astype(np.float32)
        ),
        desired_distance,
        0.0,
    )
    hit_distance = desired_distance
    if result.has_hits():
        positive_hits = [
            float(hit.ray_distance)
            for hit in result.hits
            if float(hit.ray_distance) > 0.05
        ]
        if positive_hits:
            hit_distance = min(hit_distance, min(positive_hits))
    usable_distance = max(0.35, min(desired_distance, hit_distance - 0.12))
    return ray_origin + usable_distance * ray_direction, 0.0, usable_distance


def register_drone(sim: habitat_sim.Simulator, model_path: Path, scale: float):
    attributes = habitat_sim.attributes.ObjectAttributes()
    attributes.render_asset_handle = str(model_path)
    attributes.collision_asset_handle = str(model_path)
    attributes.scale = mn.Vector3(scale, scale, scale)
    attributes.is_collidable = False
    template_manager = sim.get_object_template_manager()
    template_id = template_manager.register_template(attributes, "third_person_drone")
    if template_id < 0:
        raise RuntimeError(f"Habitat could not register drone model: {model_path}")
    drone = sim.get_rigid_object_manager().add_object_by_template_id(template_id)
    if drone is None:
        raise RuntimeError(f"Habitat could not instantiate drone model: {model_path}")
    drone.motion_type = habitat_sim.physics.MotionType.KINEMATIC
    return drone


def main() -> None:
    args = parse_args()
    if args.source_hz <= 0.0 or args.playback_rate <= 0.0:
        raise ValueError("source-hz and playback-rate must be positive")
    if not 0.0 < args.camera_smoothing <= 1.0:
        raise ValueError("camera-smoothing must be in (0, 1]")
    output_fps = args.source_hz * args.playback_rate
    if abs(output_fps - round(output_fps)) > 1e-6:
        raise ValueError("source-hz * playback-rate must produce an integer output FPS")
    output_fps = int(round(output_fps))

    execution = load_json(args.execution.resolve())
    generated = load_json(args.generated_config.resolve())
    scene_path = Path(generated["scene"]).resolve()
    scene_config = Path(generated["scene_config"]).resolve()
    model_path = args.drone_model.resolve()
    for required in (scene_path, scene_config, model_path):
        if not required.is_file():
            raise FileNotFoundError(required)

    positions_h, yaws = trajectory_in_habitat(execution, generated)
    discovered_boxes = found_boxes(execution, generated) if args.show_found_boxes else []
    if args.start_frame < 0 or args.start_frame >= len(positions_h):
        raise ValueError("start-frame is outside the trajectory")
    positions_h = positions_h[args.start_frame :]
    yaws = yaws[args.start_frame :]
    if args.max_frames > 0:
        positions_h = positions_h[: args.max_frames]
        yaws = yaws[: args.max_frames]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    preview_path = args.preview or args.output.with_suffix(".preview.png")
    preview_path.parent.mkdir(parents=True, exist_ok=True)

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(scene_path)
    sim_cfg.scene_dataset_config_file = str(scene_config)
    sim_cfg.enable_physics = True
    sim_cfg.gpu_device_id = args.gpu_device_id

    sensor_spec = habitat_sim.CameraSensorSpec()
    sensor_spec.uuid = "third_person_rgb"
    sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    sensor_spec.resolution = [args.height, args.width]
    sensor_spec.position = mn.Vector3(0.0, 0.0, 0.0)
    sensor_spec.hfov = args.hfov
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_spec]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        agent = sim.initialize_agent(0)
        drone = register_drone(sim, model_path, args.drone_scale)
        # The model is +Y-up/+Z-forward; Habitat is +Y-up/-Z-forward.
        model_forward_correction = mn.Quaternion.rotation(
            mn.Rad(math.pi), mn.Vector3.y_axis()
        )

        writer = imageio.get_writer(
            args.output,
            fps=output_fps,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_log_level="error",
            output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
        )
        middle_index = len(positions_h) // 2
        smoothed_camera_position = None
        smoothed_view_direction = None
        box_hold_frames = max(1, int(round(args.found_box_hold_s * args.source_hz)))
        try:
            for index, (position, yaw) in enumerate(zip(positions_h, yaws)):
                forward = forward_in_habitat(float(yaw))
                desired_camera_position, camera_angle, camera_clearance = collision_aware_camera(
                    sim,
                    position,
                    forward,
                    args.chase_distance,
                    args.chase_height,
                )
                if smoothed_camera_position is None:
                    smoothed_camera_position = desired_camera_position.copy()
                else:
                    smoothed_camera_position += args.camera_smoothing * (
                        desired_camera_position - smoothed_camera_position
                    )
                camera_position = smoothed_camera_position
                target = (
                    position
                    + args.look_ahead * forward
                    + np.asarray([0.0, args.look_height, 0.0])
                )
                desired_view_direction = target - camera_position
                desired_view_direction /= max(np.linalg.norm(desired_view_direction), 1e-9)
                # Do not cap angular speed: the view follows the UAV yaw directly.
                # Position smoothing remains enabled to suppress translational jitter.
                smoothed_view_direction = desired_view_direction
                target = camera_position + smoothed_view_direction

                drone.translation = mn.Vector3(*position)
                drone.rotation = yaw_quaternion(float(yaw)) * model_forward_correction

                camera_mn = mn.Vector3(*camera_position)
                target_mn = mn.Vector3(*target)
                agent.scene_node.translation = camera_mn
                agent.scene_node.rotation = mn.Quaternion.from_matrix(
                    mn.Matrix4.look_at(
                        camera_mn, target_mn, mn.Vector3.y_axis()
                    ).rotation()
                )

                if index == 0:
                    actual_camera = np.asarray(agent.scene_node.absolute_translation)
                    print(
                        "camera_distance_m="
                        f"{np.linalg.norm(actual_camera - position):.3f} "
                        f"camera_angle_deg={camera_angle:.0f} "
                        f"clearance_m={camera_clearance:.3f} "
                        f"position={position.tolist()} yaw={float(yaw):.3f} "
                        f"drone_aabb={drone.aabb}",
                        flush=True,
                    )

                rgba = np.asarray(sim.get_sensor_observations()["third_person_rgb"])
                rgb = np.ascontiguousarray(rgba[..., :3].astype(np.uint8))
                active_boxes = [
                    box for box in discovered_boxes
                    if box["frame"] <= args.start_frame + index
                    < box["frame"] + box_hold_frames
                ]
                if active_boxes:
                    rgb = draw_found_boxes(
                        rgb, active_boxes, camera_position,
                        smoothed_view_direction, args.hfov,
                    )
                writer.append_data(rgb)
                if index == middle_index:
                    Image.fromarray(rgb).save(preview_path)
                if index == 0 or (index + 1) % max(1, len(positions_h) // 10) == 0:
                    print(f"rendered {index + 1}/{len(positions_h)}", flush=True)
        finally:
            writer.close()

    manifest = {
        "format": "pre_map_vln.habitat_third_person_video.v1",
        "execution": str(args.execution.resolve()),
        "generated_config": str(args.generated_config.resolve()),
        "scene": str(scene_path),
        "drone_model": str(model_path),
        "source_pose_count": len(positions_h),
        "resolution": [args.width, args.height],
        "fps": output_fps,
        "playback_rate": args.playback_rate,
        "duration_s": len(positions_h) / output_fps,
        "chase_distance_m": args.chase_distance,
        "chase_height_m": args.chase_height,
        "camera_smoothing": args.camera_smoothing,
        "drone_scale": args.drone_scale,
        "found_box_hold_s": args.found_box_hold_s,
        "found_box_count": len(discovered_boxes),
        "show_found_boxes": args.show_found_boxes,
        "video": str(args.output.resolve()),
        "preview": str(preview_path.resolve()),
    }
    manifest_path = args.output.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"video={args.output}")
    print(f"preview={preview_path}")
    print(f"manifest={manifest_path}")


if __name__ == "__main__":
    main()
