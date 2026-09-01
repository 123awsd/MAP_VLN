#!/usr/bin/env python3
"""Render a Stage2 executed trajectory over its textured HM3D scene."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import habitat_sim
import imageio.v2 as imageio
import magnum as mn
import numpy as np
from PIL import Image


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)
FALCON_INITIAL_POSITION = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
BACKGROUND_COLORS = {
    "white": mn.Color4(1.0, 1.0, 1.0, 1.0),
    "light-gray": mn.Color4(0.92, 0.92, 0.92, 1.0),
    "black": mn.Color4(0.0, 0.0, 0.0, 1.0),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a 360-degree textured HM3D view with a Stage2 trajectory."
    )
    parser.add_argument("--execution", required=True, type=Path)
    parser.add_argument("--generated-config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--frames", type=int, default=360)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--hfov", type=float, default=72.0)
    parser.add_argument("--elevation-deg", type=float, default=24.0)
    parser.add_argument(
        "--target-height-offset-m",
        type=float,
        default=0.0,
        help="Vertical offset of the orbit look-at target; negative values move the scene up.",
    )
    parser.add_argument(
        "--orbit-padding",
        type=float,
        default=0.86,
        help="Camera distance multiplier; smaller values make the scene larger.",
    )
    parser.add_argument("--trajectory-radius", type=float, default=0.015)
    parser.add_argument("--trajectory-line-width", type=int, default=2)
    parser.add_argument("--trajectory-outline-width", type=int, default=4)
    parser.add_argument("--task-marker-radius", type=int, default=4)
    parser.add_argument("--endpoint-marker-radius", type=int, default=4)
    parser.add_argument("--frustum-depth", type=float, default=1.0)
    parser.add_argument("--frustum-line-width", type=int, default=2)
    parser.add_argument("--observation-hfov", type=float, default=90.0)
    parser.add_argument(
        "--start-hold-fraction",
        type=float,
        default=0.04,
        help="Fraction of the video held before the trajectory begins growing.",
    )
    parser.add_argument(
        "--end-hold-fraction",
        type=float,
        default=0.08,
        help="Fraction of the video held after the full trajectory is revealed.",
    )
    parser.add_argument(
        "--static-trajectory",
        action="store_true",
        help="Show the full trajectory for the entire orbit instead of revealing it.",
    )
    parser.add_argument("--gpu-device-id", type=int, default=0)
    parser.add_argument(
        "--background",
        choices=sorted(BACKGROUND_COLORS),
        default="white",
    )
    parser.add_argument(
        "--no-xray-overlay",
        action="store_true",
        help="Only render the depth-tested Habitat trajectory tube.",
    )
    parser.add_argument(
        "--no-caption",
        action="store_true",
        help="Render without the title and exploration progress panel.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def trajectory_in_habitat(execution: dict, generated: dict) -> np.ndarray:
    trajectory_f = np.asarray(execution["trajectory_xyz_yaw"], dtype=np.float64)
    if trajectory_f.ndim != 2 or trajectory_f.shape[0] < 2 or trajectory_f.shape[1] < 3:
        raise ValueError("execution trajectory_xyz_yaw must contain at least two XYZ poses")
    initial_agent_h = np.asarray(generated["initial_agent_habitat_xyz"], dtype=np.float64)
    return initial_agent_h + (
        S_HABITAT_TO_FALCON.T
        @ (trajectory_f[:, :3] - FALCON_INITIAL_POSITION).T
    ).T


def falcon_points_in_habitat(points_f: np.ndarray, generated: dict) -> np.ndarray:
    points_f = np.asarray(points_f, dtype=np.float64).reshape(-1, 3)
    initial_agent_h = np.asarray(generated["initial_agent_habitat_xyz"], dtype=np.float64)
    return initial_agent_h + (
        S_HABITAT_TO_FALCON.T
        @ (points_f - FALCON_INITIAL_POSITION).T
    ).T


def observation_frustum_in_falcon(
    pose: list[float], depth: float, hfov_deg: float, aspect: float = 4.0 / 3.0
) -> np.ndarray:
    x_value, y_value, z_value, yaw = map(float, pose)
    origin = np.asarray([x_value, y_value, z_value], dtype=np.float64)
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
    left = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0], dtype=np.float64)
    up = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    half_width = depth * math.tan(math.radians(hfov_deg) * 0.5)
    half_height = half_width / aspect
    center = origin + depth * forward
    corners = [
        center + side * half_width * left + vertical * half_height * up
        for side, vertical in ((1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0))
    ]
    return np.vstack([origin, corners])


def completion_events_in_habitat(
    execution: dict,
    generated: dict,
    frustum_depth: float,
    observation_hfov: float,
) -> list[dict]:
    events = []
    for observation in execution.get("observations", []):
        if observation.get("outcome") != "found" or "pose" not in observation:
            continue
        pose = observation["pose"]
        point_h = falcon_points_in_habitat(np.asarray([pose[:3]]), generated)[0]
        frustum_f = observation_frustum_in_falcon(
            pose, frustum_depth, observation_hfov
        )
        events.append(
            {
                "trajectory_index": int(observation.get("frame_index", 0)),
                "point": point_h,
                "frustum": falcon_points_in_habitat(frustum_f, generated),
            }
        )
    return events


def remove_repeated_points(points: np.ndarray, epsilon: float = 1e-4) -> np.ndarray:
    keep = np.ones(len(points), dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1) > epsilon
    return points[keep]


def color_for_height(value: float, low: float, high: float) -> tuple[int, int, int]:
    ratio = float(np.clip((value - low) / max(high - low, 1e-6), 0.0, 1.0))
    stops = (
        (0.00, np.asarray([255, 185, 35], dtype=np.float64)),
        (0.50, np.asarray([40, 225, 255], dtype=np.float64)),
        (1.00, np.asarray([255, 75, 205], dtype=np.float64)),
    )
    for (left_t, left), (right_t, right) in zip(stops, stops[1:]):
        if ratio <= right_t:
            local = (ratio - left_t) / (right_t - left_t)
            rgb = left + local * (right - left)
            return tuple(int(round(channel)) for channel in rgb)
    return tuple(int(channel) for channel in stops[-1][1])


def project_points(
    points: np.ndarray,
    camera_position: np.ndarray,
    target: np.ndarray,
    width: int,
    height: int,
    hfov_deg: float,
) -> tuple[np.ndarray, np.ndarray]:
    forward = target - camera_position
    forward /= np.linalg.norm(forward)
    world_up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    camera_up = np.cross(right, forward)

    relative = points - camera_position
    depth = relative @ forward
    focal = 0.5 * width / math.tan(math.radians(hfov_deg) * 0.5)
    pixels = np.empty((len(points), 2), dtype=np.float64)
    pixels[:, 0] = width * 0.5 + (relative @ right) * focal / np.maximum(depth, 1e-6)
    pixels[:, 1] = height * 0.5 - (relative @ camera_up) * focal / np.maximum(depth, 1e-6)
    return pixels, depth


def draw_xray_trajectory(
    rgb: np.ndarray,
    points: np.ndarray,
    camera_position: np.ndarray,
    target: np.ndarray,
    hfov_deg: float,
    line_width: int,
    outline_width: int,
    completion_points: np.ndarray,
    task_marker_radius: int,
    endpoint_marker_radius: int,
    frustums: list[np.ndarray],
    frustum_line_width: int,
    color_height_range: tuple[float, float],
    show_current_position: bool,
) -> np.ndarray:
    height, width = rgb.shape[:2]
    pixels, depth = project_points(points, camera_position, target, width, height, hfov_deg)
    canvas = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    overlay = canvas.copy()
    path_low, path_high = color_height_range

    visible_segments = []
    for index in range(len(points) - 1):
        if depth[index] <= 0.05 or depth[index + 1] <= 0.05:
            continue
        start = tuple(np.rint(pixels[index]).astype(int))
        end = tuple(np.rint(pixels[index + 1]).astype(int))
        visible_segments.append((index, start, end))
        cv2.line(overlay, start, end, (20, 20, 20), outline_width, cv2.LINE_AA)

    # Draw the colored cores in a separate pass. Otherwise the outline of a
    # dense following segment repeatedly paints over the preceding segment.
    for index, start, end in visible_segments:
        color_rgb = color_for_height(
            0.5 * (points[index, 1] + points[index + 1, 1]), path_low, path_high
        )
        cv2.line(overlay, start, end, color_rgb[::-1], line_width, cv2.LINE_AA)

    frustum_edges = (
        (0, 1), (0, 2), (0, 3), (0, 4),
        (1, 2), (2, 3), (3, 4), (4, 1),
    )
    projected_frustums = []
    for frustum in frustums:
        frustum_pixels, frustum_depth = project_points(
            frustum, camera_position, target, width, height, hfov_deg
        )
        for start_index, end_index in frustum_edges:
            if frustum_depth[start_index] <= 0.05 or frustum_depth[end_index] <= 0.05:
                continue
            start = tuple(np.rint(frustum_pixels[start_index]).astype(int))
            end = tuple(np.rint(frustum_pixels[end_index]).astype(int))
            projected_frustums.append((start, end))
            cv2.line(
                overlay,
                start,
                end,
                (245, 245, 245),
                frustum_line_width + 2,
                cv2.LINE_AA,
            )
    for start, end in projected_frustums:
        cv2.line(
            overlay, start, end, (210, 105, 35), frustum_line_width, cv2.LINE_AA
        )

    if len(completion_points):
        task_pixels, task_depth = project_points(
            completion_points, camera_position, target, width, height, hfov_deg
        )
        for task_pixel, task_z in zip(task_pixels, task_depth):
            if task_z <= 0.05:
                continue
            center = tuple(np.rint(task_pixel).astype(int))
            cv2.circle(
                overlay, center, task_marker_radius + 2, (20, 20, 20), -1, cv2.LINE_AA
            )
            cv2.circle(
                overlay, center, task_marker_radius + 1, (245, 245, 245), -1, cv2.LINE_AA
            )
            cv2.circle(
                overlay, center, task_marker_radius, (35, 125, 245), -1, cv2.LINE_AA
            )

    # Start is fixed and the red head advances with the revealed trajectory.
    # At the final frame the moving head is also the true endpoint.
    endpoint_markers = [(0, (60, 220, 70))]
    if show_current_position and len(points) > 1:
        endpoint_markers.append((-1, (45, 55, 245)))
    for point_index, color_bgr in endpoint_markers:
        center = tuple(np.rint(pixels[point_index]).astype(int))
        cv2.circle(
            overlay, center, endpoint_marker_radius + 2, (20, 20, 20), -1, cv2.LINE_AA
        )
        cv2.circle(overlay, center, endpoint_marker_radius, color_bgr, -1, cv2.LINE_AA)

    composed = cv2.addWeighted(overlay, 0.92, canvas, 0.08, 0.0)
    return cv2.cvtColor(composed, cv2.COLOR_BGR2RGB)


def add_caption(
    rgb: np.ndarray,
    execution: dict,
    scene_name: str,
    light_theme: bool,
    revealed_pose_count: int,
) -> np.ndarray:
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    height, width = bgr.shape[:2]
    panel = bgr.copy()
    panel_color = (250, 250, 250) if light_theme else (15, 18, 24)
    title_color = (28, 28, 28) if light_theme else (255, 255, 255)
    subtitle_color = (80, 80, 80) if light_theme else (215, 225, 235)
    panel_right = min(width - 22, 680)
    cv2.rectangle(panel, (22, 20), (panel_right, 92), panel_color, -1)
    bgr = cv2.addWeighted(panel, 0.88 if light_theme else 0.68, bgr, 0.12 if light_theme else 0.32, 0.0)
    cv2.rectangle(bgr, (22, 20), (panel_right, 92), (205, 205, 205), 1)
    cv2.putText(
        bgr,
        f"HM3D {scene_name} | Executed task trajectory",
        (42, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        title_color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        bgr,
        f"Exploration {revealed_pose_count}/{len(execution['trajectory_xyz_yaw'])} poses"
        f"  |  {float(execution.get('path_length_m', 0.0)):.2f} m total",
        (42, 79),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        subtitle_color,
        1,
        cv2.LINE_AA,
    )
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def render(args: argparse.Namespace) -> None:
    if args.width <= 0 or args.height <= 0 or args.frames <= 0 or args.fps <= 0:
        raise ValueError("width, height, frames and fps must be positive")
    if args.trajectory_line_width <= 0:
        raise ValueError("trajectory line width must be positive")
    if args.trajectory_outline_width < args.trajectory_line_width:
        raise ValueError("trajectory outline width must not be thinner than the line")
    if args.task_marker_radius <= 0:
        raise ValueError("task marker radius must be positive")
    if args.endpoint_marker_radius <= 0:
        raise ValueError("endpoint marker radius must be positive")
    if args.frustum_depth <= 0 or args.frustum_line_width <= 0:
        raise ValueError("frustum depth and line width must be positive")
    if args.start_hold_fraction < 0 or args.end_hold_fraction < 0:
        raise ValueError("trajectory hold fractions must be non-negative")
    if args.start_hold_fraction + args.end_hold_fraction >= 1.0:
        raise ValueError("trajectory hold fractions must sum to less than one")

    execution = load_json(args.execution.resolve())
    generated = load_json(args.generated_config.resolve())
    scene_path = Path(generated["scene"]).resolve()
    if not scene_path.is_file():
        raise FileNotFoundError(scene_path)

    # Keep the source indexing intact: observation frame_index values refer to
    # this exact pose sequence and drive marker/frustum reveal timing.
    points = trajectory_in_habitat(execution, generated)
    completion_events = completion_events_in_habitat(
        execution, generated, args.frustum_depth, args.observation_hfov
    )
    completion_marker_count = int(sum(
        np.linalg.norm(event["point"] - points[-1]) > 0.10
        for event in completion_events
    ))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    video_path = args.output_dir / "textured_trajectory_turntable.mp4"
    preview_path = args.output_dir / "textured_trajectory_preview.png"
    manifest_path = args.output_dir / "render_manifest.json"

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(scene_path)
    sim_cfg.enable_physics = True
    sim_cfg.gpu_device_id = args.gpu_device_id

    sensor_spec = habitat_sim.CameraSensorSpec()
    sensor_spec.uuid = "orbit_rgb"
    sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    sensor_spec.resolution = [args.height, args.width]
    sensor_spec.position = mn.Vector3(0.0, 0.0, 0.0)
    sensor_spec.hfov = args.hfov
    sensor_spec.clear_color = BACKGROUND_COLORS[args.background]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_spec]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
        scene_center = np.asarray(scene_bb.center(), dtype=np.float64)
        scene_size = np.asarray(scene_bb.size(), dtype=np.float64)
        target = scene_center.copy()
        target[1] += args.target_height_offset_m

        # The native tube can only represent the complete path. Use it for the
        # depth-tested-only mode; the normal overlay mode draws a growing path.
        if args.no_xray_overlay:
            trajectory_points = [mn.Vector3(*point) for point in remove_repeated_points(points)]
            sim.add_trajectory_object(
                "executed_task_trajectory",
                trajectory_points,
                num_segments=12,
                radius=args.trajectory_radius,
                color=mn.Color4(1.0, 0.18, 0.03, 1.0),
                smooth=False,
            )

        vfov = 2.0 * math.atan(
            math.tan(math.radians(args.hfov) * 0.5) * args.height / args.width
        )
        bounding_radius = 0.5 * float(np.linalg.norm(scene_size))
        orbit_distance = args.orbit_padding * bounding_radius / math.sin(vfov * 0.5)
        elevation = math.radians(args.elevation_deg)
        horizontal_radius = orbit_distance * math.cos(elevation)
        vertical_offset = orbit_distance * math.sin(elevation)

        writer = imageio.get_writer(
            video_path,
            fps=args.fps,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_log_level="error",
            output_params=["-movflags", "+faststart"],
        )
        try:
            for frame_index in range(args.frames):
                normalized_time = (
                    1.0 if args.frames == 1 else frame_index / (args.frames - 1)
                )
                if args.static_trajectory:
                    reveal_fraction = 1.0
                else:
                    reveal_fraction = float(np.clip(
                        (normalized_time - args.start_hold_fraction)
                        / (1.0 - args.start_hold_fraction - args.end_hold_fraction),
                        0.0,
                        1.0,
                    ))
                revealed_pose_count = (
                    len(points)
                    if reveal_fraction >= 1.0
                    else max(1, 1 + int(math.floor(reveal_fraction * (len(points) - 1))))
                )
                revealed_points = points[:revealed_pose_count]
                reached_events = [
                    event
                    for event in completion_events
                    if event["trajectory_index"] < revealed_pose_count
                ]
                visible_task_points = np.asarray(
                    [
                        event["point"]
                        for event in reached_events
                        if np.linalg.norm(event["point"] - points[-1]) > 0.10
                    ],
                    dtype=np.float64,
                ).reshape(-1, 3)
                visible_frustums = [event["frustum"] for event in reached_events]

                angle = 2.0 * math.pi * frame_index / args.frames
                camera_position = target + np.asarray(
                    [
                        horizontal_radius * math.cos(angle),
                        vertical_offset,
                        horizontal_radius * math.sin(angle),
                    ],
                    dtype=np.float64,
                )
                camera_mn = mn.Vector3(*camera_position)
                target_mn = mn.Vector3(*target)
                agent_node = sim.agents[0].scene_node
                agent_node.translation = camera_mn
                agent_node.rotation = mn.Quaternion.from_matrix(
                    mn.Matrix4.look_at(
                        camera_mn, target_mn, mn.Vector3(0.0, 1.0, 0.0)
                    ).rotation()
                )

                rgba = np.asarray(sim.get_sensor_observations()["orbit_rgb"])
                rgb = np.ascontiguousarray(rgba[..., :3].astype(np.uint8))
                if not args.no_xray_overlay:
                    rgb = draw_xray_trajectory(
                        rgb,
                        revealed_points,
                        camera_position,
                        target,
                        args.hfov,
                        args.trajectory_line_width,
                        args.trajectory_outline_width,
                        visible_task_points,
                        args.task_marker_radius,
                        args.endpoint_marker_radius,
                        visible_frustums,
                        args.frustum_line_width,
                        (float(points[:, 1].min()), float(points[:, 1].max())),
                        show_current_position=revealed_pose_count > 1,
                    )
                if not args.no_caption:
                    rgb = add_caption(
                        rgb,
                        execution,
                        scene_path.parent.name,
                        light_theme=args.background != "black",
                        revealed_pose_count=revealed_pose_count,
                    )
                if frame_index == args.frames - 1:
                    Image.fromarray(rgb).save(preview_path)
                writer.append_data(rgb)
                if frame_index == 0 or (frame_index + 1) % max(1, args.frames // 10) == 0:
                    print(f"rendered {frame_index + 1}/{args.frames}", flush=True)
        finally:
            writer.close()

    manifest = {
        "format": "pre_map_vln.textured_trajectory_render.v1",
        "execution": str(args.execution.resolve()),
        "generated_config": str(args.generated_config.resolve()),
        "scene": str(scene_path),
        "source_pose_count": len(execution["trajectory_xyz_yaw"]),
        "rendered_trajectory_point_count": len(points),
        "path_length_m": float(execution.get("path_length_m", 0.0)),
        "resolution": [args.width, args.height],
        "frames": args.frames,
        "fps": args.fps,
        "duration_s": args.frames / args.fps,
        "xray_overlay": not args.no_xray_overlay,
        "background": args.background,
        "orbit_padding": args.orbit_padding,
        "elevation_deg": args.elevation_deg,
        "target_height_offset_m": args.target_height_offset_m,
        "hfov_deg": args.hfov,
        "trajectory_radius_m": args.trajectory_radius,
        "trajectory_line_width_px": args.trajectory_line_width,
        "trajectory_outline_width_px": args.trajectory_outline_width,
        "task_completion_marker_count": completion_marker_count,
        "task_marker_radius_px": args.task_marker_radius,
        "endpoint_marker_radius_px": args.endpoint_marker_radius,
        "observation_frustum_count": len(completion_events),
        "frustum_depth_m": args.frustum_depth,
        "frustum_line_width_px": args.frustum_line_width,
        "observation_hfov_deg": args.observation_hfov,
        "progressive_trajectory": not args.static_trajectory,
        "caption": not args.no_caption,
        "start_hold_fraction": args.start_hold_fraction,
        "end_hold_fraction": args.end_hold_fraction,
        "video": str(video_path.resolve()),
        "preview": str(preview_path.resolve()),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"video={video_path}")
    print(f"preview={preview_path}")
    print(f"manifest={manifest_path}")


if __name__ == "__main__":
    render(parse_args())
