#!/usr/bin/env python3
"""Execute a dynamic stage-two mission in Habitat with RGB-D/semantic verification."""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

import habitat_sim
import numpy as np
from habitat_sim.utils.common import quat_from_angle_axis
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.grid_map import OccupancyGrid  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.joint_planner import plan_joint_mission  # noqa: E402
from stage2.mission_executor import MissionState  # noqa: E402


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
ALIASES = {
    "tv": {"tv", "television"}, "television": {"tv", "television"},
    "couch": {"couch", "sofa"}, "sofa": {"couch", "sofa"},
    "cabinet": {"cabinet", "bathroom cabinet", "kitchen cabinet"},
    "plant": {"plant", "decorative plant", "indoor plant"},
    "chair": {"chair", "office chair", "dining chair"},
    "lamp": {"lamp", "floor lamp", "table lamp"},
}


def sensor(uuid: str, sensor_type: habitat_sim.SensorType) -> habitat_sim.CameraSensorSpec:
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = uuid
    spec.sensor_type = sensor_type
    spec.resolution = [480, 640]
    spec.position = [0.0, 1.0, 0.0]
    spec.hfov = 90.0
    return spec


def category_by_semantic_id(sim, semantic: np.ndarray) -> tuple[dict[str, int], dict[int, str]]:
    counts: dict[str, int] = {}
    id_to_label: dict[int, str] = {}
    ids, pixel_counts = np.unique(semantic, return_counts=True)
    objects = sim.semantic_scene.objects
    for semantic_id, count in zip(ids.tolist(), pixel_counts.tolist()):
        label = "unknown"
        if 0 <= semantic_id < len(objects) and objects[semantic_id] is not None:
            label = str(objects[semantic_id].category.name()).strip().lower()
        id_to_label[int(semantic_id)] = label
        counts[label] = counts.get(label, 0) + int(count)
    return counts, id_to_label


def verify_target(sim, observations: dict, target_label: str, minimum_pixels: int) -> dict:
    semantic = np.asarray(observations["semantic"], dtype=np.int32)
    counts, id_to_label = category_by_semantic_id(sim, semantic)
    accepted = ALIASES.get(target_label, {target_label})
    matching = {
        label: count
        for label, count in counts.items()
        if label in accepted or any(word in label for word in accepted)
    }
    pixels = sum(matching.values())
    return {
        "target_label": target_label,
        "found": pixels >= minimum_pixels,
        "matching_pixels": pixels,
        "matching_categories": matching,
        "visible_categories": dict(sorted(counts.items(), key=lambda item: item[1], reverse=True)[:20]),
        "semantic_id_labels": {str(key): value for key, value in id_to_label.items()},
    }


def falcon_to_agent_h(initial_agent_h: np.ndarray, pose_f: list[float]) -> np.ndarray:
    sensor_delta_f = np.asarray(pose_f[:3], dtype=np.float64) - np.asarray([0.0, 0.0, 1.0])
    return initial_agent_h + S_HABITAT_TO_FALCON.T @ sensor_delta_f


def interpolate_polyline(points: list[list[float]], step_m: float) -> list[list[float]]:
    if not points:
        return []
    result = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = math.dist(start, end)
        steps = max(1, int(math.ceil(distance / step_m)))
        for index in range(1, steps + 1):
            ratio = index / steps
            result.append([start[0] + ratio * (end[0] - start[0]), start[1] + ratio * (end[1] - start[1])])
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--grid-prefix", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=ROOT / "data/scene_datasets/hm3d/example/00861-GLAQ4DNUx5U/GLAQ4DNUx5U.basis.glb")
    parser.add_argument("--scene-config", type=Path, default=ROOT / "data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--step-m", type=float, default=0.12)
    parser.add_argument("--minimum-pixels", type=int, default=40)
    parser.add_argument(
        "--minimum-pixels-by-task",
        default="{}",
        help="JSON object overriding the detector threshold for selected tasks",
    )
    parser.add_argument("--inflation", type=float, default=0.10)
    parser.add_argument("--save-every", type=int, default=3)
    args = parser.parse_args()
    minimum_pixels_by_task = {
        str(key): int(value) for key, value in json.loads(args.minimum_pixels_by_task).items()
    }

    task_graph = load_json(args.task_graph)
    tasks = {task["id"]: task for task in task_graph["tasks"]}
    candidates = load_json(args.candidates)["by_task"]
    updated_scene_graph = copy.deepcopy(load_json(args.scene_graph))
    objects_by_id = {
        obj["id"]: obj
        for room in updated_scene_graph.get("rooms", []) for obj in room.get("objects", [])
    }
    grid = OccupancyGrid.load(args.grid_prefix, inflation_m=args.inflation)
    state = MissionState(task_graph)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = args.output_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(args.scene)
    sim_cfg.scene_dataset_config_file = str(args.scene_config)
    sim_cfg.enable_physics = True
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [
        sensor("rgb", habitat_sim.SensorType.COLOR),
        sensor("depth", habitat_sim.SensorType.DEPTH),
        sensor("semantic", habitat_sim.SensorType.SEMANTIC),
    ]

    current_f = [0.0, 0.0, 1.0, 0.0]
    trajectory = [list(current_f)]
    observations_log = []
    map_updates = []
    replans = []
    frame_index = 0
    started = time.monotonic()
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("HM3D navmesh did not load")
        sim.pathfinder.seed(7)
        initial_agent_h = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float64)
        initial_state = habitat_sim.AgentState()
        initial_state.position = initial_agent_h
        initial_state.rotation = quat_from_angle_axis(0.0, np.asarray([0.0, 1.0, 0.0]))
        agent = sim.initialize_agent(0, initial_state)

        while not state.is_finished():
            if not state.active:
                break
            plan = plan_joint_mission(
                grid,
                task_graph,
                candidates,
                current_f,
                active_task_ids=state.active,
                completed_task_ids=state.completed,
            )
            replans.append({"index": len(replans), "active_task_ids": sorted(state.active), "plan": plan})
            visit = plan["visits"][0]
            segment = plan["segments"][0]
            terminal = visit["pose"]
            points = interpolate_polyline(segment["points_xy_m"], args.step_m)
            previous = np.asarray(current_f[:2], dtype=np.float64)
            start_z = current_f[2]
            for local_index, point in enumerate(points[1:], start=1):
                point_xy = np.asarray(point, dtype=np.float64)
                movement = point_xy - previous
                yaw = current_f[3] if np.linalg.norm(movement) < 1e-6 else float(math.atan2(movement[1], movement[0]))
                ratio = local_index / max(1, len(points) - 1)
                z = start_z + ratio * (float(terminal["z"]) - start_z)
                pose_f = [float(point_xy[0]), float(point_xy[1]), float(z), yaw]
                agent_state = agent.get_state()
                agent_state.position = falcon_to_agent_h(initial_agent_h, pose_f)
                agent_state.rotation = quat_from_angle_axis(yaw, np.asarray([0.0, 1.0, 0.0]))
                agent.set_state(agent_state)
                observations = sim.get_sensor_observations()
                trajectory.append(pose_f)
                if frame_index % max(1, args.save_every) == 0:
                    Image.fromarray(np.asarray(observations["rgb"])[..., :3].astype(np.uint8)).save(frames_dir / f"frame_{frame_index:06d}.jpg", quality=90)
                frame_index += 1
                previous = point_xy

            current_f = [float(terminal[key]) for key in ("x", "y", "z", "yaw")]
            terminal_state = agent.get_state()
            terminal_state.position = falcon_to_agent_h(initial_agent_h, current_f)
            terminal_state.rotation = quat_from_angle_axis(current_f[3], np.asarray([0.0, 1.0, 0.0]))
            agent.set_state(terminal_state)
            observations = sim.get_sensor_observations()
            task_threshold = minimum_pixels_by_task.get(visit["task_id"], args.minimum_pixels)
            verification = verify_target(
                sim,
                observations,
                tasks[visit["task_id"]]["verification_label"],
                task_threshold,
            )
            verification["minimum_pixels"] = task_threshold
            action = tasks[visit["task_id"]]["action"]
            if action in {"inspect", "find", "observe"}:
                outcome = "found" if verification["found"] else "not_found"
            else:
                outcome = "done"
            terminal_rgb = frames_dir / f"terminal_{len(observations_log):02d}_{visit['task_id']}.jpg"
            Image.fromarray(np.asarray(observations["rgb"])[..., :3].astype(np.uint8)).save(terminal_rgb, quality=95)
            observation_record = {
                "sequence": len(observations_log),
                "task_id": visit["task_id"],
                "candidate_id": visit["candidate_id"],
                "pose": current_f,
                "outcome": outcome,
                "frame_index": frame_index,
                "verification": verification,
                "rgb": str(terminal_rgb.relative_to(args.output_dir)),
            }
            observations_log.append(observation_record)
            mapped_object = objects_by_id.get(visit["object_id"])
            if mapped_object is not None:
                old_probability = float(mapped_object.get("probability", 0.0))
                new_probability = (
                    old_probability + 0.2 * (1.0 - old_probability)
                    if verification["found"] else old_probability * 0.5
                )
                mapped_object["probability"] = new_probability
                mapped_object["online_verification"] = {
                    "task_id": visit["task_id"],
                    "outcome": outcome,
                    "matching_pixels": verification["matching_pixels"],
                    "sequence": len(observations_log) - 1,
                }
                map_updates.append({
                    "object_id": visit["object_id"],
                    "old_probability": old_probability,
                    "new_probability": new_probability,
                    "outcome": outcome,
                })
            state.finish_task(visit["task_id"], outcome)
            print(f"task={visit['task_id']} outcome={outcome} pixels={verification['matching_pixels']} active={sorted(state.active)}", flush=True)

    trace = {
        "format": "pre_map_vln.habitat_execution.v1",
        "status": "completed" if state.is_finished() else "incomplete",
        "scene": str(args.scene),
        "task_status": state.status,
        "events": state.events,
        "observations": observations_log,
        "map_updates": map_updates,
        "replans": replans,
        "trajectory_xyz_yaw": trajectory,
        "path_length_m": sum(math.dist(a[:3], b[:3]) for a, b in zip(trajectory, trajectory[1:])),
        "elapsed_wall_s": time.monotonic() - started,
        "frame_count": frame_index,
        "final_pose": current_f,
    }
    atomic_json(args.output_dir / "habitat_execution.json", trace)
    atomic_json(args.output_dir / "updated_scene_graph.json", updated_scene_graph)
    print(f"status={trace['status']} tasks={len(observations_log)} frames={frame_index} path={trace['path_length_m']:.2f}m elapsed={trace['elapsed_wall_s']:.1f}s")


if __name__ == "__main__":
    main()
