#!/usr/bin/env python3
"""Execute a dynamic stage-two mission in Habitat with RGB-D/semantic verification."""

from __future__ import annotations

import argparse
import atexit
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
from stage2.open_vocab_detector import (  # noqa: E402
    AsyncOpenVocabularyDetector,
    LocalOpenVocabularyDetector,
    PILOT_CLASS_THRESHOLDS,
    apply_class_thresholds,
    associate_projection,
    project_detection_to_world,
    target_found,
)
from stage2.semantic_fusion import OnlineSemanticFusion  # noqa: E402
from stage2.viewpoint_recovery import ViewpointRecovery  # noqa: E402
from stage2.vlm_verifier import QwenImageVerifier  # noqa: E402


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
    parser.add_argument(
        "--verification-mode",
        choices=("semantic", "owlv2", "qwen_vl", "hybrid", "owlv2_qwen_fallback"),
        default="owlv2",
        help="Decision source; owlv2 is the fast local-GPU default and Qwen is optional fallback",
    )
    parser.add_argument("--owlv2-threshold", type=float, default=0.20)
    parser.add_argument(
        "--owlv2-class-thresholds",
        default=json.dumps(PILOT_CLASS_THRESHOLDS),
        help="JSON per-class thresholds calibrated by the local Semantic-GT pilot",
    )
    parser.add_argument(
        "--owlv2-every", type=int, default=10,
        help="Run local open-vocabulary mapping every N flight frames; 0 disables keyframes",
    )
    parser.add_argument("--vlm-confidence-threshold", type=float, default=0.65)
    parser.add_argument("--vlm-no-cache", action="store_true")
    parser.add_argument("--max-viewpoint-attempts", type=int, default=3)
    parser.add_argument("--novel-object-min-support", type=int, default=2)
    args = parser.parse_args()
    minimum_pixels_by_task = {
        str(key): int(value) for key, value in json.loads(args.minimum_pixels_by_task).items()
    }
    class_thresholds = {
        str(key).strip().lower(): float(value)
        for key, value in json.loads(args.owlv2_class_thresholds).items()
    }

    task_graph = load_json(args.task_graph)
    tasks = {task["id"]: task for task in task_graph["tasks"]}
    candidates = load_json(args.candidates)["by_task"]
    updated_scene_graph = copy.deepcopy(load_json(args.scene_graph))
    objects_by_id = {
        obj["id"]: obj
        for room in updated_scene_graph.get("rooms", []) for obj in room.get("objects", [])
    }
    mapped_objects = list(objects_by_id.values())
    grid = OccupancyGrid.load(args.grid_prefix, inflation_m=args.inflation)
    state = MissionState(task_graph)
    vlm_verifier = QwenImageVerifier() if args.verification_mode in {"qwen_vl", "hybrid", "owlv2_qwen_fallback"} else None
    open_vocab_modes = {"owlv2", "owlv2_qwen_fallback"}
    detector_prompts = sorted({task["verification_label"].strip().lower() for task in tasks.values()})
    open_vocab_detector = None
    async_detector = None
    if args.verification_mode in open_vocab_modes:
        open_vocab_detector = LocalOpenVocabularyDetector(
            detector_prompts,
            threshold=args.owlv2_threshold,
            timeout_s=300.0,
        )
        async_detector = AsyncOpenVocabularyDetector(open_vocab_detector)
        atexit.register(open_vocab_detector.close)
        atexit.register(async_detector.close)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = args.output_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    for stale in list(frames_dir.glob("frame_*.jpg")) + list(frames_dir.glob("terminal_*.jpg")):
        stale.unlink()

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
    open_vocab_observations = []
    fusion = OnlineSemanticFusion(minimum_novel_support=args.novel_object_min_support)
    recovery = ViewpointRecovery(maximum_attempts=args.max_viewpoint_attempts)
    replans = []
    frame_index = 0
    started = time.monotonic()

    def consume_async(packet: dict | None) -> None:
        if packet is None:
            return
        metadata = packet["metadata"]
        detected = apply_class_thresholds(packet["result"], args.owlv2_threshold, class_thresholds)
        lifted = [
            associate_projection(projected, mapped_objects, ALIASES)
            for item in detected["detections"]
            if (projected := project_detection_to_world(
                item, metadata["depth"], metadata["pose"]
            )) is not None
        ]
        fused = fusion.update(lifted, metadata["frame_index"])
        open_vocab_observations.append({
            "frame_index": metadata["frame_index"], "pose": metadata["pose"],
            "latency_ms": detected["latency_ms"], "detections_2d": detected["detections"],
            "detections_3d": [item for item in lifted if item["confirmed"]],
            "unassociated_3d": [item for item in lifted if not item["confirmed"]],
            "fused_objects_3d": fused,
        })

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
            planning_candidates = recovery.filtered_candidates(candidates)
            plan = plan_joint_mission(
                grid,
                task_graph,
                planning_candidates,
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
                if async_detector is not None:
                    consume_async(async_detector.poll())
                trajectory.append(pose_f)
                save_frame = frame_index % max(1, args.save_every) == 0
                detect_frame = bool(
                    open_vocab_detector is not None
                    and args.owlv2_every > 0
                    and frame_index % args.owlv2_every == 0
                )
                frame_path = frames_dir / f"frame_{frame_index:06d}.jpg"
                if save_frame or detect_frame:
                    Image.fromarray(np.asarray(observations["rgb"])[..., :3].astype(np.uint8)).save(frame_path, quality=90)
                if detect_frame and async_detector is not None:
                    async_detector.submit(frame_path, {
                        "frame_index": frame_index, "pose": list(pose_f),
                        "depth": np.asarray(observations["depth"], dtype=np.float32).copy(),
                    })
                frame_index += 1
                previous = point_xy

            current_f = [float(terminal[key]) for key in ("x", "y", "z", "yaw")]
            terminal_state = agent.get_state()
            terminal_state.position = falcon_to_agent_h(initial_agent_h, current_f)
            terminal_state.rotation = quat_from_angle_axis(current_f[3], np.asarray([0.0, 1.0, 0.0]))
            agent.set_state(terminal_state)
            observations = sim.get_sensor_observations()
            task_threshold = minimum_pixels_by_task.get(visit["task_id"], args.minimum_pixels)
            semantic_verification = verify_target(
                sim,
                observations,
                tasks[visit["task_id"]]["verification_label"],
                task_threshold,
            )
            semantic_verification["minimum_pixels"] = task_threshold
            terminal_rgb = frames_dir / f"terminal_{len(observations_log):02d}_{visit['task_id']}.jpg"
            Image.fromarray(np.asarray(observations["rgb"])[..., :3].astype(np.uint8)).save(terminal_rgb, quality=95)
            open_vocab_verification = None
            if open_vocab_detector is not None:
                consume_async(async_detector.flush() if async_detector is not None else None)
                local_result = apply_class_thresholds(
                    open_vocab_detector.detect(terminal_rgb), args.owlv2_threshold, class_thresholds
                )
                local_found = target_found(local_result, tasks[visit["task_id"]]["verification_label"], ALIASES)
                accepted = ALIASES.get(
                    tasks[visit["task_id"]]["verification_label"],
                    {tasks[visit["task_id"]]["verification_label"]},
                )
                target_detections = [
                    item for item in local_result["detections"] if item["label"] in accepted
                ]
                lifted_targets = [
                    associate_projection(projected, mapped_objects, ALIASES)
                    for item in target_detections
                    if (projected := project_detection_to_world(
                        item, np.asarray(observations["depth"]), current_f
                    )) is not None
                ]
                terminal_fused = fusion.update(lifted_targets, frame_index)
                target_label = tasks[visit["task_id"]]["verification_label"]
                novel_confirmed = any(
                    item["source"] == "online_novel"
                    and item["label"] in ALIASES.get(target_label, {target_label})
                    and any(
                        math.dist(item["center"], projected["center"]) <= fusion.association_radius_m
                        for projected in lifted_targets
                    )
                    for item in terminal_fused
                )
                geometry_confirmed = any(item["confirmed"] for item in lifted_targets) or novel_confirmed
                planned_object_confirmed = any(
                    item["confirmed"] and item["associated_object_id"] == visit["object_id"]
                    for item in lifted_targets
                )
                open_vocab_verification = {
                    "found": local_found and geometry_confirmed,
                    "detected_2d": local_found,
                    "geometry_confirmed": geometry_confirmed,
                    "planned_object_confirmed": planned_object_confirmed,
                    "novel_track_confirmed": novel_confirmed,
                    "target_label": tasks[visit["task_id"]]["verification_label"],
                    "latency_ms": local_result["latency_ms"],
                    "detections": target_detections,
                    "projected_3d": lifted_targets,
                    "all_detections": local_result["detections"],
                }
            vlm_verification = None
            should_call_vlm = bool(
                vlm_verifier is not None
                and (
                    args.verification_mode != "owlv2_qwen_fallback"
                    or not open_vocab_verification
                    or not open_vocab_verification["found"]
                )
            )
            if should_call_vlm:
                vlm_verification = vlm_verifier.verify(
                    [terminal_rgb], tasks[visit["task_id"]], use_cache=not args.vlm_no_cache
                )
            vlm_found = bool(
                vlm_verification
                and vlm_verification["found"]
                and float(vlm_verification["confidence"]) >= args.vlm_confidence_threshold
            )
            if args.verification_mode == "semantic":
                selected_found = bool(semantic_verification["found"])
            elif args.verification_mode == "owlv2":
                selected_found = bool(open_vocab_verification and open_vocab_verification["found"])
            elif args.verification_mode == "qwen_vl":
                selected_found = vlm_found
            elif args.verification_mode == "hybrid":
                selected_found = bool(semantic_verification["found"]) or vlm_found
            else:
                selected_found = bool(open_vocab_verification and open_vocab_verification["found"]) or vlm_found
            verification = dict(semantic_verification)
            verification.update({
                "mode": args.verification_mode,
                "found": selected_found,
                "semantic_found": bool(semantic_verification["found"]),
                "owlv2_found": None if open_vocab_verification is None else open_vocab_verification["found"],
                "owlv2": open_vocab_verification,
                "vlm_found_at_threshold": vlm_found if vlm_verification is not None else None,
                "vlm_confidence_threshold": args.vlm_confidence_threshold,
                "vlm": vlm_verification,
            })
            action = tasks[visit["task_id"]]["action"]
            if action in {"inspect", "find", "observe"}:
                outcome = "found" if selected_found else "not_found"
            else:
                outcome = "done"
            recovery_decision = recovery.decide(
                visit["task_id"], visit["candidate_id"], selected_found,
                candidates[visit["task_id"]],
            ) if action in {"inspect", "find", "observe"} else {
                "retry": False, "attempt_index": 1, "reason": "non_visual_action",
            }
            mission_outcome = "retry" if recovery_decision["retry"] else outcome
            observation_record = {
                "sequence": len(observations_log),
                "task_id": visit["task_id"],
                "candidate_id": visit["candidate_id"],
                "pose": current_f,
                "outcome": mission_outcome,
                "verification_outcome": outcome,
                "recovery": recovery_decision,
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
                    "outcome": mission_outcome,
                    "matching_pixels": verification["matching_pixels"],
                    "mode": args.verification_mode,
                    "vlm_confidence": None if vlm_verification is None else vlm_verification["confidence"],
                    "owlv2_score": None if not open_vocab_verification or not open_vocab_verification["detections"] else open_vocab_verification["detections"][0]["score"],
                    "projected_3d": None if not open_vocab_verification else open_vocab_verification["projected_3d"],
                    "sequence": len(observations_log) - 1,
                }
                map_updates.append({
                    "object_id": visit["object_id"],
                    "old_probability": old_probability,
                    "new_probability": new_probability,
                    "outcome": mission_outcome,
                })
            if not recovery_decision["retry"]:
                state.finish_task(visit["task_id"], outcome)
            print(f"task={visit['task_id']} outcome={mission_outcome} attempt={recovery_decision['attempt_index']} pixels={verification['matching_pixels']} active={sorted(state.active)}", flush=True)

    if async_detector is not None:
        consume_async(async_detector.flush())
        async_detector.close()
    if open_vocab_detector is not None:
        open_vocab_detector.close()

    updated_scene_graph, added_novel_objects = fusion.materialize_scene_graph(updated_scene_graph)

    trace = {
        "format": "pre_map_vln.habitat_execution.v1",
        "status": "completed" if state.is_finished() else "incomplete",
        "scene": str(args.scene),
        "verification_mode": args.verification_mode,
        "task_status": state.status,
        "events": state.events,
        "observations": observations_log,
        "map_updates": map_updates,
        "open_vocab_observations": open_vocab_observations,
        "semantic_fusion": {
            "tracks": fusion.snapshot(confirmed_only=False),
            "confirmed_tracks": fusion.snapshot(confirmed_only=True),
            "added_novel_object_ids": added_novel_objects,
        },
        "open_vocab_detector": None if open_vocab_detector is None else {
            "backend": "local_owlv2",
            "prompts": detector_prompts,
            "threshold": args.owlv2_threshold,
            "class_thresholds": class_thresholds,
            "keyframe_interval": args.owlv2_every,
            "startup": open_vocab_detector.ready,
            "async": None if async_detector is None else async_detector.stats,
        },
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
