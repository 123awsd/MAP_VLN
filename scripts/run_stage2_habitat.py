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
from stage2.astar_3d import CoarseAstar3D  # noqa: E402
from stage2.bspline_3d import BsplineSettings, anchor_astar_path, plan_collision_checked_bspline  # noqa: E402
from stage2.candidate_poses import generate_all_candidates  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.joint_planner import PlanningError, plan_joint_mission  # noqa: E402
from stage2.mission_executor import MissionState  # noqa: E402
from stage2.motion_cost_oracle import MotionCostOracle  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.spatial_verification import (  # noqa: E402
    effective_relation_context,
    scoped_reference_objects,
    verify_target_reference_relation,
)
from stage2.task_graph import normalize_and_validate_task_graph  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402
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
from stage2.semantic_region_search import (  # noqa: E402
    SemanticRegionBelief,
    materialize_room_frontier_fallback,
)
from stage2.semantic_recovery import (  # noqa: E402
    QwenSemanticRecoveryPlanner,
    materialize_recovery_candidates,
    validate_hypotheses,
)
from stage2.viewpoint_recovery import ViewpointRecovery  # noqa: E402
from stage2.vlm_verifier import QwenImageVerifier  # noqa: E402
from stage2.vocabulary import load_semantic_aliases  # noqa: E402
from stage2.yaw_motion import blend_yaw, rotation_steps, step_yaw  # noqa: E402
from stage2.representative_viewpoints import select_location_representatives  # noqa: E402


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
ALIASES = load_semantic_aliases()


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


def interpolate_xyz(points: list[list[float]], step_m: float) -> list[list[float]]:
    if not points:
        return []
    result = [list(points[0])]
    for start, end in zip(points, points[1:]):
        distance = math.dist(start, end)
        steps = max(1, int(math.ceil(distance / step_m)))
        for index in range(1, steps + 1):
            ratio = index / steps
            result.append([
                float(start[axis] + ratio * (end[axis] - start[axis])) for axis in range(3)
            ])
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, default=None)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--grid-prefix", type=Path, default=None)
    parser.add_argument("--voxel-snapshot", type=Path, default=None)
    parser.add_argument("--generated-stage1-config", type=Path, default=None)
    parser.add_argument(
        "--planning-config", type=Path,
        default=ROOT / "config/uav_3d_planning_habitat.yaml",
    )
    parser.add_argument("--scene", type=Path, default=ROOT / "data/scene_datasets/hm3d/example/00861-GLAQ4DNUx5U/GLAQ4DNUx5U.basis.glb")
    parser.add_argument("--scene-config", type=Path, default=ROOT / "data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--step-m", type=float, default=0.12)
    parser.add_argument(
        "--execution-hz", type=float, default=5.0,
        help="nominal trajectory sample rate used to enforce the simulated yaw rate",
    )
    parser.add_argument("--yaw-rate-rps", type=float, default=1.0)
    parser.add_argument("--terminal-yaw-blend-distance-m", type=float, default=0.8)
    parser.add_argument(
        "--max-candidates", type=int, default=8,
        help="maximum complementary viewpoints per physical location hypothesis",
    )
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
        choices=("semantic", "owlv2", "qwen_vl", "hybrid", "owlv2_qwen_fallback", "controlled"),
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
    parser.add_argument(
        "--max-viewpoint-attempts", type=int, default=0,
        help="optional global viewpoint cap; 0 exhausts the finite per-location candidate pools",
    )
    parser.add_argument(
        "--planning-horizon-tasks", type=int, default=3,
        help="maximum currently executable tasks in each rolling joint optimization",
    )
    parser.add_argument("--semantic-recovery", action="store_true")
    parser.add_argument("--max-recovery-hypotheses", type=int, default=3)
    parser.add_argument(
        "--max-recovery-visits", type=int, default=8,
        help="deprecated compatibility option; recovery is now budgeted by --max-recovery-locations",
    )
    parser.add_argument(
        "--max-recovery-locations", type=int, default=4,
        help="maximum distinct anchor/region hypotheses; viewpoints within one location do not consume this budget",
    )
    parser.add_argument("--max-viewpoints-per-location", type=int, default=2)
    parser.add_argument("--recovery-budget-cny", type=float, default=20.0)
    parser.add_argument(
        "--recovery-plan", type=Path, default=None,
        help="optional offline recovery hypotheses; skips the external Qwen call",
    )
    parser.add_argument(
        "--controlled-stale-object-ids",
        default="[]",
        help="JSON list of mapped anchors whose terminal result is forced missing for reproducible stale-map demos",
    )
    parser.add_argument(
        "--controlled-found-object-ids", default="[]",
        help="JSON list of anchors with a controlled positive detector result for branch-logic demos",
    )
    parser.add_argument("--novel-object-min-support", type=int, default=2)
    args = parser.parse_args()
    if (args.grid_prefix is None) == (args.voxel_snapshot is None):
        parser.error("provide exactly one of --grid-prefix or --voxel-snapshot")
    use_3d = args.voxel_snapshot is not None
    minimum_pixels_by_task = {
        str(key): int(value) for key, value in json.loads(args.minimum_pixels_by_task).items()
    }
    class_thresholds = {
        str(key).strip().lower(): float(value)
        for key, value in json.loads(args.owlv2_class_thresholds).items()
    }
    controlled_stale_object_ids = {str(value) for value in json.loads(args.controlled_stale_object_ids)}
    controlled_found_object_ids = {str(value) for value in json.loads(args.controlled_found_object_ids)}

    task_graph = normalize_and_validate_task_graph(load_json(args.task_graph))
    tasks = {task["id"]: task for task in task_graph["tasks"]}

    def allows_semantic_recovery(task: dict) -> bool:
        return task.get("intent", {}).get("not_found_policy") == "semantic_recovery"
    updated_scene_graph = copy.deepcopy(load_json(args.scene_graph))
    objects_by_id = {
        obj["id"]: obj
        for room in updated_scene_graph.get("rooms", []) for obj in room.get("objects", [])
    }
    mapped_objects = list(objects_by_id.values())
    if use_3d:
        profile = PlannerProfile.load(args.planning_config)
        voxel_map = VoxelMap3D.load(args.voxel_snapshot, profile)
        grid = MotionCostOracle(
            CoarseAstar3D(voxel_map), args.output_dir / "motion_cost_cache.json",
            save_interval=32,
        )
        bspline_settings = BsplineSettings.load(args.planning_config)
    else:
        voxel_map = None
        bspline_settings = None
        grid = OccupancyGrid.load(args.grid_prefix, inflation_m=args.inflation)
    if args.candidates is not None:
        candidates = load_json(args.candidates)["by_task"]
    else:
        candidates = generate_all_candidates(
            grid, updated_scene_graph, task_graph, max_candidates=args.max_candidates,
        )
    state = MissionState(task_graph)
    vlm_verifier = QwenImageVerifier() if args.verification_mode in {"qwen_vl", "hybrid", "owlv2_qwen_fallback"} else None
    open_vocab_modes = {"owlv2", "owlv2_qwen_fallback"}
    detector_prompts = sorted({
        label
        for task in tasks.values()
        for label in (
            [task["verification_label"].strip().lower()]
            + [str(value).strip().lower() for value in task["target"].get("references", [])]
        )
        if label
    })
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

    generated = load_json(args.generated_stage1_config) if args.generated_stage1_config else None
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(Path(generated["scene"]).resolve()) if generated else str(args.scene)
    sim_cfg.scene_dataset_config_file = (
        str(Path(generated["scene_config"]).resolve()) if generated else str(args.scene_config)
    )
    sim_cfg.enable_physics = True
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [
        sensor("rgb", habitat_sim.SensorType.COLOR),
        sensor("depth", habitat_sim.SensorType.DEPTH),
    ]
    if args.verification_mode in {"semantic", "hybrid"}:
        agent_cfg.sensor_specifications.append(sensor("semantic", habitat_sim.SensorType.SEMANTIC))

    current_f = [0.0, 0.0, 1.0, 0.0]
    trajectory = [list(current_f)]
    observations_log = []
    map_updates = []
    open_vocab_observations = []
    fusion = OnlineSemanticFusion(minimum_novel_support=args.novel_object_min_support)
    recovery = ViewpointRecovery(
        maximum_attempts=(
            args.max_viewpoint_attempts if args.max_viewpoint_attempts > 0 else None
        ),
        maximum_attempts_per_location=(
            args.max_viewpoints_per_location if args.semantic_recovery else None
        ),
        maximum_locations=args.max_recovery_locations if args.semantic_recovery else None,
    )
    semantic_recovery_planner = (
        QwenSemanticRecoveryPlanner(budget_cny=args.recovery_budget_cny)
        if args.semantic_recovery and args.recovery_plan is None else None
    )
    offline_recovery_plan = load_json(args.recovery_plan) if args.recovery_plan else None
    semantic_recovery_events = []
    semantic_region_belief = SemanticRegionBelief()
    recovery_plans: dict[str, dict] = {}
    room_fallback_candidates: dict[str, list[dict]] = {}
    failed_location_object_ids: dict[str, set[str]] = {}
    replans = []
    executed_segments = []
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
        if generated:
            initial_agent_h = np.asarray(generated["initial_agent_habitat_xyz"], dtype=np.float64)
        else:
            if not sim.pathfinder.is_loaded:
                raise RuntimeError("HM3D navmesh did not load")
            sim.pathfinder.seed(7)
            initial_agent_h = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float64)
        initial_state = habitat_sim.AgentState()
        initial_state.position = initial_agent_h
        initial_state.rotation = quat_from_angle_axis(0.0, np.asarray([0.0, 1.0, 0.0]))
        agent = sim.initialize_agent(0, initial_state)

        def record_motion_frame(pose_f, observations) -> None:
            nonlocal frame_index
            if async_detector is not None:
                consume_async(async_detector.poll())
            trajectory.append(list(pose_f))
            save_frame = frame_index % max(1, args.save_every) == 0
            detect_frame = bool(
                open_vocab_detector is not None
                and args.owlv2_every > 0
                and frame_index % args.owlv2_every == 0
            )
            frame_path = frames_dir / f"frame_{frame_index:06d}.jpg"
            if save_frame or detect_frame:
                Image.fromarray(
                    np.asarray(observations["rgb"])[..., :3].astype(np.uint8)
                ).save(frame_path, quality=90)
            if detect_frame and async_detector is not None:
                async_detector.submit(frame_path, {
                    "frame_index": frame_index,
                    "pose": list(pose_f),
                    "depth": np.asarray(observations["depth"], dtype=np.float32).copy(),
                })
            frame_index += 1

        while not state.is_finished():
            if not state.active:
                break
            planning_candidates = recovery.filtered_candidates(candidates)
            focused_tasks = recovery.focused_task_ids(state.active, planning_candidates)
            available = state.available()
            if focused_tasks:
                planning_task_ids = focused_tasks
            else:
                def task_lower_bound(task_id: str) -> float:
                    values = planning_candidates.get(task_id, [])
                    return min((
                        math.dist(
                            current_f[:3],
                            [item["pose"][axis] for axis in ("x", "y", "z")],
                        ) + float(item.get("terminal_cost", 0.0))
                        for item in values
                    ), default=math.inf)
                planning_task_ids = set(sorted(
                    available, key=lambda task_id: (task_lower_bound(task_id), task_id)
                )[:max(1, args.planning_horizon_tasks)])
            representative_limit = None
            representative_errors = []
            planning_input = planning_candidates
            if not focused_tasks:
                # Global ordering needs one entry pose per physical location,
                # not every local camera angle around that location. If an
                # entry pose proves unreachable, lazily admit the second-ranked
                # pose and finally the full pool as a correctness fallback.
                plan = None
                for representative_limit in (1, 2, None):
                    planning_input = (
                        planning_candidates
                        if representative_limit is None
                        else select_location_representatives(
                            planning_candidates, current_f, planning_task_ids,
                            maximum_per_location=representative_limit,
                            yaw_rate_rps=args.yaw_rate_rps,
                        )
                    )
                    try:
                        plan = plan_joint_mission(
                            grid, task_graph, planning_input, current_f,
                            active_task_ids=planning_task_ids,
                            completed_task_ids=state.completed,
                            yaw_rate_rps=args.yaw_rate_rps,
                        )
                        break
                    except PlanningError as error:
                        representative_errors.append({
                            "maximum_per_location": representative_limit,
                            "error": str(error),
                        })
                if plan is None:
                    raise PlanningError("representative and full candidate planning both failed")
            else:
                plan = plan_joint_mission(
                    grid, task_graph, planning_input, current_f,
                    active_task_ids=planning_task_ids,
                    completed_task_ids=state.completed,
                    yaw_rate_rps=args.yaw_rate_rps,
                )
            replans.append({
                "index": len(replans),
                "active_task_ids": sorted(state.active),
                "focused_task_ids": sorted(focused_tasks),
                "candidate_scope": (
                    "focused_local" if focused_tasks else "location_representatives"
                ),
                "representative_limit_per_location": representative_limit,
                "planning_candidate_counts": {
                    task_id: len(planning_input.get(task_id, []))
                    for task_id in sorted(planning_task_ids)
                },
                "representative_fallback_errors": representative_errors,
                "plan": plan,
            })
            visit = plan["visits"][0]
            executed_candidate = next(
                item for item in candidates[visit["task_id"]]
                if item["id"] == visit["candidate_id"]
            )
            segment = plan["segments"][0]
            terminal = visit["pose"]
            if use_3d:
                actual_goal = [float(terminal[key]) for key in ("x", "y", "z")]
                anchored = anchor_astar_path(
                    segment["points_xyz_m"], current_f[:3], actual_goal, voxel_map,
                )
                bspline = plan_collision_checked_bspline(anchored, voxel_map, bspline_settings)
                points_xyz = interpolate_xyz(bspline.points_xyz_m, args.step_m)
            else:
                points_xy = interpolate_polyline(segment["points_xy_m"], args.step_m)
                points_xyz = [
                    [point[0], point[1], current_f[2] + index / max(1, len(points_xy) - 1) * (float(terminal["z"]) - current_f[2])]
                    for index, point in enumerate(points_xy)
                ]
                bspline = None
            executed_segment = {
                "sequence": len(executed_segments),
                "task_id": visit["task_id"],
                "candidate_id": visit["candidate_id"],
                "astar_length_m": float(segment["length_m"]),
                "astar_points_xyz_m": segment.get("points_xyz_m"),
                "bspline_mode": None if bspline is None else bspline.mode,
                "bspline_degree": None if bspline is None else bspline.degree,
                "bspline_piece_count": None if bspline is None else bspline.piece_count,
                "bspline_minimum_clearance_m": None if bspline is None else bspline.minimum_clearance_m,
                "bspline_points_xyz_m": None if bspline is None else bspline.points_xyz_m,
                "collision_checked_3d": bool(use_3d),
                "yaw_rate_rps": float(args.yaw_rate_rps),
                "execution_hz": float(args.execution_hz),
                "terminal_yaw_blend_distance_m": float(args.terminal_yaw_blend_distance_m),
            }
            executed_segments.append(executed_segment)
            previous = np.asarray(current_f[:3], dtype=np.float64)
            executed_yaw = float(current_f[3])
            terminal_yaw = float(terminal["yaw"])
            maximum_yaw_step = max(1e-6, float(args.yaw_rate_rps)) / max(
                0.1, float(args.execution_hz)
            )
            remaining_distances = [0.0] * len(points_xyz)
            for index in range(len(points_xyz) - 2, -1, -1):
                remaining_distances[index] = (
                    remaining_distances[index + 1]
                    + math.dist(points_xyz[index], points_xyz[index + 1])
                )
            for local_index, point in enumerate(points_xyz[1:], start=1):
                point_xyz = np.asarray(point, dtype=np.float64)
                if use_3d and not voxel_map.is_state_valid(point_xyz):
                    raise RuntimeError(f"B-spline entered non-FREE voxel at {point_xyz.tolist()}")
                movement = point_xyz - previous
                path_yaw = (
                    executed_yaw
                    if np.linalg.norm(movement[:2]) < 1e-6
                    else float(math.atan2(movement[1], movement[0]))
                )
                blend_distance = max(0.0, float(args.terminal_yaw_blend_distance_m))
                blend_fraction = (
                    0.0 if blend_distance <= 1e-6
                    else 1.0 - min(1.0, remaining_distances[local_index] / blend_distance)
                )
                desired_yaw = blend_yaw(path_yaw, terminal_yaw, blend_fraction)
                yaw = step_yaw(executed_yaw, desired_yaw, maximum_yaw_step)
                pose_f = [float(point_xyz[0]), float(point_xyz[1]), float(point_xyz[2]), yaw]
                agent_state = agent.get_state()
                agent_state.position = falcon_to_agent_h(initial_agent_h, pose_f)
                agent_state.rotation = quat_from_angle_axis(yaw, np.asarray([0.0, 1.0, 0.0]))
                agent.set_state(agent_state)
                observations = sim.get_sensor_observations()
                record_motion_frame(pose_f, observations)
                previous = point_xyz
                executed_yaw = yaw

            terminal_xyz = [float(terminal[key]) for key in ("x", "y", "z")]
            terminal_rotation = rotation_steps(
                executed_yaw, terminal_yaw, maximum_yaw_step
            )
            for yaw in terminal_rotation:
                pose_f = terminal_xyz + [yaw]
                terminal_state = agent.get_state()
                terminal_state.position = falcon_to_agent_h(initial_agent_h, pose_f)
                terminal_state.rotation = quat_from_angle_axis(
                    yaw, np.asarray([0.0, 1.0, 0.0])
                )
                agent.set_state(terminal_state)
                observations = sim.get_sensor_observations()
                record_motion_frame(pose_f, observations)
            executed_segment["terminal_rotation_frame_count"] = len(terminal_rotation)
            executed_segment["terminal_yaw_error_before_rotation_rad"] = abs(
                math.atan2(
                    math.sin(terminal_yaw - executed_yaw),
                    math.cos(terminal_yaw - executed_yaw),
                )
            )

            current_f = [float(terminal[key]) for key in ("x", "y", "z", "yaw")]
            terminal_state = agent.get_state()
            terminal_state.position = falcon_to_agent_h(initial_agent_h, current_f)
            terminal_state.rotation = quat_from_angle_axis(current_f[3], np.asarray([0.0, 1.0, 0.0]))
            agent.set_state(terminal_state)
            observations = sim.get_sensor_observations()
            task_threshold = minimum_pixels_by_task.get(visit["task_id"], args.minimum_pixels)
            semantic_verification = (
                verify_target(
                    sim, observations, tasks[visit["task_id"]]["verification_label"],
                    task_threshold,
                )
                if "semantic" in observations else {
                    "target_label": tasks[visit["task_id"]]["verification_label"],
                    "found": False, "matching_pixels": 0,
                    "matching_categories": {}, "visible_categories": {},
                    "semantic_id_labels": {}, "unavailable": True,
                }
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
            if args.verification_mode == "controlled":
                selected_found = False
            elif args.verification_mode == "semantic":
                selected_found = bool(semantic_verification["found"])
            elif args.verification_mode == "owlv2":
                selected_found = bool(open_vocab_verification and open_vocab_verification["found"])
            elif args.verification_mode == "qwen_vl":
                selected_found = vlm_found
            elif args.verification_mode == "hybrid":
                selected_found = bool(semantic_verification["found"]) or vlm_found
            else:
                selected_found = bool(open_vocab_verification and open_vocab_verification["found"]) or vlm_found
            relation_context = effective_relation_context(
                tasks[visit["task_id"]], executed_candidate, objects_by_id,
                [] if open_vocab_verification is None else open_vocab_verification.get("projected_3d", []),
            )
            target_object = relation_context.get("target")
            reference_objects = relation_context.get("references")
            if reference_objects is None:
                reference_objects = (
                    scoped_reference_objects(
                        updated_scene_graph, visit["object_id"],
                        relation_context.get("reference_labels", []),
                    ) if target_object is not None else []
                )
            relation_verification = verify_target_reference_relation(
                relation_context.get("relation"),
                target_object or {
                    "id": visit["object_id"], "center_xyz_m": executed_candidate["target_xyz_m"],
                    "size_xyz_m": [0.1, 0.1, 0.1],
                },
                reference_objects,
            )
            relation_verification["scope"] = relation_context["scope"]
            # Spatial language grounds the search region and observation poses.
            # Keep the post-detection geometry check for audit only: a genuine
            # visual detection is sufficient for ordinary inspect/find tasks.
            relation_verification["enforced_for_success"] = False
            controlled_stale = visit["object_id"] in controlled_stale_object_ids
            controlled_found = visit["object_id"] in controlled_found_object_ids
            if controlled_found:
                selected_found = True
            if controlled_stale:
                selected_found = False
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
                "controlled_stale_map": controlled_stale,
                "controlled_found": controlled_found,
                "target_reference_relation": relation_verification,
            })
            action = tasks[visit["task_id"]]["action"]
            if action in {"inspect", "find", "observe"}:
                outcome = "found" if selected_found else "not_found"
            else:
                outcome = "done"
            region_belief_update = semantic_region_belief.observe(
                executed_candidate, selected_found
            ) if action in {"inspect", "find", "observe"} else None
            semantic_region_belief.apply(candidates[visit["task_id"]])
            recovery_decision = recovery.decide(
                visit["task_id"], visit["candidate_id"], selected_found,
                candidates[visit["task_id"]],
            ) if action in {"inspect", "find", "observe"} else {
                "retry": False, "attempt_index": 1, "reason": "non_visual_action",
            }
            search_activated = False
            if (
                (semantic_recovery_planner is not None or offline_recovery_plan is not None)
                and action in {"inspect", "find", "observe"}
                and allows_semantic_recovery(tasks[visit["task_id"]])
                and not selected_found
                and recovery_decision.get("location_exhausted")
            ):
                failed = failed_location_object_ids.setdefault(visit["task_id"], set())
                failed.add(visit["object_id"])
                if visit["task_id"] not in recovery_plans:
                    if offline_recovery_plan is not None:
                        search_plan = validate_hypotheses(
                            offline_recovery_plan, updated_scene_graph, failed,
                            args.max_recovery_hypotheses,
                        )
                        search_plan["provenance"] = {"planner": "offline_fixture", "external_call": False}
                    else:
                        search_plan = semantic_recovery_planner.plan(
                            tasks[visit["task_id"]]["verification_label"],
                            task_graph.get("instruction", ""),
                            updated_scene_graph,
                            failed,
                            maximum_hypotheses=args.max_recovery_hypotheses,
                        )
                    new_candidates = materialize_recovery_candidates(
                        grid, updated_scene_graph, tasks[visit["task_id"]], search_plan,
                    )
                    known_ids = {item["id"] for item in candidates[visit["task_id"]]}
                    new_candidates = [item for item in new_candidates if item["id"] not in known_ids]
                    candidates[visit["task_id"]].extend(new_candidates)
                    semantic_region_belief.register(new_candidates)
                    room_fallback_candidates[visit["task_id"]] = materialize_room_frontier_fallback(
                        grid, updated_scene_graph, tasks[visit["task_id"]], search_plan,
                    )
                    recovery_plans[visit["task_id"]] = search_plan
                    search_activated = bool(new_candidates)
                    semantic_recovery_events.append({
                        "sequence": len(observations_log),
                        "task_id": visit["task_id"],
                        "missing_target": tasks[visit["task_id"]]["verification_label"],
                        "failed_expected_object_id": visit["object_id"],
                        "plan": search_plan,
                        "generated_candidate_ids": [item["id"] for item in new_candidates],
                    })
                    if search_activated:
                        recovery_decision.update({
                            "retry": True,
                            "reason": "semantic_recovery_activated",
                            "remaining_candidate_ids": [item["id"] for item in new_candidates],
                            "semantic_search_plan": search_plan,
                        })
            if (
                (semantic_recovery_planner is not None or offline_recovery_plan is not None)
                and action in {"inspect", "find", "observe"}
                and not selected_found
                and not recovery_decision["retry"]
                and visit["task_id"] in room_fallback_candidates
                and room_fallback_candidates[visit["task_id"]]
                and recovery_decision.get("location_attempt_index", 1) < args.max_recovery_locations
            ):
                fallback = room_fallback_candidates.pop(visit["task_id"])
                known_ids = {item["id"] for item in candidates[visit["task_id"]]}
                fallback = [item for item in fallback if item["id"] not in known_ids]
                if fallback:
                    candidates[visit["task_id"]].extend(fallback)
                    semantic_region_belief.register(fallback)
                    recovery_decision.update({
                        "retry": True,
                        "reason": "room_frontier_fallback_activated",
                        "remaining_candidate_ids": [item["id"] for item in fallback],
                    })
                    if semantic_recovery_events:
                        semantic_recovery_events[-1]["fallback_candidate_ids"] = [
                            item["id"] for item in fallback
                        ]
            mission_outcome = "retry" if recovery_decision["retry"] else outcome
            if search_activated:
                mission_outcome = "recovery_activated"
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
            target_labels = ALIASES.get(
                tasks[visit["task_id"]]["verification_label"],
                {tasks[visit["task_id"]]["verification_label"]},
            )
            mapped_is_target = bool(
                mapped_object is not None
                and str(mapped_object.get("label", "")).lower() in target_labels
            )
            object_map_update = None
            if mapped_object is not None and mapped_is_target:
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
                object_map_update = {
                    "type": "target_instance_confidence",
                    "object_id": visit["object_id"],
                    "old_probability": old_probability,
                    "new_probability": new_probability,
                    "outcome": mission_outcome,
                }
            if region_belief_update is not None:
                if object_map_update is not None:
                    region_belief_update["target_instance_update"] = object_map_update
                map_updates.append(region_belief_update)
            elif object_map_update is not None:
                map_updates.append(object_map_update)
            else:
                # A semantic recovery view may be anchored to a shelf/table.  A
                # failed target observation must update the search-region belief,
                # never the anchor object's existence confidence.
                map_updates.append({
                    "type": "observation_no_target_instance_update",
                    "object_id": visit["object_id"],
                    "outcome": mission_outcome,
                })
            if not recovery_decision["retry"]:
                state.finish_task(visit["task_id"], outcome)
            print(
                f"task={visit['task_id']} outcome={mission_outcome} "
                f"view={recovery_decision.get('viewpoint_attempt_index', recovery_decision['attempt_index'])} "
                f"location={recovery_decision.get('location_attempt_index', 1)}/{args.max_recovery_locations if args.semantic_recovery else 1} "
                f"pixels={verification['matching_pixels']} active={sorted(state.active)}",
                flush=True,
            )

    if async_detector is not None:
        consume_async(async_detector.flush())
        async_detector.close()
    if open_vocab_detector is not None:
        open_vocab_detector.close()

    updated_scene_graph, added_novel_objects = fusion.materialize_scene_graph(updated_scene_graph)
    if hasattr(grid, "flush"):
        grid.flush()
    clearance_audit = None
    if voxel_map is not None:
        clearances = [voxel_map.clearance(item[:3]) for item in trajectory]
        clearance_audit = {
            "minimum_esdf_m": min(clearances),
            "occupied_or_unknown_pose_count": sum(
                not voxel_map.is_state_valid(item[:3]) for item in trajectory
            ),
            "poses_below_soft_preference": sum(
                value < voxel_map.profile.preferred_esdf_distance_m for value in clearances
            ),
            "hard_distance_m": voxel_map.profile.minimum_esdf_distance_m,
            "soft_preference_m": voxel_map.profile.preferred_esdf_distance_m,
        }

    trace = {
        "format": "pre_map_vln.habitat_execution.v1",
        "status": state.result_status(),
        "unresolved_task_ids": state.unresolved_task_ids(),
        "scene": sim_cfg.scene_id,
        "planning_dimension": "3d" if use_3d else "2d",
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
        "executed_segments": executed_segments,
        "clearance_audit": clearance_audit,
        "semantic_recovery": {
            "enabled": args.semantic_recovery,
            "planner": (
                "offline_fixture" if offline_recovery_plan is not None
                else "qwen3.7-plus" if args.semantic_recovery else None
            ),
            "events": semantic_recovery_events,
            "plans_by_task": recovery_plans,
            "failed_location_object_ids": {
                key: sorted(value) for key, value in failed_location_object_ids.items()
            },
            "region_beliefs": semantic_region_belief.beliefs,
            "region_observed_sample_ids": {
                key: sorted(value) for key, value in semantic_region_belief.observed_samples.items()
            },
            "maximum_hypotheses": args.max_recovery_hypotheses,
            "legacy_maximum_visits_ignored": args.max_recovery_visits,
            "maximum_locations": args.max_recovery_locations,
            "budget_unit": "location_hypothesis",
            "maximum_viewpoints_per_location": args.max_viewpoints_per_location,
            "controlled_stale_object_ids": sorted(controlled_stale_object_ids),
            "controlled_found_object_ids": sorted(controlled_found_object_ids),
        },
        "trajectory_xyz_yaw": trajectory,
        "path_length_m": sum(math.dist(a[:3], b[:3]) for a, b in zip(trajectory, trajectory[1:])),
        "elapsed_wall_s": time.monotonic() - started,
        "frame_count": frame_index,
        "final_pose": current_f,
    }
    atomic_json(args.output_dir / "habitat_execution.json", trace)
    atomic_json(args.output_dir / "updated_scene_graph.json", updated_scene_graph)
    atomic_json(args.output_dir / "candidates_with_recovery.json", {"by_task": candidates})
    print(f"status={trace['status']} tasks={len(observations_log)} frames={frame_index} path={trace['path_length_m']:.2f}m elapsed={trace['elapsed_wall_s']:.1f}s")


if __name__ == "__main__":
    main()
