#!/usr/bin/env python3
"""Generate a scene-aligned FALCON 3-D config from a Habitat navmesh."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import habitat_sim
import numpy as np


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--horizontal-margin", type=float, default=2.0)
    parser.add_argument("--vertical-margin", type=float, default=1.0)
    parser.add_argument("--output-yaml", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    return parser.parse_args()


def outward_bounds(
    raw_min: np.ndarray,
    raw_max: np.ndarray,
    horizontal_margin: float,
    vertical_margin: float,
) -> tuple[np.ndarray, np.ndarray]:
    margin = np.asarray(
        [horizontal_margin, horizontal_margin, vertical_margin], dtype=np.float64
    )
    return np.floor(raw_min - margin), np.ceil(raw_max + margin)


def yaml_text(map_name: str, map_min: np.ndarray, map_max: np.ndarray) -> str:
    box_min = map_min + 0.5
    box_max = map_max - 0.5
    values = {
        "min_x": map_min[0],
        "min_y": map_min[1],
        "min_z": map_min[2],
        "max_x": map_max[0],
        "max_y": map_max[1],
        "max_z": map_max[2],
        "box_min_x": box_min[0],
        "box_min_y": box_min[1],
        "box_min_z": box_min[2],
        "box_max_x": box_max[0],
        "box_max_y": box_max[1],
        "box_max_z": box_max[2],
    }
    return f"""# Auto-generated from the Habitat navmesh. Do not flatten z.
map_config:
  map_file: "{map_name}"
  map_dimension: 3
  init_x: 0.0
  init_y: 0.0
  init_z: 1.0
  init_yaw: 0.0
  map_size:
    map_min_x: {values['min_x']:.1f}
    map_min_y: {values['min_y']:.1f}
    map_min_z: {values['min_z']:.1f}
    map_max_x: {values['max_x']:.1f}
    map_max_y: {values['max_y']:.1f}
    map_max_z: {values['max_z']:.1f}
    box_min_x: {values['box_min_x']:.1f}
    box_min_y: {values['box_min_y']:.1f}
    box_min_z: {values['box_min_z']:.1f}
    box_max_x: {values['box_max_x']:.1f}
    box_max_y: {values['box_max_y']:.1f}
    box_max_z: {values['box_max_z']:.1f}
    vbox_min_x: {values['box_min_x']:.1f}
    vbox_min_y: {values['box_min_y']:.1f}
    vbox_min_z: {values['box_min_z']:.1f}
    vbox_max_x: {values['box_max_x']:.1f}
    vbox_max_y: {values['box_max_y']:.1f}
    vbox_max_z: {values['box_max_z']:.1f}
  scale: 1.0
  T_b_c:
    - [0.0, 0.0, 1.0, 0.0]
    - [-1.0, 0.0, 0.0, 0.0]
    - [0.0, -1.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  T_m_w:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]

frontier_finder:
  viewpoint_height: -1.0
  use_3d_viewpoints: true
  cluster_min: 40
  min_visib_num: 8
  candidate_rmin: 0.5
  candidate_rmax: 2.0
  candidate_rnum: 4
  min_candidate_clearance: 0.1
  min_candidate_occupied_clearance: 0.3

exploration_manager:
  dormant_finish_block_threshold: 3
  voxel_astar_fallback_enabled: true
  voxel_astar_fallback_max_search_time: 0.5
  fsm:
    replan_thresh2: 1.0

traj_server:
  init_dx: 0.0
  init_dy: 0.0
  init_dz: 0.0
"""


def main() -> None:
    args = parse_args()
    if not args.scene.is_file():
        raise FileNotFoundError(args.scene)
    if not args.scene_config.is_file():
        raise FileNotFoundError(args.scene_config)
    if args.horizontal_margin < 0.0 or args.vertical_margin < 0.0:
        raise ValueError("map margins must be non-negative")

    simulator_config = habitat_sim.SimulatorConfiguration()
    simulator_config.scene_id = str(args.scene)
    simulator_config.scene_dataset_config_file = str(args.scene_config)
    agent_config = habitat_sim.agent.AgentConfiguration()

    with habitat_sim.Simulator(
        habitat_sim.Configuration(simulator_config, [agent_config])
    ) as simulator:
        if not simulator.pathfinder.is_loaded:
            raise RuntimeError("HM3D navmesh did not load")
        simulator.pathfinder.seed(args.seed)
        initial_agent = np.asarray(
            simulator.pathfinder.get_random_navigable_point(), dtype=np.float64
        )
        habitat_bounds = np.asarray(simulator.pathfinder.get_bounds(), dtype=np.float64)

    if initial_agent.shape != (3,) or not np.all(np.isfinite(initial_agent)):
        raise RuntimeError("Habitat returned an invalid initial position")
    if habitat_bounds.shape != (2, 3) or not np.all(np.isfinite(habitat_bounds)):
        raise RuntimeError("Habitat returned invalid navmesh bounds")

    initial_sensor = initial_agent + np.asarray([0.0, 1.0, 0.0])
    corners = np.asarray(
        [
            [x, y, z]
            for x in habitat_bounds[:, 0]
            for y in habitat_bounds[:, 1]
            for z in habitat_bounds[:, 2]
        ],
        dtype=np.float64,
    )
    falcon_corners = (
        S_HABITAT_TO_FALCON @ (corners - initial_sensor).T
    ).T + np.asarray([0.0, 0.0, 1.0])
    raw_min = falcon_corners.min(axis=0)
    raw_max = falcon_corners.max(axis=0)
    map_min, map_max = outward_bounds(
        raw_min, raw_max, args.horizontal_margin, args.vertical_margin
    )

    if np.any(map_max - map_min < 1.0):
        raise RuntimeError("generated map bounds are degenerate")
    initial_falcon = np.asarray([0.0, 0.0, 1.0])
    if np.any(initial_falcon <= map_min) or np.any(initial_falcon >= map_max):
        raise RuntimeError("generated bounds do not contain the initial FALCON pose")

    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_yaml.write_text(
        yaml_text(args.map_name, map_min, map_max), encoding="utf-8"
    )
    metadata = {
        "format": "pre_map_vln.generated_hm3d_3d_config.v1",
        "scene": str(args.scene),
        "scene_config": str(args.scene_config),
        "map_name": args.map_name,
        "seed": args.seed,
        "initial_agent_habitat_xyz": initial_agent.tolist(),
        "initial_sensor_habitat_xyz": initial_sensor.tolist(),
        "habitat_navmesh_bounds": habitat_bounds.tolist(),
        "falcon_raw_bounds": [raw_min.tolist(), raw_max.tolist()],
        "falcon_map_bounds": [map_min.tolist(), map_max.tolist()],
        "horizontal_margin": args.horizontal_margin,
        "vertical_margin": args.vertical_margin,
    }
    args.output_json.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(
        f"generated={args.output_yaml} initial_h={initial_agent.round(4).tolist()} "
        f"map_min={map_min.tolist()} map_max={map_max.tolist()}",
        flush=True,
    )


if __name__ == "__main__":
    main()
