#!/usr/bin/env python3
"""Render a roofless, textured orthographic view of one HM3D floor."""

from __future__ import annotations

import argparse
from pathlib import Path

import habitat_sim
import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=Path,
        default=ROOT / "data/versioned_data/hm3d-0.2/hm3d/example/00861-GLAQ4DNUx5U/GLAQ4DNUx5U.basis.glb",
    )
    parser.add_argument(
        "--scene-config",
        type=Path,
        default=ROOT / "data/versioned_data/hm3d-0.2/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json",
    )
    parser.add_argument("--floor-height", type=float, default=1.207113265991211)
    parser.add_argument(
        "--cut-height",
        type=float,
        default=1.52,
        help="camera height above the selected floor; keep it below the ceiling",
    )
    parser.add_argument("--resolution", type=int, default=2048)
    parser.add_argument("--margin", type=float, default=0.45)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "outputs/topdown/hm3d_00861_floor_cutaway.png",
    )
    args = parser.parse_args()

    navmesh_path = args.scene.expanduser().resolve().with_suffix(".navmesh")
    pathfinder = habitat_sim.PathFinder()
    if not pathfinder.load_nav_mesh(str(navmesh_path)):
        raise RuntimeError("Habitat navmesh did not load: {}".format(navmesh_path))
    lower, upper = (np.asarray(value, dtype=np.float64) for value in pathfinder.get_bounds())
    span_x = float(upper[0] - lower[0])
    span_z = float(upper[2] - lower[2])
    scale = max(span_x, span_z) + 2.0 * args.margin

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(args.scene.expanduser().resolve())
    sim_cfg.scene_dataset_config_file = str(args.scene_config.expanduser().resolve())
    sim_cfg.enable_physics = False

    sensor = habitat_sim.CameraSensorSpec()
    sensor.uuid = "roofless_rgb"
    sensor.sensor_type = habitat_sim.SensorType.COLOR
    sensor.sensor_subtype = habitat_sim.SensorSubType.ORTHOGRAPHIC
    sensor.resolution = [args.resolution, args.resolution]
    # CameraSensorSpec Euler angles use Habitat's sensor-node convention; a
    # positive 90-degree X pitch points the optical axis toward world -Y.
    sensor.orientation = [0.5 * np.pi, 0.0, 0.0]
    sensor.ortho_scale = scale

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("Habitat navmesh did not load")
        state = habitat_sim.AgentState()
        state.position = np.asarray(
            [0.5 * (lower[0] + upper[0]), args.floor_height + args.cut_height,
             0.5 * (lower[2] + upper[2])],
            dtype=np.float32,
        )
        sim.initialize_agent(0, state)
        rgb = np.asarray(sim.get_sensor_observations()["roofless_rgb"])[..., :3].astype(np.uint8)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb).save(args.output)
    print(
        "wrote {} ({}x{}, floor={:.3f}m, cut={:.2f}m, span={:.2f}m)".format(
            args.output, args.resolution, args.resolution, args.floor_height,
            args.cut_height, scale,
        )
    )


if __name__ == "__main__":
    main()
