#!/usr/bin/env python3
"""Load the pinned HM3D example and export one synchronized observation."""

from pathlib import Path

import habitat_sim
import numpy as np
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = PROJECT_ROOT / "data/scene_datasets/hm3d/example"
SCENE_DIR = SCENE_ROOT / "00861-GLAQ4DNUx5U"
SCENE = SCENE_DIR / "GLAQ4DNUx5U.basis.glb"
SCENE_CONFIG = SCENE_ROOT / "hm3d_annotated_example_basis.scene_dataset_config.json"
OUTPUT_DIR = PROJECT_ROOT / "outputs/smoke_habitat"


def sensor(uuid: str, sensor_type: habitat_sim.SensorType) -> habitat_sim.CameraSensorSpec:
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = uuid
    spec.sensor_type = sensor_type
    spec.resolution = [480, 640]
    spec.position = [0.0, 0.0, 0.0]
    spec.hfov = 90.0
    return spec


def main() -> None:
    if not SCENE.exists():
        raise FileNotFoundError(SCENE)

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(SCENE)
    sim_cfg.scene_dataset_config_file = str(SCENE_CONFIG)
    sim_cfg.enable_physics = True

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [
        sensor("rgb", habitat_sim.SensorType.COLOR),
        sensor("depth", habitat_sim.SensorType.DEPTH),
        sensor("semantic", habitat_sim.SensorType.SEMANTIC),
    ]

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("HM3D navmesh did not load")

        point = sim.pathfinder.get_random_navigable_point()
        state = habitat_sim.AgentState()
        state.position = point
        sim.initialize_agent(0, state)
        obs = sim.get_sensor_observations()

        rgb = np.asarray(obs["rgb"])[..., :3].astype(np.uint8)
        depth = np.asarray(obs["depth"], dtype=np.float32)
        semantic = np.asarray(obs["semantic"], dtype=np.int32)

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb).save(OUTPUT_DIR / "rgb.png")
        np.save(OUTPUT_DIR / "depth_m.npy", depth)
        np.save(OUTPUT_DIR / "semantic.npy", semantic)
        np.save(OUTPUT_DIR / "agent_position.npy", np.asarray(point))

        print(f"scene={SCENE}")
        print(f"position={np.asarray(point).tolist()}")
        print(f"rgb={rgb.shape} {rgb.dtype}")
        print(f"depth={depth.shape} {depth.dtype} range=({depth.min():.3f}, {depth.max():.3f})")
        print(f"semantic={semantic.shape} {semantic.dtype} unique={np.unique(semantic).size}")
        print(f"semantic_objects={len(sim.semantic_scene.objects)}")
        print(f"output={OUTPUT_DIR}")


if __name__ == "__main__":
    main()
