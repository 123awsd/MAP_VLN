#!/usr/bin/env python3
"""Load one HM3D scene and export a synchronized RGB-D observation."""

import argparse
import os
from pathlib import Path

import habitat_sim
import numpy as np
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def sensor(uuid: str, sensor_type: habitat_sim.SensorType) -> habitat_sim.CameraSensorSpec:
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = uuid
    spec.sensor_type = sensor_type
    spec.resolution = [480, 640]
    spec.position = [0.0, 0.0, 0.0]
    spec.hfov = 90.0
    return spec


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", default="00166-RaYrxWt5pR1")
    parser.add_argument("--scene-root", type=Path, default=None)
    parser.add_argument("--scene-config", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if "-" not in args.scene_id:
        parser.error("--scene-id must include numeric ID and HM3D token")
    token = args.scene_id.split("-", 1)[1]
    scene_root = args.scene_root or Path(os.environ.get(
        "PRE_MAP_VLN_HM3D_TRAIN_ROOT",
        "/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train",
    ))
    scene_config = args.scene_config or Path(os.environ.get(
        "PRE_MAP_VLN_HM3D_SCENE_CONFIG",
        "/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/hm3d_annotated_basis.scene_dataset_config.json",
    ))
    scene = scene_root / args.scene_id / f"{token}.basis.glb"
    output_dir = args.output or PROJECT_ROOT / "outputs/smoke_habitat" / args.scene_id
    for path in (scene, scene_config):
        if not path.is_file():
            raise FileNotFoundError(path)

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(scene)
    sim_cfg.scene_dataset_config_file = str(scene_config)
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

        output_dir.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb).save(output_dir / "rgb.png")
        np.save(output_dir / "depth_m.npy", depth)
        np.save(output_dir / "semantic.npy", semantic)
        np.save(output_dir / "agent_position.npy", np.asarray(point))

        print(f"scene={scene}")
        print(f"position={np.asarray(point).tolist()}")
        print(f"rgb={rgb.shape} {rgb.dtype}")
        print(f"depth={depth.shape} {depth.dtype} range=({depth.min():.3f}, {depth.max():.3f})")
        print(f"semantic={semantic.shape} {semantic.dtype} unique={np.unique(semantic).size}")
        print(f"semantic_objects={len(sim.semantic_scene.objects)}")
        print(f"output={output_dir}")


if __name__ == "__main__":
    main()
