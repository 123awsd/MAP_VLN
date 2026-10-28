#!/usr/bin/env python3
"""Render selected, logged camera poses at native 1600 x 1200 resolution.

Run with PRE_MAP_VLN/.envs/habitat/bin/python (local GPU required).
This is image replay, not another navigation or model-inference experiment.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import habitat_sim
import numpy as np
from habitat_sim.utils.common import quat_from_angle_axis
from PIL import Image

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1] / "PRE_MAP_VLN"
sys.path.insert(0, str(REPO))
from scripts.run_stage2_habitat import falcon_to_agent_h, sensor

EPISODES = [
    {
        "name": "00337_all_found_showcase_v1_20260830",
        "config": "outputs/stage2_3d/prepared/00337_reference/generated_stage1_config.json",
        "scene_id": "00337-CFVBbU9Rsyb",
        "selected": [("frame", 150), ("obs", 0), ("obs", 1),
                     ("obs", 2), ("frame", 900), ("obs", 3)],
    },
    {
        "name": "00166_cushion_recovery_20261024",
        "config": "outputs/stage2_3d/tasks/00166_cushion_recovery_20261024/generated_stage1_config.json",
        "scene_id": "00166-RaYrxWt5pR1",
        "selected": [("frame", 205), ("obs", 0), ("frame", 245),
                     ("frame", 380), ("frame", 420), ("obs", 3)],
    },
    {
        "name": "00070_pillow_conditional_20261024",
        "config": "outputs/stage2_3d/tasks/00070_pillow_conditional_20261024/generated_stage1_config.json",
        "scene_id": "00070-w7QyjJ3H9Bp",
        "selected": [("frame", 160), ("frame", 210), ("obs", 0),
                     ("frame", 310), ("frame", 324), ("obs", 1)],
    },
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    result = {
        "kind": "Native Habitat RGB replay at exact logged poses",
        "resolution_wh": [1600, 1200],
        "hfov_degrees": 90,
        "sensor_position_relative_to_agent": [0.0, 1.0, 0.0],
        "cropped": False,
        "retouched": False,
        "upscaled": False,
        "pose_indexing": "Motion frame k uses trajectory[k+1]; trajectory[0] is the initial pose. Terminal images use observation.pose.",
        "generator_sha256": sha(Path(__file__)),
        "episodes": [],
    }
    for episode in EPISODES:
        execution = REPO / "outputs/stage2_3d/tasks" / episode["name"] / "execution"
        trace = json.loads((execution / "habitat_execution.json").read_text())
        generated = json.loads((REPO / episode["config"]).read_text())
        assert Path(trace["scene"]).resolve() == Path(generated["scene"]).resolve()
        assert len(trace["trajectory_xyz_yaw"]) == trace["frame_count"] + 1
        simulator = habitat_sim.SimulatorConfiguration()
        simulator.scene_id = generated["scene"]
        simulator.scene_dataset_config_file = generated["scene_config"]
        simulator.enable_physics = False
        rgb = sensor("rgb_hd", habitat_sim.SensorType.COLOR)
        rgb.resolution = [1200, 1600]
        reference = sensor("rgb_reference", habitat_sim.SensorType.COLOR)
        agent_cfg = habitat_sim.agent.AgentConfiguration()
        agent_cfg.sensor_specifications = [rgb, reference]
        initial = np.asarray(generated["initial_agent_habitat_xyz"], dtype=np.float64)
        output_dir = ROOT / "keyframes" / episode["name"]
        output_dir.mkdir(parents=True, exist_ok=True)
        record = {"run": episode["name"], "scene_id": episode["scene_id"],
                  "config": episode["config"], "config_sha256": sha(REPO / episode["config"]),
                  "trace_sha256": sha(execution / "habitat_execution.json"), "frames": []}
        with habitat_sim.Simulator(habitat_sim.Configuration(simulator, [agent_cfg])) as sim:
            agent = sim.initialize_agent(0)
            for kind, index in episode["selected"]:
                if kind == "obs":
                    obs = trace["observations"][index]
                    pose = obs["pose"]
                    source = execution / obs["rgb"]
                    frame_index = obs["frame_index"]
                else:
                    pose = trace["trajectory_xyz_yaw"][index + 1]
                    source = execution / "frames" / f"frame_{index:06d}.jpg"
                    frame_index = index
                state = habitat_sim.AgentState()
                state.position = falcon_to_agent_h(initial, pose)
                state.rotation = quat_from_angle_axis(pose[3], np.asarray([0., 1., 0.]))
                agent.set_state(state)
                pixels = sim.get_sensor_observations()
                output = output_dir / f"{kind}_{index:06d}.png"
                Image.fromarray(pixels["rgb_hd"][..., :3]).save(output)
                # Validate the camera transform against the actual logged JPEG.
                original = np.asarray(Image.open(source).convert("RGB"), dtype=np.float32)
                replay = pixels["rgb_reference"][..., :3].astype(np.float32)
                difference = float(np.mean(np.abs(original - replay)))
                assert difference < 5.0, (source, "camera replay mismatch", difference)
                record["frames"].append({
                    "key": f"{kind}:{index}", "frame_index": frame_index,
                    "observation_sequence": index if kind == "obs" else None,
                    "pose": pose, "source_image": str(source.relative_to(REPO)),
                    "source_sha256": sha(source), "image": str(output.relative_to(ROOT)),
                    "image_sha256": sha(output),
                    "reference_640x480_mean_absolute_pixel_difference": difference,
                })
                print(episode["scene_id"], kind, index, "camera replay MAE:", round(difference, 3), flush=True)
        result["episodes"].append(record)
    (ROOT / "keyframes/render_manifest.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
