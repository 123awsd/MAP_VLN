"""Inspect generated candidate views in scene 00166; no forced outcomes."""
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("MAGNUM_LOG", "quiet")
os.environ.setdefault("HABITAT_SIM_LOG", "quiet")
ROOT = Path(__file__).resolve().parents[2] / "PRE_MAP_VLN"
sys.path.insert(0, str(ROOT))
OUT = ROOT / "outputs/stage2_3d/tasks/00166_branch_probe_20261023"
OUT.mkdir(exist_ok=True)
PREP = ROOT / "outputs/stage2_3d/prepared/00166_figure_20261023"

import habitat_sim
import numpy as np
from habitat_sim.utils.common import quat_from_angle_axis
from PIL import Image
from stage2.task_graph import normalize_and_validate_task_graph
from stage2.planning_contract import PlannerProfile
from stage2.voxel_map_3d import VoxelMap3D
from stage2.motion_cost_oracle import MotionCostOracle
from stage2.astar_3d import CoarseAstar3D
from stage2.candidate_poses import generate_all_candidates
from scripts.run_stage2_habitat import sensor, falcon_to_agent_h

scene = json.loads((PREP / "scene_graph.json").read_text())
profile = PlannerProfile.load(ROOT / "config/uav_3d_planning_habitat.yaml")
voxel = VoxelMap3D.load(PREP / "voxel_snapshot", profile)
oracle = MotionCostOracle(CoarseAstar3D(voxel), OUT / "motion_cache.json")
graph = normalize_and_validate_task_graph({
    "instruction": "Candidate-view inspection, not a navigation experiment.",
    "tasks": [
        {"id": task_id, "action": "find", "target": {"label": anchor, "room_id": room},
         "verification_label": "pillow", "spatial_constraints": {"relation": "on"},
         "prerequisites": []}
        for task_id, anchor, room in [("bedroom_chair", "armchair", "6"),
                                       ("living_chair", "armchair", "8"),
                                       ("living_sofa", "sofa", "8")]
    ], "conditional_rules": [],
})
candidates = generate_all_candidates(oracle, scene, graph, max_candidates=8)
(OUT / "candidates.json").write_text(json.dumps(candidates, indent=2))
generated = json.loads((PREP / "generated_stage1_config.json").read_text())
cfg = habitat_sim.SimulatorConfiguration()
cfg.scene_id = generated["scene"]
cfg.scene_dataset_config_file = generated["scene_config"]
cfg.enable_physics = False
agent_cfg = habitat_sim.agent.AgentConfiguration()
agent_cfg.sensor_specifications = [sensor("rgb", habitat_sim.SensorType.COLOR)]
records = []
with habitat_sim.Simulator(habitat_sim.Configuration(cfg, [agent_cfg])) as sim:
    agent = sim.initialize_agent(0)
    initial = np.asarray(generated["initial_agent_habitat_xyz"])
    for task_id, views in candidates.items():
        print(task_id, len(views), flush=True)
        for index, view in enumerate(views[:6]):
            pose = [view["pose"][key] for key in ("x", "y", "z", "yaw")]
            state = habitat_sim.AgentState()
            state.position = falcon_to_agent_h(initial, pose)
            state.rotation = quat_from_angle_axis(pose[3], np.asarray([0.,1.,0.]))
            agent.set_state(state)
            image = OUT / f"{task_id}_{index}.jpg"
            Image.fromarray(sim.get_sensor_observations()["rgb"][...,:3]).save(image, quality=95)
            records.append({"task_id": task_id, "index": index, "candidate_id": view["id"],
                            "object_id": view["object_id"], "pose": pose,
                            "reachable": oracle.path([0,0,1], pose[:3]) is not None,
                            "image": str(image)})
(OUT / "probe.json").write_text(json.dumps(records, indent=2))
