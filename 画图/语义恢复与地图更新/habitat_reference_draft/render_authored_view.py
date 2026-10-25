"""Render a native Habitat RGB view at an illustrative, explicitly authored pose."""
from pathlib import Path
import json
import math
import numpy as np
import habitat_sim
from habitat_sim.utils.common import quat_from_angle_axis
from PIL import Image

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
cfg=json.loads((ROOT/'PRE_MAP_VLN/outputs/stage2_3d/tasks/00166_cushion_recovery_20261024/generated_stage1_config.json').read_text())
pos=[-2.65,-5.25,1.0]
target=[-4.03,-3.43,.75]
yaw=math.atan2(target[1]-pos[1],target[0]-pos[0])
rot=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
sc=habitat_sim.SimulatorConfiguration()
sc.scene_id=cfg['scene'];sc.scene_dataset_config_file=cfg['scene_config'];sc.enable_physics=False
spec=habitat_sim.CameraSensorSpec()
spec.uuid='rgb';spec.sensor_type=habitat_sim.SensorType.COLOR
spec.resolution=[1200,1600];spec.position=[0.,1.,0.];spec.hfov=90.
agent=habitat_sim.agent.AgentConfiguration();agent.sensor_specifications=[spec]
with habitat_sim.Simulator(habitat_sim.Configuration(sc,[agent])) as sim:
    state=habitat_sim.AgentState()
    state.position=np.array(cfg['initial_agent_habitat_xyz'])+rot.T@(np.array(pos)-[0,0,1])
    state.rotation=quat_from_angle_axis(yaw,np.array([0,1,0]))
    sim.initialize_agent(0).set_state(state)
    rgb=sim.get_sensor_observations()['rgb'][...,:3]
    Image.fromarray(rgb).save(OUT/'assets/recovery_observation.png')
(OUT/'assets/recovery_observation.json').write_text(json.dumps(dict(
    kind='Native Habitat render at an authored pose; not an algorithm-run result.',
    pose_falcon=pos+[yaw],source_scene=cfg['scene'],hfov=90,resolution=[1600,1200],
    retouched=False),indent=2)+'\n')
print('Authored view rendered.')
