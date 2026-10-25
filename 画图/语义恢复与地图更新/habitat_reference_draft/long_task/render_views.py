"""Render candidate viewpoints for an authored long-task mechanism example."""
from pathlib import Path
import json
import math
import numpy as np
from PIL import Image, ImageDraw
import habitat_sim
from habitat_sim.utils.common import quat_from_angle_axis

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
(OUT/'assets').mkdir(exist_ok=True)
cfg=json.loads((ROOT/'PRE_MAP_VLN/outputs/stage2_3d/tasks/00166_cushion_recovery_20261024/generated_stage1_config.json').read_text())
R=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
origin=np.array(cfg['initial_sensor_habitat_xyz'])
candidates=[
 ('start',[0,0,1.35],[-2,-.4,1.1]),
 ('kitchen_a',[-.856,-4.776,1.5],[.513,-4.373,.85]),
 ('kitchen_b',[-.9,-5.4,1.55],[.513,-4.373,.85]),
 ('chair_a',[-3.453,-7.55,1.3],[-4.70,-7.55,.65]),
 ('chair_b',[-3.3,-7.5,1.35],[-4.70,-7.55,.65]),
 ('retry_a',[-3.966,-7.1305,1.35],[-4.70,-7.55,.65]),
 ('retry_b',[-3.55,-7.25,1.35],[-4.70,-7.55,.65]),
 ('retry_c',[-3.75,-7.75,1.35],[-4.70,-7.55,.65]),
 ('sofa_a',[-4.12,-5.29,1.25],[-4.03,-3.43,.75]),
 ('sofa_b',[-4.5,-5.3,1.30],[-4.4,-3.43,.7]),
 ('sofa_c',[-4.7,-5.9,1.35],[-4.3,-3.4,.7]),
 ('dining_a',[-1.528,-4.015,1.35],[-.946,-2.537,.80]),
 ('dining_b',[-.85,-4.15,1.4],[-.946,-2.537,.80]),
]
sc=habitat_sim.SimulatorConfiguration()
sc.scene_id=cfg['scene'];sc.scene_dataset_config_file=cfg['scene_config'];sc.enable_physics=False
spec=habitat_sim.CameraSensorSpec()
spec.uuid='rgb';spec.sensor_type=habitat_sim.SensorType.COLOR
spec.resolution=[1000,1600];spec.position=[0.,0.,0.];spec.hfov=74.
agent=habitat_sim.agent.AgentConfiguration();agent.sensor_specifications=[spec]
records=[]
with habitat_sim.Simulator(habitat_sim.Configuration(sc,[agent])) as sim:
    robot=sim.initialize_agent(0)
    for name,pos,target in candidates:
        d=np.array(target)-np.array(pos)
        yaw=math.atan2(d[1],d[0]);pitch=math.atan2(d[2],np.linalg.norm(d[:2]))
        state=habitat_sim.AgentState()
        state.position=origin+R.T@(np.array(pos)-[0,0,1])
        state.rotation=quat_from_angle_axis(yaw,np.array([0,1,0]))*quat_from_angle_axis(pitch,np.array([1,0,0]))
        robot.set_state(state)
        im=Image.fromarray(sim.get_sensor_observations()['rgb'][...,:3])
        im.save(OUT/'assets'/f'{name}.png')
        records.append(dict(name=name,pose_falcon=pos,yaw=yaw,pitch=pitch,target_falcon=target))
        print(name,flush=True)
(OUT/'view_manifest.json').write_text(json.dumps(dict(
    kind='Native Habitat RGB renders at authored camera poses; mechanism illustration, not planner output.',
    scene=cfg['scene'],resolution=[1600,1000],hfov=74,retouched=False,views=records),indent=2)+'\n')
sheet=Image.new('RGB',(1280,math.ceil(len(candidates)/3)*227),'white');draw=ImageDraw.Draw(sheet)
for i,(name,_,_) in enumerate(candidates):
    x=(i%3)*426;y=(i//3)*227
    im=Image.open(OUT/'assets'/f'{name}.png');im.thumbnail((416,200));sheet.paste(im,(x,y+22));draw.text((x+4,y+4),name,fill='black')
sheet.save(OUT/'view_contact.png')
