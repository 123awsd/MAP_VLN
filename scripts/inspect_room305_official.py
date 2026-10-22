"""Offline official textured scene inspection; no controller, physics, GT planner input."""
from pathlib import Path
import numpy as np,json
import habitat_sim,magnum as mn
from PIL import Image
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/room305_official';O.mkdir(exist_ok=True)
S=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);hab=lambda q:origin+S.T@(np.asarray(q)-[0,0,1])
cfg=habitat_sim.SimulatorConfiguration();cfg.scene_id=str(R/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb');cfg.scene_dataset_config_file=str(R/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json');cfg.enable_physics=False
spec=habitat_sim.CameraSensorSpec();spec.uuid='rgb';spec.sensor_type=habitat_sim.SensorType.COLOR;spec.resolution=[720,960];spec.position=mn.Vector3(0,0,0);spec.hfov=90
ag=habitat_sim.agent.AgentConfiguration();ag.sensor_specifications=[spec]
views=[('main_to_right',[-3.3,5.2,6.9],[-5.2,4.2,7.1]),('opening_to_right',[-3.6,4.25,7.1],[-5.6,4.2,7.1]),('right_to_main',[-5.2,4.2,7.1],[-3.5,4.5,6.8]),('main_sloped',[-2.4,5.4,6.8],[-4.4,5.3,6.7])]
with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[ag])) as sim:
 for name,pos,target in views:
  cp=mn.Vector3(*hab(pos));tp=mn.Vector3(*hab(target));node=sim.agents[0].scene_node;node.translation=cp;node.rotation=mn.Quaternion.from_matrix(mn.Matrix4.look_at(cp,tp,mn.Vector3(0,1,0)).rotation());rgb=sim.get_sensor_observations()['rgb'][...,:3];Image.fromarray(rgb).save(O/f'{name}.png')
(O/'manifest.json').write_text(json.dumps({'source':cfg.scene_id,'kind':'offline novel-view GT inspection, not actual capture','views':views,'transform_origin':origin.tolist(),'S_HF':S.tolist(),'physics':False},indent=2))
