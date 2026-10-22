from pathlib import Path
import sys,json,itertools
import numpy as np
r=Path(__file__).resolve().parents[1];sys.path.insert(0,str(r/'scripts'))
import habitat_sim
from habitat_scene_repairs import load_scene_repair,mesh_safe_fraction
scene=r/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb'
cfg=habitat_sim.SimulatorConfiguration();cfg.scene_id=str(scene);cfg.scene_dataset_config_file=str(r/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json');cfg.enable_physics=True
agent=habitat_sim.agent.AgentConfiguration();agent.sensor_specifications=[]
S=np.array([[0,0,-1],[-1,0,0],[0,1,0]],float);origin=np.array([9.714411735534668,-2.6005380153656006,-2.374816417694092])
with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[agent])) as sim:
 objects=load_scene_repair(sim,r/'scene_repairs/00337_three_floor_shell/repair.json',scene)
 for name in ['exploration_structural_execution_504','exploration_structural_search_504']:
  out=r/'outputs'/name;rows=[json.loads(l) for l in (out/'live_trajectory.jsonl').read_text().splitlines()];p=np.array(rows[-1]['position']);start=origin+S.T@p
  tests=[]
  for d in itertools.product([-1,0,1],repeat=3):
   direction=np.array(d,float)
   if not direction.any():continue
   direction/=np.linalg.norm(direction)
   for length in [.1,.2,.4]:
    fraction=mesh_safe_fraction(sim,start,start+S.T@direction*length,radius=.07)
    tests.append({'direction_falcon':direction.tolist(),'length':length,'fraction':fraction})
  report={'method':'Offline diagnostic only: identical scene repair and mesh_safe_fraction(radius=.07), 26 directions at .1/.2/.4m; not planner guidance, not proof of a full exit path','actual_position':p.tolist(),'full_steps':[t for t in tests if t['fraction']>.999999],'max_fractions_by_length':{str(length):max(t['fraction'] for t in tests if t['length']==length) for length in [.1,.2,.4]}}
  (out/'escape_probe.json').write_text(json.dumps(report,indent=2));print('PROBE',name,json.dumps(report))
