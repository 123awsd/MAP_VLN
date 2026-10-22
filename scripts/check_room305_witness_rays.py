from pathlib import Path
import json,numpy as np,habitat_sim
from habitat_scene_repairs import load_scene_repair
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/room305_provenance';a=json.loads((O/'audit.json').read_text());d=np.load(O/'depth_witnesses.npz');S=np.array([[0,0,-1],[-1,0,0],[0,1,0]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);hab=lambda p:origin+S.T@(np.asarray(p)-[0,0,1]);scene=R/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb';cfg=habitat_sim.SimulatorConfiguration();cfg.scene_id=str(scene);cfg.scene_dataset_config_file=str(R/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json');cfg.create_renderer=False;cfg.enable_physics=True;ag=habitat_sim.agent.AgentConfiguration();ag.sensor_specifications=[];rows=[]
with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[ag])) as sim:
 objects=load_scene_repair(sim,R/'scene_repairs/00337_three_floor_shell/repair.json',scene)
 print('OBJECTS',[(v.object_id,v.handle)for v in objects],flush=True)
 for b in a['best']:
  f=next(f for f in a['frames']if f['stamp']==b['stamp']);p=d['points'+str(b['index'])][::max(1,b['pixels']//100)];cp=hab(f['position']);diff=[];obj=[]
  for q in p:
   vec=hab(q)-cp;length=np.linalg.norm(vec);hits=sim.cast_ray(habitat_sim.geo.Ray(cp,vec/length),float(length+.15),0.).hits
   if hits:diff.append(float(hits[0].ray_distance)-length);obj.append(int(hits[0].object_id))
  rows.append({'index':b['index'],'elapsed':f['elapsed'],'tested':len(p),'hits':len(diff),'first_hit_matches_3cm':sum(abs(v)<.03 for v in diff),'earlier_hit_over_3cm':sum(v<-.03 for v in diff),'object_ids':sorted(set(obj)), 'hit_counts':{str(k):obj.count(k) for k in set(obj)}})
(Path('/tmp/room305_ray_check.json')).write_text(json.dumps(rows,indent=2,default=int));print(rows)
