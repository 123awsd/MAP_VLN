from pathlib import Path
import numpy as np,json,habitat_sim
p=Path(__file__).resolve().parents[2];c=habitat_sim.SimulatorConfiguration();c.scene_id=str(p/'data/versioned_data/hm3d-0.2/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb');c.scene_dataset_config_file=str(Path(c.scene_id).parent.parent/'hm3d_annotated_basis.scene_dataset_config.json');c.enable_physics=True;ag=habitat_sim.agent.AgentConfiguration();ag.sensor_specifications=[]
S=np.array([[0,0,-1],[-1,0,0],[0,1,0]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);rows=[]
with habitat_sim.Simulator(habitat_sim.Configuration(c,[ag]))as sim:
 for y in [4,5,6,7,7.5,8.5,9,10]:
  for x in [0,1,2,3,4]:
   pos=np.array([x,y,4.0]);h=origin+S.T@(pos-[0,0,1]);hits=sim.cast_ray(habitat_sim.geo.Ray(h.astype(np.float32),np.array([0,1,0],np.float32))).hits
   vals=[float(v.ray_distance)+4.0 for v in hits if float(v.ray_distance)>.05];rows.append({'x':x,'y':y,'first_surface_z':min(vals)if vals else None})
 (Path(__file__).parent/'ceiling_probes.json').write_text(json.dumps(rows,indent=2));print(rows)
