from pathlib import Path
import sys,json,numpy as np,cv2,habitat_sim
from habitat_sim.utils.common import quat_from_angle_axis
p=Path(__file__).resolve().parents[2];sys.path.insert(0,str(p/'scripts'))
from habitat_scene_repairs import load_scene_repair,mesh_safe_fraction
from run_habitat_falcon import sensor,follow_command
S=np.array([[0,0,-1],[-1,0,0],[0,1,0]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092])
def to_h(f):return origin+S.T@(np.asarray(f)-[0,0,1])
c=habitat_sim.SimulatorConfiguration();c.scene_id=str(p/'data/versioned_data/hm3d-0.2/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb');c.scene_dataset_config_file=str(Path(c.scene_id).parent.parent/'hm3d_annotated_basis.scene_dataset_config.json');c.enable_physics=True
ag=habitat_sim.agent.AgentConfiguration();ag.sensor_specifications=[sensor('rgb',habitat_sim.SensorType.COLOR),sensor('depth',habitat_sim.SensorType.DEPTH)]
r=Path(__file__).parent;result={}
with habitat_sim.Simulator(habitat_sim.Configuration(c,[ag]))as sim:
 pos=[1.80,9.5,6.8];state=habitat_sim.AgentState();state.position=to_h(pos)-[0,1,0];state.rotation=quat_from_angle_axis(np.pi,np.array([0.,1.,0.]));agent=sim.initialize_agent(0,state)
 obs=sim.get_sensor_observations();cv2.imwrite(str(r/'before_rgb.jpg'),obs['rgb'][:,:,:3][:,:,::-1])
 objs=load_scene_repair(sim,r/'repair.json',c.scene_id)
 obs=sim.get_sensor_observations();cv2.imwrite(str(r/'after_rgb.jpg'),obs['rgb'][:,:,:3][:,:,::-1]);np.save(r/'after_depth.npy',obs['depth'])
 result['static_objects']=len(objs);result['depth_center_m']=float(obs['depth'][240,320])
 hits=sim.cast_ray(habitat_sim.geo.Ray(to_h([1.65,9.5,6.5]).astype(np.float32),(S.T@np.array([-1.,0,0])).astype(np.float32)),1.,0.).hits
 result['new_wall_ray_hits']=[{'distance':float(h.ray_distance),'object_id':int(h.object_id)}for h in hits if h.object_id>0]
 assert result['new_wall_ray_hits'],'New virtual wall is not in collision scene'
 assert abs(result['depth_center_m']-.26)<.03,'Depth camera did not see the new wall at its configured location'
 start=np.array([1.251862,7.059984,5.084431]);end=np.array([1.344932,6.90054,5.221329])
 state=agent.get_state();state.position=to_h(start)-[0,1,0];agent.set_state(state)
 cmd={'position':end.tolist(),'yaw':0.,'trajectory_id':999}
 follow_command(sim,origin-[0,1,0],cmd,.2,use_planner_yaw=True,allow_vertical_motion=True,scene_collision_check=True)
 actual=S@(np.asarray(agent.get_state().sensor_states['depth'].position)-origin)+[0,0,1]
 result['bad_transition_blocked']=cmd['_geometry_blocked'];result['bad_transition_actual_falcon']=actual.tolist()
 assert result['bad_transition_blocked'],'Recorded mesh crossing was not blocked'
 stair=[]
 for x in np.linspace(-4.35,-.30,15):
  z=2.8+(x+4.57)/4.3*2.6+.8;stair.append([x,8.45,z])
 fractions=[mesh_safe_fraction(sim,to_h(a),to_h(b))for a,b in zip(stair,stair[1:])]
 result['stairs_min_safe_fraction']=float(min(fractions));result['stairs_blocked_segments']=sum(f<.999 for f in fractions)
 # Check the new sealing mesh never intersects the genuine sloping stair route.
 repair_ids={o.object_id for o in objs};repair_hits=[]
 for a,b in zip(stair,stair[1:]):
  delta=to_h(b)-to_h(a);d=float(np.linalg.norm(delta));hits=sim.cast_ray(habitat_sim.geo.Ray(to_h(a).astype(np.float32),(delta/d).astype(np.float32)),d,0.).hits
  repair_hits +=[int(h.object_id)for h in hits if h.object_id in repair_ids]
 result['stairs_repair_intersections']=repair_hits;assert not repair_hits,'Repair closed the original staircase'
 assert result['stairs_min_safe_fraction']==1.0,'Geometry check blocked the original staircase'
(r/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
