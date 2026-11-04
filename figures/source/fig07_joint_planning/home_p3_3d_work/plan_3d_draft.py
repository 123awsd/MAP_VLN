"""Experimental project candidate generator and 3D planner on sampled GLB.
Assumed inch units and surface-only occupancy; not measured flight evidence.
Run after unpacking the real-fly branch's stage2 and config/semantic_aliases.json
into /tmp/fig7_project_code. Never use this map for real flight.
"""
from pathlib import Path
import json,sys,hashlib,math
import numpy as np
from scipy.ndimage import distance_transform_edt
sys.path.insert(0,'/tmp/fig7_project_code')
from stage2.candidate_poses import generate_candidates
from stage2.planning_contract import PlannerProfile,MapIdentity
from stage2.voxel_map_3d import VoxelMap3D
from stage2.astar_3d import CoarseAstar3D
from stage2.motion_cost_oracle import MotionCostOracle
from stage2.joint_planner import plan_joint_mission
HERE=Path(__file__).resolve().parent
S=.0254; res=.1
cloud=np.load(HERE/'home_p3_surface_sample_draft.npz')['points_glb_xyz']*S
# The sampling region is treated as free except for sampled surfaces: this is
# an illustration-only assumption, not a sensor-observed free-space map.
origin=np.array([-10.3,-11.3,-.1]); dims=np.array([106,104,26])
idx=np.floor((cloud-origin)/res).astype(int)
valid=np.all((idx>=0)&(idx<dims),axis=1); idx=idx[valid]
occ=np.zeros(tuple(dims[::-1]),dtype=bool); occ[idx[:,2],idx[:,1],idx[:,0]]=True
raw=np.where(occ,100,0).astype(np.int8)
esdf=distance_transform_edt(~occ,sampling=res).astype(np.float32)
profile=PlannerProfile(vehicle_radius_xy_m=0.10,vehicle_radius_z_m=0.10,safety_margin_m=0.05,minimum_esdf_distance_m=0.15,coarse_resolution_m=0.10); identity=MapIdentity('home-p3-illustration',1,1,'sampled-glb','surface-only-assumed-free',profile.profile_hash)
vm=VoxelMap3D(raw,esdf,origin,res,profile,identity)
np.savez_compressed(HERE/'voxel_map_draft.npz',raw_occupancy_zyx=raw,esdf_zyx_m=esdf,origin_xyz_m=origin,resolution_m=res)
planner=CoarseAstar3D(vm); oracle=MotionCostOracle(planner)
bounds=json.loads((HERE/'target_selection_draft.json').read_text())['targets']
rooms=[]; tasks=[]
labels=['bed','desk','sink','sofa']
for n,(entry,label) in enumerate(zip(bounds,labels)):
 xmin,xmax,ymin,ymax,zmin,zmax=entry['bbox_xyz']; box=np.array([[xmin,ymin,zmin],[xmax,ymax,zmax]])*S
 center=box.mean(axis=0).tolist(); size=(box[1]-box[0]).tolist(); tid=chr(65+n)
 obj={'id':'object_'+tid,'label':label,'center_xyz_m':center,'size_xyz_m':size,'probability':1.0}
 rooms.append({'id':'room_'+tid,'floor_id':1,'semantic_type':['bedroom','study','bathroom','living'][n], 'camera_height_band_m':[.65,1.8], 'objects':[obj]})
 tasks.append({'id':tid,'target':{'label':label,'room_id':'room_'+tid},'spatial_constraints':{'visibility_required':True,'height_range_m':[0.8,1.6]},'prerequisites':[],'active_initially':True})
scene={'rooms':rooms}; graph={'tasks':tasks}
cs={}
for t in tasks:
 pool=generate_candidates(oracle,scene,t,max_candidates=90)
 chosen=[]
 for candidate in pool:
  angle=candidate['candidate_angle_rad']
  if all(abs(math.atan2(math.sin(angle-c['candidate_angle_rad']),math.cos(angle-c['candidate_angle_rad']))) >= math.radians(14) for c in chosen):
   chosen.append(candidate)
  if len(chosen)==3: break
 for candidate in pool:
  if len(chosen)>=3: break
  if candidate not in chosen: chosen.append(candidate)
 cs[t['id']]=chosen
 print(t['id'], 'pool',len(pool),'azimuth-diverse',len(chosen),flush=True)

print('candidate counts:',{k:len(v) for k,v in cs.items()},flush=True)
result={'status':'draft','assumptions':['unverified 0.0254 m per GLB unit','surface-sampled occupancy; remaining map cells assumed free','manually estimated 3D object boxes','not measured execution or certified collision free'],'targets':rooms,'tasks':graph,'candidates':cs,'planner_profile':profile.__dict__}
(HERE/'candidate_results_draft.json').write_text(json.dumps(result,indent=2),encoding='utf8')
if any(len(c)<3 for c in cs.values()):
 print('Insufficient candidates; joint planner intentionally not run.',flush=True); sys.exit(2)
# Starting position in open living-room area; project planner resolves reachability.
start=[-2.5,-7.0,1.1,0.0]
print('start valid:',vm.is_state_valid(start[:3]),flush=True)
try:
 plan=plan_joint_mission(oracle,graph,cs,start)
 result['start_xyz_yaw']=start; result['plan']=plan
 (HERE/'candidate_results_draft.json').write_text(json.dumps(result,indent=2),encoding='utf8')
 print('plan:',[(v.get('task_id'),v.get('candidate_id')) for v in plan['visits']], 'length:',plan['total_path_length_m'],flush=True)
except Exception as exc:
 result['planning_error']=str(exc); (HERE/'candidate_results_draft.json').write_text(json.dumps(result,indent=2),encoding='utf8')
 print('planning failed:',repr(exc),flush=True); raise
