"""Run project joint planner and mission state for a conditional figure draft."""
from pathlib import Path
import json,sys,copy
import numpy as np
sys.path.insert(0,'/tmp/fig7_project_code')
from stage2.planning_contract import PlannerProfile,MapIdentity
from stage2.voxel_map_3d import VoxelMap3D
from stage2.astar_3d import CoarseAstar3D
from stage2.motion_cost_oracle import MotionCostOracle
from stage2.joint_planner import plan_joint_mission
from stage2.mission_executor import MissionState
H=Path(__file__).resolve().parent
d=json.loads((H/'candidate_results_height_limited.json').read_text()); a=np.load(H/'voxel_map_draft.npz'); p=PlannerProfile(**d['planner_profile'])
id=MapIdentity('home-p3-conditional',3,1,'sampled-glb','surface-only-assumed-free-height-band',p.profile_hash)
m=VoxelMap3D(a['raw_occupancy_zyx'],a['esdf_zyx_m'],a['origin_xyz_m'],float(a['resolution_m']),p,id)
lo,hi=d['height_band_m']; zs=m.origin[2]+(np.arange(m.raw.shape[0])+.5)*m.resolution; m.inflated_free[(zs<lo-1e-9)|(zs>hi+1e-9),:,:]=False
oracle=MotionCostOracle(CoarseAstar3D(m)); graph=copy.deepcopy(d['tasks'])
for t in graph['tasks']:
 t['active_initially']=t['id']!='D'; t['success_outcome']='found'
 if t['id'] in ('B','C'): t['prerequisites']=['A']
graph['conditional_rules']=[{'source_task_id':'A','if_outcome':'not_found','activate_task_ids':['D'],'skip_task_ids':[]}]
start=[-7.6,-9.2,1.2,0.0]
if not m.is_state_valid(start[:3]):
 xyz=m.nearest_valid(start[:3],.8)
 if xyz is None: raise ValueError('No valid start near bedroom')
 start[:3]=xyz
print('start:',start,flush=True)
initial=plan_joint_mission(oracle,graph,d['candidates'],start)
print('initial order:',[v['task_id'] for v in initial['visits']],flush=True)
if initial['visits'][0]['task_id']!='A': raise RuntimeError('Initial planner did not visit conditional source A first')
state=MissionState(graph); state.finish_task('A','not_found'); v=initial['visits'][0]; pose=[v['pose'][k] for k in ('x','y','z','yaw')]
updated_graph=copy.deepcopy(graph)
for task in updated_graph['tasks']:
 if task['id'] in ('B','C'): task['prerequisites']=['A','D']
updated=plan_joint_mission(oracle,updated_graph,d['candidates'],pose,active_task_ids=state.active,completed_task_ids=state.completed)
print('updated order:',[v['task_id'] for v in updated['visits']],flush=True)
for label,plan in [('initial',initial),('updated',updated)]:
 for seg,visit in zip(plan['segments'],plan['visits']):
  p=visit['pose']; pts=[seg['from_xyz_m']]+seg['points_xyz_m']+[[p['x'],p['y'],p['z']]]
  if not all(m.line_is_valid(a,b) for a,b in zip(pts,pts[1:])): raise RuntimeError(label+' invalid path')
  if min(v[2] for v in pts)<lo or max(v[2] for v in pts)>hi: raise RuntimeError(label+' invalid altitude')
found=MissionState(graph); found.finish_task('A','found')
result={'status':'conditional_planning_draft','assumptions':d['assumptions'],'height_band_m':d['height_band_m'],'planner_profile':d['planner_profile'],'targets':d['targets'],'candidates':d['candidates'],'candidate_review_numbers':d['candidate_review_numbers'],'removed_review_numbers':d['removed_review_numbers'],'task_graph':graph,'start_xyz_yaw':start,'initial_plan':initial,'observation':{'task_id':'A','outcome':'not_found','events':state.events,'active':sorted(state.active),'completed':sorted(state.completed)},'updated_plan':updated,'updated_task_graph':updated_graph,'scheduling_rule':'explicit scenario priority: visit fallback D before B/C after A not_found','found_branch':{'events':found.events,'status':found.status}}
(H/'conditional_plans_start_room.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print('lengths:',initial['total_path_length_m'],updated['total_path_length_m'],flush=True)
