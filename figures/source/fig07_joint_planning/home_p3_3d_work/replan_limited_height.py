"""Replan original candidate poses after removing B3/C2, with a 3D height band."""
from pathlib import Path
import json,sys
import numpy as np
sys.path.insert(0,'/tmp/fig7_project_code')
from stage2.planning_contract import PlannerProfile,MapIdentity
from stage2.voxel_map_3d import VoxelMap3D
from stage2.astar_3d import CoarseAstar3D
from stage2.motion_cost_oracle import MotionCostOracle
from stage2.joint_planner import plan_joint_mission
H=Path(__file__).resolve().parent
d=json.loads((H/'candidate_results_draft.json').read_text()); numbers=json.loads((H/'view_numbers.json').read_text())
removed={'B3','C2'}; number_by_id={v['candidate_id']:k for k,v in numbers.items()}; excluded={numbers[k]['candidate_id'] for k in removed}
d['candidates']={k:[c for c in v if c['id'] not in excluded] for k,v in d['candidates'].items()}
lo,hi=.65,1.75
profile_values=dict(d['planner_profile']); profile_values.update(safety_margin_m=.04,minimum_esdf_distance_m=.14); p=PlannerProfile(**profile_values); d['planner_profile']=profile_values; a=np.load(H/'voxel_map_draft.npz')
id=MapIdentity('home-p3-height-limited',2,1,'sampled-glb','surface-only-assumed-free-height-band',p.profile_hash)
m=VoxelMap3D(a['raw_occupancy_zyx'],a['esdf_zyx_m'],a['origin_xyz_m'],float(a['resolution_m']),p,id)
zs=m.origin[2]+(np.arange(m.raw.shape[0])+.5)*m.resolution
m.inflated_free[(zs<lo-1e-9)|(zs>hi+1e-9),:,:]=False
print('height band:',lo,hi,'remaining:',{k:[number_by_id[c['id']] for c in v] for k,v in d['candidates'].items()},flush=True)
planner=CoarseAstar3D(m); oracle=MotionCostOracle(planner)
plan=plan_joint_mission(oracle,d['tasks'],d['candidates'],d['start_xyz_yaw'])
for segment,visit in zip(plan['segments'],plan['visits']):
 pose=visit['pose']; pts=[segment['from_xyz_m']]+segment['points_xyz_m']+[[pose['x'],pose['y'],pose['z']]]
 if not all(m.line_is_valid(a,b) for a,b in zip(pts,pts[1:])): raise RuntimeError('Failed connector or path validation')
 if min(p[2] for p in pts)<lo or max(p[2] for p in pts)>hi: raise RuntimeError('Failed height validation')
visit_numbers=[number_by_id[v['candidate_id']] for v in plan['visits']]
d['assumptions'].append('small illustrative drone radius 0.10 m with reduced 0.04 m map safety margin'); d['plan']=plan; d['height_band_m']=[lo,hi]; d['removed_review_numbers']=sorted(removed); d['candidate_review_numbers']=number_by_id; d['status']='height-limited draft'; d['assumptions'].append('flight altitude limited to 0.65–1.75 m under assumed scene scale')
(H/'candidate_results_height_limited.json').write_text(json.dumps(d,indent=2),encoding='utf8')
print('order:',visit_numbers,'length:',plan['total_path_length_m'],flush=True)
print('actual height:',min(p[2] for s in plan['segments'] for p in s['points_xyz_m']),max(p[2] for s in plan['segments'] for p in s['points_xyz_m']),flush=True)
