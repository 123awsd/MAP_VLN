from pathlib import Path
import json,sys,math
import numpy as np,yaml
sys.path.insert(0,'/tmp/fig7_project_code')
from stage2.planning_contract import PlannerProfile,MapIdentity
from stage2.voxel_map_3d import VoxelMap3D
H=Path(__file__).resolve().parent; O=H/'minco_start_room'; O.mkdir(exist_ok=True)
d=json.loads((H/'conditional_plans_start_room.json').read_text()); a=np.load(H/'voxel_map_draft.npz'); p=PlannerProfile(**d['planner_profile']); vm=VoxelMap3D(a['raw_occupancy_zyx'],a['esdf_zyx_m'],a['origin_xyz_m'],float(a['resolution_m']),p,MapIdentity('x',1,1,'x','x',p.profile_hash)); lo,hi=d['height_band_m']; zs=vm.origin[2]+(np.arange(vm.raw.shape[0])+.5)*vm.resolution; vm.inflated_free[(zs<lo-1e-9)|(zs>hi+1e-9),:,:]=False
for name,plan,start in [('initial',d['initial_plan'],d['start_xyz_yaw']),('updated',d['updated_plan'],[d['initial_plan']['visits'][0]['pose'][k] for k in ('x','y','z','yaw')])]:
 rows=[(*start[:3],.4,math.degrees(start[3]),0.,'')]
 for seg,visit in zip(plan['segments'],plan['visits']):
  pose=visit['pose']; pts=[seg['from_xyz_m']]+seg['points_xyz_m']+[[pose['x'],pose['y'],pose['z']]]
  # Greedy shortcuts only if each 3-D connector passes the same voxel predicate.
  j=0; indices=[0]
  while j<len(pts)-1:
   k=len(pts)-1
   while k>j+1 and not vm.line_is_valid(pts[j],pts[k]): k-=1
   indices.append(k); j=k
  for index in indices[1:]:
   last=index==len(pts)-1
   rows.append((*pts[index],.4,math.degrees(pose['yaw']) if last else float('nan'),1.0 if last else 0.,'stop' if last else 'pass'))
 (O/(name+'_route.txt')).write_text('# x y z speed yaw dwell mode\n'+''.join(' '.join(str(v) for v in row)+'\n' for row in rows)); print(name,'guide points',len(rows))
cloud=np.load(H/'home_p3_surface_sample_draft.npz')['points_glb_xyz']*.0254
# Downsample by surface voxel. Still an approximate map.
keys=np.floor(cloud/.05).astype(np.int32); _,ids=np.unique(keys,axis=0,return_index=True); cloud=cloud[ids].astype('<f4')
pcd=O/'collision.pcd'
with pcd.open('wb') as f:
 f.write((f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(cloud)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(cloud)}\nDATA binary\n').encode()); f.write(cloud.tobytes())
config=yaml.safe_load(Path('/tmp/fig7_project_code/real_fly/stage2_runtime/config/super_indoor_stage2.yaml').read_text()); config['super_planner']['robot_r']=.14; config['traj_opt']['boundary']['max_vel']=.6
(O/'planner.yaml').write_text(yaml.safe_dump(config,sort_keys=False)); print('collision points',len(cloud))
