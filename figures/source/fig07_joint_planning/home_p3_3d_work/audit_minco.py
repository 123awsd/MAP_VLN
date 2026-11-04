from pathlib import Path
import sys,json,math
import numpy as np,trimesh
sys.path.insert(0,'/tmp/fig7_project_code'); sys.path.insert(0,'/tmp/fig7_project_code/real_fly/stage2_runtime/scripts')
from saved_minco_artifact import read_trajectory
from stage2.planning_contract import PlannerProfile,MapIdentity
from stage2.voxel_map_3d import VoxelMap3D
H=Path(__file__).resolve().parent; d=json.loads((H/'conditional_plans.json').read_text()); a=np.load(H/'voxel_map_draft.npz'); p=PlannerProfile(**d['planner_profile']); vm=VoxelMap3D(a['raw_occupancy_zyx'],a['esdf_zyx_m'],a['origin_xyz_m'],float(a['resolution_m']),p,MapIdentity('x',1,1,'x','x',p.profile_hash)); lo,hi=d['height_band_m']; zs=vm.origin[2]+(np.arange(vm.raw.shape[0])+.5)*vm.resolution; vm.inflated_free[(zs<lo-1e-9)|(zs>hi+1e-9),:,:]=False
mesh=trimesh.load('/home/uav/下载/home_-_p3.glb',force='scene').to_geometry(); v=np.asarray(mesh.vertices).copy(); mesh.vertices=np.column_stack((v[:,0],-v[:,2],v[:,1]))*.0254
out={}
for cycle in ('initial','updated'):
 start,phases=read_trajectory(H/'minco_trial'/f'{cycle}_minco.txt')
 allpts=np.array([pt for ph in phases for pt in ph['points']]); bad=sum(not vm.is_state_valid(pt) for pt in allpts); bad_edges=0
 for ph in phases:
  bad_edges+=sum(not vm.line_is_valid(a,b) for a,b in zip(ph['points'],ph['points'][1:]))
 # Check the exact source mesh with sample spacing at most 0.04m.
 sub=[allpts[0]]
 for x in allpts[1:]:
  if np.linalg.norm(x-sub[-1])>=.035: sub.append(x)
 if np.linalg.norm(allpts[-1]-sub[-1])>1e-6: sub.append(allpts[-1])
 distances=[]
 for i in range(0,len(sub),20):
  _,dist,_=trimesh.proximity.closest_point(mesh,np.array(sub[i:i+20])); distances.extend(dist.tolist())
 out[cycle]={'phases':len(phases),'sample_count':len(allpts),'altitude_min_m':float(allpts[:,2].min()),'altitude_max_m':float(allpts[:,2].max()),'voxel_invalid_samples':bad,'voxel_invalid_sample_edges':bad_edges,'mesh_proximity_samples':len(sub),'mesh_min_sample_distance_m':float(min(distances)),'mesh_sampling_method':'approx 0.035 m spacing; no guarantee between samples'}
 print(cycle,out[cycle],flush=True)
(H/'minco_trial'/'independent_audit.json').write_text(json.dumps(out,indent=2))
