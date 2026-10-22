#!/usr/bin/env python3
"""Infer broad observed horizontal surfaces seen from above, for offline assessment.

Never supplies planner goals. Area/normal evidence does not prove floor semantics.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from audit_boxer_depth_support import rotation
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--geometry',type=Path,required=True)
p.add_argument('--episode',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
with np.load(a.geometry)as f:
 points=np.concatenate([f[f'p{i}']for i in range(3)]);normals=np.concatenate([f[f'n{i}']for i in range(3)])
tree=cKDTree(points);manifest=json.loads((a.episode/'manifest.json').read_text());fx,fy,cx,cy=[manifest[k]for k in ['fx','fy','cx','cy']];surface={}
for path in sorted(a.episode.glob('frame_*.npz')):
 with np.load(path)as f:depth=f['depth_m'];pos=f['position'];R=rotation(f['orientation_xyzw'])
 v,u=np.mgrid[0:depth.shape[0]:6,0:depth.shape[1]:6];u,v=u.ravel(),v.ravel();z=depth[v,u];ok=np.isfinite(z)&(z>.1)&(z<=5);u,v,z=u[ok],v[ok],z[ok];world=np.c_[(u-cx)*z/fx,(v-cy)*z/fy,z]@R.T+pos
 distance,index=tree.query(world);keep=(distance<.08)&(np.abs(normals[index,2])>.95)&(pos[2]-world[:,2]>.25);q=world[keep]
 bins=np.unique(np.floor(q/np.array([.15,.15,.03])).astype(int),axis=0)
 for x,y,z in bins:surface.setdefault(int(z),set()).add((int(x),int(y)))
records=[];remaining=set(surface)
while remaining:
 centre=max(remaining,key=lambda z:len(surface[z]));near={z for z in remaining if abs(z-centre)<=2};xy=set().union(*(surface[z]for z in near));weight=sum(len(surface[z])for z in near);height=sum((z+.5)*.03*len(surface[z])for z in near)/weight
 records.append({'observed_plane_z_m':height,'measured_xy_support_m2':len(xy)*.15**2,'height_bin_min_max_m':[min(near)*.03,(max(near)+1)*.03],'floor_candidate_area_over_20m2':len(xy)*.15**2>=20});remaining-=near
records.sort(key=lambda r:r['observed_plane_z_m'])
a.output.write_text(json.dumps({'source_episode':str(a.episode),'source_geometry':str(a.geometry),'planes':records,'broad_floor_candidates':[r for r in records if r['floor_candidate_area_over_20m2']],'scene_truth_used':False,'floor_semantics_proven':False,'limitations':'Horizontal normals and same-frame camera-above evidence exclude many ceilings/furniture, but large elevated surfaces and incomplete floors still need review. This is offline assessment, not planner input.'},indent=2)+'\n')
print([r for r in records if r['floor_candidate_area_over_20m2']])
