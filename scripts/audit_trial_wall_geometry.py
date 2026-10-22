#!/usr/bin/env python3
"""Audit measured wall provenance and orientation, without claiming completeness."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--trial-dir',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();records=[]
with np.load(a.trial_dir/'geometry.npz')as geometry:
 for floor in range(3):
  points=geometry[f'p{floor}'];normals=geometry[f'n{floor}']
  with np.load(a.trial_dir/f'L{floor+1}_new_wall_points.npz')as data:walls=data['wall_points'];roof=data['ceiling_points']
  if not len(points):
   if len(walls):raise ValueError('Walls without observed source geometry')
   records.append({'floor':floor+1,'observations_insufficient':True});continue
  tree=cKDTree(points);distance,idx=tree.query(walls)if len(walls)else(np.array([]),np.array([],dtype=int))
  roof_distance=cKDTree(roof).query(walls)[0]if len(roof) and len(walls)else np.array([])
  records.append({'floor':floor+1,'source_points':len(points),'wall_points':len(walls),'wall_points_without_exact_measured_source':int((distance>1e-6).sum()),'maximum_source_distance_m':float(distance.max())if len(distance)else None,'wall_points_with_abs_normal_z_over_035':int((np.abs(normals[idx,2])>.350001).sum()),'wall_points_also_in_ceiling_set':int((roof_distance<1e-6).sum()),'wall_points_within_010m_of_ceiling':int((roof_distance<.1).sum()),'wall_quality_proven':False})
a.output.write_text(json.dumps({'floors':records,'scene_truth_used':False,'full_wall_correctness_proven':False,'limitations':'Exact measured support and vertical normals do not distinguish coplanar furniture from walls. Ceiling proximity can be legitimate wall attachment; omissions require independent observed-wall review.'},indent=2)+'\n');print(records)
