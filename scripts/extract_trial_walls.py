#!/usr/bin/env python3
"""Extract new depth-supported walls for a recorded trial with fixed baseline settings."""
import argparse,hashlib
from pathlib import Path
import importlib.util,json,sys
import numpy as np
from scipy.spatial import cKDTree
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--trial-dir',required=True,type=Path)
parser.add_argument('--export-name',default='synchronized_export_matched')
parser.add_argument('--metrics',type=Path,default=root/'outputs/wall_refinement/round11/metrics.json')
args=parser.parse_args();out=args.trial_dir
if (out/'geometry.npz').exists() or (out/'new_wall_extraction_manifest.json').exists():raise FileExistsError('Existing extraction: use a fresh trial directory')
spec=importlib.util.spec_from_file_location('walls',root/'scripts/refine_measured_walls.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
point_file=out/args.export_name/'observed_points.npz'
raw=np.load(point_file)['points'];metrics=json.loads(args.metrics.read_text());data={};records=[]
provenance={'point_file':str(point_file),'point_sha256':hashlib.sha256(point_file.read_bytes()).hexdigest(),'settings_file':str(args.metrics),'settings_sha256':hashlib.sha256(args.metrics.read_bytes()).hexdigest(),'offline_floor_bands':[[-.25,2.5],[2.5,5.5],[5.5,8.05]],'full_wall_quality_proven':False}
(out/'wall_extraction_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
for f,(lo,hi) in enumerate([(-.25,2.5),(2.5,5.5),(5.5,8.05)]):
 p=raw[(raw[:,2]>lo)&(raw[:,2]<hi)];print('normal estimation floor',f+1,'points',len(p),flush=True)
 if len(p)<30:
  data[f'p{f}']=p;data[f'n{f}']=np.empty_like(p)
  np.savez_compressed(out/f'L{f+1}_new_wall_points.npz',wall_points=np.empty((0,3)),ceiling_points=np.empty((0,3)))
  (out/f'L{f+1}_new_wall_patches.json').write_text('[]\n')
  records.append({'floor':f+1,'status':'insufficient_observations','points':len(p)});continue
 tree=cKDTree(p);n=np.empty_like(p)
 for start in range(0,len(p),5000):
  _,idx=tree.query(p[start:start+5000],k=20,workers=2);near=p[idx];near-=near.mean(1,keepdims=True);cov=np.einsum('nki,nkj->nij',near,near);_,v=np.linalg.eigh(cov);n[start:start+len(idx)]=v[:,:,0]
 data[f'p{f}']=p;data[f'n{f}']=n
 cfg=next(m['config']for m in metrics if m['floor']==f+1 and m['method'].startswith('AE_'))
 walls,roof,patches,roofplanes=mod.extract(p,n,mod.GROUNDS[f],cfg)
 np.savez_compressed(out/f'L{f+1}_new_wall_points.npz',wall_points=walls,ceiling_points=roof)
 (out/f'L{f+1}_new_wall_patches.json').write_text(json.dumps(patches,indent=2))
 records.append({'floor':f+1,'input_points':len(p),'wall_points':len(walls),'wall_patches':len(patches),'roof_planes':roofplanes,'config':cfg,'source':str(point_file),'status':'fresh_extraction; quality_unverified'})
 (out/'new_wall_extraction_manifest.json').write_text(json.dumps(records,indent=2));print('extracted',f+1,len(walls),len(patches),flush=True)
np.savez_compressed(out/'geometry.npz',**data)
(out/'new_wall_extraction_manifest.json').write_text(json.dumps(records,indent=2)+'\n')
