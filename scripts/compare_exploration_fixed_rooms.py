#!/usr/bin/env python3
"""Offline surface support in fixed room regions; never used by planning."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('baseline');p.add_argument('trial');a=p.parse_args();base=Path(a.baseline);out=Path(a.trial)
base_geometry=base/'evaluation_geometry.npz' if (base/'evaluation_geometry.npz').exists() else base/'original_style_video/geometry.npz'
b=np.load(base_geometry);n=np.load(out/'evaluation_geometry.npz');reference=Path(__file__).resolve().parents[1]/'outputs/room_wall_guided_trial/adaptive_merge'
rows=[]
for floor,ground in enumerate([-.155,2.595,5.595],1):
 s=np.load(reference/f'L{floor}/segmentation.npz');labels=s['labels'];origin=s['origin'];resolution=float(s['resolution'])
 def groups(points):
  q=points[(points[:,2]>ground+.15)&(points[:,2]<ground+2.2)]
  ij=np.floor((q[:,:2]-origin)/resolution).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<labels.shape[1])&(ij[:,1]>=0)&(ij[:,1]<labels.shape[0]);q=q[ok];ij=ij[ok];lab=labels[ij[:,1],ij[:,0]]
  return {int(k):np.unique(np.floor(q[lab==k]/.1).astype(np.int32),axis=0) for k in np.unique(labels[labels>=0])}
 before=groups(b[f'p{floor-1}']);after=groups(n[f'p{floor-1}'])
 for room,old in before.items():
  new=after[room];oldset={tuple(v) for v in old.tolist()};newset={tuple(v) for v in new.tolist()}
  matched=sum(any((x+dx,y+dy,z+dz) in newset for dx in [-1,0,1] for dy in [-1,0,1] for dz in [-1,0,1]) for x,y,z in oldset)
  rows.append({'floor':floor,'fixed_region':room,'baseline_surface_cells':len(oldset),'trial_surface_cells':len(newset),'baseline_surface_reobserved_within_one_neighbor':matched/len(oldset) if len(oldset) else None})
report={'baseline':base.name,'trial':out.name,'method':'Fixed existing segmentation regions; 10cm surface support, ±1 voxel matching. Historical sampled surface reference, not GT completeness and not room semantic labels. Counts depend on viewpoint and measurement sampling.','regions':rows}
(out/'fixed_room_comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
