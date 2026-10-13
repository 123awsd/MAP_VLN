"""Audit measured wall candidates against recorded 3D depth free-space rays.

No RGB, predicted semantics or flattened XY rays. Endpoint margin and incidence
angle prevent surface returns and grazing rays from being counted as crossings.
"""
from pathlib import Path
import json
import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/wall_refinement'

def main():
    geo=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz')
    cache=OUT/'depth_crossing_counts.npz'
    if not cache.exists():
        rays=np.load(ROOT/'outputs/wall_occlusion_completion/fullheight_rays.npz')
        trees=[cKDTree(geo[f'p{f}'])for f in range(3)]
        counts=[np.zeros(len(geo[f'p{f}']),np.uint16)for f in range(3)]
        step=.04
        for frame,pose in enumerate(rays['poses']):
            endpoints=rays[f'r{frame:04d}'][::32]
            delta=endpoints-pose[1:4];length=np.linalg.norm(delta,axis=1)
            dirs=delta/np.maximum(length[:,None],1e-8)
            t=np.arange(.2,5.0,step)
            valid=t[None,:]<length[:,None]-.18
            xyz=(pose[1:4]+dirs[:,None,:]*t[None,:,None])[valid]
            direction=np.broadcast_to(dirs[:,None,:],(len(dirs),len(t),3))[valid]
            for f,(low,high) in enumerate(zip([-.205,2.545,5.545],[2.495,5.495,8.05])):
                mask=(xyz[:,2]>=low)&(xyz[:,2]<high)
                if not mask.any():continue
                dist,idx=trees[f].query(xyz[mask],workers=2)
                transverse=np.abs(np.einsum('ij,ij->i',geo[f'n{f}'][idx],direction[mask]))>.35
                hit=np.unique(idx[(dist<.035)&transverse]);counts[f][hit]+=1
            if frame%100==0:print('depth frames',frame+1,'/',len(rays['poses']),flush=True)
        np.savez_compressed(cache,**{f'counts{f}':counts[f]for f in range(3)})
    crossings=np.load(cache);results=[]
    for f in range(3):
        tree=cKDTree(geo[f'p{f}'])
        files=[ROOT/'outputs/wall_patch_context'/f'L{f+1}_局部高度宽松.npz']
        for rnd in [1,2,5,6,7,8,9,10,11]:files+=sorted((OUT/f'round{rnd}').glob(f'L{f+1}_*.npz'))
        for path in files:
            q=np.load(path)['wall_points'];dist,idx=tree.query(q)
            c=crossings[f'counts{f}'][idx]
            result=dict(floor=f+1,method=str(path.relative_to(ROOT/'outputs')),points=len(q),unmeasured_points=int((dist>1e-6).sum()),crossed_3_frames=int((c>=3).sum()),crossed_5_frames=int((c>=5).sum()),crossed_10_frames=int((c>=10).sum()))
            results.append(result)
    report=dict(note='Depth-only consistency check, not a wall classification accuracy score. 4cm ray samples, 3.5cm hit radius, 18cm endpoint margin, >0.35 incidence cosine, per-frame counts.',results=results)
    (OUT/'depth_consistency_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    for r in results:print(r,flush=True)

if __name__=='__main__':main()
