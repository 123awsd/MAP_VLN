from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
p=Path(__file__).parent;cache=p.parent/'video_cache';d=np.load(cache/'increments.npz');raw=np.concatenate([d[k] for k in d.files if k.startswith('new_')]);cells=np.floor(raw/.04).astype(np.int32);_,ids=np.unique(cells,axis=0,return_index=True);raw=raw[ids];data={}
for f,(lo,hi) in enumerate([(-.25,2.5),(2.5,5.5),(5.5,8.05)]):
 q=raw[(raw[:,2]>lo)&(raw[:,2]<hi)];tree=cKDTree(q);n=np.empty_like(q)
 for start in range(0,len(q),5000):
  _,idx=tree.query(q[start:start+5000],k=20,workers=2);near=q[idx];near-=near.mean(1,keepdims=True);cov=np.einsum('nki,nkj->nij',near,near);_,v=np.linalg.eigh(cov);n[start:start+len(idx)]=v[:,:,0]
 data[f'p{f}']=q;data[f'n{f}']=n;print('floor',f+1,len(q),flush=True)
np.savez_compressed(p/'geometry.npz',**data)
