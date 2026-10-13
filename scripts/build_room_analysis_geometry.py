"""Build analysis geometry from measured depths, independent of video clipping."""
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[1];src=ROOT.parent/'视频制作/探索地图完整性检查/原始观测地图_分高度_4cm.npz';out=ROOT/'outputs/room_quality_fullheight';out.mkdir(exist_ok=True);raw=np.load(src);data={}
for f in range(3):
 p=raw[f'L{f+1}_structure'];tree=cKDTree(p);normals=np.empty_like(p)
 for start in range(0,len(p),5000):
  _,idx=tree.query(p[start:start+5000],k=20,workers=2);neighbors=p[idx];centered=neighbors-neighbors.mean(1,keepdims=True);cov=np.einsum('nki,nkj->nij',centered,centered);_,v=np.linalg.eigh(cov);normals[start:start+len(idx)]=v[:,:,0]
 data[f'p{f}']=p;data[f'n{f}']=normals;print('floor',f+1,len(p),flush=True)
np.savez_compressed(out/'geometry.npz',**data)
