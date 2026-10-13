"""Build analysis geometry from measured depths, independent of video clipping."""
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[1];src=ROOT.parent/'视频制作/探索地图完整性检查/原始观测地图_分高度_4cm.npz';out=ROOT/'outputs/room_quality_ceiling';out.mkdir(exist_ok=True);inc=np.load(ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed/increments.npz');allp=np.concatenate([inc[k]for k in inc.files if k.startswith('new_')]);data={};floorz=[-.155,2.595,5.595];tops=[2.495,5.495,8.05]
for f in range(3):
 p=allp[(allp[:,2]>=floorz[f]-.05)&(allp[:,2]<tops[f])];_,idx=np.unique(np.floor(p/.04).astype('int32'),axis=0,return_index=True);p=p[np.sort(idx)];tree=cKDTree(p);normals=np.empty_like(p)
 for start in range(0,len(p),5000):
  _,idx=tree.query(p[start:start+5000],k=20,workers=2);neighbors=p[idx];centered=neighbors-neighbors.mean(1,keepdims=True);cov=np.einsum('nki,nkj->nij',centered,centered);_,v=np.linalg.eigh(cov);normals[start:start+len(idx)]=v[:,:,0]
 data[f'p{f}']=p;data[f'n{f}']=normals;print('floor',f+1,len(p),flush=True)
np.savez_compressed(out/'geometry.npz',**data)
