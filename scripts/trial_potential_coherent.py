from pathlib import Path
import sys,json
sys.path[:0]=['/tmp/room-research-deps','/tmp/room-research-potential-host']
import numpy as np
from roomseg import segment
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'outputs/room_method_research';raw=np.load(ROOT.parent/'视频制作/探索地图完整性检查/原始观测地图_分高度_4cm.npz');geo=np.load(ROOT/'outputs/room_quality_fullheight/geometry.npz')
for f in [3,2,1]:
 floor=raw[f'L{f}_floor'];upper=raw[f'L{f}_upper'];structure=geo[f'p{f-1}'];p=np.concatenate([floor,structure,upper]);ns=np.zeros((len(floor)+len(upper),3),np.float32);ns[:,2]=1;n=np.concatenate([ns[:len(floor)],geo[f'n{f-1}'],ns[len(floor):]])
 print('START',f,len(p),flush=True)
 try:
  labels,art=segment(p,n,voxel=.18,vis_targets=256,min_cluster_size=40,new_cluster_dist=.45);grid=art['grid'];np.savez_compressed(out/f'APF_coherent_L{f}.npz',cells=art['cells'],cell_labels=art['cell_labels'],pf2d=art['pf2d'],origin=grid.origin,voxel=grid.voxel);(out/f'APF_coherent_L{f}_metrics.json').write_text(json.dumps({'rooms':art['n_rooms'],'bench':art['bench'],'labeled_fraction':float((labels>=0).mean())},indent=2));print('DONE',f,art['n_rooms'],art['bench'],flush=True)
 except Exception as e:
  import traceback;traceback.print_exc()
