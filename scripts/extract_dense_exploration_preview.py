from pathlib import Path
import rosbag,numpy as np
from extract_bag_exploration_timeline import xyz_from_cloud
base=Path('/workspace/shared')
cache=np.load(base/'outputs/paper_visualization/00337_online_floor_priority_timeline_101.npz')
targets=cache['snapshot_times'];store={};payload={};i=0
bagpath=base/'outputs/bags/trimmed_lz4/hm3d_stage1_3d_00337_online_floor_priority_historyfix_trim420.bag'
with rosbag.Bag(str(bagpath)) as bag:
 for _,m,t in bag.read_messages(topics=['/voxel_mapping/depth_pointcloud']):
  now=t.to_sec()
  if now-float(cache['bag_start'])>380:break
  pts=xyz_from_cloud(m);pts=pts[np.isfinite(pts).all(axis=1)&(pts[:,2]<8.2)]
  if len(pts)==0:continue
  cells=np.floor(pts/.04).astype(np.int32)
  _,idx=np.unique(cells,axis=0,return_index=True)
  for k in idx:store[tuple(cells[k])]=pts[k]
  while i<len(targets) and now>=targets[i]:
   payload[f'cloud_{i:03d}']=np.asarray(list(store.values()),dtype=np.float32);print(i,len(store),flush=True);i+=1
for k in cache.files:
 if not k.startswith('cloud_'):payload[k]=cache[k]
np.savez_compressed(base/'outputs/paper_visualization/00337_dense_preview_380.npz',**payload)
