from pathlib import Path
import rosbag,numpy as np,cv2,json
from extract_bag_exploration_timeline import xyz_from_cloud
base=Path('/workspace/shared');out=base.parent/'视频制作/三层探索_流畅稠密版'
# Workspace mount is PRE_MAP_VLN only; keep extracted data under its outputs.
out=base/'outputs/paper_visualization/smooth_dense_preview';out.mkdir(exist_ok=True)
segments=[(10,243),(288,323),(355,380)];length=sum(b-a for a,b in segments)
u=np.linspace(0,length,800);targets=[]
for v in u:
 for a,b in segments:
  if v<=b-a:targets.append(a+v);break
  v-=b-a
 else:targets.append(380)
targets=np.array(targets);seen=set();pending=[];rgb=None;i=0;clouds={}
with rosbag.Bag(str(base/'outputs/bags/trimmed_lz4/hm3d_stage1_3d_00337_online_floor_priority_historyfix_trim420.bag')) as bag:
 start=bag.get_start_time()
 for topic,m,t in bag.read_messages(topics=['/habitat/rgb','/voxel_mapping/depth_pointcloud']):
  rel=t.to_sec()-start
  if rel>381:break
  while i<800 and rel>targets[i]:
   new=np.concatenate(pending) if pending else np.empty((0,3),np.float32)
   clouds[f'new_{i:03d}']=new;pending=[]
   if rgb is not None:cv2.imwrite(str(out/f'rgb_{i:03d}.jpg'),rgb)
   if i%40==0:print(i,len(seen),flush=True)
   i+=1
  if topic=='/habitat/rgb':
   rgb=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
   if m.encoding=='rgb8':rgb=rgb[:,:,::-1]
  elif not any(a<=rel<=b for a,b in [(243,288),(323,355)]) and rel<=380:
   pts=xyz_from_cloud(m);pts=pts[np.isfinite(pts).all(1)&(pts[:,2]<8.2)]
   if not len(pts):continue
   cells=np.floor(pts/.025).astype(np.int32);_,idx=np.unique(cells,axis=0,return_index=True)
   take=[]
   for j in idx:
    key=tuple(cells[j])
    if key not in seen:seen.add(key);take.append(j)
   if take:pending.append(pts[take])
np.savez_compressed(out/'increments.npz',targets=targets,**clouds)
(out/'source.json').write_text(json.dumps({'segments_s':segments,'frames':i,'voxel_m':.025,'note':'Actual sampled points retained, no synthetic noise or interpolation.'},indent=2))
