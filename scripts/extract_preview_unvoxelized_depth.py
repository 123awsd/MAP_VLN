"""Reconstruct preview points from recorded depth pixels without changing geometry."""
from pathlib import Path
import json
import rosbag
import numpy as np
base=Path('/workspace/shared');old=base/'outputs/paper_visualization/smooth_dense_preview'
out=base/'outputs/paper_visualization/raw_depth_unvoxelized_preview';out.mkdir(exist_ok=True)
targets=np.load(old/'increments.npz')['targets']
bag_path=base/'outputs/bags/trimmed_lz4/hm3d_stage1_3d_00337_online_floor_priority_historyfix_trim420.bag'
poses={}
with rosbag.Bag(str(bag_path)) as bag:
 start=bag.get_start_time()
 for _,m,t in bag.read_messages(topics=['/uav_simulator/sensor_pose']):
  if t.to_sec()-start>381:break
  p=m.transform.translation;q=m.transform.rotation
  poses[m.header.stamp.to_nsec()]=([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rng=np.random.default_rng(20261012);seen=set();pending=[];clouds={};i=0;last=-100;count=0;missing=0
with rosbag.Bag(str(bag_path)) as bag:
 for _,m,t in bag.read_messages(topics=['/uav_simulator/depth_image']):
  rel=t.to_sec()-start
  if rel>381:break
  while i<len(targets) and rel>targets[i]:
   clouds[f'new_{i:03d}']=np.concatenate(pending) if pending else np.empty((0,3),np.float32);pending=[]
   if i%50==0:print(i,len(seen),'frames',count,flush=True)
   i+=1
  if i==len(targets):break
  if rel-last<.25 or any(a<=rel<=b for a,b in [(243,288),(323,355)]):continue
  pose=poses.get(m.header.stamp.to_nsec())
  if pose is None:missing+=1;continue
  assert m.encoding=='16UC1',m.encoding
  dep=np.frombuffer(m.data,dtype='>u2' if m.is_bigendian else '<u2').reshape(m.height,m.step//2)[:,:m.width].astype(np.float32)*.001
  # Sample real pixels without a fixed stride; never jitter their coordinates.
  v,u=np.where((dep>0)&(dep<=5)&(rng.random(dep.shape)<.05))
  z=dep[v,u];fx=m.width*.5;fy=fx;cx=m.width*.5;cy=m.height*.5
  pts=np.column_stack([(u-cx)*z/fx,(v-cy)*z/fy,z])@rotation(pose[1]).T+pose[0]
  pts=pts[np.isfinite(pts).all(1)&(pts[:,2]<8.2)].astype(np.float32)
  if len(pts):pending.append(pts)
  last=rel;count+=1
 while i<len(targets):
  clouds[f'new_{i:03d}']=np.concatenate(pending) if pending else np.empty((0,3),np.float32);pending=[];i+=1
np.savez_compressed(out/'increments.npz',targets=targets,**clouds)
(out/'source.json').write_text(json.dumps({'source_topic':'/uav_simulator/depth_image','pose_matching':'exact header stamp','pixel_sampling':'random 5% of original pixels, fixed seed','depth_frames':count,'missing_pose':missing,'range_max_m':5,'voxel_dedup_m':None,'point_coordinates':'original measured pixels, no synthetic noise','intrinsics':'simulator HFOV90, fx=fy=width/2 cx=width/2 cy=height/2'},indent=2))
print('DONE',sum(len(v) for v in clouds.values()),'missing',missing,flush=True)
