#!/usr/bin/env python3
"""Export comparable measured surface points from a completed ROS bag."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag
p=argparse.ArgumentParser();p.add_argument('trial');a=p.parse_args();out=Path(a.trial)
poses={};rng=np.random.default_rng(42);points=[];last=-100.;first=None;frames=0
with rosbag.Bag(str(out/'trial.bag')) as bag:
 for _,m,_ in bag.read_messages(topics=['/uav_simulator/sensor_pose']):
  q=m.transform.rotation;v=m.transform.translation;x,y,z,w=q.x,q.y,q.z,q.w
  matrix=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
  poses[m.header.stamp.to_nsec()]=(np.array([v.x,v.y,v.z]),matrix)
 for _,m,_ in bag.read_messages(topics=['/uav_simulator/depth_image']):
  t=m.header.stamp.to_sec();stamp=m.header.stamp.to_nsec()
  if first is None:first=t
  elapsed=t-first
  if elapsed-last<.25 or stamp not in poses:continue
  depth=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width]*.001
  v,u=np.where((depth>0)&(depth<=5)&(rng.random(depth.shape)<.05));z=depth[v,u]
  translation,rotation=poses[stamp]
  xyz=np.column_stack([(u-320)*z/320,(v-240)*z/320,z])@rotation.T+translation
  points.append(xyz[xyz[:,2]<8.05].astype(np.float32));last=elapsed;frames+=1
raw=np.concatenate(points);cells=np.floor(raw/.04).astype(np.int32);_,indices=np.unique(cells,axis=0,return_index=True);raw=raw[indices]
fields={f'p{i}':raw[(raw[:,2]>lo)&(raw[:,2]<hi)] for i,(lo,hi) in enumerate([(-.25,2.5),(2.5,5.5),(5.5,8.05)])}
np.savez_compressed(str(out/'evaluation_geometry.npz'),**fields)
report={'method':'Measured depth only; same original 5% random sampling seed42, min interval .25s, depth0-5m; 4cm voxel dedup; no offline yaw rerender','frames':frames,'points':[len(fields[f'p{i}']) for i in range(3)]}
(out/'evaluation_geometry.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
