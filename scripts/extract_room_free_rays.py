#!/usr/bin/env python3
"""ROS Python: extract camera-height free rays with exact depth/pose header matching."""
import argparse
from pathlib import Path
import numpy as np,rosbag
p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('output',type=Path);p.add_argument('--end',type=float,default=454.12733125686646);args=p.parse_args();poses={};frames=[];out={};rng=np.random.default_rng(7)
def rotation(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
with rosbag.Bag(str(args.bag))as b:
 start=b.get_start_time()
 for _,m,t in b.read_messages(topics=['/uav_simulator/sensor_pose']):
  xyz=m.transform.translation;q=m.transform.rotation;poses[m.header.stamp.to_nsec()]=(np.array([xyz.x,xyz.y,xyz.z]),rotation([q.x,q.y,q.z,q.w]))
 last=-1
 for _,m,t in b.read_messages(topics=['/uav_simulator/depth_image']):
  stamp=t.to_sec()-start
  if stamp>args.end:break
  if stamp-last<.25:continue
  pose=poses.get(m.header.stamp.to_nsec())
  if pose is None:continue
  xyz,R=pose;dep=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width]*.001;v,u=np.where((dep>.3)&(dep<=5.));z=dep[v,u];q=np.column_stack([(u-m.width/2)*z/(m.width/2),(v-m.height/2)*z/(m.width/2),z])@R.T+xyz;q=q[np.abs(q[:,2]-xyz[2])<.15]
  if len(q)>160:q=q[rng.choice(len(q),160,replace=False)]
  out[f'r{len(frames):04d}']=q.astype('float32');frames.append([stamp,*xyz]);last=stamp
args.output.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(args.output,poses=np.array(frames),**out);print('extracted',len(frames),'measured frames')
