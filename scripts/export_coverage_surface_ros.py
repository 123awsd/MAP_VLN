"""Export recorded RGB-D surface chunks with source time, for offline evaluation."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rosbag
p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('--interval',type=float,default=.4);a=p.parse_args();root=Path('/workspace/shared');case=root/'outputs'/a.case;dest=root/'outputs/consistency_validation_20261015/coverage_surface_exports'/a.case;dest.mkdir(parents=True,exist_ok=True);frames=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];stamps=np.array([f['capture_wall_s']for f in frames]);t0=stamps[0];exact={round(t,6):i for i,t in enumerate(stamps)};parts=[];times=[];last=-1e30;counts={'exact':0,'approximate':0,'unmatched':0};chunk=0;offsets=[];used=set()
def flush():
 global chunk
 if not parts:return
 np.savez_compressed(str(dest/('chunk%04d.npz'%chunk)),points=np.concatenate(parts),elapsed_s=np.concatenate(times));parts.clear();times.clear();chunk+=1
with rosbag.Bag(str(case/'trial.bag'))as bag:
 for _,m,_ in bag.read_messages(topics=['/uav_simulator/depth_image']):
  stamp=m.header.stamp.to_sec();fi=exact.get(round(stamp,6));kind='exact'
  if fi is None:
   fi=int(np.argmin(abs(stamps-stamp)));kind='approximate'
   if abs(stamps[fi]-stamp)>.05:counts['unmatched']+=1;continue
  if fi in used or stamps[fi]-last<a.interval-.01:continue
  used.add(fi);f=frames[fi];last=stamps[fi];counts[kind]+=1;offsets.append(stamp-stamps[fi]);depth=np.frombuffer(m.data,dtype='<u2').reshape(m.height,m.step//2)[:,:m.width][::4,::4].astype(np.float32)*.001;v,u=np.mgrid[0:m.height:4,0:m.width:4];z=depth.ravel();valid=(z>.15)&(z<=5);z=z[valid];q=np.column_stack(((u.ravel()[valid]-320)*z/320,(v.ravel()[valid]-240)*z/320,z));x,y,zz,w=f['orientation_xyzw'];r=np.array([[1-2*(y*y+zz*zz),2*(x*y-zz*w),2*(x*zz+y*w)],[2*(x*y+zz*w),1-2*(x*x+zz*zz),2*(y*zz-x*w)],[2*(x*zz-y*w),2*(y*zz+x*w),1-2*(x*x+y*y)]]);q=q@r.T+f['position'];mask=(q[:,0]>=-8.5)&(q[:,0]<10.5)&(q[:,1]>=-2.5)&(q[:,1]<18.5);mask&=((q[:,2]>=-.005)&(q[:,2]<2.045))|((q[:,2]>=2.745)&(q[:,2]<4.795))|((q[:,2]>=5.745)&(q[:,2]<7.795));q=q[mask].astype(np.float32);parts.append(q);times.append(np.full(len(q),stamps[fi]-t0,np.float32))
  if len(parts)>=100:flush();print('chunks',chunk,counts,flush=True)
flush();(dest/'manifest.json').write_text(json.dumps({'case':a.case,'capture_duration_s':stamps[-1]-t0,'counts':counts,'maximum_depth_pose_stamp_offset_s':max(map(abs,offsets),default=None),'pixel_stride':4,'frame_interval_s':a.interval,'optical_depth_range_m':[.15,5.],'time_basis':'actual capture wall-clock elapsed, not independently recorded physics time','scope':'fixed three interior-height bands; raw endpoints, not final occupancy or full mesh','chunks':chunk},indent=2))
