#!/usr/bin/env python3
"""Fuse recorded depth into dense surface points, without occupancy interpolation."""
import json,time
from pathlib import Path
import numpy as np
import rosbag
root=Path('/workspace/shared');case=root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016';out=root/'outputs/consistency_validation_20261015/latest_dense_floor_topdown';out.mkdir(parents=True,exist_ok=True)
frames=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];poses={round(f['capture_wall_s'],6):f for f in frames};stride=2;interval=.4;voxel=.025;grounds=[-.155,2.595,5.595];parts=[[],[],[]];chunks=[[],[],[]];last=-1e30;used=0;unmatched=0;counts=[0,0,0];started=time.time()
def fuse(q):
 keys=np.floor(q/voxel).astype(np.int32);_,idx=np.unique(keys,axis=0,return_index=True);return q[idx]
def flush():
 for i in range(3):
  if parts[i]:chunks[i].append(fuse(np.concatenate(parts[i])));parts[i].clear()
with rosbag.Bag(str(case/'trial.bag'))as bag:
 for _,m,t in bag.read_messages(topics=['/uav_simulator/depth_image']):
  stamp=m.header.stamp.to_sec()
  if stamp-last<interval-.01:continue
  f=poses.get(round(stamp,6))
  if f is None:unmatched+=1;continue
  if m.encoding not in ['16UC1','mono16']:raise RuntimeError(m.encoding)
  dep=np.frombuffer(m.data,dtype='<u2').reshape(m.height,m.step//2)[:,:m.width][::stride,::stride].astype(np.float32)*.001
  vv,uu=np.mgrid[0:m.height:stride,0:m.width:stride];z=dep.ravel();valid=(z>.15)&(z<=5.)&np.isfinite(z);z=z[valid];v=vv.ravel()[valid];u=uu.ravel()[valid]
  optical=np.column_stack(((u-320.)*z/320.,(v-240.)*z/320.,z)).astype(np.float32)
  x,y,zz,w=f['orientation_xyzw'];rot=np.array([[1-2*(y*y+zz*zz),2*(x*y-zz*w),2*(x*zz+y*w)],[2*(x*y+zz*w),1-2*(x*x+zz*zz),2*(y*zz-x*w)],[2*(x*zz-y*w),2*(y*zz+x*w),1-2*(x*x+y*y)]],dtype=np.float32)
  q=optical@rot.T+np.array(f['position'],dtype=np.float32);xy=(q[:,0]>=-8.5)&(q[:,0]<10.5)&(q[:,1]>=-2.5)&(q[:,1]<18.5)
  for i,g in enumerate(grounds):
   pts=q[xy&(q[:,2]>=g+.15)&(q[:,2]<g+2.2)];counts[i]+=len(pts)
   if len(pts):parts[i].append(pts)
  used+=1;last=stamp
  if used%60==0:flush();print('frames',used,'elapsed',round(time.time()-started,1),flush=True)
flush();result={}
for i in range(3):
 pts=fuse(np.concatenate(chunks[i]))if chunks[i]else np.empty((0,3),np.float32);result[f'floor{i+1}']=pts;print('floor',i+1,'densepoints',len(pts),flush=True)
np.savez_compressed(str(out/'dense_observed_surfaces.npz'),**result)
meta={'case':case.name,'source':'trial.bag depth images + exact matching captured optical-camera pose','frames_used':used,'unmatched_depth_frames':unmatched,'pixel_stride':stride,'minimum_frame_interval_s':interval,'depth_range_m':[.15,5.],'fusion_voxel_m':voxel,'floor_z':grounds,'slice_above_floor_m':[.15,2.2],'raw_points_by_floor':counts,'fused_points_by_floor':{k:len(v)for k,v in result.items()},'duration_s':time.time()-started,'no_interpolation':True};(out/'dense_metadata.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta),flush=True)
