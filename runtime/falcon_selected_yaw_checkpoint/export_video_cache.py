from pathlib import Path
import rosbag,numpy as np,cv2,json
root=Path('/workspace/shared/outputs/exploration_yaw_consistent_trial');out=root/'video_cache';out.mkdir(exist_ok=False)
poses={};stamps=[]
with rosbag.Bag(str(root/'trial.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/uav_simulator/sensor_pose','/uav_simulator/depth_image']):
  stamp=m.header.stamp.to_nsec()
  if topic.endswith('sensor_pose'):
   p=m.transform.translation;q=m.transform.rotation;poses[stamp]=([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
  else:stamps.append(stamp)
stamps=[s for s in stamps if s in poses];selected=[stamps[i] for i in np.linspace(0,len(stamps)-1,800).round().astype(int)];lookup={s:i for i,s in enumerate(selected)};clouds={};rgbset=set()
def rot(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
with rosbag.Bag(str(root/'trial.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/habitat/rgb','/uav_simulator/depth_image']):
  stamp=m.header.stamp.to_nsec()
  if stamp not in lookup:continue
  i=lookup[stamp]
  if topic=='/habitat/rgb':
   a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3);cv2.imwrite(str(out/f'rgb_{i:03d}.jpg'),a[:,:,::-1]);rgbset.add(i)
  else:
   dep=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width]*.001
   v,u=np.mgrid[0:m.height:12,0:m.width:12];z=dep[v,u];keep=(z>.3)&(z<=5);v,u,z=v[keep],u[keep],z[keep];xyz,q=poses[stamp]
   pts=np.column_stack([(u-320)*z/320,(v-240)*z/320,z])@rot(q).T+xyz;clouds[f'new_{i:03d}']=pts.astype(np.float32)
assert len(rgbset)==800 and len(clouds)==800
trajectory=np.array([poses[s][0] for s in stamps],np.float32);times=(np.array(stamps)-stamps[0])*1e-9;targets=(np.array(selected)-stamps[0])*1e-9
np.savez_compressed(out/'increments.npz',targets=targets,**clouds)
np.savez_compressed(out/'timeline.npz',trajectory=trajectory,trajectory_times=times,snapshot_times=targets,bag_start=0.,bag_end=targets[-1],cloud_100=np.concatenate(list(clouds.values()))[::5])
(out/'manifest.json').write_text(json.dumps({'source_bag':str(root/'trial.bag'),'frames':800,'source_duration_s':float(targets[-1]),'fps':20,'playback_duration_s':40,'rgb':'original acquired RGB, exact-stamp matched to depth and pose; no yaw rerender','cloud':'progressive original measured depth, pixel step12, depth .3-5m; no height crop','timeline':'uniform samples across all recorded depth, including final stationary period'},indent=2))
print('Exported800 frames',flush=True)
