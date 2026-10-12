from pathlib import Path
import rosbag,numpy as np,cv2
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
root=Path('/workspace/shared/outputs/exploration_three_floor_trial');out=root/'preview';out.mkdir(exist_ok=True)
poses={};rows=[]
with rosbag.Bag(str(root/'trial.bag')) as bag:
 start=bag.get_start_time()
 for topic,m,t in bag.read_messages(topics=['/uav_simulator/sensor_pose','/uav_simulator/odometry']):
  if topic.endswith('sensor_pose'):
   p=m.transform.translation;q=m.transform.rotation;poses[m.header.stamp.to_nsec()]=[t.to_sec()-start,p.x,p.y,p.z,q.x,q.y,q.z,q.w]
  else:
   p=m.pose.pose.position;rows.append([t.to_sec()-start,p.x,p.y,p.z])
a=np.array(rows);moving=np.linalg.norm(a[:,1:]-a[0,1:],axis=1)>.15;begin=max(0,a[np.where(moving)[0][0],0]-1);end=a[-1,0];targets=np.linspace(begin,end,800)
np.save(out/'camera_pose.npy',np.array(list(poses.values())));np.save(out/'trajectory.npy',a)
rng=np.random.default_rng(42);pending=[];clouds={};i=0;rgb=None;last=-100
with rosbag.Bag(str(root/'trial.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/habitat/rgb','/uav_simulator/depth_image']):
  rel=t.to_sec()-start
  while i<800 and rel>targets[i]:
   clouds[f'new_{i:03d}']=np.concatenate(pending) if pending else np.empty((0,3),np.float32);pending=[]
   if rgb is not None:cv2.imwrite(str(out/f'rgb_{i:03d}.jpg'),rgb)
   i+=1
  if i==800:break
  if topic=='/habitat/rgb':
   rgb=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
   if m.encoding=='rgb8':rgb=rgb[:,:,::-1]
  elif rel-last>=.25:
   pose=poses.get(m.header.stamp.to_nsec())
   if pose is None:continue
   dep=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width]*.001
   v,u=np.where((dep>0)&(dep<=5)&(rng.random(dep.shape)<.05));z=dep[v,u]
   pts=np.column_stack([(u-m.width/2)*z/(m.width/2),(v-m.height/2)*z/(m.width/2),z])@rotation(pose[4:]).T+pose[1:4]
   pts=pts[pts[:,2]<8.05].astype(np.float32);pending.append(pts);last=rel
 while i<800:
  clouds[f'new_{i:03d}']=np.concatenate(pending) if pending else np.empty((0,3),np.float32);pending=[]
  if rgb is not None:cv2.imwrite(str(out/f'rgb_{i:03d}.jpg'),rgb)
  i+=1
np.savez_compressed(out/'increments.npz',targets=targets,**clouds)
allpts=np.concatenate(list(clouds.values()));np.savez_compressed(out/'timeline.npz',trajectory=a[:,1:],trajectory_times=a[:,0],bag_start=0.,cloud_100=allpts[::10]);print('exported',len(allpts),begin,end,flush=True)
