"""Export valid full-height depth samples and exactly synchronized sensor poses."""
import rosbag,numpy as np
from pathlib import Path
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
root=Path('/workspace/project/PRE_MAP_VLN/outputs/exploration_recovery_local_trial/round6_extended')
poses={};out={};positions=[]
with rosbag.Bag(str(root/'trial.bag')) as bag:
 start=bag.get_start_time()
 for _,m,t in bag.read_messages(topics=['/uav_simulator/sensor_pose']):
  p=m.transform.translation;q=m.transform.rotation
  poses[m.header.stamp.to_nsec()]=(np.array([p.x,p.y,p.z]),rotation([q.x,q.y,q.z,q.w]))
 last=-100
 for _,m,t in bag.read_messages(topics=['/uav_simulator/depth_image']):
  rel=t.to_sec()-start
  if rel>454.12733125686646:break
  if rel-last<.75:continue
  pose=poses.get(m.header.stamp.to_nsec())
  if pose is None:continue
  dep=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width]*.001
  v,u=np.mgrid[0:m.height:12,0:m.width:12];v=v.ravel();u=u.ravel();z=dep[v,u];ok=(z>.1)&(z<4.95);v,u,z=v[ok],u[ok],z[ok]
  pts=np.column_stack([(u-m.width/2)*z/(m.width/2),(v-m.height/2)*z/(m.width/2),z])@pose[1].T+pose[0]
  out[f'r{len(positions):04d}']=pts.astype('float32');positions.append(np.r_[rel,pose[0]]);last=rel
out['poses']=np.array(positions);np.savez_compressed('/tmp/wall_fullheight_rays.npz',**out)
print('exported',len(positions),'frames',sum(len(x)for k,x in out.items()if k.startswith('r')),flush=True)
