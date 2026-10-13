import rosbag,numpy as np,json
from pathlib import Path
root=Path('/workspace/project/PRE_MAP_VLN/outputs/exploration_recovery_local_trial/round6_extended');poses={};maxz=-100;above=0;total=0;frames=0
def rotation(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
with rosbag.Bag(str(root/'trial.bag')) as bag:
 start=bag.get_start_time()
 for _,m,t in bag.read_messages(topics=['/uav_simulator/sensor_pose']):
  p=m.transform.translation;q=m.transform.rotation
  if p.z>5.55:poses[m.header.stamp.to_nsec()]=(np.array([p.x,p.y,p.z]),rotation([q.x,q.y,q.z,q.w]))
 last=-100
 for _,m,t in bag.read_messages(topics=['/uav_simulator/depth_image']):
  rel=t.to_sec()-start
  if rel>454.12733125686646:break
  if rel-last<.5:continue
  pose=poses.get(m.header.stamp.to_nsec())
  if pose is None:continue
  dep=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width]*.001
  v,u=np.mgrid[0:m.height:6,0:m.width:6];v=v.ravel();u=u.ravel();z=dep[v,u];ok=(z>.1)&(z<=5);v,u,z=v[ok],u[ok],z[ok]
  p=np.column_stack([(u-m.width/2)*z/(m.width/2),(v-m.height/2)*z/(m.width/2),z])@pose[1].T+pose[0];maxz=max(maxz,float(p[:,2].max()));above+=int((p[:,2]>=8.05).sum());total+=len(p);frames+=1;last=rel
print(json.dumps({'third_floor_frames':frames,'sampled_points':total,'world_z_max_m':maxz,'points_above_export_cap_8_05m':above},indent=2))
