from pathlib import Path
import numpy as np,rosbag
p=Path('/workspace/shared/outputs/exploration_yaw_consistent_trial');poses=[];commands=[]
with rosbag.Bag(str(p/'trial.bag')) as b:
 first=next(b.read_messages(topics=['/uav_simulator/depth_image']))[1].header.stamp.to_sec()
 for topic,m,t in b.read_messages(topics=['/uav_simulator/sensor_pose','/planning/pos_cmd']):
  stamp=m.header.stamp.to_sec()-first
  if topic.endswith('sensor_pose'):
   q=m.transform.rotation;v=m.transform.translation;poses.append([stamp,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
  else:
   v=m.position;commands.append([stamp,v.x,v.y,v.z,m.yaw])
np.save(p/'video_cache/camera_pose.npy',np.array(poses));np.save(p/'command_xyz_yaw.npy',np.array(commands));print(len(poses),len(commands))
