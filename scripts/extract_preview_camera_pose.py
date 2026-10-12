from pathlib import Path
import rosbag,numpy as np
base=Path('/workspace/shared');rows=[]
with rosbag.Bag(str(base/'outputs/bags/trimmed_lz4/hm3d_stage1_3d_00337_online_floor_priority_historyfix_trim420.bag')) as bag:
 start=bag.get_start_time()
 for _,m,t in bag.read_messages(topics=['/uav_simulator/sensor_pose']):
  if t.to_sec()-start>381:break
  p=m.transform.translation;q=m.transform.rotation
  rows.append([t.to_sec()-start,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
np.save(base/'outputs/paper_visualization/smooth_dense_preview/camera_pose.npy',np.asarray(rows))
print(len(rows))
