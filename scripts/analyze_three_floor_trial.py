from pathlib import Path
import rosbag,numpy as np,json
root=Path('/workspace/shared/outputs/exploration_three_floor_trial')
for name in ['round1','']:
 path=root/name/'trial.bag'
 if not path.exists():continue
 rows=[];coverage=[]
 with rosbag.Bag(str(path)) as bag:
  start=bag.get_start_time()
  for topic,m,t in bag.read_messages(topics=['/uav_simulator/odometry','/voxel_mapping/map_coverage']):
   if topic.endswith('odometry'):
    p=m.pose.pose.position;rows.append([t.to_sec()-start,p.x,p.y,p.z])
   else:coverage.append([t.to_sec()-start,str(m)])
 a=np.array(rows);np.save(root/name/'trajectory.npy',a)
 z=a[:,3];f=np.where(z>=6.1,2,np.where(z>=3.3,1,0))
 transitions=[];last=0
 for row,level in zip(a,f):
  if level!=last:transitions.append([round(float(row[0]),1),int(last+1),int(level+1)]);last=level
 result={'duration_s':float(a[-1,0]),'path_m':float(np.linalg.norm(np.diff(a[:,1:],axis=0),axis=1).sum()),'z_max':float(z.max()),'height_band_transitions':transitions,'coverage_tail':coverage[-3:]}
 (root/name/'analysis.json').write_text(json.dumps(result,indent=2));print(name,result,flush=True)
