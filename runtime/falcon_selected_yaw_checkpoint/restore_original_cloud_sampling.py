from pathlib import Path
import numpy as np,rosbag,json
p=Path('/workspace/shared/outputs/exploration_yaw_consistent_trial');cache=p/'video_cache';d=np.load(cache/'increments.npz');targets=d['targets'];poses={};rng=np.random.default_rng(42);clouds={f'new_{i:03d}':[] for i in range(len(targets))};last=-100
with rosbag.Bag(str(p/'trial.bag')) as b:
 first=next(b.read_messages(topics=['/uav_simulator/depth_image']))[1].header.stamp.to_sec()
 for _,m,_ in b.read_messages(topics=['/uav_simulator/sensor_pose']):
  q=m.transform.rotation;v=m.transform.translation;x,y,z,w=q.x,q.y,q.z,q.w;R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);poses[m.header.stamp.to_nsec()]=(np.array([v.x,v.y,v.z]),R)
 for _,m,_ in b.read_messages(topics=['/uav_simulator/depth_image']):
  stamp=m.header.stamp.to_nsec();elapsed=m.header.stamp.to_sec()-first
  if elapsed-last<.25 or stamp not in poses:continue
  dep=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width]*.001;v,u=np.where((dep>0)&(dep<=5)&(rng.random(dep.shape)<.05));z=dep[v,u];xyz,R=poses[stamp];q=np.column_stack([(u-320)*z/320,(v-240)*z/320,z])@R.T+xyz;q=q[q[:,2]<8.05].astype(np.float32)
  i=min(len(targets)-1,np.searchsorted(targets,elapsed,side='left'));clouds[f'new_{i:03d}'].append(q);last=elapsed
clouds={k:np.concatenate(v) if v else np.empty((0,3),np.float32) for k,v in clouds.items()};d.close();np.savez_compressed(cache/'increments.npz',targets=targets,**clouds)
a=np.load(cache/'timeline.npz');fields={k:a[k] for k in a.files};fields['cloud_100']=np.concatenate(list(clouds.values()))[::10];a.close();np.savez_compressed(cache/'timeline.npz',**fields)
report={'points':sum(len(v) for v in clouds.values()),'sampling':'original: random5percent,seed42,min interval .25s, depth0-5m,display z<8.05','timeline':'full rerun recording retained; original video was trimmed so total point counts are not direct quality comparison','reason':'replace generic pixel-step12 sparse export'};(cache/'density_fix.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
