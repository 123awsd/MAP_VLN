import json
from pathlib import Path
import numpy as np
import rosbag
root=Path('/workspace/shared');out=root/'outputs/consistency_validation_20261015/global_observability'
names=['consistency_A3_observation_lifecycle_504_final_20261016','consistency_R0_light_504_r1_20261015','consistency_R0_light_504_r2_20261015','consistency_A3_acceptance_504_r1_20261016']
for ci,name in enumerate(names):
 frames=[json.loads(l)for l in(root/'outputs'/name/'capture_trace.jsonl').open()];poses={round(f['capture_wall_s'],6):i for i,f in enumerate(frames)};dest=out/('depth%d.npy'%ci)
 if dest.exists() and (out/('export%d.json'%ci)).exists():continue
 if ci==0 and dest.exists():
  (out/('export%d.json'%ci)).write_text(json.dumps({'exact_frames':len(frames),'note':'prior exact export completed'}));continue
 a=np.lib.format.open_memmap(str(dest),mode='w+',dtype=np.uint16,shape=(len(frames),480,640));count=0;offsets=[];stamps=np.array([f["capture_wall_s"]for f in frames]);used=set()
 with rosbag.Bag(str(root/'outputs'/name/'trial.bag'))as bag:
  for _,m,_ in bag.read_messages(topics=['/uav_simulator/depth_image']):
   i=poses.get(round(m.header.stamp.to_sec(),6))
   if i is None:
    i=int(np.argmin(abs(stamps-m.header.stamp.to_sec())))
    if abs(stamps[i]-m.header.stamp.to_sec())>.05:continue
   if i in used:continue
   used.add(i);offsets.append(float(m.header.stamp.to_sec()-stamps[i]))
   a[i]=np.frombuffer(m.data,dtype='<u2').reshape(m.height,m.step//2)[:,:m.width];count+=1
 a.flush();print(ci,count,len(frames),flush=True);(out/('export%d.json'%ci)).write_text(json.dumps({'exported_frames':count,'capture_frames':len(frames),'max_absolute_stamp_offset_s':max(map(abs,offsets)),'offsets_s':offsets}))
