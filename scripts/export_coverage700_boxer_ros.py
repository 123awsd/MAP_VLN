"""Export exact-stamp measured RGB-D for offline Boxer; no rendering or GT."""
import json
from pathlib import Path
import numpy as np
import rosbag
R=Path('/workspace/shared');run=R/'outputs/coverage_lifecycle_700_r1_20261016';out=R/'outputs/boxer/coverage700_measured_20261017/episode'
out.mkdir(parents=True,exist_ok=True)
trace=[json.loads(s) for s in (run/'capture_trace.jsonl').read_text().splitlines()]
frames={};last=-1e20
for f in trace:
 if f['capture_wall_s']-last>=1.99:
  frames[round(f['capture_wall_s'],6)]=f;last=f['capture_wall_s']
rgb={};done=set()
with rosbag.Bag(str(run/'trial.bag')) as bag:
 for _,m,_ in bag.read_messages(topics=['/habitat/rgb']):
  stamp=round(m.header.stamp.to_sec(),6)
  if stamp not in frames:continue
  a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
  if m.encoding=='bgr8':a=a[:,:,::-1].copy()
  elif m.encoding!='rgb8':raise ValueError(m.encoding)
  rgb[stamp]=a
 for _,m,_ in bag.read_messages(topics=['/uav_simulator/depth_image']):
  stamp=round(m.header.stamp.to_sec(),6)
  if stamp not in frames or stamp not in rgb or stamp in done:continue
  f=frames[stamp];depth=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width].astype(np.float32)*.001
  np.savez_compressed(out/f"frame_{f['sequence']:06d}.npz",rgb=rgb.pop(stamp),depth_m=depth,position=np.asarray(f['position'],np.float32),orientation_xyzw=np.asarray(f['orientation_xyzw'],np.float32),time_ns=np.int64(m.header.stamp.to_nsec()))
  done.add(stamp)
manifest={'format':'pre_map_vln.habitat_episode.v1','width':640,'height':480,'fx':320.,'fy':320.,'cx':320.,'cy':240.,'coordinate_frame':'falcon_world_z_up_camera_optical','source_bag':str(run/'trial.bag'),'exact_stamp_match':True,'sample_interval_s':1.99,'frames':len(done),'missing_selected_frames':len(frames)-len(done),'uses_gt':False}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2));print(manifest,flush=True)
