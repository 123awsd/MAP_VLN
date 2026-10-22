"""Exact-time depth backprojection diagnostic; ROI used only for provenance."""
from pathlib import Path
import json,numpy as np,rosbag
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
from PIL import Image
R=Path('/workspace/shared');run=R/'outputs/coverage_lifecycle_700_r1_20261016';O=R/'outputs/coverage700_wall_room_preview/room305_provenance';O.mkdir(exist_ok=True)
frames=[json.loads(l)for l in (run/'capture_trace.jsonl').open()];lookup={round(f['capture_wall_s'],6):f for f in frames};rows=[];best=[]
with rosbag.Bag(str(run/'trial.bag')) as bag:
 for _,m,_ in bag.read_messages(topics=['/uav_simulator/depth_image']):
  f=lookup.get(round(m.header.stamp.to_sec(),6))
  if f is None or f['position'][2]<5.5:continue
  p=np.array(f['position']);
  if np.linalg.norm(p-np.array([-5.2,4.2,7.3]))>6:continue
  d=np.frombuffer(m.data,dtype='<u2').reshape(m.height,m.step//2)[:,:m.width].astype(float)*.001;y,x=np.indices(d.shape);v=(d>=.15)&(d<=5);xyz=np.stack([(x-320)*d/320,(y-240)*d/320,d],axis=-1);rot=rotation(f['orientation_xyzw']);q=xyz@rot.T+p
  hit=v&(q[:,:,0]<-4.5)&(q[:,:,0]>-6)&(q[:,:,1]>4.05)&(q[:,:,1]<4.45)&(q[:,:,2]>5.5)&(q[:,:,2]<8.3)
  count=int(hit.sum())
  if count:
   row={'sequence':f['sequence'],'stamp':f['capture_wall_s'],'elapsed':f['capture_wall_s']-frames[0]['capture_wall_s'],'pixels':count,'position':f['position'],'z_min':float(q[hit,2].min()),'z_max':float(q[hit,2].max())};rows.append(row);best.append((count,f['capture_wall_s'],hit.copy(),d.copy(),q[hit]));best=sorted(best,key=lambda t:t[0],reverse=True)[:8]
np.savez_compressed(O/'depth_witnesses.npz',**{f'mask{i}':v[2]for i,v in enumerate(best)},**{f'depth{i}':v[3]for i,v in enumerate(best)},**{f'points{i}':v[4]for i,v in enumerate(best)})
stamps={round(v[1],6):i for i,v in enumerate(best)}
with rosbag.Bag(str(run/'trial.bag')) as bag:
 topics=['/habitat/rgb'];print(topics,flush=True)
 for topic,m,_ in bag.read_messages(topics=topics):
  i=stamps.get(round(m.header.stamp.to_sec(),6))
  if i is None:continue
  c=3 if m.encoding in ('rgb8','bgr8')else 4;im=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*c].reshape(m.height,m.width,c)[:,:,:3].copy()
  if m.encoding=='bgr8':im=im[:,:,::-1]
  Image.fromarray(im).save(O/f'rgb{i}.png');over=im.copy();over[best[i][2]]=(.35*over[best[i][2]]+.65*np.array([255,30,30])).astype(np.uint8);Image.fromarray(over).save(O/f'overlay{i}.png')
(O/'audit.json').write_text(json.dumps({'frames':rows,'best':[{'index':i,'stamp':v[1],'pixels':v[0]}for i,v in enumerate(best)],'source':'exact stamped raw depth and RGB; ROI diagnostic only'},indent=2));print(len(rows),flush=True)
