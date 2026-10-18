"""Project measured frame-local Boxer 3-D predictions into original RGB images."""
from pathlib import Path
import csv,itertools,json,argparse
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];O=R/'outputs/boxer/coverage700_measured_20261017'
parser=argparse.ArgumentParser();parser.add_argument('--run-dir',type=Path,default=O);parser.add_argument('--episode',type=Path,default=O/'episode');args=parser.parse_args();O=args.run_dir;episode=args.episode
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP'
rows=list(csv.DictReader((O/'detections/episode/boxer_3dbbs.csv').open()));bytime={}
for v in rows:
 if float(v['prob'])>=.45:bytime.setdefault(int(v['time_ns']),[]).append(v)
frames=[]
for p in sorted(episode.glob('frame_*.npz')):
 with np.load(p)as a:
  t=int(a['time_ns']);pos=a['position'].copy();n=len(bytime.get(t,[]));f=int(np.digitize(pos[2],[2.3,5.3]))+1
  if n:frames.append((n,f,p,pos,t))
selected=[]
for f in (1,2,3):
 chosen=[]
 for frame in sorted([v for v in frames if v[1]==f],key=lambda v:-v[0]):
  if all(np.linalg.norm(frame[3]-old[3])>1.5 for old in chosen):chosen.append(frame)
  if len(chosen)==2:break
 selected+=chosen
signs=np.array(list(itertools.product((-.5,.5),repeat=3)));edges=[(i,j)for i in range(8)for j in range(i+1,8)if np.sum(signs[i]!=signs[j])==1]
fig,axs=plt.subplots(3,2,figsize=(14,15));evidence=[]
for ax,frame in zip(axs.ravel(),selected):
 n,f,p,pos,t=frame
 with np.load(p)as a:
  rgb=a['rgb'];rc=Rotation.from_quat(a['orientation_xyzw']).as_matrix()
 ax.imshow(rgb)
 for i,row in enumerate(bytime[t]):
  c=np.array([float(row[k+'_world_object'])for k in ('tx','ty','tz')]);sz=np.array([float(row['scale_'+k])for k in 'xyz']);rot=Rotation.from_quat([float(row[k+'_world_object'])for k in ('qx','qy','qz','qw')]).as_matrix();corners=(signs*sz)@rot.T+c;local=(corners-pos)@rc
  uv=local[:,:2]/np.maximum(local[:,2:3],1e-6)*320+[320,240];color=plt.get_cmap('tab10')(i%10)
  for k,j in edges:
   if local[k,2]>.1 and local[j,2]>.1:ax.plot(uv[[k,j],0],uv[[k,j],1],c=color,lw=1.4)
  front=local[:,2]>.1
  if front.any():
   u,v=np.clip(uv[front].mean(axis=0),[10,12],[610,460]);ax.text(u,v,f"{row['name']} {float(row['prob']):.2f}",color=color,fontsize=8,bbox={'facecolor':'white','alpha':.7,'edgecolor':'none','pad':1})
 ax.set(xlim=(0,640),ylim=(480,0),title=f'{f}楼 · {p.stem} · {n}个≥0.45单帧预测');ax.axis('off');evidence.append({'frame':p.name,'time_ns':t,'floor':f,'detections':n})
for ax in axs.ravel()[len(selected):]:ax.axis('off')
fig.suptitle('实测RGB上的Boxer三维框投影｜单帧预测示例，非真值标注',fontsize=16);fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(O/'boxer_rgb_examples.png',dpi=170);(O/'rgb_example_frames.json').write_text(json.dumps(evidence,indent=2))
