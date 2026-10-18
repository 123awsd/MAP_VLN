"""Render real-frame SAM3.1 checkpoint detector masks and resulting Boxer predictions."""
from pathlib import Path
import csv,json,itertools
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
R=Path(__file__).resolve().parents[1];O=R/'outputs/boxer/sam31_measured_smoke_20261018';source=R/'outputs/boxer/coverage700_measured_20261017/episode';a=json.loads((O/'manifest.json').read_text());rows=list(csv.DictReader((O/'boxer_3dbbs.csv').open()))
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP'
signs=np.array(list(itertools.product((-.5,.5),repeat=3)));edges=[(i,j)for i in range(8)for j in range(i+1,8)if np.sum(signs[i]!=signs[j])==1]
for mode in ['masks','boxer']:
 fig,axs=plt.subplots(4,2,figsize=(15,20))
 for ax,record in zip(axs.ravel(),a['frames']):
  seq=record['sequence']
  with np.load(source/f'frame_{seq:06d}.npz')as d:rgb=d['rgb'];pos=d['position'];rot=Rotation.from_quat(d['orientation_xyzw']).as_matrix()
  ax.imshow(rgb);ax.set(xlim=(0,640),ylim=(480,0));ax.axis('off');ax.set_title(f"frame_{seq:06d} · SAM {record['detections']}个 · Boxer {record.get('boxer_3d_detections',0)}个")
  if mode=='masks':
   with np.load(O/f'frame_{seq:06d}_sam.npz')as d:boxes=d['boxes_xyxy'];scores=d['scores'];ids=d['label_indices'];masks=d['masks']
   for b,s,i,m in zip(boxes,scores,ids,masks):
    color=plt.get_cmap('tab20')(int(i)%20);rgba=np.zeros((*m.shape,4));rgba[m]=[*color[:3],.30];ax.imshow(rgba);x,y,x2,y2=b;ax.add_patch(Rectangle((x,y),x2-x,y2-y,fill=False,edgecolor=color,lw=1.3));ax.text(x,max(10,y),f"{a['labels'][i]} {s:.2f}",fontsize=7,color=color,bbox={'facecolor':'white','alpha':.75,'edgecolor':'none','pad':1})
  else:
   for j,v in enumerate([v for v in rows if int(v['time_ns'])==record['time_ns']]):
    c=np.array([float(v[k+'_world_object'])for k in ('tx','ty','tz')]);size=np.array([float(v['scale_'+k])for k in 'xyz']);rotation=Rotation.from_quat([float(v[k+'_world_object'])for k in ('qx','qy','qz','qw')]).as_matrix();q=((signs*size)@rotation.T+c-pos)@rot;uv=q[:,:2]/np.maximum(q[:,2:3],1e-6)*320+[320,240];color=plt.get_cmap('tab20')(a['labels'].index(v['name'])%20)
    for k,l in edges:
     if q[k,2]>.1 and q[l,2]>.1:ax.plot(uv[[k,l],0],uv[[k,l],1],color=color,lw=1.3)
    if (q[:,2]>.1).any():
     u,w=np.clip(uv[q[:,2]>.1].mean(0),[5,12],[600,465]);ax.text(u,w,f"{v['name']} {float(v['prob']):.2f}",fontsize=7,color=color,bbox={'facecolor':'white','alpha':.75,'edgecolor':'none','pad':1})
 title='SAM3.1权重中的图像检测模块：框＋掩码（阈值0.5）'if mode=='masks'else'SAM图像检测＋掩码深度过滤 → Boxer三维框｜单帧输出，未多帧融合'
 fig.suptitle(title+'\n8个实测视角，无GT输入；本次未测试3.1视频跟踪',fontsize=15);fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(O/f'sam31_{mode}_examples.png',dpi=150);plt.close(fig)
stamps={v['time_ns']for v in a['frames']};comp={}
for key,p in [('gdino_sam2',R/'outputs/boxer/coverage700_measured_20261017/detections/episode/owl_2dbbs.csv'),('owlv2',R/'outputs/boxer/coverage700_owlv2_20261018/detections/episode/owl_2dbbs.csv'),('sam31_image',O/'sam_2dbbs.csv')]:
 r=[v for v in csv.DictReader(p.open())if int(v['time_ns'])in stamps];comp[key]={'detections':len(r),'labels':dict(Counter(v['name']for v in r))}
(O/'comparison.json').write_text(json.dumps({'same_frames':len(stamps),'same_labels':30,'counts':comp,'score_thresholds_not_calibrated':True,'accuracy_against_gt_not_measured':True,'sam31_video_tracker_not_tested':True},indent=2));print(comp)
