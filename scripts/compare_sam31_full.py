"""Same-frame count comparison and raw projection gallery for three front ends."""
from pathlib import Path
import csv,json,itertools
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];BASE=R/'outputs/boxer';O=BASE/'coverage700_sam31_20261018';episode=BASE/'coverage700_measured_20261017/episode'
runs=[('Grounding DINO＋SAM2','coverage700_measured_20261017'),('OWLv2','coverage700_owlv2_20261018'),('SAM3.1图像检测＋掩码','coverage700_sam31_20261018')];data={};table=[]
for name,folder in runs:
 d=BASE/folder/'detections/episode';raw=list(csv.DictReader((d/'boxer_3dbbs.csv').open()));two=list(csv.DictReader((d/'owl_2dbbs.csv').open()));fused=json.loads((BASE/folder/'fusion_fixed/boxer_summary.json').read_text());data[name]=raw
 table.append({'pipeline':name,'frames':350,'detections_2d':len(two),'detections_3d':len(raw),'detections_3d_prob_ge045':sum(float(v['prob'])>=.45 for v in raw),'fused':fused['fused_objects'],'fused_floor_counts':fused['floor_counts'],'fused_classes':fused['class_counts'],'raw_classes':dict(Counter(v['name']for v in raw))})
checks=json.loads((O/'checks.json').read_text());report={'runs':table,'sam_checks':checks,'same_frames':True,'same_prompts':30,'same_fusion_iou':.2,'same_min_unique_frames':4,'same_fusion_confidence':.45,'thresholds':{'sam31':.5,'grounding_dino':.25,'owlv2':.25},'limitations':['scores are not cross-model calibrated','not a detector-only ablation: SAM depth filtering differs from OWLv2','no instance GT precision/recall or 3D IoU evaluation','SAM3.1 video tracking not exercised']};(O/'full_comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';signs=np.array(list(itertools.product((-.5,.5),repeat=3)));edges=[(i,j)for i in range(8)for j in range(i+1,8)if np.sum(signs[i]!=signs[j])==1]
fig,axs=plt.subplots(3,3,figsize=(20,15))
for row,seq in enumerate([990,2610,2870]):
 with np.load(episode/f'frame_{seq:06d}.npz')as a:rgb=a['rgb'];pos=a['position'];rot=Rotation.from_quat(a['orientation_xyzw']).as_matrix();stamp=int(a['time_ns'])
 for col,(name,folder)in enumerate(runs):
  ax=axs[row,col];ax.imshow(rgb);ax.axis('off');ax.set(xlim=(0,640),ylim=(480,0));selected=[v for v in data[name]if int(v['time_ns'])==stamp and float(v['prob'])>=.45]
  for i,v in enumerate(selected):
   c=np.array([float(v[k+'_world_object'])for k in ('tx','ty','tz')]);size=np.array([float(v['scale_'+k])for k in 'xyz']);rotation=Rotation.from_quat([float(v[k+'_world_object'])for k in ('qx','qy','qz','qw')]).as_matrix();q=((signs*size)@rotation.T+c-pos)@rot;uv=q[:,:2]/np.maximum(q[:,2:3],1e-6)*320+[320,240];color=plt.get_cmap('tab20')(i%20)
   for k,l in edges:
    if q[k,2]>.1 and q[l,2]>.1:ax.plot(uv[[k,l],0],uv[[k,l],1],c=color,lw=1.2)
   if (q[:,2]>.1).any():
    u,w=np.clip(uv[q[:,2]>.1].mean(0),[5,12],[610,460]);ax.text(u,w,v['name'],fontsize=7,color=color,bbox={'facecolor':'white','alpha':.7,'edgecolor':'none','pad':1})
  ax.set_title(f'{name}｜frame_{seq:06d}｜{len(selected)}个≥0.45单帧框',fontsize=11)
fig.suptitle('同一实测视角：三种前端 → 同一个Boxer三维预测\n阈值未跨模型校准，图例不是准确率评测；未显示多帧融合结果',fontsize=16);fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(O/'same_frame_comparison.png',dpi=160);print(json.dumps(report,ensure_ascii=False,indent=2))
