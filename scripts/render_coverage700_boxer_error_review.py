"""Review saved 2-D proposals beside frame-local 3-D output; no new inference."""
from pathlib import Path
import csv,json,itertools
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];O=R/'outputs/boxer/coverage700_measured_20261017';D=O/'detections/episode'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP'
two=list(csv.DictReader((D/'owl_2dbbs.csv').open()));three=list(csv.DictReader((D/'boxer_3dbbs.csv').open()));frames={}
for p in (O/'episode').glob('frame_*.npz'):
 with np.load(p)as a:frames[int(a['time_ns'])]=p
best=max((v for v in two if v['name']=='toilet'),key=lambda v:float(v['prob']))
paths=[O/'episode/frame_000080.npz',O/'episode/frame_000990.npz',frames[int(best['time_ns'])]]
signs=np.array(list(itertools.product((-.5,.5),repeat=3)));edges=[(i,j)for i in range(8)for j in range(i+1,8)if np.sum(signs[i]!=signs[j])==1]
fig,axs=plt.subplots(3,2,figsize=(14,15))
for ri,p in enumerate(paths):
 with np.load(p)as a:rgb=a['rgb'];pos=a['position'];rot=Rotation.from_quat(a['orientation_xyzw']).as_matrix();t=int(a['time_ns'])
 proposals=[v for v in two if int(v['time_ns'])==t];preds=[v for v in three if int(v['time_ns'])==t]
 for ax in axs[ri]:ax.imshow(rgb);ax.set(xlim=(0,640),ylim=(480,0));ax.axis('off')
 for j,v in enumerate(proposals):
  if float(v['prob'])<.3:continue
  color=plt.get_cmap('tab10')(j%10);x=float(v['x1'])*640/float(v['img_width']);y=float(v['y1'])*480/float(v['img_height']);w=(float(v['x2'])-float(v['x1']))*640/float(v['img_width']);h=(float(v['y2'])-float(v['y1']))*480/float(v['img_height'])
  axs[ri,0].add_patch(Rectangle((x,y),w,h,fill=False,edgecolor=color,lw=1.5));axs[ri,0].text(x,max(10,y),f"{v['name']} {float(v['prob']):.2f}",fontsize=8,color=color,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none','pad':1})
 for j,v in enumerate(preds):
  if float(v['prob'])<.45:continue
  color=plt.get_cmap('tab10')(j%10);c=np.array([float(v[k+'_world_object'])for k in ('tx','ty','tz')]);sz=np.array([float(v['scale_'+k])for k in 'xyz']);ro=Rotation.from_quat([float(v[k+'_world_object'])for k in ('qx','qy','qz','qw')]).as_matrix();q=((signs*sz)@ro.T+c-pos)@rot;uv=q[:,:2]/np.maximum(q[:,2:3],1e-6)*320+[320,240]
  for k,l in edges:
   if q[k,2]>.1 and q[l,2]>.1:axs[ri,1].plot(uv[[k,l],0],uv[[k,l],1],c=color,lw=1.3)
  if (q[:,2]>.1).any():
   u,w=np.clip(uv[q[:,2]>.1].mean(axis=0),[10,12],[610,460]);axs[ri,1].text(u,w,f"{v['name']} {float(v['prob']):.2f}",fontsize=8,color=color,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none','pad':1})
 axs[ri,0].set_title(f'{p.stem} · 原二维分类/框（分数≥0.30）');axs[ri,1].set_title('同帧三维框（2D/3D平均分≥0.45）')
fig.suptitle('误差来源核对：左侧已经出现的类别错误，并非多帧融合才产生\n右侧分数混合了二维分类与三维模型输出，不代表类别正确率',fontsize=15);fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(O/'error_review_2d_vs_3d.png',dpi=160)
