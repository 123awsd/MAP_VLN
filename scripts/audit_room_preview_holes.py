"""Classify enclosed blanks in cached room proposals without altering geometry."""
from pathlib import Path
import json
import numpy as np
import cv2
from scipy import ndimage as ndi
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,2,figsize=(11,16));rows=[]
for f in range(1,4):
 for j,src in enumerate(['measured','official']):
  a=np.load(O/src/f'L{f}_small_room_preserved.npz');labels=a['labels'];domain=a['observed_floor_domain'];wall=a['wall_grid'];res=float(a['resolution']);org=a['origin_xy'];barrier=cv2.dilate(wall,np.ones((3,3),np.uint8))>0
  holes=ndi.binary_fill_holes(labels>0)&(labels==0)
  reasons={'wall_projection':holes&domain&barrier,'no_domain_support':holes&~domain,'unassigned_free':holes&domain&~barrier}
  rgb=np.ones((*labels.shape,3));rgb[labels>0]=[.75,.83,.88]
  colors={'wall_projection':[.85,.15,.25],'no_domain_support':[1.,.65,.1],'unassigned_free':[.6,.2,.8]}
  for k,m in reasons.items():rgb[m]=colors[k]
  cc,n=ndi.label(holes);blobs=[]
  for k in range(1,n+1):
   m=cc==k;area=m.sum()*res**2
   if area>=.03:
    y,x=ndi.center_of_mass(m);blobs.append({'area_m2':round(area,4),'raw_xy_m':(org+np.array([x+.5,y+.5])*res).tolist(),'reason_area_m2':{s:round((v&m).sum()*res**2,4) for s,v in reasons.items()}})
  row={'floor':f,'source':src,'enclosed_blank_m2':round(holes.sum()*res**2,4),'reason_area_m2':{k:round(v.sum()*res**2,4) for k,v in reasons.items()},'largest_holes':sorted(blobs,key=lambda b:-b['area_m2'])[:12]};rows.append(row)
  ax=axs[f-1,j];ax.imshow(rgb[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);p=a['wall_points'];ax.scatter(-p[:,0],p[:,1],s=.1,c='#333333',lw=0);ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f}楼 '+('实测'if j==0 else'官方几何'))
fig.suptitle('分区内部空洞归因：红=墙投影遮罩，橙=支持域缺失，紫=可用域未分配',fontsize=12);fig.tight_layout(rect=[0,.03,1,.96]);fig.text(.5,.01,'仅统计分区包围的空白；开口连接外部的缺口未计入。支持域缺失不等于原始深度完全无点。',ha='center',fontsize=9);fig.savefig(O/'enclosed_holes_audit.png',dpi=150)
(O/'enclosed_holes_audit.json').write_text(json.dumps({'definition':'enclosed blanks in binary fill of positive labels; not all room-interior unknowns','grid_resolution_m':.05,'wall_mask':'3x3 dilation adds .05m each side, total approximately .15m cells','rows':rows},ensure_ascii=False,indent=2));print(json.dumps([{k:v for k,v in r.items() if k!='largest_holes'}for r in rows],ensure_ascii=False))
