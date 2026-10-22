#!/usr/bin/env python3
"""Render saved observed occupancy voxels by floor; no synthetic geometry."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.patches import Rectangle,Patch
from matplotlib.lines import Line2D
root=Path(__file__).resolve().parents[1];case=root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016';out=root/'outputs/consistency_validation_20261015/latest_floor_topdown';out.mkdir(parents=True,exist_ok=True)
grid=np.load(case/'observed_grid_final.npz');frames=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];trajectory=np.array([f['position']for f in frames]);t=np.array([f['capture_wall_s']for f in frames]);t-=t[0]
from matplotlib import font_manager
font_path='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font_path)
plt.rcParams['font.family']=font_manager.FontProperties(fname=font_path).get_name()
plt.rcParams['font.sans-serif']=[plt.rcParams['font.family'][0],'DejaVu Sans'];plt.rcParams['axes.unicode_minus']=False
extent=(-8.5,10.5,-2.5,18.5);res=.1;shape=(210,190);grounds=[-.155,2.595,5.595];bands=[(-1.5,2.595),(2.595,5.595),(5.595,8.05)];cmap=ListedColormap(['#ffffff','#e8f0f4','#173d50']);norm=BoundaryNorm([-.5,.5,1.5,2.5],3);audit=json.loads((case/'consistency_audit.json').read_text())
def panel(ax,i,overlay=True):
 z0=grounds[i]+.15;z1=min(grounds[i]+2.2,bands[i][1]);image=np.zeros(shape,dtype=np.uint8);counts={}
 for k,v in [('free',1),('occupied',2)]:
  pts=grid[k];q=pts[(pts[:,2]>=z0)&(pts[:,2]<z1)&(pts[:,0]>=extent[0])&(pts[:,0]<extent[1])&(pts[:,1]>=extent[2])&(pts[:,1]<extent[3])];ij=np.floor((q[:,:2]-[extent[0],extent[2]])/res).astype(int);image[ij[:,1],ij[:,0]]=v;counts[k]=len(q)
 ax.imshow(image,origin='lower',extent=extent,cmap=cmap,norm=norm,interpolation='nearest');ax.set_aspect('equal');ax.set_xlim(extent[:2]);ax.set_ylim(extent[2:]);ax.set_xlabel('X / m');ax.set_ylabel('Y / m');ax.set_xticks([-5,0,5,10]);ax.set_yticks([0,5,10,15]);ax.grid(alpha=.13,linewidth=.4)
 if overlay:
  m=(trajectory[:,2]>=bands[i][0])&(trajectory[:,2]<bands[i][1]);path=trajectory[:,:2].copy();path[~m]=np.nan;ax.plot(path[:,0],path[:,1],color='#ed8339',linewidth=.8,alpha=.9,zorder=4)
  idx=np.flatnonzero(m)
  if len(idx):ax.scatter(*trajectory[idx[-1],:2],s=24,c='#b82832',edgecolors='white',linewidths=.5,zorder=5)
  if i==1:
   for x,y,w,h,lab,color in [(-1.4,4.3,1.25,1.9,'2号旁：已进入','#a139b1'),(-1.3,9.7,1.2,2.,'9号右上：未进入','#d92d43')]:
    ax.add_patch(Rectangle((x,y),w,h,fill=False,edgecolor=color,linewidth=1.6,zorder=6));ax.annotate(lab,(x,y+h/2),(x-4.8,y+h/2+.6),color=color,fontsize=8,arrowprops={'arrowstyle':'-','color':color,'lw':.8},bbox={'fc':'white','ec':'none','alpha':.8,'pad':1},zorder=7)
 known=audit['floor_volumes']['floors'][i]['known_m3'];ax.set_title(f'{i+1}楼｜已知体积 {known:.3f} m³\n点云切片 z = {z0:.3f}–{z1:.3f} m',fontsize=11,pad=9)
 return {'floor':i+1,'point_slice_z_m':[z0,z1],'trajectory_band_z_m':bands[i],'points':counts}
meta=[]
for overlay,name in [(True,'latest_three_floors_topdown'),(False,'latest_three_floors_cloud_only')]:
 fig,axes=plt.subplots(1,3,figsize=(16,7.5),facecolor='white')
 for i,ax in enumerate(axes):meta.append(panel(ax,i,overlay))
 fig.suptitle('最新504秒建图｜分层点云俯视图',fontsize=17,y=.97)
 handles=[Patch(facecolor='#173d50',label='观测占据点云'),Patch(facecolor='#e8f0f4',label='观测自由体素投影'),Patch(facecolor='white',edgecolor='#cccccc',label='切片中无观测点')]
 if overlay:handles += [Line2D([0],[0],color='#ed8339',lw=1.5,label='该层真实轨迹'),Line2D([0],[0],marker='o',color='white',markerfacecolor='#b82832',label='该层最后采集位置')]
 fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.075),ncol=len(handles),frameon=False,fontsize=10)
 fig.text(.5,.035,'来源：A3_observation_lifecycle 最终运行（503.91 s）｜0.1 m体素；去除地板/天花板高度，仅显示已观测数据。',ha='center',fontsize=9,color='#555555')
 fig.subplots_adjust(left=.05,right=.985,bottom=.17,top=.84,wspace=.18);fig.savefig(out/(name+'.png'),dpi=180,facecolor='white');plt.close(fig)
for i in range(3):
 fig,ax=plt.subplots(figsize=(8,9));panel(ax,i);fig.tight_layout();fig.savefig(out/f'floor{i+1}_topdown.png',dpi=180);plt.close(fig)
(out/'metadata.json').write_text(json.dumps({'case':case.name,'source':'observed_grid_final.npz','extent_m':extent,'slice_metadata':meta[:3],'note':'Per-XY occupied point wins over free projection; white is no observed point in this slice, not GT room completeness. Known volumes in titles use original full floor bands, not the display slice.'},indent=2))
print(out/'latest_three_floors_topdown.png')
