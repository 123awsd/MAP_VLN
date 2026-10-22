"""Measured cloud only, diagnostic ROI and display reflection; no segmentation edits."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/room305_sections';O.mkdir(exist_ok=True)
p=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')['p2'];g=5.595
m=(p[:,0]>-6.2)&(p[:,0]<-1.7)&(p[:,1]>3.7)&(p[:,1]<6.4)&(p[:,2]>g-.15)&(p[:,2]<g+2.65);p=p[m];x=-p[:,0];y=p[:,1];z=p[:,2]-g
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig=plt.figure(figsize=(15,11));ax=fig.add_subplot(221)
s=ax.scatter(x,y,c=z,cmap='viridis',vmin=0,vmax=2.5,s=1.6,lw=0);ax.add_patch(Rectangle((4.5,3.8),1.5,.75,fill=False,ec='#ed5a21',lw=2));ax.text(4.6,4.72,'橙框：右侧凸出对应范围',color='#d64918',fontsize=10);ax.set(title='① 实测点云俯视（包含各高度表面）',xlabel='−X / m',ylabel='Y / m',aspect='equal',xlim=(1.7,6.2),ylim=(3.7,6.4))
ax=fig.add_subplot(222,projection='3d');ax.scatter(x,y,z,c=z,cmap='viridis',vmin=0,vmax=2.5,s=.9,lw=0,depthshade=False);ax.set(xlabel='−X / m',ylabel='Y / m',zlabel='相对楼面高度 / m',title='② 原始表面三维关系（未补面）');ax.view_init(elev=19,azim=-65);ax.set_box_aspect((4.5,2.7,2.8))
ax=fig.add_subplot(223);m=(y>=4)&(y<4.5);ax.scatter(x[m],z[m],c=z[m],cmap='viridis',vmin=0,vmax=2.5,s=2,lw=0);ax.axvspan(4.5,6,facecolor='#ff8b42',alpha=.09);ax.set(title='③ 凸出处侧视：仅Y=4.0–4.5m切片',xlabel='−X / m',ylabel='相对楼面高度 / m',xlim=(1.7,6.2),ylim=(-.15,2.65));ax.grid(alpha=.15)
ax=fig.add_subplot(224);m=(y>=5)&(y<5.5);ax.scatter(x[m],z[m],c=z[m],cmap='viridis',vmin=0,vmax=2.5,s=2,lw=0);ax.set(title='④ 主体侧视对照：仅Y=5.0–5.5m切片',xlabel='−X / m',ylabel='相对楼面高度 / m',xlim=(1.7,6.2),ylim=(-.15,2.65));ax.grid(alpha=.15)
fig.suptitle('305及右侧凸出：700秒运行实测点云｜颜色表示高度，不表示房间类别',fontsize=16)
fig.subplots_adjust(left=.07,right=.91,bottom=.08,top=.91,hspace=.3,wspace=.25);cax=fig.add_axes([.94,.18,.015,.65]);fig.colorbar(s,cax=cax,label='相对楼面高度 / m');fig.text(.5,.025,'原点云保留；仅裁剪查看。没有官方真值替代、没有表面填补；−X/Y沿用之前的展示朝向。',ha='center',fontsize=10)
fig.savefig(O/'room305_raw_cloud.png',dpi=180);plt.close(fig)
