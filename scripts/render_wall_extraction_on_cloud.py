"""Measured wall extraction visualization; no algorithm or label changes."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_cloud_review';O.mkdir(exist_ok=True)
G=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,2,figsize=(16,23));records=[]
for f,g in enumerate([-.155,2.595,5.595],1):
 p=G[f'p{f-1}'];n=G[f'n{f-1}'];w=np.load(B/'measured'/f'L{f}_small_room_preserved.npz')['wall_points']
 selected=cKDTree(w).query(p,workers=2)[0]<1e-5
 vertical=(abs(n[:,2])<.35)&(p[:,2]>g+.15);rejected=vertical&~selected
 # Same height slab on both panels, consistent with prior dense top-down views.
 slab=(p[:,2]>g+.15)&(p[:,2]<g+2.2)
 for j in (0,1):
  ax=axs[f-1,j];q=p[slab];ax.scatter(-q[:,0],q[:,1],s=.16,c='#b5bec5',lw=0,rasterized=True,label='实测点云')
  if j:
   q=p[rejected&slab];ax.scatter(-q[:,0],q[:,1],s=.3,c='#e8a02e',lw=0,rasterized=True,label='未保留竖直候选（不等于墙）')
  q=p[selected&slab];ax.scatter(-q[:,0],q[:,1],s=.34,c='#bd1738',lw=0,rasterized=True,label='已提取墙面')
  ax.set(aspect='equal',xlim=(-10.5,8.5),ylim=(-2.5,18.5),xlabel='−X / m',ylabel='Y / m',title=f'{f}楼｜'+('实测点云＋提取墙面'if not j else'增加未保留候选对照'));ax.legend(loc='lower right',fontsize=8,markerscale=8)
 records.append({'floor':f,'vertical_candidates':int(vertical.sum()),'selected_vertical':int((vertical&selected).sum()),'display_height_above_ground_m':[.15,2.2]})
 if f==1:
  fig2,aa=plt.subplots(2,2,figsize=(14,12));roi=(p[:,0]>-.2)&(p[:,0]<3.3)&(p[:,1]>11.5)&(p[:,1]<16.4)
  for j in (0,1):
   ax=aa[0,j];q=p[roi&slab];ax.scatter(-q[:,0],q[:,1],s=1,c='#bbc3c9',lw=0)
   if j:
    q=p[roi&slab&rejected];ax.scatter(-q[:,0],q[:,1],s=1,c='#e8a02e',lw=0)
   q=p[roi&slab&selected];ax.scatter(-q[:,0],q[:,1],s=1,c='#bd1738',lw=0)
   ax.set(aspect='equal',xlim=(-3.4,.3),ylim=(11.4,16.5),xlabel='−X / m',ylabel='Y / m',title='房间俯视：'+('点云＋已提取墙'if not j else'橙色为未保留竖直候选'))
  for ax,xpos,title in zip(aa[1],[3.025,.175],['低保留率一侧墙','对侧墙']):
   m=(abs(p[:,0]-xpos)<.12)&(p[:,1]>12)&(p[:,1]<15.8)
   q=p[m];ax.scatter(q[:,1],q[:,2]-g,s=.5,c='#bbc3c9',lw=0,label='实测点云')
   q=p[m&rejected];ax.scatter(q[:,1],q[:,2]-g,s=.7,c='#e8a02e',lw=0,label='未保留竖直候选')
   q=p[m&selected];ax.scatter(q[:,1],q[:,2]-g,s=.8,c='#bd1738',lw=0,label='已提取墙面')
   den=int((m&vertical).sum());num=int((m&vertical&selected).sum());ax.set(xlim=(12,15.8),ylim=(0,2.6),xlabel='沿墙 Y / m',ylabel='相对楼面高度 / m',title=f'{title}：{num}/{den}个竖直候选保留');ax.legend(fontsize=8,markerscale=5)
  fig2.suptitle('一楼上方房间：上排俯视，下排墙面立面\n灰＝实测点云，红＝已提取墙，橙＝未保留竖直候选',fontsize=15);fig2.tight_layout(rect=[0,.02,1,.94]);fig2.savefig(O/'L1_upper_room_wall_detail.png',dpi=200);plt.close(fig2)
fig.suptitle('墙面直接叠在实测稠密点云上｜原始坐标不变，显示使用−X/Y\n红＝已提取墙；橙＝未保留竖直候选，可能包含家具及其他物体',fontsize=17);fig.tight_layout(rect=[0,.01,1,.96]);fig.savefig(O/'three_floors_walls_on_cloud.png',dpi=200);plt.close(fig)
(O/'manifest.json').write_text(json.dumps({'algorithm_changed':False,'uses_GT':False,'source_run':'coverage_lifecycle_700_r1_20261016','wall_source':'measured/L*_small_room_preserved.npz wall_points','candidate_definition':'abs(normal.z)<.35 and z>ground+.15; not semantic wall truth','floors':records},ensure_ascii=False,indent=2))
print(O)
