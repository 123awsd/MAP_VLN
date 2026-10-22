"""Read-only measured 3D sections; ROI selects diagnostic output, not algorithm rules."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'room305_sections';O.mkdir(exist_ok=True)
a=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz');p=a['p2'];n=a['n2'];w=np.load(B/'wall_structural_evidence/L3_walls.npz')['wall_points'];g=5.595
roi=(p[:,0]>-6.2)&(p[:,0]<-1.7)&(p[:,1]>3.7)&(p[:,1]<6.4);q=p[roi];nn=n[roi];wr=w[(w[:,0]>-6.2)&(w[:,0]<-1.7)&(w[:,1]>3.7)&(w[:,1]<6.4)]
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(2,3,figsize=(18,11));bins=[(0,.5),(.5,1.6),(1.6,2.6)]
for ax,(lo,hi)in zip(axs[0],bins):
 m=(q[:,2]-g>=lo)&(q[:,2]-g<hi);ax.scatter(-q[m,0],q[m,1],s=.6,c='#a4b2bb',lw=0);v=(wr[:,2]-g>=lo)&(wr[:,2]-g<hi);ax.scatter(-wr[v,0],wr[v,1],s=1,c='#b9193d',lw=0);ax.set(xlim=(1.7,6.2),ylim=(3.7,6.4),aspect='equal',title=f'相对楼面{lo}–{hi}m：点云＋墙',xlabel='−X / m',ylabel='Y / m')
records=[]
for ax,(lo,hi,title)in zip(axs[1],[(4.,4.5,'凸出细条'),(4.5,4.8,'主体与凸出接合'),(5.,5.5,'房间主体')]):
 m=(q[:,1]>=lo)&(q[:,1]<hi);ax.scatter(-q[m,0],q[m,2]-g,s=.8,c='#b8c0c6',lw=0);h=m&(abs(nn[:,2])>.9);v=m&(abs(nn[:,2])<.35);ax.scatter(-q[h,0],q[h,2]-g,s=1,c='#168b97',lw=0,label='近水平表面');ax.scatter(-q[v,0],q[v,2]-g,s=.7,c='#bd2943',lw=0,label='竖直候选');ax.set(xlim=(1.7,6.2),ylim=(-.15,2.6),title=f'{title}：Y={lo}–{hi}m',xlabel='−X / m',ylabel='相对楼面高度 / m');ax.legend(fontsize=8)
 # Raw point density is not area; histogram only describes sampled surface heights.
 for xmin,xmax,name in [(1.8,4.4,'主体侧'),(4.5,6.,'凸出侧')]:
  mask=h&(-q[:,0]>=xmin)&(-q[:,0]<xmax);hist,edges=np.histogram(q[mask,2]-g,bins=np.arange(-.2,2.81,.1));top=np.argsort(hist)[-5:][::-1];records.append({'section_y':[lo,hi],'side':name,'horizontal_points':int(mask.sum()),'height_peaks':[{'z_relative':round(float((edges[i]+edges[i+1])/2),2),'points':int(hist[i])}for i in top if hist[i]]})
fig.suptitle('3-05三维剖面：上排分高度俯视，下排沿−X剖面｜实际测量，不补面',fontsize=16);fig.tight_layout(rect=[0,.02,1,.95]);fig.savefig(O/'height_sections.png',dpi=180);plt.close(fig)
(O/'audit.json').write_text(json.dumps({'ground_reference':g,'uses_GT':False,'changes_algorithm':False,'records':records},ensure_ascii=False,indent=2));print(json.dumps(records,ensure_ascii=False))
