"""Inspect stair-area blank masks against current measured cloud and official surfaces."""
from pathlib import Path
import json
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview';geo=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz');gt=np.load(R/'outputs/consistency_validation_20261015/latest_official_comparison/official_sampled_surfaces.npz')['points'];traj=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/video_cache/trajectory.npy')
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,3,figsize=(15,11));rows=[]
for f,g in enumerate([-.155,2.595,5.595]):
 d=np.load(O/'measured'/f'L{f+1}_small_room_preserved.npz');l=d['labels'];org=d['origin_xy'];res=float(d['resolution']);domain=d['observed_floor_domain'];wall=cv2.dilate(d['wall_grid'],np.ones((3,3),np.uint8))>0
 yy,xx=np.indices(l.shape);x=org[0]+(xx+.5)*res;y=org[1]+(yy+.5)*res;roi=(x>=-5.5)&(x<-1)&(y>=7.3)&(y<9.3);blank=roi&(l==0);nod=blank&~domain
 p,n=geo[f'p{f}'],geo[f'n{f}'];ij=np.floor((p[:,:2]-org)/res).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<l.shape[1])&(ij[:,1]>=0)&(ij[:,1]<l.shape[0]);p,n,ij=p[ok],n[ok],ij[ok];inblank=nod[ij[:,1],ij[:,0]];q=p[inblank];nn=n[inblank]
 allowed_height=(abs(q[:,2]-g)<.12)|((q[:,2]>g+1.8)&(q[:,2]<g+2.75));horizontal=abs(nn[:,2])>.9
 rows.append({'floor':f+1,'roi_raw_xy':[-5.5,-1,7.3,9.3],'blank_m2':round(blank.sum()*res**2,4),'no_domain_m2':round(nod.sum()*res**2,4),'wall_mask_m2':round((blank&domain&wall).sum()*res**2,4),'unassigned_m2':round((blank&domain&~wall).sum()*res**2,4),'measured_4cm_points_below_missing_domain_xy':len(q),'horizontal_points':int(horizontal.sum()),'horizontal_rejected_height':int((horizontal&~allowed_height).sum()),'accepted_support_points':int((horizontal&allowed_height).sum()),'z_range_m':q[:,2].min().item()if len(q)else None,'z_max_m':q[:,2].max().item()if len(q)else None})
 ax=axs[f,0];rgb=np.ones((*l.shape,3));rgb[l>0]=[.7,.82,.9];rgb[nod]=[1,.65,.15];rgb[blank&domain&wall]=[.8,.2,.2];ax.imshow(rgb[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);t=traj[(traj[:,1]>=-5.5)&(traj[:,1]<-1)&(traj[:,2]>=7.3)&(traj[:,2]<9.3)&(traj[:,3]>=g)&(traj[:,3]<g+2.75)];ax.scatter(-t[:,1],t[:,2],s=2,c='#111111');ax.set(xlim=(1,5.5),ylim=(7.3,9.3),aspect='equal',title=f'{f+1}楼 楼梯附近分区空白',xlabel='−X / m',ylabel='Y / m')
 for j,points in enumerate([p,gt],1):
  a=points[(points[:,0]>=-5.5)&(points[:,0]<-1)&(points[:,1]>=7.3)&(points[:,1]<9.3)&(points[:,2]>=g-.12)&(points[:,2]<g+2.75)];ax=axs[f,j];ax.scatter(-a[:,0],a[:,2],s=.25,c=a[:,1],cmap='viridis',lw=0,rasterized=True);ax.axhspan(g-.12,g+.12,color='green',alpha=.1);ax.axhspan(g+1.8,g+2.75,color='green',alpha=.1);ax.scatter(-t[:,1],t[:,3],s=3,c='red');ax.set(xlim=(1,5.5),ylim=(g-.2,g+2.8),title=('700秒实际点云'if j==1 else'官方几何')+'：−X/Z投影',xlabel='−X / m',ylabel='Z / m')
fig.suptitle('楼梯区域核查｜橙：支持域缺失；红：墙遮罩；黑/红点：本轮实际轨迹',fontsize=13);fig.tight_layout(rect=[0,.04,1,.95]);fig.text(.5,.015,'右侧绿色高度带为当前分割允许的水平面高度；投影跨越2m宽Y切片，不用于证明具体可通行空隙。',ha='center',fontsize=10);fig.savefig(O/'stair_gap_diagnosis.png',dpi=160)
(O/'stair_gap_diagnosis.json').write_text(json.dumps({'scope':'stair vicinity from prior location, current run data only; ROI is diagnostic not room truth','rows':rows},ensure_ascii=False,indent=2));print(json.dumps(rows,ensure_ascii=False))
