"""Audit and retain small geometric seeds using cached walls; no exploration/GT mixing."""
from pathlib import Path
import json
import numpy as np
from scipy import ndimage as ndi
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,4,figsize=(18,16));report=[]
for f in range(1,4):
 for j,src in enumerate(['measured','official']):
  d=np.load(O/src/f'L{f}_walls_rooms.npz');old=d['labels'];domain=d['observed_floor_domain'];wall=d['wall_grid'];res=float(d['resolution']);org=d['origin_xy'];free=domain&~(cv2.dilate(wall,np.ones((3,3),np.uint8))>0);dist=ndi.distance_transform_edt(free)*res
  core,n=ndi.label(dist>.45);seed=np.zeros_like(old);count=0;rescued=[]
  for k in range(1,n+1):
   m=core==k;area=m.sum()*res**2
   if area>=.04:
    count+=1;seed[m]=count
    if area<1:rescued.append({'seed_id':count,'core_area_m2':round(area,4)})
  # Small isolated free components may be too narrow to contain any .45m core.
  comp,ncomp=ndi.label(free)
  for k in range(1,ncomp+1):
   m=comp==k
   if not np.any(seed[m]) and m.sum()*res**2>=.3 and dist[m].max()>=.2:
    count+=1;seed[m&(dist>=max(.2,dist[m].max()-.05))]=count
  labels=ndi.watershed_ift(np.uint8(255*(1-np.clip(dist/2,0,1))),seed);labels[~free]=0
  for k in range(1,ncomp+1):
   m=comp==k
   if not np.any(seed[m]):labels[m]=0
  for k in range(1,count+1):
   # Reject tiny output regions as noise, not on the eroded core area.
   if (labels==k).sum()*res**2<.3:labels[labels==k]=0
  ids=np.unique(labels);ids=ids[ids>0];new=np.zeros_like(labels)
  for k,v in enumerate(ids,1):new[labels==v]=k
  np.savez_compressed(O/src/f'L{f}_small_room_preserved.npz',labels=new,wall_grid=wall,observed_floor_domain=domain,origin_xy=org,resolution=res,wall_points=d['wall_points'])
  row={'floor':f,'source':src,'before':int(old.max()),'after':len(ids),'rescued_core_candidates':rescued,'areas_m2':[round((new==k).sum()*res**2,3) for k in range(1,len(ids)+1)]};report.append(row)
  for mode,l in enumerate([old,new]):
   ax=axes[f-1,2*j+mode];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest');p=d['wall_points'];ax.scatter(-p[:,0],p[:,1],s=.2,c='#ad2340',lw=0,rasterized=True)
   for k in np.unique(l):
    if k:
     y,x=ndi.center_of_mass(l==k);ax.text(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,str(k),fontsize=8,ha='center')
   ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f}楼 '+('实测'if j==0 else'官方几何')+(' 原版'if mode==0 else' 保留小区域'))
fig.suptitle('小区域保留对照：修正对腐蚀后种子面积的过强过滤',fontsize=16);fig.tight_layout(rect=[0,.025,1,.96]);fig.text(.5,.012,'数字为自动分区ID，不是原房间编号；小区域含走廊/碎片，仍需门洞和边界验证。',ha='center');fig.savefig(O/'small_room_preservation_comparison.png',dpi=160)
(O/'small_room_seed_audit.json').write_text(json.dumps({'change':'minimum eroded core 1.0 -> 0.04 m2; final output minimum .3m2; isolated narrow component rescue','unchanged':'source pointcloud, walls, domain, .45m distance threshold','results':report},ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
