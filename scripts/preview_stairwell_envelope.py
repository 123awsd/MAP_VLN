"""Separate logical stairwell envelope from physical floor support and navigation."""
from pathlib import Path
import json,numpy as np,cv2
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview';D=O/'measured';items=[];envelopes=[]
for f in range(1,4):
 a=dict(np.load(D/f'L{f}_stair_aware_regions.npz'));b=cv2.morphologyEx(a['wall_grid'],cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;cc,n=nd.label(nd.binary_fill_holes(b)&~b);s=a['stair_logical_ids']>0
 ids,c=np.unique(cc[s],return_counts=True);valid=ids>0;mask=np.zeros_like(s)
 if valid.any():mask=cc==ids[valid][np.argmax(c[valid])];mask=nd.binary_fill_holes(mask)
 items.append(a);envelopes.append(mask)
# Shared overlap is a conservative cross-floor envelope proposal, not measured floor.
shared=envelopes[1]&envelopes[2]
assert shared.sum()*.0025>1
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,2,figsize=(12,14));report=[]
for i,a in enumerate(items):
 mask=envelopes[i].copy() if envelopes[i].any()else shared.copy()
 # Upper-floor shared core constrains stair extent; preserve separate room labels
 # outside that core even when a doorway joins their wall-enclosed components.
 before=mask.copy()
 local=np.load(D/f'L{i+1}_small_room_preserved.npz')['labels']
 removed_ids=[]
 if i==1:
  # Prevent an attached room/doorway from extending the cross-floor stair core.
  yy,xx=np.where(shared);coords=np.column_stack([xx,yy]);_,vec=np.linalg.eigh(np.cov(coords.T));across=vec[:,0]
  grid_y,grid_x=np.indices(mask.shape);projection=grid_x*across[0]+grid_y*across[1];limits=coords@across
  mask &= (projection>=limits.min()-1)&(projection<=limits.max()+1)
  for rid in np.unique(local[mask&~shared]):
   if not rid:continue
   region=local==rid
   overlap=(region&shared).sum()/max(1,region.sum())
   if overlap<.1:
    mask[region]=False;removed_ids.append(int(rid))
  cc,nn=nd.label(mask);ids,c=np.unique(cc[shared&mask],return_counts=True);valid=ids>0
  if valid.any():mask=cc==ids[valid][np.argmax(c[valid])]
 origin=a['origin_xy'];res=float(a['resolution']);old=a['labels'];rooms=old.copy();rooms[mask]=old.max()+1;barrier=cv2.dilate(a['wall_grid'],np.ones((3,3),np.uint8))>0
 # Keep physical wall/floor masks intact in separate arrays; envelope is semantic extent only.
 missing=mask&~a['original_room_domain'];np.savez_compressed(D/f'L{i+1}_stairwell_envelope.npz',logical_labels=rooms,stairwell_envelope=mask,original_room_domain=a['original_room_domain'],wall_grid=a['wall_grid'],origin_xy=origin,resolution=res,inferred_from_other_floors=np.array(i==0))
 for j in [0,1]:
  ax=axs[i,j];rgba=plt.get_cmap('tab20')((old%20)/19);rgba[:,:,3]=(old>0)*.6
  if j:rgba[mask]=[.05,.72,.78,.85]
  ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);y,x=np.where(a['wall_grid']>0);ax.scatter(-(origin[0]+(x+.5)*res),origin[1]+(y+.5)*res,s=.3,c='#a51b32',lw=0)
  if j:
   y,x=nd.center_of_mass(mask);ax.text(-(origin[0]+(x+.5)*res),origin[1]+(y+.5)*res,'SF1',ha='center',weight='bold');ax.contour(- (origin[0]+(np.arange(old.shape[1])+.5)*res),origin[1]+(np.arange(old.shape[0])+.5)*res,mask.astype(int),levels=[.5],colors=['#007f89'],linewidths=[1.2])
  ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',title=f'{i+1}楼｜'+('原局部轨迹支持'if not j else'独立楼梯间空间轮廓'+('（跨层推断）'if i==0 else'')),xlabel='−X / m',ylabel='Y / m')
 report.append({'floor':i+1,'logical_envelope_area_m2':round(mask.sum()*res**2,3),'includes_previously_unsupported_projection_m2':round(missing.sum()*res**2,3),'source':'shared overlap of upper-floor wall-enclosed components'if i==0 else'wall-enclosed component intersecting witnessed stair route','physical_maps_unchanged':True,'preserved_separate_room_ids':removed_ids,'removed_from_stair_envelope_m2':round((before&~mask).sum()*res**2,3)})
fig.suptitle('楼梯间单独建模：青色表示空间归属，不表示存在水平楼板',fontsize=14);fig.tight_layout(rect=[0,.04,1,.96]);fig.text(.5,.015,'墙缝闭合仅用于生成空间轮廓候选；一楼轮廓由上层共同范围推断，待核验。未改占据、FREE、墙体或导航地图。',ha='center',fontsize=9);fig.savefig(O/'stairwell_envelope_before_after.png',dpi=150)
(O/'stairwell_envelope_audit.json').write_text(json.dumps({'status':'provisional logical space envelope, not navigation geometry','wall_gap_close_kernel_m':.35,'floor1_inferred':True,'results':report},ensure_ascii=False,indent=2));print(report)
