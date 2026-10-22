"""Offline ownership trial: direct in-region visibility to sampled existing room cores.
Unverified space stays observed/untyped, never converted to walls or exterior.
"""
from pathlib import Path
import json
import numpy as np
from scipy import ndimage as nd
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'core_visibility_ownership';O.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False

def visible_targets(mask,targets,cores):
 """Test all supplied core samples. Dense quarter-cell sampling checks both sides at corners."""
 visible=np.zeros(len(targets),bool)
 for core in cores:
  ids=np.flatnonzero(~visible)
  if not len(ids):break
  for start in range(0,len(ids),2048):
   ix=ids[start:start+2048];dst=targets[ix];delta=dst-core;n=max(1,int(np.max(np.abs(delta))*4))
   q=core[None,None,:]+delta[:,None,:]*np.linspace(0,1,n+1)[None,:,None]
   # Pixel centers are integer; floor/ceil around epsilon rejects corner cutting.
   lo=np.floor(q+.5-1e-7).astype(int);hi=np.floor(q+.5+1e-7).astype(int)
   good=mask[lo[:,:,0],lo[:,:,1]]&mask[hi[:,:,0],hi[:,:,1]]&mask[lo[:,:,0],hi[:,:,1]]&mask[hi[:,:,0],lo[:,:,1]]
   visible[ix[good.all(axis=1)]]=True
 return visible
# Controlled open room and wall-turn examples use the same production check.
test=np.ones((12,12),bool);assert visible_targets(test,np.array([[10,10]]),np.array([[1,1]]))[0]
test[2:10,6]=False;assert not visible_targets(test,np.array([[5,9]]),np.array([[5,2]]))[0]
rows=[];datasets=[]
for f in (1,2,3):
 a=np.load(B/'independent_room_support'/f'L{f}_constrained.npz');old=a['labels'];res=float(a['resolution']);final=old.copy();coremap=np.zeros_like(old,bool);unknown=np.zeros_like(old,bool);detail=[]
 for rid in np.unique(old[old>0]):
  mask=old==rid;dist=nd.distance_transform_edt(mask)*res;core=mask&(dist>.45);coremap|=core
  if not core.any():
   detail.append({'id':int(rid),'status':'no core, unchanged'});continue
  # Use each connected core boundary sampled every 2 raster cells, plus its maximum-clearance point.
  boundary=core&~nd.binary_erosion(core);points=np.argwhere(boundary);sample=points[(points[:,0]%2==0)|(points[:,1]%2==0)]
  cc,n=nd.label(core);extra=[]
  for k in range(1,n+1):extra.append(np.unravel_index(np.where(cc==k,dist,-1).argmax(),dist.shape))
  sample=np.unique(np.vstack([sample,np.array(extra)]),axis=0)
  targets=np.argwhere(mask&~core);ok=visible_targets(mask,targets,sample)
  rejected=targets[~ok];unknown[rejected[:,0],rejected[:,1]]=True;final[rejected[:,0],rejected[:,1]]=0
  detail.append({'id':int(rid),'before_m2':round(mask.sum()*res**2,4),'untyped_m2':round(len(rejected)*res**2,4),'core_samples':len(sample),'tested_cells':len(targets)})
 assert not np.any((final>0)&a['wall_barrier']);assert np.all((final==0)|(final==old));assert np.array_equal(final[coremap],old[coremap])
 np.savez_compressed(O/f'L{f}_rooms.npz',labels=final,original_labels=old,untyped=unknown,core=coremap,wall_grid=a['measured_wall'],observed_domain=a['constrained_domain'],origin_xy=a['origin_xy'],resolution=res)
 row={'floor':f,'untyped_area_m2':round(unknown.sum()*res**2,4),'rooms':detail};rows.append(row);datasets.append((f,a,final,unknown,coremap));print(row,flush=True)

def panel(ax,f,a,l,title,unknown=None,core=None):
 rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7
 if unknown is not None:rgba[unknown]=[.65,.65,.65,1]
 if core is not None:rgba[core]=[.95,.2,.4,1]
 ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest');o=a['origin_xy'];r=float(a['resolution']);y,x=np.where(a['measured_wall']);ax.scatter(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,s=.3,c='#852c2c',lw=0)
 for rid in np.unique(l[l>0]):
  d=nd.distance_transform_edt(l==rid);yy,xx=np.unravel_index(d.argmax(),d.shape);t=ax.text(-(o[0]+(xx+.5)*r),o[1]+(yy+.5)*r,f'{f}-{rid:02d}',ha='center',fontsize=8,clip_on=True);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 ax.set(title=title,xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m')
fig,axs=plt.subplots(3,2,figsize=(13,18))
for f,a,l,u,c in datasets:
 panel(axs[f-1,0],f,a,a['labels'],f'{f}楼：原归属');panel(axs[f-1,1],f,a,l,f'{f}楼：归属检查（灰色待定）',u)
 fig2,aa=plt.subplots(1,3,figsize=(16,7));panel(aa[0],f,a,a['labels'],'① 原分区');panel(aa[1],f,a,a['labels'],'② 沿用房间核心（粉色）',core=c);panel(aa[2],f,a,l,'③ 核心联系不足：灰色待定',u);fig2.tight_layout();fig2.savefig(O/f'L{f}_stages.png',dpi=160);plt.close(fig2)
 if f==3:
  fig2,aa=plt.subplots(1,2,figsize=(14,7));panel(aa[0],f,a,a['labels'],'原归属');panel(aa[1],f,a,l,'归属检查，灰色保留为待定空间',u)
  for ax in aa:ax.set(xlim=(-1.5,6.3),ylim=(3.5,10.2))
  fig2.tight_layout();fig2.savefig(O/'L3_detail.png',dpi=180);plt.close(fig2)
fig.suptitle('三层统一归属检查：保留空间证据，不把细条直接判成室外');fig.tight_layout(rect=[0,.02,1,.97]);fig.savefig(O/'three_floors_before_after.png',dpi=160)
(O/'audit.json').write_text(json.dumps({'status':'experimental ownership heuristic, not promoted','uses_GT':False,'coordinate_or_room_specific_rules':False,'core_radius_m':.45,'core_boundary_sample_grid_cells':2,'limits':['sampled cores, unverified does not prove invisible to every core pixel','direct 2D visibility is a conservative ownership heuristic, not semantic room truth','L-shaped narrow branches may become untyped','observed space and physical map preserved'],'results':rows},ensure_ascii=False,indent=2))
