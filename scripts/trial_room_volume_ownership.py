"""Height-resolved measured-air ownership. No GT, room IDs, ROI or occupancy edits.
A core-derived lower air layer provides ownership evidence; high air alone cannot
claim a lower floor footprint. Ambiguous cells remain observed and untyped.
"""
from pathlib import Path
import json,ast
import numpy as np
from scipy import ndimage as nd
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'volume_room_ownership';O.mkdir(exist_ok=True)
F=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/observed_grid_final.npz');tree=cKDTree(F['free']);G=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')

def ownership_decision(room,core,lower_connected,floor_support,air_count):
    # Missing observations alone never reject a room. Need positive upper-air evidence.
    return room&~core&~lower_connected&~floor_support&(air_count>=2)
# Evidence-state checks: no data unchanged, upper-only untyped, floor / same-level / core protected.
m=np.ones(5,bool);z=np.zeros(5,bool)
assert np.array_equal(ownership_decision(m,np.array([0,0,0,0,1],bool),np.array([0,0,0,1,0],bool),np.array([0,0,1,0,0],bool),np.array([0,3,3,3,3])),[0,1,0,0,0])
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
rows=[];items=[]
for f,g in enumerate([-.155,2.595,5.595],1):
 a=np.load(B/'wall_structural_rooms'/f'L{f}_constrained.npz');old=a['labels'];r=float(a['resolution']);o=a['origin_xy'];shape=old.shape;y,x=np.indices(shape);xy=o+np.column_stack([x.ravel()+.5,y.ravel()+.5])*r
 heights=np.arange(np.ceil((g+.15-.05)/.1)*.1+.05,g+2.3,.1)
 free=[]
 for h in heights:
  q=np.column_stack([xy,np.full(len(xy),h)]);free.append((tree.query(q,workers=2)[0]<=.09).reshape(shape))
 free=np.array(free);counts=free.sum(axis=0)
 p=G[f'p{f-1}'];n=G[f'n{f-1}'];p=p[(abs(p[:,2]-g)<=.15)&(abs(n[:,2])>=.9)]
 floor=(cKDTree(p[:,:2]).query(xy,workers=2)[0]<=.12).reshape(shape) if len(p) else np.zeros(shape,bool)
 new=old.copy();coreall=a['seeds']>0;lowerall=np.zeros(shape,bool);detail=[]
 for k in np.unique(old[old>0]):
  room=old==k;core=room&coreall
  if not core.any():continue
  coverage=free[:,core].mean(axis=1);valid=np.flatnonzero(coverage>=.5)
  if not len(valid):
   detail.append({'id':int(k),'status':'no majority core air layer; unchanged'});continue
  level=int(valid[0]);band=free[level:min(level+3,len(free))].any(axis=0)
  cc,_=nd.label(band&room);ids=np.unique(cc[core&band]);ids=ids[ids>0];connected=room&np.isin(cc,ids)
  lowerall|=connected
  uncertain=ownership_decision(room,core,connected,floor,counts);new[uncertain]=0
  detail.append({'id':int(k),'reference_height_above_floor_m':round(float(heights[level]-g),3),'core_coverage':round(float(coverage[level]),3),'before_m2':round(room.sum()*r*r,4),'untyped_m2':round(uncertain.sum()*r*r,4)})
 untyped=(old>0)&(new==0)
 assert np.array_equal(new[coreall],old[coreall]);assert not np.any(untyped&floor);assert not np.any(untyped&lowerall)
 assert not np.any((new>0)&a['wall_barrier'])
 np.savez_compressed(O/f'L{f}_rooms.npz',labels=new,original_labels=old,untyped=untyped,observed_domain=a['constrained_domain'],wall_grid=a['measured_wall'],core=coreall,floor_support=floor,lower_connected=lowerall,free_by_height=free,heights=heights,origin_xy=o,resolution=r)
 row={'floor':f,'untyped_m2':round(untyped.sum()*r*r,4),'rooms_before':len(np.unique(old[old>0])),'rooms_after':len(np.unique(new[new>0])),'rooms':detail};rows.append(row);items.append((f,a,new,untyped,lowerall));print(row,flush=True)
# Shared plotting only; importing the experiment itself would rerun it.
src=R/'scripts/trial_wall_opening_ownership.py';node=next(n for n in ast.parse(src.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='panel');ns={'np':np,'plt':plt,'nd':nd,'patheffects':patheffects};exec(compile(ast.Module(body=[node],type_ignores=[]),str(src),'exec'),ns);panel=ns['panel']
fig,axs=plt.subplots(1,3,figsize=(18,8))
for ax,(f,a,new,u,lc) in zip(axs,items):panel(ax,f,a,new,f'{f}楼',u)
fig.suptitle('分高度空气证据约束归属｜灰色待定，原观测空间不变');fig.tight_layout();fig.savefig(O/'final_three_floors.png',dpi=180);plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(13,18))
for f,a,new,u,lc in items:
 panel(axs[f-1,0],f,a,a['labels'],f'{f}楼：原版');panel(axs[f-1,1],f,a,new,f'{f}楼：高度归属试验',u)
fig.tight_layout();fig.savefig(O/'comparison.png',dpi=160);plt.close(fig)
f,a,new,u,lc=items[-1];fig,axs=plt.subplots(1,3,figsize=(16,6))
for ax,l,title,uu in zip(axs,[a['labels'],a['labels'],new],['原版','蓝色：与核心低层空气相连','试验：灰色待定'],[None,None,u]):
 panel(ax,3,a,l,title,uu)
 if ax is axs[1]:
  overlay=np.zeros((*lc.shape,4));overlay[lc]=[0,.35,.85,.7];ax.imshow(overlay[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
 ax.set(xlim=(1.5,6.4),ylim=(3.7,6.6))
fig.tight_layout();fig.savefig(O/'room305_detail.png',dpi=180);plt.close(fig)
(O/'audit.json').write_text(json.dumps({'status':'experimental, not promoted','GT':False,'room_specific_rules':False,'space_unchanged':True,'core_majority_fraction':.5,'voxel_near_tolerance_m':.09,'lower_band_layers':3,'floor_surface_tolerance_m':.15,'limitations':['not a full volumetric semantic segmentation','absence of low air may be unobserved rather than blocked','same-height 2D connectivity from 3D measured voxels, not full 3D path proof','stepped floors and furniture need additional evidence','parameters are explicit exploratory assumptions, not parameter-free'],'floors':rows},ensure_ascii=False,indent=2))
