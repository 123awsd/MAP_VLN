"""Measured wall-end separators constrain room ownership, never observed space.
Reuses existing geometric opening proposals; rejects all core-intersecting proposals.
No GT, coordinate exception, area pruning or altered room seed threshold.
"""
from pathlib import Path
import json
import numpy as np
import cv2
from scipy import ndimage as nd
from segment_rooms_wall_structure import structure_barriers
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_opening_ownership';O.mkdir(exist_ok=True)

def assign(mask,core,cut):
    cc,_=nd.label(mask&~cut)
    ids=np.unique(cc[core]);ids=ids[ids>0]
    retained=mask&np.isin(cc,ids)
    # Separator cells touching the retained room stay assigned; do not punch a line-shaped hole.
    retained|=mask&cut&nd.binary_dilation(retained)
    return retained|core

def checks():
    m=np.zeros((15,22),bool);m[2:13,2:12]=1;m[7:10,12:20]=1
    core=np.zeros_like(m);core[5:10,5:9]=1
    cut=np.zeros_like(m);cut[:,12]=1
    result=assign(m,core,cut)
    assert result[core].all() and not result[8,18] and result[3,3]
    core[8,18]=1
    assert assign(m,core,cut)[8,18]  # A second supported core is never discarded.
    assert np.array_equal(assign(m,core,np.zeros_like(m)),m)
checks()
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
rows=[];items=[]
for f in (1,2,3):
    a=np.load(B/'wall_structural_rooms'/f'L{f}_constrained.npz');side=np.load(B/'wall_side_evidence'/f'L{f}_wall_sides.npz')
    old=a['labels'];core=a['seeds']>0;r=float(a['resolution']);o=a['origin_xy']
    _,_,segs,ops=structure_barriers(a['measured_wall'],r,o,max_opening=1.2,side=side)
    cut=np.zeros(old.shape,np.uint8);accepted=[];rejected=[]
    for op in ops:
        if op['length_m']>1.2:continue
        line=np.zeros_like(cut);ij=np.rint((np.array([op['a'],op['b']])-o)/r-.5).astype(int)
        cv2.line(line,tuple(ij[0]),tuple(ij[1]),1)
        if np.any((line>0)&core):rejected.append(op);continue
        cut|=line;accepted.append(op)
    new=old.copy();detail=[]
    for k in np.unique(old[old>0]):
        m=old==k;c=m&core
        if not c.any():continue
        keep=assign(m,c,cut>0);new[m&~keep]=0
        detail.append({'id':int(k),'before_m2':round(m.sum()*r*r,4),'untyped_m2':round((m&~keep).sum()*r*r,4)})
    untyped=(old>0)&(new==0)
    assert np.array_equal(new[core],old[core])
    assert np.all((new==old)|(new==0))
    assert not np.any((new>0)&a['wall_barrier'])
    np.savez_compressed(O/f'L{f}_rooms.npz',labels=new,original_labels=old,untyped=untyped,virtual_separator=cut,core=core,observed_domain=a['constrained_domain'],wall_grid=a['measured_wall'],origin_xy=o,resolution=r)
    rows.append({'floor':f,'untyped_m2':round(untyped.sum()*r*r,4),'rooms_before':len(np.unique(old[old>0])),'rooms_after':len(np.unique(new[new>0])),'accepted_separators':accepted,'core_intersection_rejected':rejected,'rooms':detail})
    items.append((f,a,new,untyped,cut))

def panel(ax,f,a,labels,title,untyped=None,cut=None):
    rgba=plt.get_cmap('tab20')((labels%20)/19);rgba[:,:,3]=(labels>0)*.7
    if untyped is not None:rgba[untyped]=[.65,.65,.65,1]
    ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
    o=a['origin_xy'];r=float(a['resolution']);y,x=np.where(a['measured_wall']);ax.scatter(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,s=.4,c='#333333',lw=0)
    if cut is not None:
        y,x=np.where(cut);ax.scatter(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,s=1,c='#cf2760',lw=0)
    for k in np.unique(labels[labels>0]):
        d=nd.distance_transform_edt(labels==k);y,x=np.unravel_index(d.argmax(),d.shape)
        t=ax.text(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,f'{f}-{k:02d}',ha='center',fontsize=8,clip_on=True);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
    ax.set(title=title,xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m')
fig,axs=plt.subplots(1,3,figsize=(18,8))
for ax,(f,a,new,u,cut) in zip(axs,items):panel(ax,f,a,new,f'{f}楼',u)
fig.suptitle('墙端分界约束归属｜灰色为保留的未确定空间，未删除观测范围');fig.tight_layout();fig.savefig(O/'final_three_floors.png',dpi=180);plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(13,18))
for f,a,new,u,cut in items:
    panel(axs[f-1,0],f,a,a['labels'],f'{f}楼：原版');panel(axs[f-1,1],f,a,new,f'{f}楼：试验，灰色归属待定',u)
fig.tight_layout();fig.savefig(O/'comparison.png',dpi=170);plt.close(fig)
f,a,new,u,cut=items[-1];fig,axs=plt.subplots(1,3,figsize=(16,6))
for ax,l,title,uu,cc in zip(axs,[a['labels'],a['labels'],new],['原版','候选分界（粉色）','试验：灰色待定'],[None,None,u],[None,cut,None]):
    panel(ax,3,a,l,title,uu,cc);ax.set(xlim=(1.5,6.4),ylim=(3.7,6.6))
fig.tight_layout();fig.savefig(O/'room305_detail.png',dpi=190);plt.close(fig)
(O/'audit.json').write_text(json.dumps({'status':'trial not promoted','uses_GT':False,'coordinate_or_room_exception':False,'space_domain_unchanged':True,'cores_unchanged':True,'max_opening_m':1.2,'tests':'synthetic branch and second core, empty separator, per-floor invariants passed','limitations':['virtual separators are geometric hypotheses, not verified doors','uses prior wall-side normals and existing room cores','same label cores may still represent different rooms','incorrect wall proposals may unassign legitimate narrow branches'],'floors':rows},ensure_ascii=False,indent=2))
print([(v['floor'],v['untyped_m2'],v['rooms_after'])for v in rows])
