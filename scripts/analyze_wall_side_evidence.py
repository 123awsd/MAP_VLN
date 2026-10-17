"""Measured-only wall-side evidence prototype. No unseen side is called exterior.
Patch normals use sign-invariant 2D dyads, not arbitrary point-normal orientation.
Independent final FREE, nonvertical surface and actual depth-endpoint view evidence.
"""
from pathlib import Path
import json
import numpy as np
from scipy.spatial import cKDTree
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_side_evidence';O.mkdir(exist_ok=True);run=R/'outputs/coverage_lifecycle_700_r1_20261016'
geo=np.load(run/'original_style_video/geometry.npz');mapdata=np.load(run/'observed_grid_final.npz');free_tree=cKDTree(mapdata['free']);rays=np.load(B/'evidence_test/measured_free_rays.npz')
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,3,figsize=(17,17));summary=[];grounds=[-.155,2.595,5.595]
def decide(free_support,struct_support,views):
 # Seeing one surface alone does not establish that the whole side is indoors.
 return (free_support>=.25)&(struct_support>=1/3)
# Regression: flipping input normals must preserve canonical side assignment.
theta=np.array([.2,.2+np.pi]);dyad=np.column_stack([np.cos(2*theta),np.sin(2*theta)]);assert np.allclose(dyad[0],dyad[1])
assert not decide(np.array([0.]),np.array([1.]),np.array([10]))[0]
assert decide(np.array([.5]),np.array([.8]),np.array([0]))[0]
for f,g in enumerate(grounds):
 old=np.load(B/'measured'/f'L{f+1}_small_room_preserved.npz');walls=old['wall_points'];p,n=geo[f'p{f}'],geo[f'n{f}'];_,nearest=cKDTree(p).query(walls,workers=2);wn=n[nearest,:2];length=np.linalg.norm(wn,axis=1);wn=wn/np.maximum(length[:,None],1e-9);angles=np.arctan2(wn[:,1],wn[:,0]);keys=np.floor(walls[:,:2]/.2).astype('int32');unique,inverse=np.unique(keys,axis=0,return_inverse=True);N=len(unique);counts=np.bincount(inverse);centers=np.column_stack([np.bincount(inverse,weights=walls[:,k])/counts for k in range(3)]);cx=np.bincount(inverse,weights=np.cos(2*angles))/counts;sy=np.bincount(inverse,weights=np.sin(2*angles))/counts;coherence=np.hypot(cx,sy);axis=.5*np.arctan2(sy,cx);normals=np.column_stack([np.cos(axis),np.sin(axis)]);valid=coherence>=.65
 view_counts=np.zeros((N,2),np.int32);w_tree=cKDTree(walls)
 for i,pose in enumerate(rays['poses']):
  if not(g+.15<=pose[3]<g+2.75):continue
  q=rays[f'r{i:04d}'];dist,ix=w_tree.query(q);keep=dist<=.12
  if not keep.any():continue
  q,ix=q[keep],ix[keep];pid=inverse[ix];delta=pose[1:3]-q[:,:2];signed=np.einsum('ij,ij->i',delta,normals[pid]);incidence=np.abs(signed)/np.maximum(np.linalg.norm(delta,axis=1),1e-6);keep=(incidence>=.25)&valid[pid];pairs=np.unique(np.column_stack([pid[keep],(signed[keep]<0).astype(int)]),axis=0)
  if len(pairs):view_counts[pairs[:,0],pairs[:,1]]+=1
 support_points=p[(abs(n[:,2])>=.35)&(p[:,2]>=g-.15)&(p[:,2]<g+2.85)];stree=cKDTree(support_points[:,:2])
 free_fraction=np.zeros((N,2));structure_fraction=np.zeros((N,2));free_heights=np.zeros((N,2),int)
 for side,sign in enumerate([1,-1]):
  probe_xy=centers[:,None,:2]+sign*np.array([.25,.45,.65])[None,:,None]*normals[:,None,:]
  probe=np.zeros((N,3,4,3));probe[:,:,:,:2]=probe_xy[:,:,None,:];probe[:,:,:,2]=g+np.array([.45,.85,1.25,1.65])[None,None,:];distance=free_tree.query(probe.reshape(-1,3),workers=2)[0].reshape(N,3,4);hits=distance<=.09;free_fraction[:,side]=hits.mean((1,2));free_heights[:,side]=hits.any(1).sum(1)
  structure_fraction[:,side]=(stree.query(probe_xy.reshape(-1,2),workers=2)[0].reshape(N,3)<=.12).mean(1)
 supported=decide(free_fraction,structure_fraction,view_counts)&(free_heights>=2)&valid[:,None]
 classification=supported.sum(1).astype('uint8');classification[~valid]=3
 # A view-vs-FREE side disagreement is diagnostic, not an instruction to flip/delete walls.
 disagreement=(view_counts>=3)&~supported
 np.savez_compressed(O/f'L{f+1}_wall_sides.npz',centers_xyz=centers,canonical_normal_xy=normals,normal_coherence=coherence,free_fraction=free_fraction,free_height_layers=free_heights,structure_fraction=structure_fraction,depth_endpoint_view_frames=view_counts,supported_side=supported,classification=classification,view_without_support=disagreement,wall_point_patch_id=inverse)
 a=np.load(B/'wall_guided/measured_review'/f'L{f+1}_rooms.npz');labels=a['labels'];org=a['origin_xy'];res=float(a['resolution']);regionrows=[]
 # Shared wall patches may border several regions; no candidate ID affects decisions.
 for rid in np.unique(labels):
  if not rid:continue
  region=labels==rid;ys,xs=np.where(region);ringxy=org+np.column_stack([xs+.5,ys+.5])*res;rtree=cKDTree(ringxy);distance,ix=rtree.query(centers[:,:2]);near=distance<=.2
  toward=ringxy[ix]-centers[:,:2];side=(np.einsum('ij,ij->i',toward,normals)<0).astype(int);usable=near&valid;frac=float(supported[np.arange(N),side][usable].mean())if usable.any()else None
  regionrows.append({'id':f'{f+1}-{int(rid):02d}','nearby_or_inside_wall_patches':int(usable.sum()),'toward_region_supported_fraction':frac,'note':'local sampling only; does not certify room/corridor/exterior'})
 row={'floor':f+1,'patches':N,'two_sides_supported':int((classification==2).sum()),'one_side_supported_other_unknown':int((classification==1).sum()),'no_side_supported':int((classification==0).sum()),'normal_ambiguous':int((classification==3).sum()),'patches_with_3frame_view_evidence':int((view_counts>=3).any(1).sum()),'regions':regionrows};summary.append(row);print({k:v for k,v in row.items()if k!='regions'},flush=True)
 colors=np.array(['#a5aab0','#e78b24','#7d4bbc','#d34b4b']);q=p[(p[:,2]>g+.15)&(p[:,2]<g+2.2)]
 for j,ax in enumerate(axs[f]):
  ax.scatter(-q[:,0],q[:,1],s=.15,c='#d5dce0',lw=0,rasterized=True)
  if j==0:
   ax.scatter(-centers[:,0],centers[:,1],s=5,c=colors[classification],lw=0);title='墙两侧状态'
  else:
   vectors=[];starts=[]
   for side,sign in enumerate([1,-1]):
    take=(supported[:,side]if j==1 else(view_counts[:,side]>=3)&valid)
    # Fixed spatial downsample only affects display arrows, never evidence.
    ids=np.flatnonzero(take);ids=ids[::2];starts.append(centers[ids,:2]);vectors.append(sign*normals[ids]*.35)
   if any(len(x)for x in starts):
    pp=np.concatenate(starts);vv=np.concatenate(vectors);ax.quiver(-pp[:,0],pp[:,1],-vv[:,0],vv[:,1],angles='xy',scale_units='xy',scale=1,width=.002,color='#009a8c'if j==1 else'#2875c4')
   title='绿箭头：FREE＋表面支持侧'if j==1 else'蓝箭头：实际相机观察侧（≥3帧）'
  ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f+1}楼｜{title}',facecolor='white')
fig.suptitle('墙体侧向证据：紫=两侧有支持；橙=单侧有支持；灰=证据不足；红=法向不稳定',fontsize=14);fig.tight_layout(rect=[0,.055,1,.96]);fig.text(.5,.02,'仅用当前700秒实测点云、最终FREE和精确匹配深度端点；未用GT。\n支持侧是室内候选，未知侧不等于室外；未修改墙体、房间分区或导航地图，不能直接证明门洞或完整房间。',ha='center',fontsize=10);fig.savefig(O/'three_floors_wall_side_evidence.png',dpi=170)
(O/'audit.json').write_text(json.dumps({'status':'wall-side evidence prototype; no certified interior/exterior classifier','uses_GT':False,'uses_coordinate_or_region_rules':False,'normal_convention':'unoriented dyad average; canonical +/- pair, low coherence retained as unknown','parameters':{'patch_xy_m':.2,'normal_coherence_min':.65,'free_probes_offsets_m':[.25,.45,.65],'free_probes_height_above_floor_m':[.45,.85,1.25,1.65],'free_match_m':.09,'surface_projection_match_m':.12,'depth_endpoint_wall_match_m':.12,'free_fraction_min':.25,'surface_fraction_min':1/3,'free_height_layers_min':2},'controlled_checks':{'normal_sign_invariance':True,'view_only_does_not_certify_side':True,'joint_support_positive_case':True},'limitations':['final map is not historical online state','nonvertical supports can include furniture and exterior surfaces','quantized geometry and sampled depth may miss observations','one-sided observation cannot distinguish exterior wall from partially explored interior wall','visibility, FREE and surface features are correlated, not independent probabilistic evidence'],'results':summary},ensure_ascii=False,indent=2))
