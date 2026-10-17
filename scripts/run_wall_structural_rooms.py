"""Offline independent room-space evidence trial.
Wall surfaces do not substantiate room air; unseeded observed space stays untyped.
Uses identical measured-run cached rays, never GT. Original outputs are preserved.
"""
from pathlib import Path
import ast,heapq,json
import numpy as np,cv2
from scipy import ndimage as nd
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_structural_rooms';O.mkdir(exist_ok=True);run=R/'outputs/coverage_lifecycle_700_r1_20261016';G=np.load(run/'original_style_video/geometry.npz');free_tree=cKDTree(np.load(run/'observed_grid_final.npz')['free']);traj=np.load(run/'video_cache/trajectory.npy')
src=R/'scripts/segment_rooms_wall_guided.py';node=next(n for n in ast.parse(src.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='watershed');ns={'np':np,'heapq':heapq};exec(compile(ast.Module(body=[node],type_ignores=[]),str(src),'exec'),ns)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
rays=np.load(B/'evidence_test/measured_free_rays.npz')
records=[];datasets=[]
def unsupported_wall_band(domain,band,surface,free,route,rays):
 return domain&band&~(surface|free|route|rays)
# Same gate used below: each evidence source independently prevents exclusion.
_d=np.ones(5,bool);_b=np.ones(5,bool);_eye=np.eye(5,dtype=bool)
assert np.array_equal(unsupported_wall_band(_d,_b,_eye[1],_eye[2],_eye[3],_eye[4]),[True,False,False,False,False])
assert not unsupported_wall_band(np.ones(1,bool),np.zeros(1,bool),*[np.zeros(1,bool)]*4)[0]
for f,g in enumerate([-.155,2.595,5.595],1):
 a=np.load(B/'wall_guided/measured'/f'L{f}_rooms.npz');old=np.load(B/'independent_room_support'/f'L{f}_constrained.npz')['labels'];side=np.load(B/'wall_side_evidence'/f'L{f}_wall_sides.npz');org=a['origin_xy'];res=float(a['resolution']);domain=a['domain'];bar=a['wall_barrier'];wall=a['wall_grid'];shape=domain.shape;y,x=np.indices(shape);xy=org+np.column_stack([x.ravel()+.5,y.ravel()+.5])*res
 # Replace wall input with the new extracted measured points; keep the same downstream method.
 nw=np.load(B/'wall_structural_evidence'/f'L{f}_walls.npz')['wall_points']
 ij=np.floor((nw[:,:2]-org)/res).astype(int);valid=(ij[:,0]>=0)&(ij[:,0]<shape[1])&(ij[:,1]>=0)&(ij[:,1]<shape[0]);ij=ij[valid]
 wall=np.zeros(shape,np.uint8);wall[ij[:,1],ij[:,0]]=1
 oldbar=bar.copy();bar=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0
 domain=(domain|oldbar)&~bar
 centers=side['centers_xyz'][:,:2];normals=side['canonical_normal_xy'];support=side['supported_side']
 # Horizontal floor/ceiling evidence only. Vertical walls cannot support room air.
 p=G[f'p{f-1}'];norm=G[f'n{f-1}']
 mask=(p[:,2]>=g-.15)&(p[:,2]<g+2.85)&(np.abs(norm[:,2])>=.9)
 p=p[mask]
 surface_dist=cKDTree(p[:,:2]).query(xy,workers=2)[0].reshape(shape)
 protected_surface=surface_dist<=.12
 # Independently protect observed FREE at several heights, not merely point density.
 probes=np.zeros((len(xy),4,3));probes[:,:,:2]=xy[:,None,:];probes[:,:,2]=g+np.array([.45,.85,1.25,1.65])[None,:]
 free_layers=(free_tree.query(probes.reshape(-1,3),workers=2)[0].reshape(-1,4)<=.09).sum(1);protected_free=(free_layers>=2).reshape(shape)
 t=traj[(traj[:,3]>=g+.2)&(traj[:,3]<g+2.2),1:3];protected_route=(cKDTree(t).query(xy,workers=2)[0]<=.15).reshape(shape)if len(t)else np.zeros(shape,bool)
 # Reuse measured-ray raster from the identical input run; no new simulation.
 previous=np.load(B/'wall_band_support'/f'L{f}_constrained.npz')
 ray_votes=previous['ray_frame_votes'];protected_rays=ray_votes>=3
 wall_dist=previous['wall_distance_m']
 direct=protected_surface|protected_free|protected_route|protected_rays
 # No distance-to-wall exemption and no filled-hole evidence.
 exclude=domain&~direct
 positive=direct.ravel().astype(int);negative=(domain&~direct).ravel().astype(int)
 assert not np.any(exclude&direct)
 newdomain=domain&~exclude
 dist=nd.distance_transform_edt(newdomain)*res;cc,n=nd.label(dist>.45);seed=np.zeros(shape,np.int32);fallback=np.zeros(shape,bool);count=0
 for k in range(1,n+1):
  m=cc==k
  if m.sum()*res**2>=.04:count+=1;seed[m]=count
 # Unseeded observed components remain untyped; never auto-promote a narrow strip.
 labels=ns['watershed'](dist,seed,newdomain);merges=[]
 for _ in range(3):
  changed=False
  for rid in np.unique(labels):
   if not rid:continue
   m=labels==rid;ring=nd.binary_dilation(m)&~m;neighbors=np.unique(labels[ring]);neighbors=neighbors[(neighbors>0)&(neighbors!=rid)];wallfrac=float(bar[ring].mean())if ring.any()else 0;opened=bool(np.any(ring&~newdomain&~bar))
   if len(neighbors)==1 and wallfrac<.2 and not opened:target=int(neighbors[0]);labels[m]=target;merges.append({'from':int(rid),'to':target});changed=True
  if not changed:break
 final=np.zeros(shape,np.int32)
 for k,rid in enumerate(np.unique(labels[labels>0]),1):final[labels==rid]=k
 # Existing display-only fragment rule applied identically to candidate result.
 closed=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;inside=nd.binary_fill_holes(closed)&~closed;hidden=[]
 for rid in np.unique(final):
  if not rid:continue
  m=final==rid
  if m.sum()*res**2<1. and nd.distance_transform_edt(m).max()*res<.3 and inside[m].mean()<.1:hidden.append(int(rid));final[m]=0
 audit=[];display=np.zeros(shape,np.int32);idmap={};nextid=int(old.max())+1
 for rid in np.unique(final):
  if not rid:continue
  m=final==rid;votes=np.bincount(old[m]);votes[0]=0;best=int(votes.argmax())if votes.sum()else 0
  if best and best not in idmap.values():did=best
  else:did=nextid;nextid+=1
  idmap[int(rid)]=did;display[m]=did
 for rid in np.unique(old):
  if rid:
   m=old==rid;audit.append({'original_id':f'{f}-{int(rid):02d}','original_area_m2':round(m.sum()*res**2,3),'side_excluded_area_m2':round((m&exclude).sum()*res**2,3),'remaining_display_fraction':round(float((display[m]>0).mean()),4)})
 assert not np.any(exclude&protected_free);assert not np.any(exclude&protected_route);assert not np.any((display>0)&bar)
 np.savez_compressed(O/f'L{f}_constrained.npz',original_labels=old,labels=display,original_domain=domain,constrained_domain=newdomain,side_excluded_candidates=exclude,positive_votes=positive.reshape(shape),negative_votes=negative.reshape(shape),free_protected=protected_free,trajectory_protected=protected_route,surface_protected=protected_surface,ray_protected=protected_rays,ray_frame_votes=ray_votes,surface_distance_m=surface_dist,wall_distance_m=wall_dist,seeds=seed,fallback_seed=fallback,measured_wall=wall,wall_barrier=bar,origin_xy=org,resolution=res)
 record={'floor':f,'excluded_area_m2':round(exclude.sum()*res**2,3),'before_regions':len(np.unique(old[old>0])),'after_regions':len(np.unique(display[display>0])),'per_original_region':audit,'all_protections_passed':True,'horizontal_surface_protection_passed':not bool(np.any(exclude&protected_surface)),'sampled_ray_protection_passed':not bool(np.any(exclude&protected_rays))};records.append(record);print({k:v for k,v in record.items()if k!='per_original_region'},flush=True)
 datasets.append({'f':f,'g':g,'old':old,'final':display,'wall':wall,'domain':domain,'newdomain':newdomain,'exclude':exclude,'positive':positive.reshape(shape),'negative':negative.reshape(shape),'seed':seed,'fallback':fallback,'org':org,'res':res,'centers':centers,'normals':normals,'support':support})
def im(ax,rgb):ax.imshow(rgb[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
def draw(ax,d,step):
 shape=d['old'].shape;rgb=np.ones((*shape,3))
 if step==0:
  p=G[f'p{d["f"]-1}'];p=p[(p[:,2]>d['g']+.15)&(p[:,2]<d['g']+2.2)];ax.scatter(-p[:,0],p[:,1],s=.25,c='#7898ac',lw=0)
  for side,sign in enumerate([1,-1]):
   ix=np.flatnonzero(d['support'][:,side])[::2];p=d['centers'][ix];v=sign*d['normals'][ix]*.3;ax.quiver(-p[:,0],p[:,1],-v[:,0],v[:,1],angles='xy',scale_units='xy',scale=1,width=.002,color='#008f83')
 elif step==1:rgb[d['domain']]=[.73,.79,.84];im(ax,rgb)
 elif step==2:rgb[d['domain']]=[.75,.85,.81];rgb[d['exclude']]=[1,.55,.15];im(ax,rgb)
 elif step==3:rgb[d['newdomain']]=[.67,.84,.76];im(ax,rgb)
 elif step==4:rgb[d['newdomain']]=[.88,.92,.92];rgb[d['seed']>0]=[.9,.15,.38];rgb[d['fallback']]=[1,.6,.1];im(ax,rgb)
 else:
  rgb[d['newdomain']]=[.83,.83,.83];im(ax,rgb);l=d['final'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;im(ax,rgba)
  for rid in np.unique(l):
   if rid:
    ds=nd.distance_transform_edt(l==rid);y,x=np.unravel_index(ds.argmax(),ds.shape);text=ax.text(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],f'{d["f"]}-{rid:02d}',ha='center',fontsize=8,clip_on=True);text.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 y,x=np.where(d['wall']);ax.scatter(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],s=.25,c='#a51b32',lw=0);ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=['① 实测点云与墙面','② 原可分割范围','③ 橙色：缺少独立空间证据','④ 有独立证据的范围','⑤ 核心种子（无兜底种子）','⑥ 分区；灰色为未定型观测空间'][step])
for d in datasets:
 fig,axs=plt.subplots(2,3,figsize=(16,13))
 for step,ax in enumerate(axs.flat):draw(ax,d,step)
 fig.suptitle(f'{d["f"]}楼：独立空间证据试验｜墙面不再自证房间',fontsize=16);fig.tight_layout(rect=[0,.045,1,.95]);fig.text(.5,.018,'橙色仅从房间候选范围排除，不代表室外/占据。墙侧只缺表面点、但有FREE/多帧射线/轨迹的空气区域仍保留。\n只用近水平表面、多高度FREE、多帧深度射线和实际轨迹；不使用GT、不按房间编号裁剪，原分区与导航数据保持不变。',ha='center',fontsize=9);fig.savefig(O/f'L{d["f"]}_six_stages.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(13,18))
for d in datasets:
 for j,key in enumerate(['old','final']):
  ax=axs[d['f']-1,j];l=d[key];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;im(ax,rgba);y,x=np.where(d['wall']);ax.scatter(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],s=.3,c='#a51b32',lw=0)
  for rid in np.unique(l):
   if rid:
    ds=nd.distance_transform_edt(l==rid);y,x=np.unravel_index(ds.argmax(),ds.shape);text=ax.text(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],f'{d["f"]}-{rid:02d}',ha='center',fontsize=8);text.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
  ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{d["f"]}楼｜'+('原版'if not j else'新墙输入＋同一分割流程'))
fig.suptitle('新墙提取接入房间分割前后：统一三层，不按特定区域裁剪',fontsize=16);fig.tight_layout(rect=[0,.04,1,.96]);fig.text(.5,.015,'编号以最大重叠尽量关联原分区；可能出现拆分，不等于语义房间身份。原结果保留，候选尚未晋升。',ha='center',fontsize=9);fig.savefig(O/'three_floors_before_after.png',dpi=170)
(O/'audit.json').write_text(json.dumps({'status':'offline experiment, not promoted','uses_GT':False,'uses_label_or_xy_exceptions':False,'original_data_unchanged':True,'wall_source':'wall_structural_evidence','downstream':'same independent support segmentation, no ownership or contour postfilter','rule':'horizontal surface or multi-height FREE or measured rays or trajectory; no wall-distance exemption; no fallback seeds','limitations':['no observation is not proof of exterior','normal .9, noise tolerance .12 and original .45m core remain parameters','unseeded observed space is retained as untyped, not a certified room','final-state evidence only; sparse evidence can fragment genuine rooms'],'results':records},ensure_ascii=False,indent=2))
