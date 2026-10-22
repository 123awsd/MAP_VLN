"""Measured paired-face wall thickness prototype, separate from side-clipping trial.
Fill only locally parallel, height-overlapping faces viewed from outward sides;
never overwrite multi-height FREE, witnessed trajectory, or GT-derived regions.
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
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'solid_wall_trial';O.mkdir(exist_ok=True);run=R/'outputs/coverage_lifecycle_700_r1_20261016';G=np.load(run/'original_style_video/geometry.npz');mp=np.load(run/'observed_grid_final.npz');ftree=cKDTree(mp['free']);trajectory=np.load(run/'video_cache/trajectory.npy')
src=R/'scripts/segment_rooms_wall_guided.py';fn=next(n for n in ast.parse(src.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='watershed');ns={'np':np,'heapq':heapq};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(src),'exec'),ns)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
results=[];data=[]
def pair_geometry(a,b,n1,n2):
 delta=b-a;normaldistance=abs(delta@n1);along=abs(delta@np.array([-n1[1],n1[0]]));return .08<=normaldistance<=.6 and along<=.12 and abs(n1@n2)>=.95
assert pair_geometry(np.array([0.,0.]),np.array([.2,0.]),np.array([1.,0.]),np.array([1.,0.]))
assert not pair_geometry(np.array([0.,0.]),np.array([1.,0.]),np.array([1.,0.]),np.array([1.,0.]))
assert not pair_geometry(np.array([0.,0.]),np.array([.2,0.]),np.array([1.,0.]),np.array([0.,1.]))
for f,g in enumerate([-.155,2.595,5.595],1):
 a=np.load(B/'wall_guided/measured'/f'L{f}_rooms.npz');old=np.load(B/'wall_guided/measured_review'/f'L{f}_rooms.npz')['labels'];wp=np.load(B/'measured'/f'L{f}_small_room_preserved.npz')['wall_points'];s=np.load(B/'wall_side_evidence'/f'L{f}_wall_sides.npz');c=s['centers_xyz'][:,:2];n=s['canonical_normal_xy'];coh=s['normal_coherence'];views=s['depth_endpoint_view_frames'];inv=s['wall_point_patch_id'];org=a['origin_xy'];res=float(a['resolution']);shape=old.shape
 bands=np.zeros(len(c),np.uint32);zb=np.floor((wp[:,2]-g)/.2).astype(int);valid=(zb>=0)&(zb<24);np.bitwise_or.at(bands,inv[valid],np.left_shift(np.uint32(1),zb[valid].astype('uint32')))
 candidate=np.zeros(shape,np.uint8);accepted=np.zeros(shape,np.uint8);pairs=[];rejects={'geometry':0,'height_overlap':0,'view_side':0,'free_or_trajectory':0};y,x=np.indices(shape);xy=org+np.column_stack([x.ravel()+.5,y.ravel()+.5])*res
 probe=np.zeros((len(xy),4,3));probe[:,:,:2]=xy[:,None,:];probe[:,:,2]=g+np.array([.45,.85,1.25,1.65])[None,:];freeprotect=((ftree.query(probe.reshape(-1,3),workers=2)[0].reshape(-1,4)<=.09).sum(1)>=2).reshape(shape)
 tr=trajectory[(trajectory[:,3]>=g+.2)&(trajectory[:,3]<g+2.2),1:3];routeprotect=(cKDTree(tr).query(xy,workers=2)[0]<=.15).reshape(shape)if len(tr)else np.zeros(shape,bool)
 pairs_idx=cKDTree(c).query_pairs(.65)
 for i,j in sorted(pairs_idx):
  if coh[i]<.8 or coh[j]<.8 or not pair_geometry(c[i],c[j],n[i],n[j]):rejects['geometry']+=1;continue
  common=int(bands[i]&bands[j]);heightbins=common.bit_count()if hasattr(int,'bit_count')else bin(common).count('1')
  if heightbins<4:rejects['height_overlap']+=1;continue
  normal=n[i].copy();normal*=1 if (c[j]-c[i])@normal>=0 else -1;tangent=np.array([-normal[1],normal[0]]);mid=(c[i]+c[j])/2;lo=min(c[i]@normal,c[j]@normal);hi=max(c[i]@normal,c[j]@normal);mid_t=mid@tangent
  poly=np.array([normal*lo+tangent*(mid_t-.11),normal*hi+tangent*(mid_t-.11),normal*hi+tangent*(mid_t+.11),normal*lo+tangent*(mid_t+.11)]);pix=np.rint((poly-org)/res).astype('int32');mask=np.zeros(shape,np.uint8);cv2.fillConvexPoly(mask,pix,1);m=mask>0;candidate|=mask
  # Camera observations must lie outside the paired faces, not in their gap.
  inward_i=0 if (c[j]-c[i])@n[i]>=0 else 1;inward_j=0 if (c[i]-c[j])@n[j]>=0 else 1;out_i=views[i,1-inward_i];out_j=views[j,1-inward_j]
  viewok=out_i>=2 and out_j>=2 and views[i,inward_i]<=max(1,.25*out_i) and views[j,inward_j]<=max(1,.25*out_j)
  if not viewok:rejects['view_side']+=1;continue
  # Refuse whole local pair if observed room-space or camera route lies in its interior.
  inner=m&~a['wall_barrier']
  if np.any(inner&(freeprotect|routeprotect)):rejects['free_or_trajectory']+=1;continue
  accepted|=mask;pairs.append({'patch_ids':[int(i),int(j)],'thickness_m':round(float(hi-lo),4),'shared_vertical_bins':heightbins,'outward_view_frames':[int(out_i),int(out_j)]})
 added=(accepted>0)&~a['wall_barrier'];assert not np.any(added&(freeprotect|routeprotect));bar=a['wall_barrier']|(accepted>0);domain=a['domain']&~bar;dist=nd.distance_transform_edt(domain)*res;cc,num=nd.label(dist>.45);seed=np.zeros(shape,np.int32);count=0;fallback=np.zeros(shape,bool)
 for k in range(1,num+1):
  m=cc==k
  if m.sum()*res**2>=.04:count+=1;seed[m]=count
 cc,num=nd.label(domain)
 for k in range(1,num+1):
  m=cc==k
  if not seed[m].any() and m.sum()*res**2>=.3:count+=1;q=m&(dist>=dist[m].max()-.05);seed[q]=count;fallback|=q
 labels=ns['watershed'](dist,seed,domain)
 for _ in range(3):
  changed=False
  for rid in np.unique(labels):
   if not rid:continue
   m=labels==rid;ring=nd.binary_dilation(m)&~m;near=np.unique(labels[ring]);near=near[(near>0)&(near!=rid)];frac=bar[ring].mean()if ring.any()else 0;opened=bool(np.any(ring&~domain&~bar))
   if len(near)==1 and frac<.2 and not opened:labels[m]=near[0];changed=True
  if not changed:break
 closed=cv2.morphologyEx(a['wall_grid'],cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;inside=nd.binary_fill_holes(closed)&~closed;final=np.zeros(shape,np.int32);used=set();nextid=int(old.max())+1
 for rid in np.unique(labels):
  if not rid:continue
  m=labels==rid
  if m.sum()*res**2<1 and nd.distance_transform_edt(m).max()*res<.3 and inside[m].mean()<.1:continue
  votes=np.bincount(old[m]);votes[0]=0;best=int(votes.argmax())if votes.sum()else 0
  if best and best not in used:did=best
  else:did=nextid;nextid+=1
  used.add(did);final[m]=did
 assert not np.any((final>0)&bar)
 regionstats=[{'id':f'{f}-{int(r):02d}','original_area_m2':round((old==r).sum()*res**2,3),'new_solid_wall_overlap_m2':round(((old==r)&added).sum()*res**2,3)}for r in np.unique(old)if r]
 rec={'floor':f,'geometric_pair_proposals':len(pairs_idx)-rejects['geometry']-rejects['height_overlap'],'accepted_local_pairs':len(pairs),'rejections':rejects,'candidate_strip_area_m2':round((candidate>0).sum()*res**2,3),'new_wall_area_m2':round(added.sum()*res**2,3),'regions_before':len(np.unique(old[old>0])),'regions_after':len(np.unique(final[final>0])),'pair_records':pairs,'per_original_region':regionstats,'FREE_and_route_protection_passed':True};results.append(rec);print({k:v for k,v in rec.items()if k not in ['pair_records','per_original_region']},flush=True)
 np.savez_compressed(O/f'L{f}_solid_wall.npz',wall_face=a['wall_grid'],original_barrier=a['wall_barrier'],candidate_thickness=candidate>0,accepted_solid=accepted>0,added_solid=added,domain=domain,seeds=seed,labels=final,original_labels=old,free_protection=freeprotect,route_protection=routeprotect,origin_xy=org,resolution=res)
 data.append({'f':f,'g':g,'a':a,'old':old,'new':final,'candidate':candidate>0,'added':added,'domain':domain,'seed':seed,'org':org,'res':res})
def im(ax,rgb):ax.imshow(rgb[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
def panel(ax,d,step):
 rgb=np.ones((*d['old'].shape,3));a=d['a']
 if step==0:
  p=G[f'p{d["f"]-1}'];p=p[(p[:,2]>d['g']+.15)&(p[:,2]<d['g']+2.2)];ax.scatter(-p[:,0],p[:,1],s=.25,c='#9bb0bd',lw=0)
 elif step==1:rgb[a['domain']]=[.88,.9,.92];rgb[d['candidate']]=[1,.65,.15];im(ax,rgb)
 elif step==2:rgb[a['domain']]=[.88,.9,.92];rgb[a['wall_barrier']]=[.65,.18,.25];rgb[d['added']]=[.28,.16,.77];im(ax,rgb)
 elif step==3:rgb[d['domain']]=[.65,.84,.77];im(ax,rgb)
 elif step==4:rgb[d['domain']]=[.88,.92,.93];rgb[d['seed']>0]=[.9,.13,.36];im(ax,rgb)
 else:
  l=d['new'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;im(ax,rgba)
  for rid in np.unique(l):
   if rid:
    dt=nd.distance_transform_edt(l==rid);y,x=np.unravel_index(dt.argmax(),dt.shape);t=ax.text(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],f'{d["f"]}-{rid:02d}',fontsize=8,ha='center',clip_on=True);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 y,x=np.where(a['wall_grid']);ax.scatter(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],s=.3,c='#a51b32',lw=0);ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=['① 实测点云＋墙面','② 橙色：平行双墙面厚度候选','③ 紫色：证据通过的新增墙实体','④ 排除墙实体后的分割范围','⑤ 重新生成种子','⑥ 最终几何分区'][step])
for d in data:
 fig,axs=plt.subplots(2,3,figsize=(16,13))
 for step,ax in enumerate(axs.flat):panel(ax,d,step)
 fig.suptitle(f'{d["f"]}楼：双墙面厚度约束分阶段试验',fontsize=17);fig.tight_layout(rect=[0,.045,1,.955]);fig.text(.5,.018,'同层平行面配对＋共同高度支持＋从两面外侧实际观察，保护多高度FREE和真实轨迹。\n橙色仅为候选，不会全部填实；未使用GT或特定房间规则，未叠加上一版墙侧裁剪。',ha='center',fontsize=9);fig.savefig(O/f'L{d["f"]}_six_stages.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(13,18))
for d in data:
 for j,key in enumerate(['old','new']):
  dd={**d,'new':d[key]};panel(axs[d['f']-1,j],dd,5);axs[d['f']-1,j].set_title(f'{d["f"]}楼｜'+('原墙面分割'if j==0 else'墙实体约束试验'))
fig.suptitle('墙厚处理前后：两面之间有证据的部分不分给房间',fontsize=16);fig.tight_layout(rect=[0,.035,1,.96]);fig.text(.5,.012,'保留原版与所有物理地图；这不是同侧面之间全局填充，也不把未知空间直接改成占据。',ha='center',fontsize=9);fig.savefig(O/'three_floors_before_after.png',dpi=170)
(O/'audit.json').write_text(json.dumps({'status':'experimental thickness mask for offline partition only','base':'original wall_guided, NOT wall_side_constrained','uses_GT_or_coordinate_exceptions':False,'controlled_geometry_checks_passed':3,'parameters':{'normal_distance_m':[.08,.6],'max_tangential_offset_m':.12,'min_normal_parallel_cos':.95,'min_normal_coherence':.8,'min_common_vertical_bins':4,'height_bin_m':.2,'outward_view_frames_min':2,'inward_to_outward_view_limit':.25},'limitations':['candidate paired faces are not a globally optimized solid wall model','thickness above.6m remains unknown, no assumption it is a wall','sampled camera observations can miss opposing-face evidence','wall envelope corners and door endpoints need further validation'],'results':results},ensure_ascii=False,indent=2))
