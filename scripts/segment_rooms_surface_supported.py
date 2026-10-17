"""Generic offline room proposals from measured surface support, no XY special cases."""
from pathlib import Path
import json
import numpy as np
import cv2
from scipy import ndimage as ndi
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];BASE=R/'outputs/coverage700_wall_room_preview';OUT=BASE/'surface_supported';OUT.mkdir(exist_ok=True)
geo=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz');gt=np.load(R/'outputs/consistency_validation_20261015/latest_official_comparison/official_sampled_surfaces.npz')['points'];grounds=[-.155,2.595,5.595]
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,4,figsize=(19,15));rows=[]
for f,g in enumerate(grounds):
 for j,source in enumerate(['measured','official']):
  a=np.load(BASE/source/f'L{f+1}_small_room_preserved.npz');old=a['labels'];wall=a['wall_grid'];org=a['origin_xy'];res=float(a['resolution']);shape=old.shape
  if source=='measured':p,n=geo[f'p{f}'],geo[f'n{f}']
  else:
   cache=OUT/f'official_L{f+1}_normals.npz'
   if cache.exists():z=np.load(cache);p,n=z['points'],z['normals']
   else:
    p=gt[(gt[:,2]>=g-.12)&(gt[:,2]<g+2.75)];_,ii=np.unique(np.floor(p/.04).astype('int32'),axis=0,return_index=True);p=p[np.sort(ii)];tree=cKDTree(p);n=np.empty_like(p)
    for lo in range(0,len(p),5000):
     _,ix=tree.query(p[lo:lo+5000],k=20,workers=2);q=p[ix];q=q-q.mean(1,keepdims=True);_,v=np.linalg.eigh(np.einsum('nki,nkj->nij',q,q));n[lo:lo+5000]=v[:,:,0]
    np.savez_compressed(cache,points=p,normals=n)
  def raster(q):
   m=np.zeros(shape,np.uint8);ij=np.floor((q[:,:2]-org)/res).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<shape[1])&(ij[:,1]>=0)&(ij[:,1]<shape[0]);ij=ij[ok];m[ij[:,1],ij[:,0]]=1;return m
  # Any non-vertical surface in the floor's full slab can support spatial extent:
  # floors, stair treads, furniture tops and sloped ceilings; never declares navigable floor.
  support=(abs(n[:,2])>=.35)&(p[:,2]>=g-.15)&(p[:,2]<=g+2.85)
  evidence=raster(p[support]);domain=cv2.morphologyEx(evidence,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9)))>0
  # Fill only small enclosed sampling gaps, not arbitrarily large unseen interiors.
  enclosed=ndi.binary_fill_holes(domain)&~domain;cc,num=ndi.label(enclosed)
  for k in range(1,num+1):
   m=cc==k
   if m.sum()*res**2<=1.:domain[m]=True
  # Preserve existing supported regions while adding slope/full-height evidence.
  domain |= a['observed_floor_domain']
  bound=np.zeros(shape,np.uint8);y,x=np.where(wall>0);cv2.fillConvexPoly(bound,cv2.convexHull(np.column_stack([x,y]).astype('int32')),1);domain &= bound>0
  barrier=cv2.dilate(wall,np.ones((3,3),np.uint8))>0;free=domain&~barrier;dist=ndi.distance_transform_edt(free)*res
  # Preserve all existing region cores so broader support cannot silently merge them.
  seeds=np.zeros(shape,np.int32);count=0
  for rid in np.unique(old):
   if not rid:continue
   mask=(old==rid)&free;inner=ndi.binary_erosion(mask,iterations=2)
   if not inner.any():inner=mask
   if inner.any():count+=1;seeds[inner]=count
  core,ncore=ndi.label((dist>.45)&(seeds==0));nearold=ndi.distance_transform_edt(seeds==0)*res if count else np.full(shape,np.inf)
  for k in range(1,ncore+1):
   m=(core==k)&(nearold>.45)
   if m.sum()*res**2>=.04:count+=1;seeds[m]=count
  components,ncomp=ndi.label(free)
  for k in range(1,ncomp+1):
   m=components==k
   if not seeds[m].any() and m.sum()*res**2>=.3:
    count+=1;seeds[m&(dist>=dist[m].max()-.05)]=count
  labels=ndi.watershed_ift(np.uint8(255*(1-np.clip(dist/2,0,1))),seeds);labels[~free]=0
  for k in range(1,ncomp+1):
   m=components==k
   if not seeds[m].any():labels[m]=0
  assert not np.any((labels>0)&barrier)
  # Production result retains wall masks; older room labels each retain an exclusive seed.
  dest=OUT/source;dest.mkdir(exist_ok=True);np.savez_compressed(dest/f'L{f+1}_rooms.npz',labels=labels,domain=domain,surface_evidence=evidence,wall_grid=wall,origin_xy=org,resolution=res)
  yy,xx=np.indices(shape);pts=np.column_stack([org[0]+(xx.ravel()+.5)*res,org[1]+(yy.ravel()+.5)*res]);near=cKDTree(p[:,:2]).query(pts,workers=2)[0].reshape(shape)<=.12
  recover=(~a['observed_floor_domain'])&domain&~barrier
  row={'floor':f+1,'source':source,'old_regions':len(np.unique(old[old>0])),'new_regions':len(np.unique(labels[labels>0])),'added_supported_nonwall_area_m2':round(recover.sum()*res**2,3),'point_backed_added_area_m2':round((recover&near).sum()*res**2,3),'preserved_old_positive_cells_fraction':round(float((labels[old>0]>0).mean()),6),'wall_overlap_cells':int(((labels>0)&barrier).sum())};rows.append(row);print(row,flush=True)
  for mode,l in enumerate([old,labels]):
   ax=axs[f,2*j+mode];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.65;ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);q=a['wall_points'];ax.scatter(-q[:,0],q[:,1],s=.2,c='#a92337',lw=0,rasterized=True);ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f+1}楼 '+('实测'if j==0 else'官方几何')+(' 原版'if mode==0 else' 自动多表面支持'))
fig.suptitle('通用离线分割：完整楼层高度内的斜面/水平面支持，无位置专用规则',fontsize=15);fig.tight_layout(rect=[0,.03,1,.96]);fig.text(.5,.012,'仅为空间分区候选；彩色支持范围不等于水平楼板或导航FREE。沿用原墙体候选，漏墙和室外轮廓误差仍可能存在。',ha='center',fontsize=9);fig.savefig(OUT/'three_floors_before_after.png',dpi=160)
(OUT/'audit.json').write_text(json.dumps({'uses_xy_roi_rules':False,'uses_truth_for_measured':False,'algorithm':'full-slab nonvertical surface support + bounded small-gap fill + preserved room seeds watershed','parameters':{'min_abs_normal_z':.35,'closing_diameter_m':.45,'max_enclosed_gap_fill_m2':1.,'floor_slab_relative_z_m':[-.15,2.85]},'limitations':['wall extractor unchanged','convex-hull bound may include exterior surfaces','floor slabs use existing floor estimates; no automatic floor discovery added','old labels provide seeds, not independent semantic truth','stair logical identity remains separate from room geometry'],'rows':rows},ensure_ascii=False,indent=2))
