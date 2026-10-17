"""Offline, identical wall extraction and geometric room proposals on measured/GT clouds."""
from pathlib import Path
import json
import numpy as np
import cv2
from scipy.spatial import cKDTree
from scipy import ndimage as ndi
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from refine_measured_walls import extract
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/coverage700_wall_room_preview';OUT.mkdir(parents=True,exist_ok=True)
geo=np.load(ROOT/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')
truth=np.load(ROOT/'outputs/consistency_validation_20261015/latest_official_comparison/official_sampled_surfaces.npz')['points']
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
origin=np.array([-8.5,-2.5]);res=.05;shape=(420,380)
def raster(p):
 a=np.zeros(shape,np.uint8);xy=np.floor((p[:,:2]-origin)/res).astype(int);ok=(xy[:,0]>=0)&(xy[:,0]<shape[1])&(xy[:,1]>=0)&(xy[:,1]<shape[0]);xy=xy[ok];a[xy[:,1],xy[:,0]]=1;return a
cfg=dict(envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.6,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04,local_top_attachment=.4,flat_ceiling_only=True)
records=[];fig,axes=plt.subplots(3,4,figsize=(20,16))
for f,g in enumerate([-.155,2.595,5.595]):
 for j,source in enumerate(['measured','official']):
  if j==0:p,n=geo[f'p{f}'],geo[f'n{f}']
  else:
   p=truth[(truth[:,2]>=g-.12)&(truth[:,2]<g+2.75)]
   _,i=np.unique(np.floor(p/.04).astype('int32'),axis=0,return_index=True);p=p[np.sort(i)]
   tree=cKDTree(p);n=np.empty_like(p)
   for lo in range(0,len(p),5000):
    _,ix=tree.query(p[lo:lo+5000],k=20,workers=2);q=p[ix];q=q-q.mean(1,keepdims=True);_,v=np.linalg.eigh(np.einsum('nki,nkj->nij',q,q));n[lo:lo+5000]=v[:,:,0]
  print('extract',f+1,source,len(p),flush=True)
  floor_cfg={**cfg,'flat_ceiling_only':f<2}
  walls,roof,patches,roofmeta=extract(p,n,g,floor_cfg)
  # Domain uses only each input's horizontal floor/ceiling supports; GT is never used to fill measured data.
  floor=p[((abs(p[:,2]-g)<.12)|((p[:,2]>g+1.8)&(p[:,2]<g+2.75)))&(abs(n[:,2])>.9)]
  domain=cv2.morphologyEx(raster(floor),cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9)))>0
  domain=ndi.binary_fill_holes(domain)
  wall=raster(walls);barrier=cv2.dilate(wall,np.ones((3,3),np.uint8))>0
  enclosure=np.zeros(shape,np.uint8)
  yx=np.column_stack(np.where(wall>0))[:,::-1].astype('int32')
  cv2.fillConvexPoly(enclosure,cv2.convexHull(yx),1)
  domain &= enclosure>0
  free=domain&~barrier
  distance=ndi.distance_transform_edt(free)*res
  cores,ncores=ndi.label(distance>.45)
  seed=np.zeros(shape,np.int32);count=0
  for k in range(1,ncores+1):
   if (cores==k).sum()*res*res>=1.0:count+=1;seed[cores==k]=count
  # Geodesic watershed expands doorway-separated cores within each evidenced free component.
  elevation=np.uint8(255*(1-np.clip(distance/2,0,1)))
  labels=ndi.watershed_ift(elevation,seed);labels[~free]=0
  # Components without their own seed are not silently assigned to a remote room.
  components,num=ndi.label(free)
  for k in range(1,num+1):
   m=components==k
   if not np.any(seed[m]):labels[m]=0
  areas=[{'id':k,'area_m2':round(float((labels==k).sum()*res*res),3)} for k in range(1,count+1)]
  dest=OUT/source;dest.mkdir(exist_ok=True)
  np.savez_compressed(dest/f'L{f+1}_walls_rooms.npz',wall_points=walls,ceiling_points=roof,labels=labels,observed_floor_domain=domain,wall_grid=wall,origin_xy=origin,resolution=res)
  row={'floor':f+1,'source':source,'input_points':len(p),'wall_points':len(walls),'wall_patches':len(patches),'room_proposals':count,'rooms':areas,'floor_supported_domain_m2':round(domain.sum()*res*res,3)};records.append(row);print(row,flush=True)
  for mode in range(2):
   ax=axes[f,j*2+mode];q=p[(p[:,2]>g+.15)&(p[:,2]<g+2.15)]
   ax.scatter(-q[:,0],q[:,1],s=.15,c='#c1c5c9',lw=0,rasterized=True)
   if mode:
    rgba=plt.get_cmap('tab20')((labels%20)/19);rgba[:,:,3]=(labels>0)*.62
    # Array columns are raw +X; reverse for display -X only.
    ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
    for room in areas:
     y,x=ndi.center_of_mass(labels==room['id']);ax.text(-(origin[0]+(x+.5)*res),origin[1]+(y+.5)*res,str(room['id']),ha='center',fontsize=9)
   ax.scatter(-walls[:,0],walls[:,1],s=.25,c='#b51b36',lw=0,rasterized=True)
   ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X (m)',ylabel='Y (m)',title=f'{f+1}楼 | '+('700秒实测' if j==0 else '官方真值几何')+' | '+('房间候选分区' if mode else '墙体候选'))
fig.suptitle('同参数离线处理：实测点云与官方几何｜红色为墙体，彩色为房间候选',fontsize=16)
fig.text(.5,.015,'房间为几何分割候选，并非语义真值；地面支持区域可能包含补洞，不能据此判断已探索完整或用于导航。',ha='center',fontsize=11)
fig.tight_layout(rect=[0,.035,1,.96]);fig.savefig(OUT/'three_floors_wall_room_comparison.png',dpi=160);plt.close(fig)
(OUT/'summary.json').write_text(json.dumps({'algorithm':'existing ceiling-supported measured wall extractor AE; horizontal floor/ceiling support domain and distance-core watershed proposals','config':cfg,'grid_resolution_m':res,'display':'-X/Y; geometry unmirrored','truth_used_in_measured_processing':False,'limitations':['geometric proposals, no semantic room annotation','horizontal support closing/fill and wall convex-hull bound may bridge unobserved holes; not room truth','furniture and doorways may cause splits/merges; not navigation map'],'results':records},ensure_ascii=False,indent=2))
