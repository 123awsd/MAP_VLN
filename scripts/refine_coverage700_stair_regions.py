"""Represent witnessed stairs as cross-floor spaces; never change wall or navigation maps."""
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
from reconstruct_transition_spaces import detect_transition_traversals
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview';run=R/'outputs/coverage_lifecycle_700_r1_20261016';a=np.load(run/'video_cache/trajectory.npy');geo=np.load(run/'original_style_video/geometry.npz');grounds=[-.155,2.595,5.595]
tr=detect_transition_traversals(a[:,0],a[:,1:],[{'flight_z_m':z}for z in [1.05,3.95,6.85]])
assert tr,'No witnessed floor traversal'
# Associate nearby routes across shared floors without floor-specific coordinates.
parent=list(range(len(tr)))
def root(i):
 while parent[i]!=i:i=parent[i]
 return i
for i,t in enumerate(tr):
 for j,u in enumerate(tr[:i]):
  if {t['lower_floor'],t['upper_floor']}&{u['lower_floor'],u['upper_floor']}:
   p=np.array(t['centerline_xyz']);q=np.array(u['centerline_xyz'])
   if cKDTree(p[:,:2]).query(q[:,:2])[0].min()<1.:parent[root(i)]=root(j)
groups={}
for i,t in enumerate(tr):groups.setdefault(root(i),[]).append(t)
groups=list(groups.values());font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,2,figsize=(12,15));rows=[]
for f,g in enumerate(grounds):
 d=np.load(O/'measured'/f'L{f+1}_small_room_preserved.npz');old=d['labels'];wall=d['wall_grid'];barrier=cv2.dilate(wall,np.ones((3,3),np.uint8))>0;org=d['origin_xy'];res=float(d['resolution']);p,n=geo[f'p{f}'],geo[f'n{f}'];h,w=old.shape;yy,xx=np.indices(old.shape);centers=np.column_stack([org[0]+(xx.ravel()+.5)*res,org[1]+(yy.ravel()+.5)*res]);stairs=np.zeros_like(old);links=[]
 for sid,ts in enumerate(groups,1):
  related=[t for t in ts if f+1 in [t['lower_floor'],t['upper_floor']]]
  if not related:continue
  route=np.concatenate([np.array(t['centerline_xyz']) for t in related]);tree=cKDTree(route[:,:2]);dd,ix=tree.query(p[:,:2]);localroutez=route[ix,2]
  # Treads/slope returns must lie below a witnessed flight path, with limited XY offset.
  support=(dd<=.65)&(abs(n[:,2])>=.65)&(p[:,2]>=g-.15)&(p[:,2]<=g+3.15)&(localroutez-p[:,2]>=.08)&(localroutez-p[:,2]<=1.9)
  evidence=np.zeros((h,w),np.uint8);xy=np.floor((p[support,:2]-org)/res).astype(int);valid=(xy[:,0]>=0)&(xy[:,0]<w)&(xy[:,1]>=0)&(xy[:,1]<h);xy=xy[valid];evidence[xy[:,1],xy[:,0]]=1
  evidence=cv2.dilate(evidence,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)))>0
  route_near=tree.query(centers)[0].reshape(h,w)<=.65
  allowed=evidence&route_near&~barrier
  # Only evidence components near the observed centerline receive the logical stair ID.
  cc,num=ndi.label(allowed);near=tree.query(centers)[0].reshape(h,w)<=.25;mask=np.zeros_like(allowed)
  for k in range(1,num+1):
   m=cc==k
   if np.any(m&near):mask|=m
  stairs[mask]=sid
  adjacent=ndi.binary_dilation(mask,iterations=2)&~mask;ids=np.unique(old[adjacent]);ids=ids[ids>0]
  links.append({'stair_id':f'SF{sid}','support_points':int(support.sum()),'projected_area_m2':round(mask.sum()*res**2,4),'newly_represented_blank_m2':round((mask&(old==0)).sum()*res**2,4),'adjacent_room_candidate_ids':ids.tolist(),'connected_floors':sorted({k for t in ts for k in [t['lower_floor'],t['upper_floor']]})})
 new=old.copy();new[stairs>0]=old.max()+stairs[stairs>0]
 assert not np.any((stairs>0)&barrier)
 assert np.array_equal(new[stairs==0],old[stairs==0])
 np.savez_compressed(O/'measured'/f'L{f+1}_stair_aware_regions.npz',labels=new,stair_logical_ids=stairs,wall_grid=wall,original_room_domain=d['observed_floor_domain'],origin_xy=org,resolution=res)
 rows.append({'floor':f+1,'stairspaces':links,'wall_mask_unchanged':True,'outside_stairs_labels_unchanged':True})
 for j,labels in enumerate([old,new]):
  ax=axs[f,j];rgba=plt.get_cmap('tab20')((labels%20)/19);rgba[:,:,3]=(labels>0)*.65
  if j:rgba[stairs>0]=[.05,.72,.78,.9]
  ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);q=d['wall_points'];ax.scatter(-q[:,0],q[:,1],s=.2,c='#b42137',lw=0,rasterized=True)
  if j:
   for sid in np.unique(stairs):
    if sid:
     y,x=ndi.center_of_mass(stairs==sid);ax.text(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,f'SF{sid}',fontsize=10,weight='bold',ha='center')
  ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f+1}楼｜'+('原房间分区'if j==0 else'楼梯连接空间（青色）'))
fig.suptitle('700秒实际点云：楼梯跨层空间修正',fontsize=16);fig.tight_layout(rect=[0,.04,1,.96]);fig.text(.5,.015,'SF为跨层逻辑空间，不是平坦可行走地面；只在实测台阶/坡面和实际跨层轨迹共同支持处建立，保留墙体。',ha='center',fontsize=9);fig.savefig(O/'stair_aware_before_after.png',dpi=160)
(O/'stair_aware_regions_audit.json').write_text(json.dumps({'source_run':str(run),'uses_truth':False,'uses_previous_run_masks':False,'traversals':tr,'parameters':{'xy_route_radius_m':.65,'support_normal_abs_z_min':.65,'support_below_route_m':[.08,1.9],'point_projection_dilation_m':.15},'limitations':['observed partial footprint, not entire stairwell truth','projection may overlap different heights; not navigation clearance','wall-mask and floorwise room boundaries still provisional'],'floors':rows},ensure_ascii=False,indent=2));print(json.dumps({'traversals':len(tr),'logical_spaces':len(groups),'floors':rows},ensure_ascii=False))
