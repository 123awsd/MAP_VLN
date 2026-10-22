#!/usr/bin/env python3
"""Compare structural room masks on one measured cloud; trajectory is audit-only."""
from pathlib import Path
import sys,json,heapq,argparse
from collections import deque
import numpy as np,cv2
from scipy.ndimage import distance_transform_edt,maximum_filter
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'));import render_exploration_three_view as display
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--trial-dir',type=Path,help='Use this trial\'s fresh geometry, extracted walls and exact-matched free rays')
parser.add_argument('--output-dir',type=Path)
parser.add_argument('--export-name',default='synchronized_export_matched',help='Name of the exact synchronized export directory under trial-dir')
parser.add_argument('--observed-floor-planes',type=Path,help='Optional offline measured floor candidates; requires exactly three ordered broad planes')
args=parser.parse_args()
OUT=args.output_dir or ROOT/'outputs/room_quality_trial';OUT.mkdir(parents=True,exist_ok=True)
if args.trial_dir:
 cache=args.trial_dir;geo=np.load(cache/'geometry.npz');inc=None;poses=None
 frame=np.genfromtxt(cache/args.export_name/'frame_evidence.csv',delimiter=',',names=True)
 traj=np.column_stack([frame[k]for k in ['elapsed_s','x','y','z']])
else:
 cache=ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed';geo=np.load(ROOT.parent/'视频制作/探索剪辑素材_无字30帧/geometry.npz');inc=np.load(cache/'increments.npz');poses=np.load(cache/'camera_pose.npy');traj=np.load(cache/'trajectory.npy')
ground=np.array([-.155,2.595,5.595]);floorbases=np.array([-.2,2.6,5.4]);res=.05
if args.observed_floor_planes:
 evidence=json.loads(args.observed_floor_planes.read_text())
 ground=np.array([p['observed_plane_z_m'] for p in evidence['broad_floor_candidates']],dtype=float)
 if len(ground)!=3 or not np.isfinite(ground).all() or not np.all(np.diff(ground)>1):raise ValueError('Three separated observed floor candidates required')
 # Classify ray origins by measured ground height, with the existing 15cm floor margin.
 floorbases=ground-.15
# Shared domain built from recorded rays at the sensor's horizontal band and observed floor returns.
data=[{'p':geo[f'p{f}'],'n':geo[f'n{f}']}for f in range(3)];allp=np.concatenate([x['p']for x in data]);origin=np.floor((allp[:,:2].min(0)-.5)/res)*res;size=np.ceil((allp[:,:2].max(0)+.5-origin)/res).astype(int)+1;w,h=size;shape=(h,w)
def pixel(p):return np.floor((np.asarray(p)[...,:2]-origin)/res).astype('int32')
def drawcells(mask,p):
 uv=pixel(p);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=uv[ok];mask[uv[:,1],uv[:,0]]=1
free=[np.zeros(shape,np.uint8)for _ in range(3)];seen=[np.zeros(shape,np.uint8)for _ in range(3)];rayframes=[np.zeros(shape,np.uint16)for _ in range(3)]
low_rayframes=[np.zeros(shape,np.uint16)for _ in range(3)]
for key in inc.files if inc is not None else []:
 if not key.startswith('new_'):continue
 idx=int(key[4:]);stamp=inc['targets'][idx];pose=poses[max(0,np.searchsorted(poses[:,0],stamp)-1)];f=int(np.clip(np.searchsorted(floorbases+.15,pose[3])-1,0,2));p=inc[key]
 if not len(p):continue
 floor=p[np.abs(p[:,2]-ground[f])<.08];drawcells(free[f],floor)
 # Sensor rays are loaded from exact header-stamp matched RGB-D poses below.
if args.trial_dir:
 for f,item in enumerate(data):drawcells(free[f],item['p'][np.abs(item['p'][:,2]-ground[f])<.08])
rays=np.load((cache if args.trial_dir else cache.parent)/'measured_free_rays.npz')
for i,pose in enumerate(rays['poses']):
 f=int(np.clip(np.searchsorted(floorbases+.15,pose[3])-1,0,2));uv=pixel(rays[f'r{i:04d}']);start=pixel(pose[1:4]);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=np.unique(uv[ok],axis=0)
 if 0<=start[0]<w and 0<=start[1]<h:
  raymask=np.zeros(shape,np.uint8)
  for end in uv:
   delta=end-start;length=float(np.linalg.norm(delta))
   if length>4:
    safeend=start+np.round(delta*(length-3)/length).astype(int);cv2.line(raymask,tuple(start),tuple(safeend),1,1)
  free[f]|=raymask;rayframes[f]+=raymask
  if .35 <= pose[3]-ground[f] <= 1.25:low_rayframes[f]+=raymask

for f,item in enumerate(data):drawcells(seen[f],item['p'])
np.savez_compressed(OUT/'observed_domain.npz',origin=origin,resolution=res,**{f'free{f}':free[f]for f in range(3)})
def grow(markers,domain):
 labels=markers.copy();queue=deque(map(tuple,np.argwhere(labels>0)))
 while queue:
  y,x=queue.popleft();label=labels[y,x]
  for dy,dx in [(0,1),(0,-1),(1,0),(-1,0)]:
   yy,xx=y+dy,x+dx
   if 0<=yy<h and 0<=xx<w and domain[yy,xx] and labels[yy,xx]==0:labels[yy,xx]=label;queue.append((yy,xx))
 return labels
wall_cache={}
def walls(f,kind):
 if (f,kind)in wall_cache:return wall_cache[f,kind]
 if kind=='measured':
  mask=np.zeros(shape,np.uint8);drawcells(mask,np.load(cache/f'L{f+1}_new_wall_points.npz')['wall_points'])
  wall_cache[f,kind]=mask.astype(bool);return wall_cache[f,kind]
 if kind=='height_supported':
  mask=walls(f,'measured').copy();points=np.load(cache/f'L{f+1}_new_wall_points.npz')['wall_points']
  low=np.zeros(shape,np.uint8);drawcells(low,points[(points[:,2]-ground[f]>=.25)&(points[:,2]-ground[f]<=1.25)])
  low=cv2.dilate(low,np.ones((3,3),np.uint8))
  opening=mask&(low==0)&(low_rayframes[f]>=4)
  mask[opening]=False
  np.savez_compressed(OUT/f'L{f+1}_opening_evidence.npz',original_wall=walls(f,'measured'),opening=opening,low_wall_support=low,low_free_ray_frames=low_rayframes[f],origin=origin,resolution=res)
  wall_cache[f,kind]=mask;return mask
 if kind=='structural':
  mask=walls(f,'ray4').copy();num,lab,stats,_=cv2.connectedComponentsWithStats(mask.astype('uint8'),8)
  p=data[f]['p'];n=data[f]['n']
  for k in range(1,num):
   x,y,ww,hh,area=stats[k]
   if max(ww,hh)*res>.30:continue
   lo=origin+np.array([x-1,y-1])*res;hi=origin+np.array([x+ww+1,y+hh+1])*res;use=(p[:,0]>=lo[0])&(p[:,0]<=hi[0])&(p[:,1]>=lo[1])&(p[:,1]<=hi[1])&(np.abs(n[:,2])<.35);z=p[use,2]-ground[f];z=z[(z>.25)&(z<2.2)]
   full_column=len(z)>0 and z.max()-z.min()>=1.6 and len(np.unique(np.floor(z/.2)))>=8
   if not full_column:mask[lab==k]=False
  wall_cache[f,kind]=mask;return mask
 if kind.startswith('ray'):
  threshold=int(kind[3:]);mask=walls(f,'thin').copy();mask[rayframes[f]>=threshold]=False;wall_cache[f,kind]=mask;return mask
 p,n=data[f]['p'],data[f]['n'];vertical=np.abs(n[:,2])<.35;rel=p[:,2]-ground[f];use=vertical&(rel>.25)&(rel<2.2);q=p[use];uv=pixel(q);z=np.floor((q[:,2]-ground[f])/.20).astype(int);counts=np.zeros((12,h,w),np.uint8)
 for zb in range(12):
  xy=uv[z==zb];counts[zb,xy[:,1],xy[:,0]]=1
 # Neighbourhood support recognizes real measured planes across sparse vertical columns.
 votes=sum(cv2.dilate(c,np.ones((3,3),np.uint8))for c in counts);heightvotes=np.zeros(shape,np.uint8);drawcells(heightvotes,p[vertical&(rel>1.45)])
 mask=((votes>=4)&(cv2.dilate(heightvotes,np.ones((3,3),np.uint8))>0)).astype('uint8');support=np.zeros(shape,np.uint8);drawcells(support,q);mask&=cv2.dilate(support,np.ones((3,3),np.uint8))
 if kind=='thick':mask=cv2.dilate(mask,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
 elif kind=='directional':
  # Only short gaps, along the observed wall direction; no per-room edits.
  horizontal=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((1,5),np.uint8));verticalmask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,1),np.uint8));mask=horizontal|verticalmask
 elif kind=='thin':mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
 wall_cache[f,kind]=mask.astype(bool);return wall_cache[f,kind]
def segment(f,kind,neck):
 if kind=='preserve_regions':
  _,_,original=segment(f,'measured',neck)
  wall=walls(f,'height_supported');known=cv2.morphologyEx(free[f],cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)).astype(bool)|seen[f].astype(bool);domain=known&~wall
  labels=grow(original,domain)
  assert np.array_equal(labels[original>0],original[original>0]),'existing seed labels must not merge or change'
  return wall,domain,labels
 wall=walls(f,kind);known=cv2.morphologyEx(free[f],cv2.MORPH_CLOSE,np.ones((5,5)if kind=='structural' else (3,3),np.uint8)).astype(bool)|seen[f].astype(bool);domain=known&~wall
 dist=distance_transform_edt(domain)*res
 core=(dist>neck).astype('uint8');num,components,stats,_=cv2.connectedComponentsWithStats(core,4);markers=np.zeros(shape,np.int32);k=0
 for j in range(1,num):
  if stats[j,cv2.CC_STAT_AREA]*res*res>=.35:k+=1;markers[components==j]=k
 labels=grow(markers,domain)
 # Components without a sufficiently wide room core remain uncertain, not silently filled.
 return wall,domain,labels
trajfloor=np.clip(np.searchsorted(floorbases+.15,traj[:,3])-1,0,2)
def audit(f,wall,domain,labels):
 path=traj[trajfloor==f,1:];valid=path[(path[:,2]>ground[f]+.35)&(path[:,2]<ground[f]+2.25)];uv=pixel(valid);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=uv[ok];hit=wall[uv[:,1],uv[:,0]];known=domain[uv[:,1],uv[:,0]];label=labels[uv[:,1],uv[:,0]]
 return {'samples':len(uv),'wall_hit_samples':int(hit.sum()),'wall_hit_fraction':float(hit.mean())if len(hit)else 0.,'domain_fraction':float(known.mean())if len(hit)else 0.,'labeled_fraction':float((label>0).mean())if len(hit)else 0.,'regions':int(labels.max()),'room_area_m2':float((labels>0).sum()*res*res)}
variants=[('A_厚墙基线','thick',.40),('B_薄墙距离分割','thin',.40),('C_方向连接距离分割','directional',.40),('D_薄墙较宽门口阈值','thin',.55),('E_方向连接较宽门口阈值','directional',.55),('F_方向连接窄门阈值','directional',.30),('G_薄墙自由证据3帧','ray3',.40),('H_薄墙自由证据6帧','ray6',.40),('I_薄墙自由证据12帧','ray12',.40),('J_自由证据3帧宽门分割','ray3',.55),('K_自由证据4帧','ray4',.40),('L_自由证据5帧','ray5',.40),('M_自由证据4帧宽门分割','ray4',.55),('N_自由证据4帧更宽门分割','ray4',.65),('O_自由证据4帧多房间核心','ray4',.75),('P_结构小片段判别','structural',.55)]
if args.trial_dir:
 variants=[('S_新实测墙自由空间','measured',.55),('T_低位自由门洞证据','height_supported',.55),('U_门洞自由区域保留','preserve_regions',.55)]
 (OUT/'input_manifest.json').write_text(json.dumps({'observed_floor_evidence':str(args.observed_floor_planes) if args.observed_floor_planes else None,'offline_ground_heights_m':ground.tolist(),'offline_ray_floor_thresholds_m':(floorbases+.15).tolist(),'trial':str(cache),'geometry':str(cache/'geometry.npz'),'walls':[str(cache/f'L{f}_new_wall_points.npz')for f in [1,2,3]],'free_rays':str(cache/'measured_free_rays.npz'),'trajectory_role':'audit only','scene_truth_used':False,'quality_validated':False,'method':'Observed wall raster and recorded free rays; distance-core partition; unseeded components remain uncertain'},ensure_ascii=False,indent=2))
colors=np.array([[121,177,196],[159,150,193],[134,182,158],[175,174,143],[122,151,195],[188,149,173],[148,180,185]],np.uint8);records=[]
for name,kind,neck in variants:
 rec={'name':name,'wall_method':kind,'neck_radius_m':neck,'floors':[]}
 for f in range(3):
  wall,domain,labels=segment(f,kind,neck);stats=audit(f,wall,domain,labels);rec['floors'].append(stats);np.savez_compressed(OUT/f'{name}_L{f+1}.npz',wall=wall,domain=domain,labels=labels,origin=origin,resolution=res)
  im=np.full((h,w,3),(18,21,26),np.uint8);im[domain]=[40,48,58]
  for k in range(1,labels.max()+1):im[labels==k]=(colors[(k-1)%len(colors)]*.5).astype('uint8')
  im[wall]=[218,223,229];path=traj[trajfloor==f,1:];uv=pixel(path);cv2.polylines(im,[uv],False,(240,211,132),1,cv2.LINE_AA)
  # Hit markers audit geometry only; they are never fed back into segmentation.
  uv=pixel(path[(path[:,2]>ground[f]+.35)&(path[:,2]<ground[f]+2.25)]);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=uv[ok];uv=uv[wall[uv[:,1],uv[:,0]]]
  for xy in uv[::5]:cv2.circle(im,tuple(xy),2,(245,91,99),-1)
  image=Image.fromarray(np.flipud(im).copy());image=image.resize((int(image.width*min(720/image.width,850/image.height)),int(image.height*min(720/image.width,850/image.height))),Image.Resampling.NEAREST);canvas=Image.new('RGB',(800,1000),(18,21,26));canvas.paste(image,((800-image.width)//2,110));d=ImageDraw.Draw(canvas);d.text((20,15),f'{name} · {f+1}层',font=display.font(25),fill=(229,238,245));d.text((20,60),f"区域 {stats['regions']}，轨迹落墙 {stats['wall_hit_samples']}/{stats['samples']}",font=display.font(21),fill=(181,199,214));d.text((20,955),'白：结构墙；黄：轨迹；红：投影落墙',font=display.font(20),fill=(182,198,214));canvas.save(OUT/f'{name}_L{f+1}.png')
 print(name,rec['floors'],flush=True);records.append(rec)
(OUT/'metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
for f in range(3):
 b=Image.new('RGB',(2400,6000),(18,21,26))
 for i,(name,_,_)in enumerate(variants):b.paste(Image.open(OUT/f'{name}_L{f+1}.png'),((i%3)*800,(i//3)*1000))
 b.save(OUT/f'L{f+1}_十六方法对比.png')
