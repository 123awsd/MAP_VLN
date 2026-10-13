#!/usr/bin/env python3
"""Compare structural room masks on one measured cloud; trajectory is audit-only."""
from pathlib import Path
import sys,json,heapq
from collections import deque
import numpy as np,cv2
from scipy.ndimage import distance_transform_edt,maximum_filter
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'));import render_exploration_three_view as display
OUT=ROOT/'outputs/room_quality_fullheight';OUT.mkdir(exist_ok=True)
cache=ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed';geo=np.load(OUT/'geometry.npz');inc=np.load(cache/'increments.npz');poses=np.load(cache/'camera_pose.npy');traj=np.load(cache/'trajectory.npy');ground=np.array([-.155,2.595,5.595]);floorbases=np.array([-.2,2.6,5.4]);res=.05
# Shared domain built from recorded rays at the sensor's horizontal band and observed floor returns.
data=[{'p':geo[f'p{f}'],'n':geo[f'n{f}']}for f in range(3)];allp=np.concatenate([x['p']for x in data]);origin=np.floor((allp[:,:2].min(0)-.5)/res)*res;size=np.ceil((allp[:,:2].max(0)+.5-origin)/res).astype(int)+1;w,h=size;shape=(h,w)
def pixel(p):return np.floor((np.asarray(p)[...,:2]-origin)/res).astype('int32')
def drawcells(mask,p):
 uv=pixel(p);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=uv[ok];mask[uv[:,1],uv[:,0]]=1
free=[np.zeros(shape,np.uint8)for _ in range(3)];seen=[np.zeros(shape,np.uint8)for _ in range(3)];rayframes=[np.zeros(shape,np.uint16)for _ in range(3)]
for key in inc.files:
 if not key.startswith('new_'):continue
 idx=int(key[4:]);stamp=inc['targets'][idx];pose=poses[max(0,np.searchsorted(poses[:,0],stamp)-1)];f=int(np.clip(np.searchsorted(floorbases+.15,pose[3])-1,0,2));p=inc[key]
 if not len(p):continue
 floor=p[np.abs(p[:,2]-ground[f])<.08];drawcells(free[f],floor)
 # Sensor rays are loaded from exact header-stamp matched RGB-D poses below.
rays=np.load(cache.parent/'measured_free_rays.npz')
for i,pose in enumerate(rays['poses']):
 f=int(np.clip(np.searchsorted(floorbases+.15,pose[3])-1,0,2));uv=pixel(rays[f'r{i:04d}']);start=pixel(pose[1:4]);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=np.unique(uv[ok],axis=0)
 if 0<=start[0]<w and 0<=start[1]<h:
  raymask=np.zeros(shape,np.uint8)
  for end in uv:
   delta=end-start;length=float(np.linalg.norm(delta))
   if length>4:
    safeend=start+np.round(delta*(length-3)/length).astype(int);cv2.line(raymask,tuple(start),tuple(safeend),1,1)
  free[f]|=raymask;rayframes[f]+=raymask

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
 if kind=='lines':
  # Extract persistent long structural lines instead of treating each vertical patch as a wall.
  p,n=data[f]['p'],data[f]['n'];rel=p[:,2]-ground[f];use=(np.abs(n[:,2])<.30)&(rel>.4)&(rel<2.2);q=p[use];uv=pixel(q)
  counts=np.zeros((11,h,w),np.uint8)
  for b in range(11):
   xy=uv[(q[:,2]-ground[f]>=b*.2)&(q[:,2]-ground[f]<(b+1)*.2)];counts[b,xy[:,1],xy[:,0]]=1
  support=sum(cv2.dilate(c,np.ones((3,3),np.uint8))for c in counts)
  candidates=((support>=5)&(sum(counts[7:11])>0)).astype('uint8')
  lines=cv2.HoughLinesP(candidates*255,1,np.pi/180,threshold=12,minLineLength=round(.65/res),maxLineGap=round(.20/res))
  mask=np.zeros(shape,np.uint8)
  if lines is not None:
   for x1,y1,x2,y2 in lines.reshape(-1,4):
    # Retain arbitrary wall directions; require direct support along most of the proposed line.
    trial=np.zeros(shape,np.uint8);cv2.line(trial,(x1,y1),(x2,y2),1,1)
    near=cv2.dilate(candidates,np.ones((3,3),np.uint8))
    if np.mean(near[trial>0])>=.75:mask|=trial
  wall_cache[f,kind]=mask.astype(bool);return wall_cache[f,kind]
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
variants=[('X_连续结构墙线','lines',.55)]
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
 b=Image.new('RGB',(800,1000),(18,21,26))
 for i,(name,_,_)in enumerate(variants):b.paste(Image.open(OUT/f'{name}_L{f+1}.png'),((i%3)*800,(i//3)*1000))
 b.save(OUT/f'L{f+1}_十六方法对比.png')
