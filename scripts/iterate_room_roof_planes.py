#!/usr/bin/env python3
"""Compare structural room masks on one measured cloud; trajectory is audit-only."""
from pathlib import Path
import sys,json,heapq
from collections import deque
import numpy as np,cv2
from scipy.ndimage import distance_transform_edt,maximum_filter
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'));import render_exploration_three_view as display
OUT=ROOT/'outputs/room_quality_ceiling';OUT.mkdir(exist_ok=True)
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
 if kind=='planes':
  # Plane hypotheses use normals and offsets in 3D, then aggregate evidence along each plane.
  p,n=data[f]['p'],data[f]['n'];rel=p[:,2]-ground[f]
  good=(np.abs(n[:,2])<.25)&(rel>.25)&(rel<2.65);q=p[good];nn=n[good,:2];nn/=np.maximum(np.linalg.norm(nn,axis=1,keepdims=True),1e-8)
  ang=np.mod(np.arctan2(nn[:,1],nn[:,0]),np.pi);hist,edges=np.histogram(ang,bins=90,range=(0,np.pi));peaks=[]
  for idx in np.argsort(hist)[::-1]:
   theta=(edges[idx]+edges[idx+1])/2
   if all(min(abs(theta-t),np.pi-abs(theta-t))>np.deg2rad(12) for t in peaks):peaks.append(theta)
   if len(peaks)==4:break
  mask=np.zeros(shape,np.uint8);plane_records=[]
  for theta in peaks:
   normal=np.array([np.cos(theta),np.sin(theta)]);tangent=np.array([-normal[1],normal[0]])
   near=np.abs(nn@normal)>.96;qq=q[near];offsets=qq[:,:2]@normal
   bins=np.round(offsets/.05).astype(int);keys,counts=np.unique(bins,return_counts=True);order=keys[np.argsort(counts)[::-1]];chosen=[]
   for key in order:
    offset=key*.05
    if any(abs(offset-o)<.15 for o in chosen):continue
    points=qq[np.abs(offsets-offset)<.06]
    if len(points)<80:continue
    heights=points[:,2]-ground[f]
    if np.ptp(heights)<1.25 or heights.max()<1.4:continue
    along=points[:,:2]@tangent;origin_t=np.floor(along.min()/.1)*.1;tb=np.floor((along-origin_t)/.1).astype(int);sz=int(tb.max()+1)
    evidence=np.zeros((14,sz),np.uint8);zb=np.clip(np.floor(heights/.2).astype(int),0,13);evidence[zb,tb]=1
    robust=(evidence.sum(0)>=4)&(evidence[7:].sum(0)>0)
    # Close only short unsupported stretches on this same measured plane.
    robust=cv2.morphologyEx(robust.astype('uint8')[None,:],cv2.MORPH_CLOSE,np.ones((1,3),np.uint8))[0]>0
    runs=np.diff(np.r_[False,robust,False].astype(int))
    accepted=False
    for start,end in zip(np.where(runs==1)[0],np.where(runs==-1)[0]):
     if (end-start)*.1<.65:continue
     lo=origin_t+start*.1;hi=origin_t+end*.1;ends=np.array([normal*offset+tangent*lo,normal*offset+tangent*hi]);xy=pixel(ends);cv2.line(mask,tuple(xy[0]),tuple(xy[1]),1,1);accepted=True
     plane_records.append({'normal_xy':normal.tolist(),'offset_m':float(offset),'extent_t_m':[float(lo),float(hi)],'height_span_m':float(np.ptp(heights))})
    if accepted:chosen.append(offset)
  (OUT/f'L{f+1}_plane_records.json').write_text(json.dumps(plane_records,indent=2))
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
 p,n=data[f]['p'],data[f]['n'];vertical=np.abs(n[:,2])<.35;rel=p[:,2]-ground[f];use=vertical&(rel>.25)&(rel<2.65);q=p[use];uv=pixel(q);z=np.floor((q[:,2]-ground[f])/.20).astype(int);counts=np.zeros((12,h,w),np.uint8)
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
variants=[('C_完整斜顶结构平面','planes',.55)]
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
