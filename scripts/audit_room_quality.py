#!/usr/bin/env python3
from pathlib import Path
import json
import numpy as np,cv2
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/room_quality_trial';variant='P_结构小片段判别';a=np.load(ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed/trajectory.npy');ground=np.array([-.155,2.595,5.595]);bases=np.array([-.2,2.6,5.4]);result=[];graphs=[]
for f in range(3):
 d=np.load(OUT/f'{variant}_L{f+1}.npz');wall,labels,domain=d['wall'],d['labels'],d['domain'];origin,res=d['origin'],float(d['resolution']);h,w=wall.shape
 pathparts=[]
 for start,end in zip(a[:-1],a[1:]):
  # Subdivide every recorded segment at <=2.5cm; no corridor-specific exclusions.
  dist=np.linalg.norm(end[1:]-start[1:]);n=max(1,int(np.ceil(dist/.025)));q=np.linspace(start[1:],end[1:],n+1);floor=np.clip(np.searchsorted(bases+.15,q[:,2])-1,0,2);keep=(floor==f)&(q[:,2]>ground[f]+.35)&(q[:,2]<ground[f]+2.25)
  if keep.any():pathparts.append(q[keep])
 p=np.concatenate(pathparts);uv=np.floor((p[:,:2]-origin)/res).astype(int);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=uv[ok];hits=wall[uv[:,1],uv[:,0]];unlabelled=labels[uv[:,1],uv[:,0]]==0
 # Shared label edges are traversable portals, not physical walls.
 portals={}
 for dy,dx in [(0,1),(1,0)]:
  aa=labels[:h-dy,:w-dx];bb=labels[dy:,dx:];yy,xx=np.where((aa>0)&(bb>0)&(aa!=bb))
  for y,x in zip(yy,xx):
   pair=tuple(sorted((int(aa[y,x]),int(bb[y,x]))));portals.setdefault(pair,[]).append([float(origin[0]+(x+.5)*res),float(origin[1]+(y+.5)*res)])
 regions=[]
 for rid in range(1,int(labels.max())+1):
  mask=(labels==rid).astype('uint8');cts,hierarchy=cv2.findContours(mask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE);parts=[]
  if hierarchy is not None:
   for i,c in enumerate(cts):
    if hierarchy[0,i,3]!=-1:continue
    poly=c[:,0,:]*res+origin+res/2;holes=[];child=hierarchy[0,i,2]
    while child!=-1:holes.append((cts[child][:,0,:]*res+origin+res/2).tolist());child=hierarchy[0,child,0]
    parts.append({'outer_xy_m':poly.tolist(),'holes_xy_m':holes})
  regions.append({'id':rid,'area_m2':float(mask.sum()*res*res),'polygon_components':parts})
 graph={'floor':f+1,'regions':regions,'portals':[{'region_ids':list(pair),'boundary_samples_xy_m':pts}for pair,pts in portals.items()],'note':'Label interfaces are traversable partitions, not obstacle walls; no automatic semantic names copied from old run.'};graphs.append(graph);(OUT/f'L{f+1}_selected_regions.json').write_text(json.dumps(graph,ensure_ascii=False,indent=2))
 metrics={'floor':f+1,'densified_samples':len(uv),'wall_hits':int(hits.sum()),'unlabeled_samples':int(unlabelled.sum()),'out_of_bounds_samples':int((~ok).sum()),'regions':int(labels.max()),'portals':len(portals),'passed':not hits.any()and not unlabelled.any()and bool(ok.all())};result.append(metrics)
 if hits.any():np.save(OUT/f'L{f+1}_continuous_conflicts.npy',p[ok][hits])
(OUT/'continuous_trajectory_audit.json').write_text(json.dumps({'sample_step_m':.025,'scope':'floorwise 2D centreline; height=.35..2.25m above measured floor; excludes near-floor/ceiling and cross-floor transitions from 2D proof; not a 3D radius-clearance certificate','results':result,'all_pass':all(x['passed']for x in result)},ensure_ascii=False,indent=2));print(result,flush=True)
