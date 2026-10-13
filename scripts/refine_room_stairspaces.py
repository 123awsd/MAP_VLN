#!/usr/bin/env python3
"""Assign observed inter-floor circulation a shared logical space, preserving wall/domain."""
from pathlib import Path
import json,sys
import numpy as np,cv2
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'));from reconstruct_transition_spaces import detect_transition_traversals,cluster_traversals
OUT=ROOT/'outputs/room_quality_trial';cache=ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed';a=np.load(cache/'trajectory.npy');traversals=detect_transition_traversals(a[:,0],a[:,1:],[{'flight_z_m':z}for z in [1.05,3.95,6.85]]);groups=cluster_traversals(traversals);parent=list(range(len(groups)))
def find(i):
 while parent[i]!=i:i=parent[i]
 return i
# Shared floor + nearby observed route footprints: logical association, not an invented 3D volume.
for i,g in enumerate(groups):
 for j in range(i):
  h=groups[j];fs={g[0]['lower_floor'],g[0]['upper_floor']};gs={h[0]['lower_floor'],h[0]['upper_floor']}
  if not fs&gs:continue
  x=np.concatenate([t['centerline_xyz']for t in g]);y=np.concatenate([t['centerline_xyz']for t in h]);near=cKDTree(x[:,:2]).query(y[:,:2])[0].min()
  if near<=1.5:parent[find(i)]=find(j)
logical={}
for i,g in enumerate(groups):logical.setdefault(find(i),[]).extend(g)
raw=np.concatenate([np.load(cache/'increments.npz')[f'new_{i:03d}']for i in range(800)]);specs=[];changed=[]
for sid,ts in enumerate(logical.values(),1):specs.append({'id':f'SF{sid}','semantic_type':'observed_interfloor_transition','classification':'stairwell_candidate','connected_floors':sorted({f for t in ts for f in [t['lower_floor'],t['upper_floor']]}),'traversals':ts,'source':'executed inter-floor traversals plus nearby measured geometry','confidence_note':'not floorplan truth; group by shared floor and route-footprint proximity <=1.5m','floor_components':{}})
for f in range(3):
 d=np.load(OUT/f'P_结构小片段判别_L{f+1}.npz');labels=d['labels'].copy();domain=d['domain'];wall=d['wall'];origin=d['origin'];res=float(d['resolution']);h,w=labels.shape;old=labels.copy();mapping={}
 def pixel(q):return np.floor((np.array(q)[...,:2]-origin)/res).astype('int32')
 for sid,sp in enumerate(specs,1):
  if f+1 not in sp['connected_floors']:continue
  lines=[np.array(t['centerline_xyz'])for t in sp['traversals']if f+1 in [t['lower_floor'],t['upper_floor']]];q=np.concatenate(lines);tree=cKDTree(q[:,:2]);floor_z=[-.155,2.595,5.595][f];selected=raw[(raw[:,2]>=floor_z-.10)&(raw[:,2]<=floor_z+3.15)];near=tree.query(selected[:,:2])[0]<=.85;measured=selected[near];footprint=np.zeros((h,w),np.uint8);uv=pixel(measured);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv=uv[ok];footprint[uv[:,1],uv[:,0]]=1;footprint=cv2.dilate(footprint,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)))
  route=np.zeros((h,w),np.uint8)
  for line in lines:cv2.polylines(route,[pixel(line)],False,1,thickness=max(1,round(.70/res)))
  # Geometric support stops arbitrary route-buffer widening; original wall/domain remain unchanged.
  seed=(route>0)&(footprint>0)&domain;allowed=(footprint>0)&domain
  # Grow at most .6m away from witnessed traversal footprint inside observed free domain.
  local=seed.copy()
  for _ in range(round(.60/res)):local=(cv2.dilate(local.astype('uint8'),np.array([[0,1,0],[1,1,1],[0,1,0]],np.uint8))>0)&allowed
  touched=[]
  for k in np.unique(old[local]):
   if k==0:continue
   mask=old==k;fraction=float((mask&local).sum()/mask.sum());area=float(mask.sum()*res*res)
   # Whole-fragment relabeling only for small, predominantly supported staircase pieces.
   if fraction>=.50 and area<=max(8.,2.*float(local.sum()*res*res)):local|=mask
   touched.append({'source_region_id':int(k),'overlap_ratio':fraction,'source_area_m2':area})
  # Absorb small adjacent leftovers of the same source region, avoiding large corridors.
  distance=distance_transform_edt(~local)*res
  for entry in touched:
   k=entry['source_region_id'];rest=((old==k)&~local).astype('uint8');nr,rr,stats,_=cv2.connectedComponentsWithStats(rest,4)
   for component in range(1,nr):
    piece=rr==component;area=float(stats[component,cv2.CC_STAT_AREA])*res*res
    if area<=1.5 and distance[piece].max()<=.90 and distance[piece].min()<=res*1.5:local|=piece
  local&=domain&~wall&(old>0)
  if not local.any():continue
  label=int(old.max())+sid;labels[local]=label;mapping[str(label)]=sp['id'];num,components,stats,_=cv2.connectedComponentsWithStats(local.astype('uint8'),4);sp['floor_components'][str(f+1)]={'label':label,'component_count':num-1,'area_m2':float(local.sum()*res*res),'source_regions':touched};changed.append({'floor':f+1,'stairwell':sp['id'],'relabelled_cells':int(local.sum())})
 assert np.array_equal(wall,d['wall'])and np.array_equal(domain,d['domain']);assert np.array_equal(labels>0,old>0),'stair annotation must not fill unlabeled cells'
 np.savez_compressed(OUT/f'Q_楼梯独立空间_L{f+1}.npz',labels=labels,domain=domain,wall=wall,origin=origin,resolution=res);(OUT/f'L{f+1}_stair_label_mapping.json').write_text(json.dumps(mapping,ensure_ascii=False,indent=2))
(OUT/'stairspaces.json').write_text(json.dumps({'logical_stairspaces':specs,'changes':changed,'policy':'cross-floor logical ID shared; disjoint observed polygons preserved; no wall removal or unknown filling'},ensure_ascii=False,indent=2));print(json.dumps(changed,ensure_ascii=False),flush=True)
