#!/usr/bin/env python3
"""Area sampled official surfaces: exact recorded depth opportunity witnesses.
Read-only bags; no simulated observations and no exploration control.
"""
import json,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
root=Path(__file__).resolve().parents[1]; out=root/'outputs/consistency_validation_20261015/global_observability';out.mkdir(exist_ok=True)
ref=np.load(root/'outputs/consistency_validation_20261015/latest_official_comparison/official_sampled_surfaces.npz')['points']
grounds=[-.155,2.595,5.595]; masks=[(ref[:,2]>=g+.15)&(ref[:,2]<g+2.2)&(ref[:,0]>=-8.5)&(ref[:,0]<10.5)&(ref[:,1]>=-2.5)&(ref[:,1]<18.5) for g in grounds]
# Uniform deterministic Bernoulli thinning of the uniform area samples.
rng=np.random.default_rng(16016);ids=np.flatnonzero(np.logical_or.reduce(masks)&(rng.random(len(ref))<1/16)); p=ref[ids]; floor=np.select([m[ids] for m in masks],[1,2,3]); weight=np.full(len(p),.01)
actual=np.load(root/'outputs/consistency_validation_20261015/latest_dense_floor_topdown/dense_observed_surfaces.npz'); distances=np.empty(len(p))
for f in (1,2,3):
 sel=floor==f; distances[sel]=cKDTree(actual['floor%d'%f]).query(p[sel],workers=2)[0]
case_names=['consistency_A3_observation_lifecycle_504_final_20261016','consistency_R0_light_504_r1_20261015','consistency_R0_light_504_r2_20261015','consistency_A3_acceptance_504_r1_20261016']
# Include all sampled surfaces so the same witnesses support all thresholds.
tree=cKDTree(p); seen=np.zeros((4,len(p)),bool); witness=np.full((4,len(p)),-1,np.int32); delta=np.full((4,len(p)),np.inf,np.float32); frame_counts=[]; tstart=time.time()
for ci,name in enumerate(case_names):
 case=root/'outputs'/name; frames=[json.loads(l)for l in (case/'capture_trace.jsonl').open()]; poses={round(f['capture_wall_s'],6):(i,f)for i,f in enumerate(frames)};count=0; missing=0
 depths=np.load(out/('depth%d.npy'%ci),mmap_mode='r')
 for fi,f in enumerate(frames):
   pos=np.asarray(f['position']);idx=np.asarray(tree.query_ball_point(pos,7.1),dtype=int);idx=idx[~seen[ci,idx]]
   if len(idx):
    x,y,z,w=f['orientation_xyzw'];r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);q=(p[idx]-pos)@r; valid=(q[:,2]>.15)&(q[:,2]<=5.);idx=idx[valid];q=q[valid];u=np.rint(q[:,0]/q[:,2]*320+320).astype(int);v=np.rint(q[:,1]/q[:,2]*320+240).astype(int);valid=(u>=0)&(u<640)&(v>=0)&(v<480);idx=idx[valid];q=q[valid];u=u[valid];v=v[valid]
    d=depths[fi,v,u]*.001;err=np.abs(d-q[:,2]);valid=(d>.15)&(d<=5.)&(err<=.025);good=idx[valid];seen[ci,good]=True;witness[ci,good]=fi;delta[ci,good]=err[valid]
   count+=1
   if count%400==0: print(name,count,'witnesses',int(seen[ci].sum()),'seconds',round(time.time()-tstart),flush=True)
 export=json.loads((out/('export%d.json'%ci)).read_text());matched=int(export.get('exported_frames',export.get('exact_frames',0)))
 frame_counts.append({'case':name,'pose_records_scanned':count,'matched_depth_frames':matched,'unmatched_capture_records':count-matched})
 np.savez_compressed(out/'surface_witnesses.npz',points=p,floor=floor,weight_m2=weight,distance_m=distances,depth_seen=seen,witness_frame=witness,depth_error_m=delta,reference_indices=ids)
 (out/'witness_manifest.json').write_text(json.dumps({'cases':frame_counts,'area_density_per_m2':100,'sample_seed':16016,'depth_z_tolerance_m':.025,'maximum_optical_depth_m':5,'pixel_projection':'nearest integer pixel at full 640x480 resolution','limits':['A depth agreement witness is positive evidence; absence is not proof of no geometric visibility.','Historical depth witnesses demonstrate prior observations, not online FREE reachability or guaranteed current safe access.','Uniform area estimates are Monte Carlo estimates, not exact CAD areas.','Existing interior-height slices exclude floors and ceilings; this is not the entire building surface.']},indent=2))
print('DONE',flush=True)
