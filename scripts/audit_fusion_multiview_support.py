#!/usr/bin/env python3
"""Compare fused boxes against exact member RGB-D and 2D proposals.

Image agreement/depth support are diagnostics, not ground-truth IoU or accuracy.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
from audit_boxer_depth_support import rotation
p=argparse.ArgumentParser(description=__doc__)
for name in ['episode','raw-boxes','proposals','baseline','baseline-audit','candidate','candidate-audit','output']:p.add_argument('--'+name,type=Path,required=True)
a=p.parse_args()
def read(path):
 with path.open()as f:return list(csv.DictReader(f))
raw=read(a.raw_boxes);proposals=read(a.proposals);baseline=read(a.baseline);candidate=read(a.candidate)
assert len(raw)==len(proposals)
assert all((b['time_ns'],b['name'])==(r['time_ns'],r['name'])for b,r in zip(raw,proposals))
b=json.loads(a.baseline_audit.read_text())['instances'];c=json.loads(a.candidate_audit.read_text())['instances'];owner={r:i['fused_index']for i in b for r in i['source_csv_row_indices']}
frames={}
for path in a.episode.glob('frame_*.npz'):
 with np.load(path)as f:frames[int(f['time_ns'])]=path
manifest=json.loads((a.episode/'manifest.json').read_text());fx,fy,cx,cy=[manifest[k]for k in ['fx','fy','cx','cy']]
def support(box,world,camera_pos,camera_R,u,v,roi):
 centre=np.array([float(box[k])for k in ['tx_world_object','ty_world_object','tz_world_object']]);size=np.array([float(box[k])for k in ['scale_x','scale_y','scale_z']]);q=[float(box[k])for k in ['qx_world_object','qy_world_object','qz_world_object','qw_world_object']]
 local=(world-centre)@rotation(q);outside=np.linalg.norm(np.maximum(np.abs(local)-size/2,0),axis=1);near=outside<=.15;mask=(u>=roi[0])&(u<=roi[2])&(v>=roi[1])&(v<=roi[3]);cam=(centre-camera_pos)@camera_R
 pixel=np.array([cx+fx*cam[0]/cam[2],cy+fy*cam[1]/cam[2]])if cam[2]>.01 else None
 return {'proposal_sampled_depth_points':int(mask.sum()),'proposal_depth_points_near_box':int((mask&near).sum()),'all_depth_points_near_box':int(near.sum()),'projected_centre_pixel':pixel.tolist()if pixel is not None else None,'projected_centre_inside_proposal':bool(pixel is not None and roi[0]<=pixel[0]<=roi[2] and roi[1]<=pixel[1]<=roi[3]),'proposal_centre_distance_normalized':float(np.linalg.norm(pixel-(roi[:2]+roi[2:])/2)/max(np.linalg.norm(roi[2:]-roi[:2]),1))if pixel is not None else None}
records=[]
for instance in c:
 idx=instance['fused_index'];parent={owner[r]for r in instance['source_csv_row_indices']}
 if len(parent)!=1:raise ValueError('Candidate joins multiple original clusters')
 parent=parent.pop();old=set(b[parent]['source_csv_row_indices'])
 if set(instance['source_csv_row_indices'])==old:continue
 for r in instance['source_csv_row_indices']:
  stamp=int(raw[r]['time_ns']);pr=proposals[r]
  with np.load(frames[stamp])as f:depth=f['depth_m'];pos=f['position'];R=rotation(f['orientation_xyzw'])
  v,u=np.mgrid[0:depth.shape[0]:4,0:depth.shape[1]:4];u=u.ravel();v=v.ravel();z=depth[v,u];valid=np.isfinite(z)&(z>0);u,v,z=u[valid],v[valid],z[valid];world=np.c_[(u-cx)*z/fx,(v-cy)*z/fy,z]@R.T+pos
  roi=np.array([float(pr[k])for k in ['x1','y1','x2','y2']])*np.array([depth.shape[1]/float(pr['img_width']),depth.shape[0]/float(pr['img_height'])]*2)
  records.append({'source_row':r,'frame':frames[stamp].name,'baseline_cluster':parent,'candidate_cluster':idx,'proposal_name':pr['name'],'baseline':support(baseline[parent],world,pos,R,u,v,roi),'candidate':support(candidate[idx],world,pos,R,u,v,roi)})
summary={}
for kind in ['baseline','candidate']:
 values=[r[kind]for r in records];dist=[v['proposal_centre_distance_normalized']for v in values if v['proposal_centre_distance_normalized']is not None]
 summary[kind]={'member_views':len(values),'projected_centres_inside_exact_proposal':sum(v['projected_centre_inside_proposal']for v in values),'zero_proposal_depth_support_views':sum(v['proposal_depth_points_near_box']==0 for v in values),'median_normalized_proposal_centre_distance':float(np.median(dist))if dist else None}
a.output.write_text(json.dumps({'changed_members':records,'summary':summary,'box_accuracy_proven':False,'limitations':'Same-frame proposal alignment and sampled depth diagnostics only. Glass/background and partial objects can distort support; class semantics and ground-truth 3D bounds remain unverified.'},indent=2)+'\n');print(summary)
