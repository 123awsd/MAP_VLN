#!/usr/bin/env python3
"""Audit oriented boxes against exact-frame depth; no accuracy/IoU claims.

CSV format verified against BoxerNet: world centre, wxyz rotation, full local
axis lengths. Same-class 2D proposal union is context, not an instance match.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode',required=True,type=Path)
    p.add_argument('--boxes',required=True,type=Path)
    p.add_argument('--proposals',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--max-depth-m',type=float,default=None,
                   help='Optional range cutoff; default audits all valid depth supplied to Boxer')
    a=p.parse_args();manifest=json.loads((a.episode/'manifest.json').read_text())
    fx,fy,cx,cy=[manifest[k]for k in ('fx','fy','cx','cy')]
    proposals={}
    for row in csv.DictReader(a.proposals.open()):
        proposals.setdefault((int(row['time_ns']),row['name']),[]).append(row)
    boxes=list(csv.DictReader(a.boxes.open()));grouped={}
    for i,row in enumerate(boxes):grouped.setdefault(int(row['time_ns']),[]).append((i,row))
    results=[]
    for path in sorted(a.episode.glob('frame_*.npz')):
        with np.load(path) as f:
            stamp=int(f['time_ns']);depth=f['depth_m'];position=f['position'];R=rotation(f['orientation_xyzw'])
        if stamp not in grouped:continue
        v,u=np.mgrid[0:depth.shape[0]:4,0:depth.shape[1]:4];v,u=v.ravel(),u.ravel();z=depth[v,u];keep=np.isfinite(z)&(z>0)
        if a.max_depth_m is not None:keep&=z<=a.max_depth_m
        v,u,z=v[keep],u[keep],z[keep]
        camera=np.c_[(u-cx)*z/fx,(v-cy)*z/fy,z];world=camera@R.T+position
        for index,row in grouped[stamp]:
            centre=np.array([float(row[k])for k in ('tx_world_object','ty_world_object','tz_world_object')])
            scale=np.array([float(row[k])for k in ('scale_x','scale_y','scale_z')])
            q=np.array([float(row[k])for k in ('qx_world_object','qy_world_object','qz_world_object','qw_world_object')])
            norm=float(np.linalg.norm(q));valid=bool(np.isfinite(np.r_[centre,scale,q]).all()and(scale>0).all()and abs(norm-1)<1e-3)
            record={'row_index':index,'time_ns':stamp,'frame':path.name,'name':row['name'],'prob':float(row['prob']),'valid_box_format':valid,'quaternion_norm':norm,'scale_m':scale.tolist()}
            if valid:
                local=(world-centre)@rotation(q/norm);outside=np.linalg.norm(np.maximum(np.abs(local)-scale/2,0),axis=1);inside=outside<1e-8
                proposal_mask=np.zeros(len(world),bool)
                for pr in proposals.get((stamp,row['name']),[]):
                    sx=depth.shape[1]/float(pr['img_width']);sy=depth.shape[0]/float(pr['img_height'])
                    proposal_mask|=(u>=float(pr['x1'])*sx)&(u<=float(pr['x2'])*sx)&(v>=float(pr['y1'])*sy)&(v<=float(pr['y2'])*sy)
                camera_centre=(centre-position)@R
                record.update(sampled_depth_points=len(world),inside_depth_points=int(inside.sum()),depth_points_within_015m=int((outside<=.15).sum()),
                              same_class_proposal_depth_points=int(proposal_mask.sum()),proposal_depth_inside_box=int((proposal_mask&inside).sum()),
                              camera_centre_z_m=float(camera_centre[2]),centre_pixel=[float(cx+fx*camera_centre[0]/camera_centre[2]),float(cy+fy*camera_centre[1]/camera_centre[2])]if camera_centre[2]>.001 else None)
            results.append(record)
    missing=set(grouped)-{r['time_ns']for r in results}
    report={'format':'pre_map_vln.boxer_depth_support.v1','source_boxes':str(a.boxes),'episode':str(a.episode),
            'box_rows':len(boxes),'audited_rows':len(results),'missing_frame_stamps':sorted(missing),
            'maximum_audited_depth_m':a.max_depth_m,
            'invalid_format_rows':sum(not r['valid_box_format']for r in results),
            'zero_depth_support_within_015m_rows':sum(r.get('depth_points_within_015m',0)==0 for r in results),
            'rows':results,'semantic_accuracy_proven':False,
            'limitations':'Depth support is not object truth or box IoU; background can support wrong boxes. Visible partial objects can legitimately have incomplete support. Same-class 2D unions do not disambiguate duplicate instances.'}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items()if k!='rows'},indent=2))


if __name__=='__main__':main()
