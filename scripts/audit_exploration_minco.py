#!/usr/bin/env python3
"""Check saved polynomials against an unchanged exploration geofence."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from matplotlib.path import Path as Polygon
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'real_fly/stage2_runtime/scripts'))
from saved_minco_artifact import read_trajectory
from build_super_collision_pcd import read_binary_xyz


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--mission-dir',type=Path,required=True)
    p.add_argument('--run-data',type=Path,required=True)
    a=p.parse_args(); d=a.mission_dir; root=a.run_data
    start,phases=read_trajectory(d/'final_minco.txt')
    points=np.vstack([v['points'] for v in phases]); samples=[]
    for first,second in zip(points[:-1],points[1:]):
        n=max(1,int(np.ceil(np.linalg.norm(second-first)/.01)))
        samples.extend(first+np.arange(n)[:,None]/n*(second-first))
    samples=np.vstack([samples,points[-1]])
    bp=root/'falcon_exploration_boundary.json'; boundary=json.loads(bp.read_text())
    legal=Polygon(boundary['allowed_polygon_xy_m']).contains_points(samples[:,:2])
    for poly in boundary['excluded_polygons']:
        legal &= ~Polygon(poly['polygon_xy_m']).contains_points(samples[:,:2])
    height=(samples[:,2]>=.5)&(samples[:,2]<=1.5)
    minimum=float(cKDTree(read_binary_xyz(d/'collision.pcd')).query(samples)[0].min())
    report={'boundary_sha256':hashlib.sha256(bp.read_bytes()).hexdigest(),
            'outside_or_no_fly_samples':int((~legal).sum()),'height_violations':int((~height).sum()),
            'z_range_m':[float(samples[:,2].min()),float(samples[:,2].max())],
            'minimum_collision_point_distance_m':minimum,
            'duration_s':sum(v['duration']+v['dwell'] for v in phases),
            'length_m':float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
            'passed':bool(legal.all() and height.all() and minimum>=.25),'sample_spacing_m':.01}
    (d/'exploration_geofence_audit.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    m=json.loads((root/'voxel_snapshot/metadata.json').read_text())
    g=np.load(root/'voxel_snapshot/voxel_map.npz');o=m['origin_xyz_m'];r=m['resolution_m']
    y,x=np.where(g['raw_occupancy_zyx'][int((1.1-o[2])//r)]>0)
    f,ax=plt.subplots(figsize=(14,9))
    ax.scatter(o[0]+(x+.5)*r,o[1]+(y+.5)*r,s=2,c='#aaaaaa',label='occupied at 1.1m')
    v=np.array(boundary['allowed_polygon_xy_m']);v=np.vstack([v,v[0]])
    ax.plot(v[:,0],v[:,1],c='purple',lw=2,label='approved boundary')
    for poly in boundary['excluded_polygons']:
        v=np.array(poly['polygon_xy_m']);ax.fill(v[:,0],v[:,1],c='red',alpha=.25,label='no-fly')
    ax.plot(points[:,0],points[:,1],c='#0066ff',lw=1.5,label='final MINCO')
    ax.scatter(*points[0,:2],s=60,c='green',label='start')
    ax.scatter(*points[-1,:2],s=60,c='orange',label='end')
    for i,phase in enumerate(phases):
        v=np.asarray(phase['points'][-1]);ax.text(v[0],v[1],str(i+1),fontsize=12)
    ax.set_aspect('equal');ax.legend();ax.set_title('Final MINCO exploration demo')
    f.savefig(d/'exploration_final_preview.png',dpi=160)
    if not report['passed']:raise SystemExit('FINAL GEOFENCE AUDIT FAILED: DO NOT EXECUTE')


if __name__=='__main__':main()
