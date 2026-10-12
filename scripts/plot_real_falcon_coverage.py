#!/usr/bin/env python3
"""Show actual simulated-ray coverage, remaining cells and FALCON motion."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', type=Path, required=True)
    args = parser.parse_args()
    result = json.loads((args.preview/'result.json').read_text())
    audit = json.loads((args.preview/'coverage_audit.json').read_text())
    snapshot = Path(result['snapshot'])
    meta = json.loads((snapshot/'metadata.json').read_text())
    arrays = np.load(snapshot/'voxel_map.npz')
    masks = np.load(args.preview/'coverage_audit_masks.npz')
    target, seen = masks['target_xy'], masks['seen_xy']
    origin = np.array(meta['origin_xyz_m']); resolution = meta['resolution_m']
    z_index = int(np.floor((1.1-origin[2])/resolution))
    occupied = arrays['raw_occupancy_zyx'][z_index] == 100
    ny,nx = target.shape
    extent = [origin[0],origin[0]+nx*resolution,origin[1],origin[1]+ny*resolution]
    rgba = np.zeros((ny,nx,4))
    rgba[seen] = [0.40,0.78,0.68,.45]
    rgba[target & ~seen] = [1.0,.55,.10,.85]
    rgba[occupied] = [.25,.29,.34,.80]
    traj = np.atleast_2d(np.loadtxt(args.preview/'falcon_trajectory.csv',delimiter=',',skiprows=1))
    boundary = json.loads(Path(result['boundary']).read_text())
    polygon = np.array(boundary['allowed_polygon_xy_m'])
    fig,ax = plt.subplots(figsize=(15,9),constrained_layout=True)
    ax.imshow(rgba,origin='lower',extent=extent,interpolation='nearest')
    closed = np.vstack((polygon,polygon[0]))
    ax.plot(*closed.T,color='#7c3aed',lw=1.6,label='approved boundary')
    for item in boundary.get('excluded_polygons',[]):
        p = np.array(item.get('polygon_xy_m',item) if isinstance(item,dict) else item)
        p = np.vstack((p,p[0]))
        ax.fill(*p.T,color='#ef4444',alpha=.2)
        ax.plot(*p.T,color='#dc2626',lw=1.5)
    ax.plot(traj[:,1],traj[:,2],color='#075985',lw=1.5,label='FALCON actual simulated motion')
    ax.scatter(*traj[0,1:3],c='#16a34a',s=60,label='start',zorder=5)
    ax.scatter(*traj[-1,1:3],c='#f59e0b',marker='*',s=110,label='end',zorder=5)
    for i in range(40,len(traj)-20,100):
        a,b = traj[i,1:3],traj[i+15,1:3]
        if np.linalg.norm(b-a) > .3:
            ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'->','color':'#075985','lw':1.4})
    ax.set_xlim(polygon[:,0].min()-1.5,polygon[:,0].max()+1.5)
    ax.set_ylim(polygon[:,1].min()-1.5,polygon[:,1].max()+1.5)
    ax.set_aspect('equal'); ax.grid(alpha=.15)
    ax.set_xlabel('world X (m)'); ax.set_ylabel('world Y (m)')
    ax.set_title(f"Offline FALCON | {audit['duration_s']:.1f} s | {audit['path_length_m']:.1f} m | sensor XY coverage {audit['observed_safe_xy_fraction']:.2%}")
    handles,labels = ax.get_legend_handles_labels()
    handles += [Patch(color='#66c7ad',alpha=.6,label='observed safe XY'),
                Patch(color='#ff8c19',label='remaining safe XY'),
                Patch(color='#404a57',label='occupancy slice z=1.1 m'),
                Patch(color='#ef4444',alpha=.4,label='no-fly zone')]
    ax.legend(handles=handles,loc='lower left',fontsize=8)
    ax.text(.99,.01,'Coverage: actual ray traversals in z=0.8..1.5 m\nNot a 3-D frontier-completion claim',
            transform=ax.transAxes,ha='right',va='bottom',fontsize=8,
            bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'})
    output = args.preview/'coverage_preview.png'
    fig.savefig(output,dpi=180)
    print(output)
    timeline_path = args.preview/'sensor_coverage.csv'
    if timeline_path.exists():
        timeline = np.atleast_2d(np.loadtxt(timeline_path,delimiter=',',skiprows=1))
        fig,ax = plt.subplots(figsize=(9,4),constrained_layout=True)
        ax.plot(timeline[:,0],100*timeline[:,1],color='#075985')
        ax.set(xlabel='time (s)',ylabel='observed safe XY (%)',ylim=(0,100))
        ax.grid(alpha=.2)
        fig.savefig(args.preview/'coverage_timeline.png',dpi=150)


if __name__ == '__main__':
    main()
