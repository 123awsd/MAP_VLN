"""Read-only comparison of archived occupancy and captured FALCON occupancy."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from run_pcd_falcon import load_environment, astar_grid

root = Path(__file__).resolve().parents[1]
p = root / 'real_fly/stage2_offline/data/lab_room_corrected_v1'
m, occ, esdf, origin, r, allowed, _, _, _ = load_environment(
    p/'voxel_snapshot', p/'approved_scene_graph.json', set(), p/'falcon_exploration_boundary.json')
base = root/'outputs/real_falcon/lab_room_corrected_v1'
data = json.loads((base/'exit_map_diagnostic_01.json').read_text())
zi = int((1.1-origin[2])/r)
walkable = allowed & (occ[zi] == 0) & (esdf[zi] >= .25)
cell = lambda x,y: (int((y-origin[1])/r), int((x-origin[0])/r))
path = astar_grid(cell(0,0), cell(-6,-3), walkable)
route = origin[:2] + (np.array(path)[:, ::-1]+.5)*r
fig, axes = plt.subplots(1,2,figsize=(13,6),sharex=True,sharey=True)
yy,xx = np.nonzero(occ[zi] == 100)
raw = origin[:2] + (np.column_stack((xx,yy))+.5)*r
axes[0].scatter(raw[:,0], raw[:,1], s=10,c='gray',label='Source occupied')
axes[0].plot(route[:,0],route[:,1],c='blue',label='Source collision-free route')
for key,color in [('free','lightgreen'),('occupied','red')]:
    pts=np.array(data[key]);pts=pts[np.abs(pts[:,2]-1.1)<.06]
    axes[1].scatter(pts[:,0],pts[:,1],s=8,c=color,label='FALCON '+key)
axes[1].plot(route[:,0],route[:,1],c='blue',alpha=.7,label='Same source route')
for ax,title in zip(axes,['Archived map z=1.1 m','Captured FALCON map z~1.1 m']):
    ax.scatter([0],[0],c='black',marker='*',s=140,label='Start')
    ax.scatter([2.1],[-.1],c='orange',marker='x',s=100,label='Exit route vicinity')
    ax.set(xlim=(-4,4),ylim=(-4,3),aspect='equal',title=title,xlabel='x (m)',ylabel='y (m)')
    ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.tight_layout()
out=base/'exit_map_comparison_01.png'
fig.savefig(out,dpi=150)
print(out)
