"""Original second-floor room labels in the reference orthographic view."""
from pathlib import Path
import importlib.util,json,math,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PlotPath
from matplotlib.patches import PathPatch

OUT=Path(__file__).resolve().parent
SOURCE=OUT.parent/'single_row/render_figure.py'
spec=importlib.util.spec_from_file_location('partition_figure',SOURCE)
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
data,check=base.reproduce(2)
labels=data['labels'];resolution=float(data['resolution'])
origin=np.asarray(data['origin_xy'])
azimuth,elevation=-58.,26.
a,e=np.radians([azimuth,elevation])
projection=np.array([[-np.sin(a),np.cos(a)],
                     [-np.sin(e)*np.cos(a),-np.sin(e)*np.sin(a)]])

def vertices(mask):
    runs=base.rectangle_runs(mask)
    if not runs:return None
    polys=np.asarray(runs,float)
    world=polys*resolution+origin
    xy=world@projection.T
    return np.concatenate([xy,xy[:,:1]],axis=1).reshape(-1,2)

def draw(ax,mask,color,z,alpha=1,offset=(0,0),edge='none',lw=0):
    points=vertices(mask)
    if points is None:return
    points=points+offset
    codes=np.tile([PlotPath.MOVETO,PlotPath.LINETO,PlotPath.LINETO,
                   PlotPath.LINETO,PlotPath.CLOSEPOLY],len(points)//5)
    ax.add_patch(PathPatch(PlotPath(points,codes),facecolor=color,
        edgecolor=edge,linewidth=lw,alpha=alpha,antialiased=False,zorder=z))

footprint=data['constrained_domain']|data['wall_barrier']
extent=vertices(footprint)
low=extent.min(axis=0);high=extent.max(axis=0);span=high-low
pad=.06*max(span)
fig=plt.figure(figsize=(10,10*(span[1]+2*pad)/(span[0]+2*pad)),facecolor='white')
ax=fig.add_axes([0,0,1,1]);ax.set_aspect('equal');ax.axis('off')
ax.set_xlim(low[0]-pad,high[0]+pad);ax.set_ylim(low[1]-pad,high[1]+pad)
# A soft display-only shadow under the zero-thickness floor silhouette.
for i in range(16,0,-1):
    draw(ax,footprint,'#46515B',0,alpha=.0035,
         offset=(.004*i,-.012*i))
draw(ax,data['constrained_domain'],base.UNTYPED,1)
colors=base.room_colors(labels)
for rid,color in colors.items():draw(ax,labels==rid,color,2)
draw(ax,data['wall_barrier'],'#697783',4)
# Thin boundaries aid separation without changing any room's footprint.
import cv2
for rid in colors:
    contours,_=cv2.findContours((labels==rid).astype('uint8'),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if len(contour)<3:continue
        world=(contour[:,0,:]+.5)*resolution+origin
        xy=world@projection.T;xy=np.vstack([xy,xy[0]])
        ax.plot(xy[:,0],xy[:,1],color='#F7FAFC',lw=.65,zorder=3)
for suffix in ['png','pdf','svg']:
    fig.savefig(OUT/f'room_partition_L2_oblique.{suffix}',dpi=350,facecolor='white')
plt.close(fig)
metadata=dict(floor=2,azimuth_degrees=azimuth,elevation_degrees=elevation,
    projection='Orthographic, zero-thickness XY floor plane',
    reference='素材处理后/第一阶段/房间分割/场景00337_三层房间语义_浅色柔和阴影版.png',
    reference_parameters='素材处理后/第一阶段/房间分割/生成说明.md',
    source=str(base.BASE/'wall_structural_rooms/L2_constrained.npz'),
    source_sha256=hashlib.sha256((base.BASE/'wall_structural_rooms/L2_constrained.npz').read_bytes()).hexdigest(),
    all_labels_rendered=[int(x) for x in np.unique(labels) if x>0],
    colors={str(k):v for k,v in colors.items()},checks=check,
    display='Original latest room colors; neutral walls; subtle display-only shadow. No semantic category inference or geometry changes.')
(OUT/'provenance.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
print(OUT/'room_partition_L2_oblique.png')
