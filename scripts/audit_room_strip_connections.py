"""Trace existing strip-to-core raster connections and render rooms without untyped space.
Diagnostic windows never feed segmentation. No map or algorithm changes.
"""
from pathlib import Path
from collections import deque
import json
import numpy as np
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'core_visibility_ownership';D=O/'connection_audit';D.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False

def trace(mask,core,start):
 h,w=mask.shape;parent=np.full(mask.size,-1,np.int32);first=start[0]*w+start[1];parent[first]=first;q=deque([first]);end=None
 while q:
  p=q.popleft();y,x=divmod(p,w)
  if core[y,x]:end=p;break
  for yy,xx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
   if 0<=yy<h and 0<=xx<w and mask[yy,xx]:
    t=yy*w+xx
    if parent[t]<0:parent[t]=p;q.append(t)
 if end is None:return np.empty((0,2),int)
 path=[end]
 while path[-1]!=first:path.append(int(parent[path[-1]]))
 return np.array([divmod(p,w)for p in path[::-1]])

def baseplot(ax,a,labels,title,bounds=None):
 o=a['origin_xy'];r=float(a['resolution']);h,w=labels.shape;rgba=plt.get_cmap('tab20')((labels%20)/19);rgba[:,:,3]=(labels>0)*.7
 extent=[-(o[0]+w*r),-o[0],o[1],o[1]+h*r]
 ax.imshow(rgba[:,::-1],origin='lower',extent=extent,interpolation='nearest')
 wall=np.zeros((*labels.shape,4));wall[a['wall_grid']>0]=[.2,.2,.2,1];ax.imshow(wall[:,::-1],origin='lower',extent=extent,interpolation='nearest')
 ax.set(aspect='equal',title=title,xlabel='−X / m',ylabel='Y / m')
 if bounds:ax.set(xlim=bounds[:2],ylim=bounds[2:])

data={f:np.load(O/f'L{f}_rooms.npz')for f in (1,2,3)}
# Display-only: zero labels already represent untyped space; no additional exclusion.
fig,axs=plt.subplots(1,3,figsize=(19,8))
for f,ax in zip((1,2,3),axs):
 a=data[f];labels=a['labels'];baseplot(ax,a,labels,f'{f}楼：仅显示房间分区',(-10.5,8.5,-2.5,18.5))
 o=a['origin_xy'];r=float(a['resolution'])
 for rid in np.unique(labels[labels>0]):
  dd=nd.distance_transform_edt(labels==rid);yy,xx=np.unravel_index(dd.argmax(),dd.shape);t=ax.text(-(o[0]+(xx+.5)*r),o[1]+(yy+.5)*r,f'{f}-{rid:02d}',ha='center',fontsize=9);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
fig.suptitle('实测房间展示：待定细条不着色，墙体仍显示；未改变观测地图',fontsize=15);fig.tight_layout(rect=[0,.03,1,.95]);fig.savefig(O/'rooms_only_three_floors.png',dpi=180);plt.close(fig)
# These ROIs choose diagnostic examples only. No data label mutation here.
examples=[(3,8,(-.9,1.5,9.05,9.4),'3-06上方细条'),(3,4,(-.9,1.5,6.2,6.6),'3-04上方细条'),(2,6,(1.2,2.1,4.2,6.4),'2-06右侧带状区')]
records=[];fig,axs=plt.subplots(1,3,figsize=(18,7))
for ax,(f,rid,roi,title)in zip(axs,examples):
 a=data[f];original=a['original_labels'];o=a['origin_xy'];r=float(a['resolution']);yy,xx=np.indices(original.shape);X=-(o[0]+(xx+.5)*r);Y=o[1]+(yy+.5)*r
 target=(original==rid)&a['untyped']&(X>=roi[0])&(X<=roi[1])&(Y>=roi[2])&(Y<=roi[3]);mask=original==rid;core=a['core']&mask
 if not target.any():
  target=mask&a['untyped']
 if not target.any():continue
 # Use deepest point inside a removed component to avoid picking an edge by chance.
 td=nd.distance_transform_edt(target);start=np.unravel_index(td.argmax(),td.shape);path=trace(mask,core,start)
 wall=np.load(B/'independent_room_support'/f'L{f}_constrained.npz')['wall_barrier']
 assert len(path)>0 and not np.any(wall[path[:,0],path[:,1]])
 pts=o+(path[:,::-1]+.5)*r;clear=nd.distance_transform_edt(mask)*r
 # Straight segment to the connected core endpoint intersects wall or leaves original room.
 qs=np.linspace(path[0],path[-1],max(2,len(path)*4));ij=np.floor(qs+.5).astype(int);straight_wall=int(np.count_nonzero(wall[ij[:,0],ij[:,1]]))
 unique=np.unique(ij,axis=0);straight_wall_unique=int(np.count_nonzero(wall[unique[:,0],unique[:,1]]))
 rec={'floor':f,'original_room_id':rid,'title':title,'path_xy_m':pts.tolist(),'path_length_m':round((len(path)-1)*r,3),'path_wall_overlap_cells':0,'straight_to_same_core_wall_overlap_cells':straight_wall_unique,'minimum_grid_clearance_on_path_m':float(clear[path[:,0],path[:,1]].min()),'start_xy':pts[0].tolist(),'core_xy':pts[-1].tolist(),'note':'raster connectivity proof, not a physically traversable robot route; ROI only selects example'};records.append(rec)
 px=-pts[:,0];py=pts[:,1];bounds=(float(px.min()-.65),float(px.max()+.65),float(py.min()-.65),float(py.max()+.65));baseplot(ax,a,original,title,bounds)
 ax.plot(px,py,color='#00a4bb',lw=2,label='栅格连通路径');ax.plot(px[[0,-1]],py[[0,-1]],'--',color='#e44520',lw=1,label='直线被墙阻隔');ax.scatter(px[0],py[0],s=60,c='orange',zorder=4);ax.scatter(px[-1],py[-1],s=60,c='#da145d',zorder=4)
 ax.legend(fontsize=8);ax.text(.02,.02,f'绕行 {rec["path_length_m"]:.2f}m，穿墙栅格 0',transform=ax.transAxes,fontsize=9,bbox={'facecolor':'white','alpha':.8})
fig.suptitle('细条为什么被染色：墙挡住直线，分区仍可沿墙端绕行',fontsize=16);fig.tight_layout(rect=[0,.03,1,.95]);fig.savefig(D/'strip_connection_paths.png',dpi=180);plt.close(fig)
(D/'paths.json').write_text(json.dumps({'algorithm_changed':False,'display_hides_untyped_only':True,'examples':records},ensure_ascii=False,indent=2))
for rec in records:print({k:v for k,v in rec.items()if k!='path_xy_m'})
