"""Plot measured-run Boxer OBBs; use original FALCON coordinates for geometry."""
from pathlib import Path
import csv,json,itertools,argparse
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
from matplotlib.patches import Polygon, Patch
R=Path(__file__).resolve().parents[1];O=R/'outputs/boxer/coverage700_measured_20261017';D=O/'detections/episode'
parser=argparse.ArgumentParser();parser.add_argument('--fixed',action='store_true');parser.add_argument('--run-dir',type=Path,default=O);parser.add_argument('--pipeline',default='Grounding DINO → SAM2深度前景 → BoxerNet');parser.add_argument('--room-dir',type=Path,default=R/'outputs/coverage700_wall_room_preview/manual_room305_boundary');args=parser.parse_args();O=args.run_dir;D=O/'detections/episode';suffix='_fixed' if args.fixed else '';
if args.fixed:
 O=O/'fusion_fixed';O.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
rows=list(csv.DictReader((D/f'boxer_3dbbs_fused{suffix}.csv').open()))
raw=list(csv.DictReader((D/'boxer_3dbbs.csv').open()))
cloud=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')
rooms=[np.load(args.room_dir/f'L{f}_constrained.npz') for f in (1,2,3)]
classes=sorted({a['name']for a in rows});colors={k:plt.get_cmap('tab20')(i%20)for i,k in enumerate(classes)}
support=json.loads((D/f'fusion_support{suffix}.json').read_text());objects=[]
for i,row in enumerate(rows,1):
 c=np.array([float(row[k+'_world_object'])for k in ('tx','ty','tz')]);size=np.array([float(row['scale_'+k])for k in 'xyz']);q=[float(row[k+'_world_object'])for k in ('qx','qy','qz','qw')]
 corners=(np.array(list(itertools.product((-.5,.5),repeat=3)))*size)@Rotation.from_quat(q).as_matrix().T+c
 xy=corners[:,:2];poly=xy[ConvexHull(xy).vertices]
 floor=int(np.digitize(c[2],[2.3,5.3]))+1;a=rooms[floor-1];ix,iy=np.floor((c[:2]-a['origin_xy'])/float(a['resolution'])).astype(int);label=int(a['labels'][iy,ix])if 0<=iy<a['labels'].shape[0]and 0<=ix<a['labels'].shape[1]else 0
 objects.append({'id':f'O{i:03d}','support_count':support[i-1]['support_count'],'raw_detection_count':support[i-1].get('raw_detection_count',support[i-1]['support_count']),'label':row['name'],'probability':float(row['prob']),'floor':floor,'room_id':f'{floor}-{label:02d}'if label else None,'room_assignment':'center_in_label_grid'if label else 'unassigned','center_xyz':c.tolist(),'size_xyz':size.tolist(),'quaternion_xyzw':q,'corners_xyz':corners.tolist(),'footprint_xy':poly.tolist()})

def panel(ax,f):
 a=rooms[f-1];l=a['labels'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.12
 ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
 p=cloud[f'p{f-1}'];ax.scatter(-p[:,0],p[:,1],c='#999999',s=.12,alpha=.28,rasterized=True,lw=0)
 for k in np.unique(l[l>0]):
  d=nd.distance_transform_edt(l==k);y,x=np.unravel_index(d.argmax(),d.shape);ax.text(-(a['origin_xy'][0]+(x+.5)*float(a['resolution'])),a['origin_xy'][1]+(y+.5)*float(a['resolution']),f'{f}-{k:02d}',ha='center',fontsize=8,color='#555555',bbox={'facecolor':'white','edgecolor':'none','alpha':.7,'pad':1})
 for obj in objects:
  if obj['floor']!=f:continue
  poly=np.array(obj['footprint_xy']);poly[:,0]*=-1;ax.add_patch(Polygon(poly,fill=False,color=colors[obj['label']],linewidth=1.4))
  x,y,_=obj['center_xyz'];t=ax.text(-x,y,obj['id'],fontsize=6,color=colors[obj['label']],ha='center');t.set_path_effects([patheffects.withStroke(linewidth=1.5,foreground='white')])
 ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',title=f'{f}楼 · {sum(v["floor"]==f for v in objects)}个融合框',xlabel='−X / m',ylabel='Y / m')
fig,axs=plt.subplots(1,3,figsize=(20,9))
for f,ax in enumerate(axs,1):panel(ax,f)
fig.suptitle('700秒实测记录｜Boxer物体三维框俯视投影＋点云＋房间分区\n'+args.pipeline+' → 多帧融合；框为模型预测，非真值',fontsize=15)
fig.legend(handles=[Patch(facecolor='none',edgecolor=colors[k],label=k) for k in classes],loc='lower center',ncol=8,fontsize=8);fig.tight_layout(rect=[0,.09,1,.92]);fig.savefig(O/'boxer_three_floors.png',dpi=190);plt.close(fig)
for f in (1,2,3):
 fig,ax=plt.subplots(figsize=(12,13));panel(ax,f);fig.suptitle('Boxer多帧融合物体框｜实测700秒');fig.tight_layout();fig.savefig(O/f'boxer_floor{f}.png',dpi=190);plt.close(fig)
fields=['id','support_count','label','probability','floor','room_id','room_assignment']
with (O/'object_index.csv').open('w') as fp:
 w=csv.DictWriter(fp,fieldnames=fields);w.writeheader();w.writerows([{k:v[k]for k in fields}for v in objects])
summary={'raw_3d_detections':len(raw),'fused_objects':len(objects),'class_counts':dict(Counter(v['label']for v in objects)),'floor_counts':dict(Counter(v['floor']for v in objects)),'unassigned_to_room':sum(v['room_id']is None for v in objects),'room_assignment':'center containment only, no fallback or room semantic inference','uses_gt':False,'depth_source':'original recorded depth; no new sensor rendering','fusion':{'iou':.2,'min_detections':4,'confidence':.45},'objects':objects}
(O/'boxer_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print({k:v for k,v in summary.items()if k!='objects'})
