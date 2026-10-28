"""Render a presentation flow chart from existing 700 s artifacts; no inference."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager, patches
from matplotlib.collections import LineCollection
from scipy import ndimage as ndi

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'outputs/stage1_pipeline_flow_20261019'
OUT.mkdir(parents=True, exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'svg.fonttype':'none'})
G=np.load(ROOT/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')
B=ROOT/'outputs/coverage700_wall_room_preview'
A=[np.load(B/'wall_structural_rooms'/f'L{i}_constrained.npz') for i in (1,2,3)]
OBJ=json.loads((ROOT/'outputs/boxer/coverage700_sam31_20261018/fusion_fixed/boxer_summary.json').read_text())['objects']

BG='#F9F7F2'; INK='#172237'; MUTED='#59667B'
ACC=['#FF5F6D','#8E5BFF','#24B995','#FFB547','#377DFF','#EB5FA9']
TITLES=['三维点云地图','楼层划分','墙体提取','房间分割','Boxer 物体提取','房间语义判定']
SUB=['700 s 实测重建','按高度分别处理','三维墙面 → 平面边界','观测支持的几何分区','SAM3.1 检测 → 3D Box','多视角 RGB + 房内物体']
TAGS=['MEASURED CLOUD','FLOOR-WISE','STRUCTURE','ROOM REGIONS','OBJECT INSTANCES','SEMANTIC INFERENCE']
fig=plt.figure(figsize=(24,8.2),facecolor=BG)
fig.text(.025,.942,'第一阶段  |  从探索地图到房间—物体结构',fontsize=28,fontweight='bold',color=INK)
fig.text(.026,.894,'同一轮 700 秒实测数据  ·  统一原始坐标  ·  几何提取与语义推断分开呈现',fontsize=12.2,color=MUTED)

left=.025; cardw=.148; gap=.016; cardy=.23; cardh=.60
for i in range(6):
 x=left+i*(cardw+gap)
 fig.add_artist(patches.FancyBboxPatch((x,cardy),cardw,cardh,transform=fig.transFigure,
      boxstyle='round,pad=0.008,rounding_size=0.018',fc='white',ec='#E4E5EA',lw=1.3,zorder=1))
 fig.add_artist(patches.FancyBboxPatch((x,cardy+cardh-.012),cardw,.013,transform=fig.transFigure,
      boxstyle='round,pad=0,rounding_size=0.005',fc=ACC[i],ec='none',zorder=2))
 fig.text(x+.012,cardy+cardh-.057,f'{i+1:02d}',fontsize=15,fontweight='bold',color=ACC[i])
 fig.text(x+.012,cardy+cardh-.105,TITLES[i],fontsize=16,fontweight='bold',color=INK)
 fig.text(x+.012,cardy+cardh-.139,SUB[i],fontsize=10.2,color=MUTED)
 fig.text(x+.012,cardy+.027,TAGS[i],fontsize=9.2,fontweight='bold',color=ACC[i])
 if i<5:
  axa=fig.add_axes([x+cardw+.003,cardy+.305,gap-.005,.05],zorder=5);axa.axis('off')
  axa.annotate('',xy=(1,.5),xytext=(0,.5),arrowprops=dict(arrowstyle='-|>',lw=2.2,color='#798395',mutation_scale=16))

# All 2-D previews use the established -X/Y display orientation. Geometry is untouched.
def map_ax(i):
 x=left+i*(cardw+gap)
 ax=fig.add_axes([x+.012,cardy+.102,cardw-.024,.345],zorder=3)
 ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5));ax.set_aspect('equal');ax.axis('off')
 return ax

def cloud(ax,f,step=5,color='#516D89',alpha=.7,size=.25):
 p=G[f'p{f-1}'][::step]
 g=[-.155,2.595,5.595][f-1]
 p=p[(p[:,2]>g+.15)&(p[:,2]<g+2.2)]
 ax.scatter(-p[:,0],p[:,1],c=color,s=size,alpha=alpha,lw=0,rasterized=True)

# 1: Real point cloud in a compact perspective view.
ax=fig.add_axes([left+.017,cardy+.11,cardw-.034,.335],projection='3d',zorder=3)
for f in (1,2,3):
 p=G[f'p{f-1}'][::55]
 ax.scatter(p[:,0],p[:,1],p[:,2],s=.45,c=[ACC[f-1]],alpha=.45,rasterized=True)
ax.view_init(elev=33,azim=-67);ax.set(xlim=(-8.5,10.5),ylim=(-2.5,18.5),zlim=(-.1,8));ax.set_axis_off()
ax.set_box_aspect((19,21,8),zoom=1.12)

# 2: Three real floor slices, offset visually only within the panel.
x=left+1*(cardw+gap)
for k,f in enumerate((3,2,1)):
 yy=cardy+.115+k*.105
 ax=fig.add_axes([x+.025,yy,.108,.095],zorder=3)
 p=G[f'p{f-1}'][::16]
 g=[-.155,2.595,5.595][f-1]
 p=p[(p[:,2]>g+.15)&(p[:,2]<g+2.2)]
 ax.scatter(-p[:,0],p[:,1],c=ACC[f-1],s=.43,alpha=.80,lw=0,rasterized=True)
 ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5));ax.set_aspect('equal');ax.axis('off')
 fig.text(x+.012,yy+.039,f'L{f}',fontsize=10,color=ACC[f-1],fontweight='bold')

# 3: Point cloud and actual extracted wall samples on floor 2.
ax=map_ax(2);cloud(ax,2,step=16,color='#A3B1C1',alpha=.5,size=.35)
w=np.load(B/'wall_structural_evidence/L2_walls.npz')['wall_points'][::15]
ax.scatter(-w[:,0],w[:,1],s=1.0,c=ACC[2],alpha=.95,lw=0,rasterized=True)

# 4: Automatic result, deliberately excluding the manual 305 correction.
ax=map_ax(3);a=A[1];labels=a['labels'];org=a['origin_xy'];res=float(a['resolution'])
rgba=plt.get_cmap('Set3')((labels%12)/12);rgba[:,:,3]=(labels>0)*.88
ax.imshow(rgba[:,::-1],origin='lower',extent=[-org[0]-labels.shape[1]*res,-org[0],org[1],org[1]+labels.shape[0]*res],interpolation='nearest')
for rid in np.unique(labels[labels>0]):
 mask=labels==rid;d=ndi.distance_transform_edt(mask);yy,xx=np.unravel_index(d.argmax(),d.shape)
 ax.text(-(org[0]+(xx+.5)*res),org[1]+(yy+.5)*res,str(rid),ha='center',va='center',fontsize=7,color=INK,fontweight='bold')

# 5: Actual fused rotated boxes, projected onto the same floor map.
ax=map_ax(4);cloud(ax,2,step=20,color='#B6C3CF',alpha=.45,size=.28)
for o in OBJ:
 if o.get('floor')!=2:continue
 q=np.asarray(o['corners_xyz'],float)
 # 3D OBB corners projected into the XY plane, preserving rotation.
 from scipy.spatial import ConvexHull
 xy=q[:,:2];h=ConvexHull(xy).vertices;v=xy[np.r_[h,h[0]]]
 ax.plot(-v[:,0],v[:,1],color=ACC[4],lw=.72,alpha=.84)

# 6: Three recorded views from the same measured room, plus its Boxer evidence.
# Camera positions are checked against the automatic L2 label grid.
x=left+5*(cardw+gap)
episode=ROOT/'outputs/boxer/coverage700_measured_20261017/episode'
room=A[1]['labels'];org=A[1]['origin_xy'];res=float(A[1]['resolution'])
for j,frame in enumerate((1280,1360)):
 with np.load(episode/f'frame_{frame:06d}.npz') as z:
  rgb=z['rgb'].copy();pos=z['position']
 ij=np.floor((pos[:2]-org)/res).astype(int)
 assert 0<=ij[0]<room.shape[1] and 0<=ij[1]<room.shape[0] and room[ij[1],ij[0]]==5
 ax=fig.add_axes([x+.012+j*.063,cardy+.267,.060,.157],zorder=3)
 ax.imshow(rgb);ax.axis('off')
 ax.add_patch(patches.Rectangle((0,0),rgb.shape[1]-1,rgb.shape[0]-1,fill=False,edgecolor='white',linewidth=3))
fig.text(x+.012,cardy+.251,'同一房间 2-05 · 两个实测视角',fontsize=8.8,color=MUTED)
fig.text(x+.012,cardy+.214,'Boxer 关联物体',fontsize=9.5,color=INK,fontweight='bold')
room_objects=[o['label'] for o in OBJ if o.get('room_id')=='2-05']
assert 'bed' in room_objects and 'wardrobe' in room_objects and 'lamp' in room_objects
for j,(label,color) in enumerate((('床',ACC[4]),('衣柜',ACC[1]),('灯',ACC[3]))):
 xx=x+.012+j*.041
 fig.add_artist(patches.FancyBboxPatch((xx,cardy+.165),.038,.036,transform=fig.transFigure,
      boxstyle='round,pad=0.001,rounding_size=0.01',fc=color,ec='none',zorder=3))
 fig.text(xx+.019,cardy+.183,label,ha='center',va='center',color='white',fontsize=8.5,fontweight='bold')
decision_file=OUT/'automated_room_semantics/semantic_results.json'
if decision_file.exists():
 decisions=json.loads(decision_file.read_text(encoding='utf-8'))['decisions']
 room_decision=next((item for item in decisions if item['room_id']==205),None)
else:
 room_decision=None
if room_decision:
 room_name={'bedroom':'卧室','bathroom':'卫生间','living_room':'客厅',
            'kitchen':'厨房','dining_room':'餐厅','unknown':'待定'}.get(room_decision['room_type'],room_decision['room_type'])
 fig.text(x+.012,cardy+.127,f'自动分类：{room_name}',fontsize=10.4,color=ACC[5],fontweight='bold')
 fig.text(x+.012,cardy+.104,f'房间 2-05 · 置信度 {room_decision["confidence"]:.2f}',fontsize=8.1,color=MUTED)
else:
 fig.text(x+.012,cardy+.127,'自动房间分类输入',fontsize=10.4,color=ACC[5],fontweight='bold')
 fig.text(x+.012,cardy+.104,'模型结果待实际运行验收',fontsize=8.1,color=MUTED)

fig.text(.025,.135,'实测输入与自动几何结果',fontsize=11.5,fontweight='bold',color=INK)
fig.text(.265,.135,'→',fontsize=14,color=ACC[0])
fig.text(.29,.135,'物体实例与房间几何关联',fontsize=11.5,fontweight='bold',color=INK)
fig.text(.575,.135,'→',fontsize=14,color=ACC[4])
fig.text(.60,.135,'语义层：房间多视角与物体证据联合推断',fontsize=11.5,fontweight='bold',color=INK)
fig.text(.025,.055,'数据：coverage_lifecycle_700_r1 · wall_structural_rooms（自动版） · SAM3.1 + Boxer。末格展示同房间实测照片、关联物体与自动分类结果。',fontsize=9.4,color=MUTED)
for ext in ('png','svg','pdf'):
 fig.savefig(OUT/f'stage1_pipeline_flow.{ext}',dpi=170,facecolor=BG,bbox_inches='tight',pad_inches=.16)
plt.close(fig)
print(OUT)
