"""Preview the automatic classifier input for one existing room."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects,patches
from PIL import Image
from scipy import ndimage as ndi

R=Path(__file__).resolve().parents[1]
O=R/'outputs/stage1_pipeline_flow_20261019/automated_room_semantics';O.mkdir(parents=True,exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
a=np.load(R/'outputs/coverage700_wall_room_preview/wall_structural_rooms/L2_constrained.npz')
l=a['labels'];org=a['origin_xy'];res=float(a['resolution']);mask=l==5
box=json.loads((R/'outputs/boxer/coverage700_sam31_20261018/fusion_fixed/boxer_summary.json').read_text())
objs=[v for v in box['objects'] if v.get('room_id')=='2-05']
verified=[]
for v in objs:
 c=np.array(v['center_xyz']);ij=np.floor((c[:2]-org)/res).astype(int)
 inside=0<=ij[0]<l.shape[1] and 0<=ij[1]<l.shape[0] and l[ij[1],ij[0]]==5
 verified.append({'id':v['id'],'label':v['label'],'confidence':v['probability'],'center_inside_auto_region':bool(inside)})
assert all(x['center_inside_auto_region'] for x in verified)
selected=json.loads((O/'selected_frames.json').read_text())['2-05']
views=[]
for record in selected:
 no=int(record['frame'][6:12])
 p=R/f'outputs/boxer/coverage700_measured_20261017/episode/frame_{no:06d}.npz'
 with np.load(p) as z:
  q=z['position'];ij=np.floor((q[:2]-org)/res).astype(int)
  inside=0<=ij[0]<l.shape[1] and 0<=ij[1]<l.shape[0] and l[ij[1],ij[0]]==5
  assert inside
  views.append({'frame':no,'position_xyz_m':q.tolist(),'camera_in_auto_region':True})

BG='#F8F7F3';INK='#172237';BLUE='#377DFF';PINK='#E951A2';YELLOW='#FDBB47'
fig=plt.figure(figsize=(16,9),facecolor=BG)
fig.text(.035,.935,'房间 2-05  |  多视角与物体证据',fontsize=24,color=INK,fontweight='bold')
fig.text(.036,.892,'二楼 · 自动几何分区 · 700 秒实测 RGB · SAM3.1 + Boxer 融合物体',fontsize=11,color='#647086')
for i,record in enumerate(selected):
 no=int(record['frame'][6:12])
 ax=fig.add_axes([.036+i*.313,.365,.295,.465])
 ax.imshow(Image.open(record['image']));ax.axis('off')
 ax.add_patch(patches.Rectangle((0,0),639,479,fill=False,edgecolor='white',lw=5))
 fig.text(.043+i*.313,.837,f'视角 {i+1}  ·  frame {no}',fontsize=10.5,color=INK,fontweight='bold')

ax=fig.add_axes([.685,.34,.28,.49]);ax.set_facecolor('white')
y,x=np.where(mask);xmin,xmax=x.min(),x.max();ymin,ymax=y.min(),y.max();pad=20
x0=max(0,xmin-pad);x1=min(l.shape[1],xmax+pad);y0=max(0,ymin-pad);y1=min(l.shape[0],ymax+pad)
sl=l[y0:y1,x0:x1];rgb=np.ones((*sl.shape,3));rgb[sl>0]=[.9,.91,.94];rgb[sl==5]=[1,.72,.82]
ax.imshow(rgb[:,::-1],origin='lower',extent=[-(org[0]+x1*res),-(org[0]+x0*res),org[1]+y0*res,org[1]+y1*res],interpolation='nearest')
wall=a['measured_wall'][y0:y1,x0:x1];yy,xx=np.where(wall);ax.scatter(-(org[0]+(xx+x0+.5)*res),org[1]+(yy+y0+.5)*res,s=.4,c='#4F6075',lw=0)
for v in objs:
 c=v['center_xyz'];ax.plot(-c[0],c[1],marker='o',ms=5.5,color=BLUE,mec='white',mew=.7)
for v in views:
 q=v['position_xyz_m'];ax.plot(-q[0],q[1],marker='^',ms=8,color=YELLOW,mec=INK,mew=.6)
ax.set(aspect='equal',xlabel='−X / m',ylabel='Y / m',title='同一自动分区与相机位置');ax.tick_params(labelsize=8)

fig.text(.038,.284,'Boxer 关联物体',fontsize=13.5,color=INK,fontweight='bold')
from collections import Counter
counts=Counter(v['label'] for v in objs)
translation={'bed':'床','wardrobe':'衣柜','lamp':'灯','cabinet':'柜','door':'门'}
chips=[f'{translation.get(k,k)} × {n}' for k,n in counts.items()]
for j,t in enumerate(chips):
 xx=.038+j*.105
 fig.add_artist(patches.FancyBboxPatch((xx,.220),.095,.045,transform=fig.transFigure,boxstyle='round,pad=.003,rounding_size=.014',fc=[BLUE,'#875DFF',YELLOW,'#27BA9C',PINK][j%5],ec='none'))
 fig.text(xx+.0475,.2425,t,ha='center',va='center',fontsize=10,color='white',fontweight='bold')
results=O/'semantic_results.json'
if results.exists():
 decisions=json.loads(results.read_text())['decisions'];match=next((x for x in decisions if x['room_id']==205),None)
else:match=None
if match:
 fig.text(.038,.142,f"自动模型输出：{match['room_type']}  ({match['confidence']:.2f})",fontsize=19,color=PINK,fontweight='bold')
else:
 fig.text(.038,.142,'自动模型输入已就绪 · 语义结果待运行',fontsize=17,color=PINK,fontweight='bold')
fig.text(.038,.083,'两张相机照片及 7 个物体中心均属于自动分区 2-05；本图没有人工赋予房间语义标签。',fontsize=9.5,color='#647086')
fig.savefig(O/'room_2_05_automation_preview.png',dpi=150,facecolor=BG,bbox_inches='tight',pad_inches=.12)
fig.savefig(O/'room_2_05_automation_preview.pdf',facecolor=BG,bbox_inches='tight',pad_inches=.12)
plt.close(fig)
(O/'local_validation.json').write_text(json.dumps({'room_id':'2-05','auto_room_source':'wall_structural_rooms/L2_constrained.npz','views':views,'boxer_objects':verified,'semantic_model_output':match,'uses_gt':False},ensure_ascii=False,indent=2))
print(O/'room_2_05_automation_preview.png')
