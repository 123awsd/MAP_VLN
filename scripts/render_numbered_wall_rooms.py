"""Number current geometric partitions and audit color between extracted walls."""
from pathlib import Path
import json,numpy as np,cv2
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/wall_guided'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,2,figsize=(13,19));records=[]
for f in range(1,4):
 for j,source in enumerate(['measured','official']):
  a=np.load(O/source/f'L{f}_rooms.npz');labels=a['labels'];wall=a['wall_grid'];bar=a['wall_barrier'];org=a['origin_xy'];res=float(a['resolution']);ax=axs[f-1,j]
  closed=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;interior=nd.binary_fill_holes(closed)&~closed
  rgba=plt.get_cmap('tab20')((labels%20)/19);rgba[:,:,3]=(labels>0)*.65;ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
  y,x=np.where(wall);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=.45,c='#a51b32',lw=0)
  for rid in np.unique(labels):
   if not rid:continue
   region=labels==rid;cc,n=nd.label(region);sizes=np.bincount(cc.ravel());sizes[0]=0;main=cc==sizes.argmax();dist=nd.distance_transform_edt(main);iy,ix=np.unravel_index(np.argmax(dist),dist.shape);px=-(org[0]+(ix+.5)*res);py=org[1]+(iy+.5)*res
   name=f'{f}-{int(rid):02d}';area=region.sum()*res**2;ring=nd.binary_dilation(region)&~region;open_fraction=float((ring&~a['domain']&~bar).sum()/max(1,ring.sum()));enclosed_fraction=float(interior[region].mean())
   # All are partition IDs, including narrow strips and corridors, not semantic rooms.
   text=ax.text(px,py,name,fontsize=8 if area>=1 else 6,ha='center',va='center',weight='bold',color='#111111',zorder=5);text.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
   records.append({'source':source,'floor':f,'display_id':name,'label_id':int(rid),'area_m2':round(area,3),'label_anchor_raw_xy_m':[-px,py],'components':n,'wall_closed_enclosure_fraction':round(enclosed_fraction,3),'open_domain_edge_fraction':round(open_fraction,3),'narrow_candidate':bool(dist.max()*res<.3),'interpretation':'geometric partition; room/corridor/exterior not semantically verified'})
  ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f}楼｜'+('700秒实测点云分区'if not j else'官方几何输入分区'))
fig.suptitle('墙体引导分区编号｜红色：提取墙体；编号：楼层−分区序号',fontsize=16);fig.tight_layout(rect=[0,.045,1,.965]);fig.text(.5,.018,'编号仅用于指认本图分区，不是历史房间编号；两组独立编号，不表示一一匹配。\n门口/走廊也可能被分配颜色；细长室外碎片和漏墙合并仍需剔除或修复，不能全部视为真实房间。',ha='center',fontsize=10);fig.savefig(O/'numbered_three_floors.png',dpi=170)
(O/'numbered_partitions.json').write_text(json.dumps({'source':'existing wall_guided labels, unchanged','wall_overlap_cells':0,'rows':records},ensure_ascii=False,indent=2));print(json.dumps([r for r in records if r['source']=='measured' and (r['narrow_candidate'] or r['area_m2']<1)],ensure_ascii=False))
