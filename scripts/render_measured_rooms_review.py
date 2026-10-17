"""Measured-only review; flag/remove narrow exterior proposals without room coordinates."""
from pathlib import Path
import json,numpy as np,cv2
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/wall_guided';dest=O/'measured_review';dest.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(1,3,figsize=(17,9));report=[]
for f,ax in enumerate(axs,1):
 a=np.load(O/'measured'/f'L{f}_rooms.npz');old=a['labels'];labels=old.copy();wall=a['wall_grid'];bar=a['wall_barrier'];org=a['origin_xy'];res=float(a['resolution']);closed=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;inside=nd.binary_fill_holes(closed)&~closed;removed=[];retained=[]
 for rid in np.unique(old):
  if not rid:continue
  m=old==rid;area=m.sum()*res**2;radius=nd.distance_transform_edt(m).max()*res;ring=nd.binary_dilation(m)&~m;openfrac=(ring&~a['domain']&~bar).sum()/max(1,ring.sum());enclosed=inside[m].mean()
  # Narrow, small, unenclosed patches are uncertain exterior fragments, not rooms.
  reject=area<1. and radius<.3 and enclosed<.1
  info={'id':f'{f}-{int(rid):02d}','area_m2':round(area,3),'inradius_m':round(radius,3),'wall_enclosed_fraction':round(float(enclosed),3),'open_edge_fraction':round(float(openfrac),3)}
  if reject:labels[m]=0;removed.append(info)
  else:retained.append(info)
 rgba=plt.get_cmap('tab20')((labels%20)/19);rgba[:,:,3]=(labels>0)*.7;ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);y,x=np.where(wall);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=.4,c='#a51b32',lw=0)
 for r in retained:
  rid=int(r['id'].split('-')[1]);m=labels==rid;distance=nd.distance_transform_edt(m);y,x=np.unravel_index(distance.argmax(),distance.shape);t=ax.text(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,r['id'],fontsize=9,weight='bold',ha='center',va='center');t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',title=f'{f}楼｜实测分区',xlabel='−X / m',ylabel='Y / m');np.savez_compressed(dest/f'L{f}_rooms.npz',labels=labels,wall_grid=wall,origin_xy=org,resolution=res,excluded_fragment_mask=(old>0)&(labels==0));report.append({'floor':f,'excluded_uncertain_fragments':removed,'retained_partitions':retained})
fig.suptitle('700秒实测点云｜墙体引导分区与编号',fontsize=17);fig.tight_layout(rect=[0,.065,1,.95]);fig.text(.5,.025,'保持原分区编号；仅隐藏“小且窄、缺乏墙体包围”的疑似外侧碎片。\n未删除原始点云、墙体或探索地图；分区仍含走廊/楼梯，非语义房间真值。',ha='center',fontsize=10);fig.savefig(dest/'measured_three_floors_numbered.png',dpi=180)
(dest/'audit.json').write_text(json.dumps({'source_labels_unchanged':True,'measured_only':True,'uses_xy_rules':False,'policy':'exclude area<1m2 AND inradius<.3m AND wall-enclosed fraction<.1; retain original IDs','limitations':'heuristic uncertainty filter, not verified building outer contour; could exclude genuinely tiny unenclosed spaces','floors':report},ensure_ascii=False,indent=2));print(json.dumps([{'floor':x['floor'],'excluded':x['excluded_uncertain_fragments']} for x in report],ensure_ascii=False))
