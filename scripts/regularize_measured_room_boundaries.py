"""Measured-only, conservative wall-supported contour simplification trial."""
from pathlib import Path
import json
import cv2
import numpy as np
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_supported_contours';O.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
rows=[];data=[]
for f in (1,2,3):
 a=np.load(B/'independent_room_support'/f'L{f}_constrained.npz');old=a['labels'];bar=a['wall_barrier'].astype(bool);res=float(a['resolution']);tol=2*res
 near=nd.distance_transform_edt(~bar)*res<=tol+1e-8
 final=old.copy();proposed=np.zeros_like(bar);accepted=np.zeros_like(bar);details=[]
 for rid in np.unique(old[old>0]):
  m=old==rid;contours,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE);poly=np.zeros_like(old,np.uint8);nv=0
  for c in contours:
   simp=cv2.approxPolyDP(c,tol/res,True);nv+=len(simp);cv2.fillPoly(poly,[simp],1)
  removal=m&~poly.astype(bool);proposed|=removal;cut=removal&near;candidate=m&~cut
  safe=candidate.any() and nd.label(candidate)[1]<=nd.label(m)[1] and np.all(candidate[nd.distance_transform_edt(m)*res>.45])
  if safe:final[cut]=0;accepted|=cut
  details.append({'id':int(rid),'proposed_m2':float(removal.sum()*res**2),'removed_m2':float(cut.sum()*res**2)if safe else 0.,'topology_check':bool(safe),'polygon_vertices':nv})
 assert np.all((final==0)|(final==old)) and not np.any((final>0)&bar) and not np.any(accepted&~near)
 np.savez_compressed(O/f'L{f}_rooms.npz',labels=final,original_labels=old,wall_barrier=bar,wall_grid=a['measured_wall'],proposed_removal=proposed,accepted_removal=accepted,origin_xy=a['origin_xy'],resolution=res)
 row={'floor':f,'removed_m2':float(accepted.sum()*res**2),'unsupported_proposal_m2':float((proposed&~accepted).sum()*res**2),'regions_before':len(np.unique(old[old>0])),'regions_after':len(np.unique(final[final>0])),'rooms':details};rows.append(row);data.append((f,a,final,proposed,accepted));print({k:v for k,v in row.items()if k!='rooms'},flush=True)
def draw(ax,f,a,l,title,overlay=None):
 org=a['origin_xy'];res=float(a['resolution']);h,w=l.shape;extent=[-(org[0]+w*res),-org[0],org[1],org[1]+h*res]
 rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.65;ax.imshow(rgba[:,::-1],origin='lower',extent=extent,interpolation='nearest')
 if overlay is not None:
  y,x=np.where(overlay);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=3,c='#ff7f00',lw=0)
 y,x=np.where(a['measured_wall']);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=.2,c='#92352d',lw=0)
 for rid in np.unique(l[l>0]):
  d=nd.distance_transform_edt(l==rid);y,x=np.unravel_index(d.argmax(),d.shape);t=ax.text(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,f'{f}-{rid:02d}',ha='center',fontsize=8,clip_on=True);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 ax.set(title=title,aspect='equal',xlim=(-10.5,8.5),ylim=(-2.5,18.5),xlabel='−X / m',ylabel='Y / m')
fig,axs=plt.subplots(3,2,figsize=(13,18))
for f,a,final,_,_ in data:
 draw(axs[f-1,0],f,a,a['labels'],f'{f}楼：规整前');draw(axs[f-1,1],f,a,final,f'{f}楼：保守规整后')
fig.suptitle('墙支持轮廓试验：不扩张、不填未知、不跨墙');fig.tight_layout(rect=[0,.02,1,.97]);fig.savefig(O/'three_floors_before_after.png',dpi=160);plt.close(fig)
for f,a,final,proposed,accepted in data:
 fig,axs=plt.subplots(1,4,figsize=(18,6))
 for ax,l,t,mask in zip(axs,[a['labels']]*3+[final],['① 输入分区与墙','② 简化提案：橙色待删','③ 有墙支持的实际删减','④ 最终轮廓'],[None,proposed,accepted,None]):draw(ax,f,a,l,t,mask)
 fig.tight_layout();fig.savefig(O/f'L{f}_stages.png',dpi=160);plt.close(fig)
 if f==3:
  fig,axs=plt.subplots(1,3,figsize=(14,6))
  for ax,l,t,mask in zip(axs,[a['labels'],final,final],['规整前','规整后','橙色：实际删减'],[None,None,accepted]):
   draw(ax,f,a,l,t,mask);ax.set(xlim=(1.5,6.5),ylim=(3.5,6.8))
  fig.tight_layout();fig.savefig(O/'L3_305_zoom.png',dpi=180);plt.close(fig)
(O/'audit.json').write_text(json.dumps({'uses_GT':False,'coordinate_specific_rules':False,'status':'trial, not promoted','tolerance_m':.1,'constraints':['no expansion','removal near existing wall only','preserve connected components and original .45m cores','no hole filling or fallback seeds'],'limitations':['2D conservative simplification, not 3D wall reconstruction','no new door closure; unsupported protrusions retained','no semantic ground truth accuracy claim'],'floors':rows},ensure_ascii=False,indent=2))
