"""Wall-guided offline partition, masked watershed, no preserved old/new seed overlays."""
from pathlib import Path
import heapq,json
import numpy as np,cv2
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_guided';O.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,4,figsize=(19,15));report=[]
def watershed(distance,seeds,mask):
 labels=seeds.copy();cost=np.full(mask.shape,np.inf);q=[];h,w=mask.shape
 # Multi-source minimax flood restricted to non-wall cells. Heap tie order deterministic.
 for y,x in np.argwhere(seeds>0):cost[y,x]=-distance[y,x];heapq.heappush(q,(cost[y,x],int(y),int(x),int(seeds[y,x])))
 while q:
  c,y,x,r=heapq.heappop(q)
  if c!=cost[y,x] or labels[y,x]!=r:continue
  for yy,xx in [(y-1,x),(y+1,x),(y,x-1),(y,x+1)]:
   if yy<0 or xx<0 or yy>=h or xx>=w or not mask[yy,xx] or seeds[yy,xx]>0:continue
   v=max(c,-distance[yy,xx])+1e-8
   if v<cost[yy,xx]:cost[yy,xx]=v;labels[yy,xx]=r;heapq.heappush(q,(v,yy,xx,r))
 return labels
for f in range(1,4):
 for j,source in enumerate(['measured','official']):
  a=np.load(B/'surface_supported'/source/f'L{f}_rooms.npz');old=a['labels'];wall=a['wall_grid'];res=float(a['resolution']);raw=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0
  closed=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0
  enclosed=nd.binary_fill_holes(closed)&~closed
  domain=(a['domain']|enclosed)&~raw;dist=nd.distance_transform_edt(domain)*res;cores,n=nd.label(dist>.45);seed=np.zeros_like(old);count=0
  for k in range(1,n+1):
   m=cores==k
   if m.sum()*res**2>=.04:count+=1;seed[m]=count
  comp,n=nd.label(domain)
  for k in range(1,n+1):
   m=comp==k
   if not seed[m].any() and m.sum()*res**2>=.3:count+=1;seed[m&(dist>=dist[m].max()-.05)]=count
  labels=watershed(dist,seed,domain);merged=[]
  # Only enclosed islands with one neighbor and little wall support can merge.
  for _ in range(3):
   changed=False
   for rid in np.unique(labels):
    if not rid:continue
    region=labels==rid;ring=nd.binary_dilation(region)&~region;neighbor=np.unique(labels[ring]);neighbor=neighbor[(neighbor>0)&(neighbor!=rid)];wallfrac=float(raw[ring].mean())if ring.any()else 0
    touches_open=bool(np.any(ring&~domain&~raw))
    if len(neighbor)==1 and wallfrac<.2 and not touches_open:
     target=int(neighbor[0]);merged.append({'from':int(rid),'to':target,'area_m2':round(region.sum()*res**2,3),'wall_boundary_fraction':round(wallfrac,3)});labels[region]=target;changed=True
   if not changed:break
  ids=np.unique(labels);ids=ids[ids>0];new=np.zeros_like(labels)
  for k,rid in enumerate(ids,1):new[labels==rid]=k
  assert not np.any((new>0)&raw)
  dest=O/source;dest.mkdir(exist_ok=True);np.savez_compressed(dest/f'L{f}_rooms.npz',labels=new,domain=domain,wall_grid=wall,wall_barrier=raw,origin_xy=a['origin_xy'],resolution=res)
  report.append({'floor':f,'source':source,'regions_before':len(np.unique(old[old>0])),'regions_after':len(ids),'merged_unsupported_islands':merged,'wall_overlap_cells':0})
  for mode,l in enumerate([old,new]):
   ax=axs[f-1,2*j+mode];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.65;ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);y,x=np.where(wall);ax.scatter(-(a['origin_xy'][0]+(x+.5)*res),a['origin_xy'][1]+(y+.5)*res,s=.2,c='#a51b32',lw=0);ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f}楼 '+('实测'if j==0 else'官方几何')+(' 原多表面预览'if mode==0 else' 墙体引导重新分割'))
fig.suptitle('墙体引导分割：重新统一生成种子，禁止穿墙扩展，检查无墙支持的内部色岛',fontsize=14);fig.tight_layout(rect=[0,.03,1,.96]);fig.text(.5,.012,'墙缝闭合仅为离线房间轮廓假设，不改导航；漏墙、室外水平面和门洞分界仍待验证。无位置专用规则。',ha='center',fontsize=9);fig.savefig(O/'wall_guided_comparison.png',dpi=160)
(O/'audit.json').write_text(json.dumps({'uses_xy_rules':False,'uses_gt_for_measured':False,'wall_source':'same AE extracted candidate wall grid as prior preview; not silently substituted legacy wall run','domain':'own measured surface support plus bounded wall-enclosed regions','seed_source':'recomputed single distance-core pass; no old/new seed overlay','flood':'4-connected masked minimax watershed','limitations':['wall gaps <=.35m closed only for room extent hypothesis','wall evidence remains provisional','room IDs not semantic ground truth'],'results':report},ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
