"""Visualize exact committed measured partition stages; no algorithm changes or GT.
Reconstruct seeds and expansion and assert every final raw label matches saved data.
"""
from pathlib import Path
import ast,heapq,json
import numpy as np,cv2
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'stage_visualization';O.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
src=R/'scripts/segment_rooms_wall_guided.py';tree=ast.parse(src.read_text());fn=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='watershed');ns={'np':np,'heapq':heapq};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(src),'exec'),ns)
G=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz');data=[];records=[]
for f,g in enumerate([-.155,2.595,5.595],1):
 a=np.load(B/'surface_supported/measured'/f'L{f}_rooms.npz');prior=np.load(B/'measured'/f'L{f}_walls_rooms.npz');rawsaved=np.load(B/'wall_guided/measured'/f'L{f}_rooms.npz');review=np.load(B/'wall_guided/measured_review'/f'L{f}_rooms.npz');wall=a['wall_grid'].astype(bool);res=float(a['resolution']);org=a['origin_xy'];bar=cv2.morphologyEx(wall.astype('uint8'),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0;closure=cv2.morphologyEx(wall.astype('uint8'),cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;enclosure=nd.binary_fill_holes(closure)&~closure;domain=(a['domain']|enclosure)&~bar;dist=nd.distance_transform_edt(domain)*res;cc,n=nd.label(dist>.45);seeds=np.zeros_like(a['labels']);fallback=np.zeros_like(domain);count=0;seedrows=[]
 for k in range(1,n+1):
  m=cc==k
  if m.sum()*res**2>=.04:count+=1;seeds[m]=count;seedrows.append({'id':count,'type':'distance_core','seed_area_m2':float(m.sum()*res**2)})
 cc,n=nd.label(domain)
 for k in range(1,n+1):
  m=cc==k
  if not seeds[m].any() and m.sum()*res**2>=.3:
   count+=1;s=m&(dist>=dist[m].max()-.05);seeds[s]=count;fallback|=s;seedrows.append({'id':count,'type':'fallback','seed_area_m2':float(s.sum()*res**2)})
 expanded=ns['watershed'](dist,seeds,domain);premerge=expanded.copy();merges=[]
 for _ in range(3):
  changed=False
  for rid in np.unique(expanded):
   if not rid:continue
   m=expanded==rid;ring=nd.binary_dilation(m)&~m;neighbor=np.unique(expanded[ring]);neighbor=neighbor[(neighbor>0)&(neighbor!=rid)];fraction=float(bar[ring].mean())if ring.any()else 0;opened=bool(np.any(ring&~domain&~bar))
   if len(neighbor)==1 and fraction<.2 and not opened:
    target=int(neighbor[0]);expanded[m]=target;changed=True;merges.append({'from':int(rid),'to':target})
  if not changed:break
 final=np.zeros_like(expanded)
 for k,rid in enumerate(np.unique(expanded[expanded>0]),1):final[expanded==rid]=k
 assert np.array_equal(final,rawsaved['labels']),f'Floor {f} does not reproduce current algorithm'
 assert np.array_equal(bar,rawsaved['wall_barrier'])
 p=G[f'p{f-1}'];shown=p[(p[:,2]>g+.15)&(p[:,2]<g+2.2)]
 d={'f':f,'g':g,'points':shown,'wall':wall,'bar':bar,'original_domain':prior['observed_floor_domain'],'surface_domain':a['domain'],'enclosure':enclosure,'domain':domain,'dist':dist,'seed':seeds,'fallback':fallback,'final':review['labels'],'rawfinal':final,'org':org,'res':res,'excluded':(final>0)&(review['labels']==0)};data.append(d)
 np.savez_compressed(O/f'L{f}_stages.npz',wall=wall,original_domain=d['original_domain'],surface_domain=d['surface_domain'],wall_enclosure_additions=enclosure&~a['domain'],wall_barrier=bar,closure_added_cells=bar&~wall,partition_domain=domain,distance_m=dist,seeds=seeds,fallback_seeds=fallback,expanded_before_merge=premerge,raw_labels=final,display_labels=d['final'],excluded_fragments=d['excluded'],origin_xy=org,resolution=res)
 records.append({'floor':f,'exact_saved_raw_labels_match':True,'seed_records':seedrows,'merges':merges,'display_excluded_area_m2':round(d['excluded'].sum()*res**2,4)})
titles=['① 实测点云','② 墙体候选投影','③ 分割范围的形成','④ 补墙与可分割区域','⑤ 距离核心与种子','⑥ 当前分区与编号']
def showmask(ax,rgb):ax.imshow(rgb[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest')
def rgbbase(d):return np.ones((*d['wall'].shape,3))
def points(ax,d,color='#c2ccd2'):
 p=d['points'];ax.scatter(-p[:,0],p[:,1],s=.3,c=color,lw=0,rasterized=True)
def walls(ax,d):
 y,x=np.where(d['wall']);ax.scatter(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],s=.4,c='#a41d32',lw=0)
def paint(ax,d,step,zoom=False):
 if step==0:points(ax,d,'#426c86')
 if step==1:points(ax,d);walls(ax,d)
 if step==2:
  rgb=rgbbase(d);rgb[d['original_domain']]=[.72,.78,.82];rgb[d['surface_domain']&~d['original_domain']]=[.35,.72,.95];rgb[d['enclosure']&~d['surface_domain']]=[1.,.65,.2];showmask(ax,rgb);walls(ax,d)
 if step==3:
  rgb=rgbbase(d);rgb[d['domain']]=[.69,.86,.77];rgb[d['wall']]=[.72,.17,.24];rgb[d['bar']&~d['wall']]=[.1,.35,.95];showmask(ax,rgb)
 if step==4:
  rgba=plt.get_cmap('YlGnBu')(np.clip(d['dist']/1.2,0,1));rgba[:,:,3]=d['domain']*.65;showmask(ax,rgba);rgb=np.zeros((*d['wall'].shape,4));rgb[d['seed']>0]=[.9,.1,.35,1];rgb[d['fallback']]=[1,.55,0,1];showmask(ax,rgb)
  for rid in np.unique(d['seed']):
   if rid:
    m=d['seed']==rid;y,x=nd.center_of_mass(m);ax.text(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],str(rid),fontsize=7,ha='center',color='black',clip_on=True,bbox={'facecolor':'white','alpha':.65,'pad':.1,'edgecolor':'none'})
 if step==5:
  l=d['final'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;showmask(ax,rgba);walls(ax,d)
  for rid in np.unique(l):
   if rid:
    ds=nd.distance_transform_edt(l==rid);y,x=np.unravel_index(ds.argmax(),ds.shape);t=ax.text(-(d['org'][0]+(x+.5)*d['res']),d['org'][1]+(y+.5)*d['res'],f'{d["f"]}-{rid:02d}',fontsize=8,ha='center',weight='bold',clip_on=True);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 if zoom:
  # Target contour is a display guide only, never an algorithm input.
  m=(d['rawfinal']==2).astype(float);xx=-(d['org'][0]+(np.arange(m.shape[1])+.5)*d['res']);yy=d['org'][1]+(np.arange(m.shape[0])+.5)*d['res'];ax.contour(xx,yy,m,levels=[.5],colors=['#e86e00'],linewidths=[1.1]);ax.set(xlim=(-3.6,1.8),ylim=(3.3,9.6))
 else:ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5))
 ax.set_aspect('equal');ax.set_xlabel('−X / m');ax.set_ylabel('Y / m');ax.set_title(titles[step],fontsize=12)
legend='③灰：早期支持域；蓝：多表面补充；橙：墙围合新增。④绿：可分割域；红：墙候选；蓝：补墙。⑤粉：核心种子；橙：兜底种子。'
for d in data:
 fig,axs=plt.subplots(2,3,figsize=(16,13))
 for step,ax in enumerate(axs.flat):paint(ax,d,step)
 fig.suptitle(f'{d["f"]}楼：实测点云到分区的六步过程',fontsize=18);fig.tight_layout(rect=[0,.055,1,.95]);fig.text(.5,.022,legend+'\n点云仅为可视化裁切；计算掩码使用原有全高度输入。彩色域不等于地图FREE或已确认房间。',ha='center',fontsize=9);fig.savefig(O/f'L{d["f"]}_six_stages.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,3,figsize=(16,13))
for step,ax in enumerate(axs.flat):paint(ax,data[0],step,True)
fig.suptitle('1-02附近放大：橙色轮廓仅指示最终1-02位置，不参与任何计算',fontsize=15);fig.tight_layout(rect=[0,.055,1,.95]);fig.text(.5,.022,legend+'\n先看③是否纳入了不该分割的空间，再看④连接是否被封闭，最后看⑤在哪生成独立种子。',ha='center',fontsize=9);fig.savefig(O/'partition102_six_stages_zoom.png',dpi=180);plt.close(fig)
(O/'manifest.json').write_text(json.dumps({'source_run':'coverage_lifecycle_700_r1_20261016','algorithm_unchanged':True,'GT_used':False,'reverted_uncertainty_version_used':False,'stage_sources':['original_style_video/geometry.npz','measured/L*_walls_rooms.npz','surface_supported/measured/L*_rooms.npz','wall_guided/measured/L*_rooms.npz','wall_guided/measured_review/L*_rooms.npz'],'display':'-X/Y only','stage3_caveat':'early domain already includes historical close/fill; not raw depth FREE','records':records},ensure_ascii=False,indent=2));print('All three floors reproduced exactly; saved six-stage and 1-02 zoom figures.')
