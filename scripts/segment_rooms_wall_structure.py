"""Experimental wall-arrangement room partition, independent of watershed labels.
Measured wall segments + explicit virtual opening/corner separators form cells.
Cells are admitted by measured space and observed wall-facing evidence.
Virtual separators are NEVER occupancy or navigation obstacles.
"""
from pathlib import Path
import argparse,json,math
import numpy as np
import cv2
from scipy import ndimage as nd
from scipy.spatial import cKDTree


def axis_angle(u,v):
 return math.acos(float(np.clip(abs(u@v),0,1)))

def extract_segments(wall,res,origin,side=None):
 # Infer a dominant orthogonal frame only when measured wall normals support it.
 # This groups actual planes before rasterization, rather than accumulating many
 # near-duplicate Hough lines that split the same physical wall into slivers.
 if side is None:raise ValueError('Measured wall-side normals are required')
 cp=side['centers_xyz'][:,:2];norm=side['canonical_normal_xy'];coh=side['normal_coherence'];valid=coh>.8
 angles=np.arctan2(norm[:,1],norm[:,0]);four=np.mean(np.exp(4j*angles[valid]))
 if abs(four)<.9:raise ValueError('No reliable orthogonal wall frame; do not force Manhattan geometry')
 theta=float(np.angle(four)/4)
 wy,wx=np.where(wall);wp=origin+(np.column_stack([wx,wy])+.5)*res
 segments=[]
 for angle in (theta,theta+np.pi/2):
  n=np.array([np.cos(angle),np.sin(angle)]);u=np.array([-n[1],n[0]])
  compatible=valid&(abs(norm@n)>np.cos(np.deg2rad(10)))
  ids=np.flatnonzero(compatible);ids=ids[np.argsort(cp[ids]@n)]
  clusters=[]
  for idx in ids:
   value=float(cp[idx]@n)
   if not clusters or value-clusters[-1][0]>2*res:clusters.append((value,[idx]))
   else:clusters[-1][1].append(idx)
  for _,members in clusters:
   if len(members)<3:continue
   points=cp[members];offset=float(np.median(points@n));t=points@u
   selected=(abs(wp@n-offset)<=res)&(wp@u>=t.min()-.15)&(wp@u<=t.max()+.15)
   q=wp[selected]
   if len(q)<6:continue
   offset=float(np.median(q@n));tt=np.sort(q@u)
   if np.ptp(tt)<.5:continue
   gaps=[(offset*n+u*tt[k],offset*n+u*tt[k+1])for k in np.flatnonzero(np.diff(tt)>3*res)]
   segments.append({'a':offset*n+u*tt[0],'b':offset*n+u*tt[-1],'u':u,
                    'length':float(np.ptp(tt)),'support_gaps':gaps,
                    'halfwidth_m':float(np.quantile(abs(q@n-offset),.95)+res/2),
                    'normal_frame_concentration':float(abs(four)),
                    'normal_frame_angle_rad':theta,'measured_patch_count':len(members)})
 return segments


def structure_barriers(wall,res,origin,max_opening=1.2,side=None):
 seg=extract_segments(wall,res,origin,side);fitted=np.zeros(wall.shape,np.uint8);virtual=np.zeros_like(fitted);records=[]
 def line(mask,a,b,width=1):
  ij=np.rint((np.array([a,b])-origin)/res-.5).astype(int);cv2.line(mask,tuple(ij[0]),tuple(ij[1]),1,int(width))
 for si,s in enumerate(seg):
  width=max(1,int(np.ceil(2*s['halfwidth_m']/res)))
  line(fitted,s['a'],s['b'],width)
  for a,b in s.get('support_gaps',[]):
   gapmask=np.zeros_like(fitted);line(gapmask,a,b,width);fitted[gapmask>0]=0;virtual|=gapmask
   records.append({'source_wall':si,'other_wall':si,'kind':'unobserved_plane_span','a':a.tolist(),'b':b.tolist(),'length_m':float(np.linalg.norm(b-a))})
 # Explicit separators bridge only collinear wall ends, or support a junction to another wall.
 # They close a room boundary hypothesis, not a physical wall or a certified door.
 for i,s in enumerate(seg):
  for end,inside in ((s['a'],s['b']),(s['b'],s['a'])):
   outward=(end-inside)/np.linalg.norm(end-inside);best=None
   for j,t in enumerate(seg):
    if i==j:continue
    if axis_angle(s['u'],t['u'])<np.deg2rad(4):
     normal=np.array([-s['u'][1],s['u'][0]])
     for target in (t['a'],t['b']):
      delta=target-end;along=delta@outward;off=abs(delta@normal)
      if 0<along<=max_opening and off<=1.5*res:
       cand=(float(np.linalg.norm(delta)),target,'collinear_opening',j)
       if best is None or cand[0]<best[0]:best=cand
    else:
     mat=np.column_stack([outward,-t['u']])
     if abs(np.linalg.det(mat))<.3:continue
     along,other=np.linalg.solve(mat,t['a']-end)
     if 0<along<=max_opening and -2*res<=other<=t['length']+2*res:
      target=end+along*outward;cand=(float(along),target,'wall_junction',j)
      if best is None or cand[0]<best[0]:best=cand
   if best is not None and best[0]>res*.5:
    line(virtual,end,best[1]);records.append({'source_wall':i,'other_wall':best[3],'kind':best[2],'a':end.tolist(),'b':best[1].tolist(),'length_m':best[0]})
 return (wall.astype(bool)|(fitted>0)),virtual>0,seg,records


def classify_cells(barrier,space,side,origin,res,cells_override=None):
 cells,num=nd.label(~barrier) if cells_override is None else (cells_override,int(cells_override.max()));edge=np.unique(np.r_[cells[0],cells[-1],cells[:,0],cells[:,-1]])
 # Each wall patch votes only for the first cell immediately on its own side.
 # Camera-side evidence is directional; top surfaces alone cannot certify a room.
 vote_pos=np.zeros(num+1);vote_neg=np.zeros(num+1);patches_pos=np.zeros(num+1,int);patches_neg=np.zeros(num+1,int)
 cp=side['centers_xyz'][:,:2];norm=side['canonical_normal_xy'];views=side['depth_endpoint_view_frames'];coh=side['normal_coherence']
 for k in range(len(cp)):
  if coh[k]<.8:continue
  for signidx,sign in enumerate((1,-1)):
   cid=0
   for offset in np.arange(res, .35+res/2,res):
    xy=cp[k]+sign*norm[k]*offset;ij=np.floor((xy-origin)/res).astype(int)
    if not(0<=ij[0]<cells.shape[1] and 0<=ij[1]<cells.shape[0]):break
    v=cells[ij[1],ij[0]]
    if v:cid=int(v);break
   if not cid:continue
   same=float(views[k,signidx]);opposite=float(views[k,1-signidx])
   total=same+opposite
   if total<2:continue
   # One patch is one length-weighted observation, not one vote per camera frame.
   vote_pos[cid]+=same/total;vote_neg[cid]+=opposite/total
   patches_pos[cid]+=int(same>=2);patches_neg[cid]+=int(opposite>=2)
 labels=np.zeros_like(cells);rows=[];rid=0
 for cid in range(1,num+1):
  if cid in edge:continue
  m=cells==cid;area=m.sum()*res**2;frac=float(space[m].mean());p=vote_pos[cid];n=vote_neg[cid]
  # Majority evidence gate; no region area, width, convexity, seed or coordinate threshold.
  accepted=frac>.5 and p>n and patches_pos[cid]>=2
  if accepted:rid+=1;labels[m]=rid
  rows.append({'cell_id':cid,'label':rid if accepted else 0,'area_m2':round(area,4),'measured_space_fraction':frac,'inward_view_weight':float(p),'opposite_view_weight':float(n),'inward_observed_patches':int(patches_pos[cid]),'opposite_observed_patches':int(patches_neg[cid]),'accepted':bool(accepted),'reason':'accepted'if accepted else('insufficient_space_evidence'if frac<=.5 else'insufficient_inward_wall_evidence')})
 return labels,cells,rows


def merge_unsupported_cells(barrier,measured_wall,openings,origin,res,max_opening):
 """Merge bounded adjacent arrangement cells across long unsupported plane spans.
 Keep geometric outer hypotheses and short openings as explicit boundaries.
 No observed wall cell can be added to a merged room.
 """
 cells,num=nd.label(~barrier);edge=set(np.unique(np.r_[cells[0],cells[-1],cells[:,0],cells[:,-1]]).tolist());parent=np.arange(num+1);links=[];fill=[]
 def root(k):
  while parent[k]!=k:parent[k]=parent[parent[k]];k=parent[k]
  return k
 for op in openings:
  if op['kind']!='unobserved_plane_span' or op['length_m']<=max_opening:continue
  a=np.array(op['a']);b=np.array(op['b']);u=(b-a)/np.linalg.norm(b-a);n=np.array([-u[1],u[0]])
  for t in np.linspace(0,1,max(2,int(op['length_m']/res))):
   q=a*(1-t)+b*t;pair=[]
   for sign in (1,-1):
    cid=0
    for distance in np.arange(res,6*res,res):
     ij=np.floor((q+sign*n*distance-origin)/res).astype(int)
     if 0<=ij[0]<cells.shape[1] and 0<=ij[1]<cells.shape[0]:
      cid=int(cells[ij[1],ij[0]])
      if cid:break
    pair.append(cid)
   left,right=pair
   if not left or not right or left==right or left in edge or right in edge:continue
   rl,rr=root(left),root(right)
   if rl!=rr:parent[rr]=rl;links.append([left,right])
   for d in np.arange(-5*res,5.5*res,res):
    ij=np.floor((q+n*d-origin)/res).astype(int)
    if 0<=ij[0]<cells.shape[1]and 0<=ij[1]<cells.shape[0]and not measured_wall[ij[1],ij[0]]:fill.append((ij[1],ij[0],left,right))
 remap=np.array([root(k)for k in range(num+1)]);merged=remap[cells]
 for y,x,left,right in fill:
  if merged[y,x]==0 and root(left)==root(right):merged[y,x]=root(left)
 ids=np.unique(merged);lookup=np.zeros(num+1,int)
 for i,k in enumerate(ids[ids>0],1):lookup[k]=i
 return lookup[merged],links


def render(out,items):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib import font_manager,patheffects
 font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
 def panel(ax,d,mode):
  rgba=np.ones((*d['labels'].shape,4));l=d['labels'];extent=[-10.5,8.5,-2.5,18.5]
  if mode==0:
   p=d['points'];ax.scatter(-p[:,0],p[:,1],s=.2,c='#8ba4b1',lw=0)
  else:
   if mode==1:rgba[d['space']]=[.8,.9,.9,1]
   if mode==2:
    l=d['cells'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.55
   if mode==3:
    rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7
   ax.imshow(rgba[:,::-1],origin='lower',extent=extent,interpolation='nearest')
  y,x=np.where(d['wall']);ax.scatter(-(d['origin'][0]+(x+.5)*d['res']),d['origin'][1]+(y+.5)*d['res'],s=.35,c='#333333',lw=0)
  if mode in (1,2):
   y,x=np.where(d['virtual']);ax.scatter(-(d['origin'][0]+(x+.5)*d['res']),d['origin'][1]+(y+.5)*d['res'],s=1,c='#e98c21',lw=0)
  if mode==3:
   for rid in np.unique(l[l>0]):
    dist=nd.distance_transform_edt(l==rid);y,x=np.unravel_index(dist.argmax(),dist.shape);t=ax.text(-(d['origin'][0]+(x+.5)*d['res']),d['origin'][1]+(y+.5)*d['res'],f'{d["floor"]}-{rid:02d}',ha='center',fontsize=8,clip_on=True);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
  ax.set(aspect='equal',xlim=(-10.5,8.5),ylim=(-2.5,18.5),xlabel='−X / m',ylabel='Y / m',title=['① 实测点云与墙面','② 墙段＋开口边界假设（橙）','③ 边界形成的空间单元','④ 观测与墙面朝向支持的单元'][mode])
 fig,axs=plt.subplots(1,3,figsize=(18,8))
 for ax,d in zip(axs,items):panel(ax,d,3);ax.set_title(f'{d["floor"]}楼：墙结构划分')
 fig.suptitle('从墙结构生成空间单元：无分水岭生长，编号为新单元身份');fig.tight_layout(rect=[0,.03,1,.95]);fig.savefig(out/'three_floors_rooms.png',dpi=180);plt.close(fig)
 for d in items:
  fig,axs=plt.subplots(1,4,figsize=(20,7))
  for mode,ax in enumerate(axs):panel(ax,d,mode)
  fig.suptitle(f'{d["floor"]}楼：墙结构驱动的独立试验');fig.tight_layout(rect=[0,.03,1,.95]);fig.savefig(out/f'L{d["floor"]}_stages.png',dpi=160);plt.close(fig)


def main():
 parser=argparse.ArgumentParser();parser.add_argument('--max-opening',type=float,default=1.2);args=parser.parse_args()
 root=Path(__file__).resolve().parents[1];base=root/'outputs/coverage700_wall_room_preview';out=base/'wall_structure_partition';out.mkdir(exist_ok=True)
 geo=np.load(root/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz');reports=[];items=[]
 for f,g in enumerate((-.155,2.595,5.595),1):
  a=np.load(base/'measured'/f'L{f}_small_room_preserved.npz');wall=a['wall_grid']>0;origin=a['origin_xy'];res=float(a['resolution']);side=np.load(base/'wall_side_evidence'/f'L{f}_wall_sides.npz')
  # Reuse only measured observation feature rasters; NEVER prior labels, cores or untyped masks.
  e=np.load(base/'independent_room_support'/f'L{f}_constrained.npz');space=e['surface_protected']|e['free_protected']|e['ray_protected']|e['trajectory_protected']
  fitted,virtual,seg,openings=structure_barriers(wall,res,origin,args.max_opening,side)
  barrier=fitted|virtual
  arrangement,merges=merge_unsupported_cells(barrier,wall,openings,origin,res,args.max_opening)
  effective_barrier=arrangement==0
  labels,cells,rows=classify_cells(effective_barrier,space,side,origin,res,arrangement)
  assert not np.any((labels>0)&wall)
  # Opening graph: report incident units, do not merge them because connected physically.
  for op in openings:
   mid=(np.array(op['a'])+op['b'])/2;u=np.array(op['b'])-op['a'];u=u/np.linalg.norm(u);n=np.array([-u[1],u[0]]);adj=[]
   for sign in (1,-1):
    ij=np.floor((mid+sign*n*res*2-origin)/res).astype(int)
    adj.append(int(cells[ij[1],ij[0]])if 0<=ij[0]<cells.shape[1]and 0<=ij[1]<cells.shape[0]else 0)
   op['adjacent_cell_ids']=adj;op['is_navigation_wall']=False
  np.savez_compressed(out/f'L{f}_rooms.npz',labels=labels,cells=cells,wall_grid=wall,fitted_wall=fitted,virtual_separators=virtual,space_evidence=space,origin_xy=origin,resolution=res)
  report={'floor':f,'wall_segments':len(seg),'virtual_separators':len(openings),'merged_unsupported_cell_edges':len(merges),'accepted_units':len(np.unique(labels[labels>0])),'bounded_cells':len(rows),'cells':rows,'openings':openings,'segments':[{'a':s['a'].tolist(),'b':s['b'].tolist(),'length_m':s['length']}for s in seg]};reports.append(report);print({k:v for k,v in report.items()if k not in ('cells','openings','segments')},flush=True)
  p=geo[f'p{f-1}'];p=p[(p[:,2]>=g+.15)&(p[:,2]<g+2.2)];items.append({'floor':f,'labels':labels,'cells':cells,'wall':wall,'virtual':virtual,'space':space,'points':p,'origin':origin,'res':res})
 render(out,items)
 (out/'audit.json').write_text(json.dumps({'status':'independent experimental algorithm, not promoted','uses_GT':False,'uses_previous_room_labels_or_core':False,'uses_watershed':False,'uses_specific_room_or_xy_rules':False,'physical_map_changed':False,'max_opening_m':args.max_opening,'limitations':['opening separators are geometric hypotheses, not certified doors','wall-side observation absence is uncertain, not exterior proof','majority feature thresholds and geometric tolerances remain','wall extraction errors can create missing or extra bounded units'],'floors':reports},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
