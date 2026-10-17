"""Feasibility prototype: boundary evidence states, controlled cases and current-run rays.
Not a joint room/topology estimator, doorway detector or novelty validation.
"""
from pathlib import Path
import json
import numpy as np,cv2
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'evidence_test_results';O.mkdir(exist_ok=True)
def classify(wall,free):
 wall=np.asarray(wall)>=3;free=np.asarray(free)>=3
 return np.where(wall&free,3,np.where(wall,1,np.where(free,2,0))).astype('uint8')
# Controlled observation-count tests use the same state classifier as real-data audit.
cases=[]
for name,w,v,expected in [('missing_wall_is_unknown',0,0,0),('wall_evidence_without_crossing',4,0,1),('repeated_free_evidence',0,4,2),('wall_free_conflict_not_silently_erased',4,4,3),('one_ray_is_insufficient',0,1,0)]:
 result=int(classify(w,v));assert result==expected;cases.append({'case':name,'passed':True,'state':result})
# Controlled two-room boundary hypothesis: unknown gap must remain undecided,
# not become a traversable adjacency merely because wall points are absent.
topology_cases=[]
for name,gap_wall,gap_free,expected in [('occluded_boundary',0,0,2),('supported_opening',0,4,1),('conflicting_opening',4,4,2),('single_ray_gap',0,1,2)]:
 boundary=np.zeros((60,80),bool);boundary[:,40]=True
 w=np.zeros(boundary.shape,int);v=np.zeros_like(w);w[boundary]=4
 w[25:35,40]=gap_wall;v[25:35,40]=gap_free
 state=classify(w,v);blocked=boundary&(state!=2)
 _,components=nd.label(~blocked)
 assert components==expected,(name,components)
 # The naive wall-only projection would merge when occluded gap lacks points.
 _,naive=nd.label(~(boundary&(w>=3)))
 topology_cases.append({'case':name,'components_with_evidence_gate':components,'naive_wall_only_components':naive,'expected':expected,'passed':True})
rays=np.load(B/'evidence_test/measured_free_rays.npz');pose=rays['poses'];font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(1,3,figsize=(16,8));rows=[]
for f,g in enumerate([-.155,2.595,5.595]):
 a=np.load(B/'wall_guided/measured'/f'L{f+1}_rooms.npz');old=np.load(B/'measured'/f'L{f+1}_small_room_preserved.npz');points=old['wall_points'];shape=a['labels'].shape;org=a['origin_xy'];res=float(a['resolution']);wall=a['wall_grid']>0
 # Count vertical height bands, not repeated cloud points, as geometric support.
 bands=np.zeros((18,*shape),np.uint8);xy=np.floor((points[:,:2]-org)/res).astype(int);zz=np.floor((points[:,2]-g)/.2).astype(int);ok=(xy[:,0]>=0)&(xy[:,0]<shape[1])&(xy[:,1]>=0)&(xy[:,1]<shape[0])&(zz>=0)&(zz<18);xy,zz=xy[ok],zz[ok];bands[zz,xy[:,1],xy[:,0]]=1
 support=sum(cv2.dilate(b,np.ones((3,3),np.uint8))for b in bands);support[~wall]=0
 visits=np.zeros(shape,np.uint16);viewsectors=np.zeros((8,*shape),bool);frames=0;origins=[]
 for i,pp in enumerate(pose):
  if not(g+.15<=pp[3]<g+2.75):continue
  origin=pp[1:4];endpoints=rays[f'r{i:04d}'];framehits=np.zeros(shape,bool)
  for endpoint in endpoints:
   delta=endpoint-origin;length=np.linalg.norm(delta)
   if length<=.25:continue
   # Stop 15cm before measured endpoint; full 3D interpolation preserved.
   ts=np.arange(0,max(0,1-.15/length),res/max(np.linalg.norm(delta[:2]),res));q=origin+ts[:,None]*delta;ij=np.floor((q[:,:2]-org)/res).astype(int);zb=np.floor((q[:,2]-g)/.2).astype(int);valid=(ij[:,0]>=0)&(ij[:,0]<shape[1])&(ij[:,1]>=0)&(ij[:,1]<shape[0])&(zb>=0)&(zb<18);ij,zb=ij[valid],zb[valid]
   if not len(ij):continue
   # Restrict this test to candidate wall cells with occupied height-band evidence.
   hit=wall[ij[:,1],ij[:,0]]&(bands[zb,ij[:,1],ij[:,0]]>0)
   ij=ij[hit];framehits[ij[:,1],ij[:,0]]=True
  if framehits.any():
   y,x=np.where(framehits);angles=np.arctan2(org[1]+(y+.5)*res-origin[1],org[0]+(x+.5)*res-origin[0]);sec=np.floor((angles+np.pi)/(2*np.pi)*8).astype(int)%8;viewsectors[sec,y,x]=True
  visits+=framehits;frames+=1;origins.append(origin)
 states=classify(support,visits);states[~wall]=0
 stable_conflict=wall&(states==3)&(viewsectors.sum(0)>=2)
 row={'floor':f+1,'ray_frames':frames,'wall_cells':int(wall.sum()),'wall_supported_cells':int((wall&(support>=3)).sum()),'crossed_wall_cells_at_least3_frames':int((wall&(visits>=3)).sum()),'conflicting_wall_cells':int((wall&(states==3)).sum()),'conflicting_cells_at_least2_view_sectors':int(stable_conflict.sum()),'conflicting_projection_area_m2':round((wall&(states==3)).sum()*res**2,4)};rows.append(row);print(row,flush=True)
 np.savez_compressed(O/f'L{f+1}_boundary_evidence.npz',states=states,wall_height_band_support=support,free_ray_frame_votes=visits,view_sector_count=viewsectors.sum(0),wall_grid=wall,origin_xy=org,resolution=res)
 ax=axs[f];lab=a['labels'];rgba=np.ones((*shape,3));rgba[lab>0]=[.85,.9,.92];rgba[wall]=[.25,.35,.65];rgba[wall&(states==3)]=[.9,.2,.1];ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',title=f'{f+1}楼：蓝墙体候选 / 红证据冲突',xlabel='−X / m',ylabel='Y / m')
fig.suptitle('当前700秒数据：墙面几何与多帧自由射线的证据一致性试验',fontsize=14);fig.tight_layout(rect=[0,.06,1,.94]);fig.text(.5,.025,'红色仅为待复核冲突，不等于门洞或应删除的墙。保留三维高度带、端点前15cm截断；未改变任何房间/导航结果。',ha='center',fontsize=9);fig.savefig(O/'boundary_evidence_feasibility.png',dpi=170)
(O/'feasibility_result.json').write_text(json.dumps({'status':'feasibility only; not innovation or accuracy acceptance','controlled_cases':cases,'controlled_topology_cases':topology_cases,'real_data':rows,'source_run':'coverage_lifecycle_700_r1_20261016','ground_truth_used':False,'source_ray_manifest':str(B/'evidence_test/measured_free_rays.manifest.json'),'limitations':['height-band counts are geometry support, not independent multi-view wall measurements','3D bands quantized at .2m and XY .05m; ray/wall conflicts need exact geometry verification','depth rays are sampled; absence of rays is not evidence of closed wall','no novel joint estimator implemented; no room accuracy or causal improvement claimed']},ensure_ascii=False,indent=2))
