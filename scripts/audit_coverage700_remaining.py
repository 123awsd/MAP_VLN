"""Offline remaining surfaces and historical viewpoint witnesses; never online GT."""
import argparse,json,itertools,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import habitat_sim
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'scripts'))
from habitat_scene_repairs import load_scene_repair,mesh_safe_fraction
parser=argparse.ArgumentParser();parser.add_argument('case',nargs='?',default='coverage_lifecycle_700_r1_20261016');parser.add_argument('--available-history-only',action='store_true');args=parser.parse_args()
base=root/'outputs/consistency_validation_20261015';g=base/'global_observability';out=base/('coverage700_remaining'if args.case=='coverage_lifecycle_700_r1_20261016'else args.case+'_remaining');out=out.with_name(out.name+'_available_history')if args.available_history_only else out;out.mkdir(exist_ok=True);a=np.load(g/'surface_witnesses.npz');p=a['points'];floor=a['floor'];exports=base/'coverage_surface_exports'/args.case;distancefile=out/'reference_distance.npz'
if distancefile.exists():d=np.load(distancefile)['distance_m']
else:
 parts=[]
 for f in sorted(exports.glob('chunk*.npz')):
  with np.load(f)as c:parts.append(c['points'])
 obs=np.concatenate(parts);print('building surface index',len(obs),flush=True);d=cKDTree(obs).query(p,workers=2)[0];np.savez_compressed(distancefile,distance_m=d);del obs,parts
names=json.loads((g/'witness_manifest.json').read_text())['cases'];missing_history=[n['case']for n in names if not(root/'outputs'/n['case']/'capture_trace.jsonl').exists()];
if missing_history and not args.available_history_only:raise FileNotFoundError('Missing historical captures: '+str(missing_history)+'; use --available-history-only for a conservative partial witness audit')
frames=[[json.loads(l)for l in(root/'outputs'/n['case']/'capture_trace.jsonl').open()]if(root/'outputs'/n['case']/'capture_trace.jsonl').exists()else[]for n in names];seen=a['depth_seen'];wi=a['witness_frame'];verified=np.zeros(len(p),bool);source=np.full(len(p),-1,int);witness=np.full(len(p),-1,int);clearcache={};S=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);hab=lambda q:origin+S.T@(np.asarray(q)-[0,0,1]);dirs=np.array([v for v in itertools.product([-1,0,1],repeat=3)if any(v)],float);dirs/=np.linalg.norm(dirs,axis=1)[:,None];cfg=habitat_sim.SimulatorConfiguration();scene=root/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb';cfg.scene_id=str(scene);cfg.scene_dataset_config_file=str(root/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json');cfg.enable_physics=True;cfg.create_renderer=False;agent=habitat_sim.agent.AgentConfiguration();agent.sensor_specifications=[]
with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[agent]))as sim:
 overlays=load_scene_repair(sim,root/'scene_repairs/00337_three_floor_shell/repair.json',scene)
 for ci in range(4):
  if not frames[ci]:continue
  for k in np.flatnonzero((d>.05)&seen[ci]&~verified):
   fi=int(wi[ci,k]);f=frames[ci][fi];h=hab(f['position']);key=(ci,fi)
   if key not in clearcache:
    clear=not f.get('preview_motion_blocked',False)
    for dr in dirs:
     if not clear:break
     if sim.cast_ray(habitat_sim.geo.Ray(h,S.T@dr),.085,0.).has_hits():clear=False
    if clear and fi>0:clear=mesh_safe_fraction(sim,hab(frames[ci][fi-1]['position']),h,radius=.07)>=.999
    clearcache[key]=bool(clear)
   if not clearcache[key]:continue
   v=hab(p[k])-h;length=np.linalg.norm(v);res=sim.cast_ray(habitat_sim.geo.Ray(h,v/length),float(length+.04),0.)
   if res.has_hits()and abs(float(res.hits[0].ray_distance)-length)<=.04:verified[k]=True;source[k]=ci;witness[k]=fi
  print('case',ci,'verified residual',int(verified.sum()),flush=True)
np.savez_compressed(out/'opportunity_witnesses.npz',verified=verified,source_case=source,witness_frame=witness);area=lambda m:round(float(a['weight_m2'][m].sum()),3);summary=[]
for radius in [.05,.08,.12]:
 for f in [0,1,2,3]:
  mask=(d>radius)&((floor==f)if f else True);summary.append({'threshold_m':radius,'floor':f,'unmatched_reference_m2':area(mask),'remaining_with_historical_mesh_checked_view_m2':area(mask&verified),'unresolved_m2':area(mask&~verified)})
tiles=np.floor(p[:,:2]/2).astype(int);regions=[]
for f,tx,ty in np.unique(np.column_stack((floor,tiles)),axis=0):
 mask=(d>.08)&(floor==f)&(tiles[:,0]==tx)&(tiles[:,1]==ty)
 if not mask.any():continue
 witnessids=np.flatnonzero(mask&verified);groups={}
 for k in witnessids:
  key=(int(source[k]),int(witness[k]));groups.setdefault(key,[]).append(int(k))
 best=sorted(groups,key=lambda key:len(groups[key]),reverse=True)[:3];views=[]
 for ci,fi in best:
  frame=frames[ci][fi];views.append({'case':names[ci]['case'],'frame':fi,'position':frame['position'],'quaternion':frame['orientation_xyzw'],'assigned_unique_witness_area_m2':round(len(groups[(ci,fi)])*.01,3)})
 regions.append({'floor':int(f),'x_range_m':[int(tx*2),int(tx*2+2)],'y_range_m':[int(ty*2),int(ty*2+2)],'remaining_m2':area(mask),'remaining_historical_opportunity_m2':area(mask&verified),'unresolved_m2':area(mask&~verified),'representative_witnesses':views})
regions.sort(key=lambda r:r['remaining_historical_opportunity_m2'],reverse=True);result={'case':args.case,'missing_history_cases':missing_history,'history_cases_available':[n['case']for n,f in zip(names,frames)if f],'opportunity_scope':'Only retained historical capture witnesses; removed history is not treated as evidence of no opportunity','surface_reference_scope':'three interior-height bands, sampled surface area not full building observable denominator','area_table':summary,'remaining_regions':regions,'checked_pose_count':len(clearcache),'limits':['Missing sparse endpoint match is not proof of never seen or final map missing.','Historical depth is only a clue; repaired first-hit and sampled radial/swept safety independently checked.','Checks are necessary conditions under discrete execution model, not exact body collision or online global path reachability.','Representative area groups use assigned witness per sample, not exhaustive maximum coverage.','No new exploration or sensor rendering; offline geometry only.']};(out/'remaining_summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(summary,indent=2))
