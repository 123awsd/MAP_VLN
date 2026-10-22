"""Verify historical opportunity witnesses against repaired geometry; no controls."""
import json,sys,itertools,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import habitat_sim
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'scripts'))
from habitat_scene_repairs import load_scene_repair,mesh_safe_fraction
out=root/'outputs/consistency_validation_20261015/global_observability';a=np.load(out/'surface_witnesses.npz');p=a['points'];seen=a['depth_seen'];wi=a['witness_frame'];dist=a['distance_m'];names=json.loads((out/'witness_manifest.json').read_text())['cases'];frames=[[json.loads(l)for l in(root/'outputs'/n['case']/'capture_trace.jsonl').open()]for n in names];S=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);hab=lambda q: origin+S.T@(np.asarray(q)-[0,0,1]);cfg=habitat_sim.SimulatorConfiguration();scene=root/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb';cfg.scene_id=str(scene);cfg.scene_dataset_config_file=str(root/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json');cfg.enable_physics=True;cfg.create_renderer=False;agent=habitat_sim.agent.AgentConfiguration();agent.sensor_specifications=[];verified=np.zeros(len(p),bool);source=np.full(len(p),-1,int);safe_cache={};linechecked=0;t=time.time()
dirs=np.array([v for v in itertools.product([-1,0,1],repeat=3)if any(v)],float);dirs/=np.linalg.norm(dirs,axis=1)[:,None]
with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[agent]))as sim:
 overlays=load_scene_repair(sim,root/'scene_repairs/00337_three_floor_shell/repair.json',scene)
 # Actual geometric visibility: all recorded camera poses, exact frustum and first hit.
 geom=seen[0].copy();gtree=cKDTree(p);geomframe=np.full(len(p),-1,int);actualray=0
 for fi,f in enumerate(frames[0]):
  pos=np.array(f['position']);ids=np.array(gtree.query_ball_point(pos,7.1),int);ids=ids[(dist[ids]>.05)&~geom[ids]]
  if not len(ids):continue
  x,y,z,w=f['orientation_xyzw'];r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);q=(p[ids]-pos)@r;keep=(q[:,2]>.15)&(q[:,2]<=5)&(abs(q[:,0])<q[:,2])&(abs(q[:,1])<.75*q[:,2]);ids=ids[keep];h=hab(pos)
  for k in ids:
   vec=hab(p[k])-h;length=np.linalg.norm(vec);result=sim.cast_ray(habitat_sim.geo.Ray(h,vec/length),float(length+.04),0.0);actualray+=1
   if result.has_hits() and abs(float(result.hits[0].ray_distance)-length)<=.025:geom[k]=True;geomframe[k]=fi
  if fi%500==0:print('actual geometry',fi,int((geom&~seen[0]).sum()),actualray,flush=True)
 np.savez_compressed(out/'actual_geometry_visibility.npz',visible=geom,witness_frame=geomframe)
 for ci in range(1,4):
  ids=np.flatnonzero((dist>.05)&~seen[0]&seen[ci]&~verified)
  for k in ids:
   fi=int(wi[ci,k]);key=(ci,fi);f=frames[ci][fi];h=hab(f['position'])
   if key not in safe_cache:
    clear=True
    for v in dirs:
     result=sim.cast_ray(habitat_sim.geo.Ray(h,S.T@v),.085,0.0)
     if result.has_hits():clear=False;break
    if clear and fi>0:
     prev=hab(frames[ci][fi-1]['position']);clear=mesh_safe_fraction(sim,prev,h,radius=.07)>=.999
    safe_cache[key]=bool(clear and not f.get('preview_motion_blocked',False))
   if not safe_cache[key]:continue
   target=hab(p[k]);v=target-h;length=np.linalg.norm(v);result=sim.cast_ray(habitat_sim.geo.Ray(h,v/length),float(length+.04),0.0);linechecked+=1
   if result.has_hits() and abs(float(result.hits[0].ray_distance)-length)<=.04:verified[k]=True;source[k]=ci
  print('verified case',ci,int(verified.sum()),'elapsed',round(time.time()-t),flush=True)
np.savez_compressed(out/'mesh_verified_opportunities.npz',verified=verified,source_case=source)
(out/'mesh_verification.json').write_text(json.dumps({'actual_geometry_rays':actualray,'actual_geometry_without_depth_witness_samples':int((geom&~seen[0]).sum()),'poses_checked':len(safe_cache),'poses_passed':sum(safe_cache.values()),'line_of_sight_checks':linechecked,'verified_samples':int(verified.sum()),'radius_m':.07,'clearance_probe_distance_m':.085,'visibility_endpoint_tolerance_m':.04,'limits':['26 radial probes plus preceding recorded segment using execution swept-ray contract are sampled necessary conditions, not an exact sphere-to-mesh distance or global path guarantee.','Only historical depth-witness poses were tested; no exhaustive viewpoint search.','Original mesh and all repair collision overlays were loaded; no renderer or exploration was run.']},indent=2))
