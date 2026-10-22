"""Three fixed surface sets; historical views vs scored/selected/executed views.
Offline first-hit tests do not establish online reachability. No controls.
"""
import json,re,bisect,sys,collections,math
from pathlib import Path
import numpy as np
import habitat_sim
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'scripts'))
from habitat_scene_repairs import load_scene_repair
out=root/'outputs/consistency_validation_20261015/global_observability';case=root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016';a=np.load(out/'surface_witnesses.npz');verified=np.load(out/'mesh_verified_opportunities.npz');geom=np.load(out/'actual_geometry_visibility.npz')['visible'];p=a['points'];mask=(a['distance_m']>.08)&~geom&verified['verified'];source=verified['source_case'];wi=a['witness_frame'];names=json.loads((out/'witness_manifest.json').read_text())['cases'];frames=[[json.loads(l)for l in(root/'outputs'/n['case']/'capture_trace.jsonl').open()]for n in names];t0=frames[0][0]['capture_wall_s'];events=[e for e in json.loads((case/'execution_events.json').read_text()) if t0<=e['wall_s']<=frames[0][-1]['capture_wall_s']];rounds=json.loads((case/'search_round_full_audit.json').read_text())['all_rounds'];starts=[r['start']['wall_s']for r in rounds]
text=re.sub(r'\x1b\[[0-9;]*m','',(case/'run.log').read_text(errors='replace'));pat=re.compile(r'\[(?:INFO|WARN|ERROR|DEBUG|FATAL)\]\s+\[(\d+\.\d+)\]:');marks=list(pat.finditer(text));sp=re.compile(r'\[ObservationDecision\] xyz=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\) gain=(\d+) confidence=([\d.]+) travel=([\d.]+) onward=([\d.]+) utility=([\d.]+)');scores=[]
for i,m in enumerate(marks):
 body=text[m.end():marks[i+1].start()if i+1<len(marks)else len(text)];q=sp.search(body)
 if not q:continue
 stamp=float(m[1]);
 if not t0<=stamp<=frames[0][-1]['capture_wall_s']:continue
 idx=bisect.bisect_right(starts,stamp)-1;r=rounds[idx]if idx>=0 else None;plan=r.get('plan')if r and stamp<=r.get('end',{}).get('wall_s',r['start']['wall_s'])else None
 scores.append({'relative_s':stamp-t0,'plan':plan,'position':[float(q[j])for j in range(1,4)],'gain':int(q[4]),'utility':float(q[8])})
S=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);hab=lambda q:origin+S.T@(np.asarray(q)-[0,0,1]);cfg=habitat_sim.SimulatorConfiguration();scene=root/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb';cfg.scene_id=str(scene);cfg.scene_dataset_config_file=str(root/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json');cfg.enable_physics=True;cfg.create_renderer=False;agent=habitat_sim.agent.AgentConfiguration();agent.sensor_specifications=[]
regions=[('floor3_south',3,[0,2],[-2,0]),('floor2_ordinary',2,[2,4],[4,6]),('floor2_room9_near',2,[-2,0],[10,12])];result=[];masksout={}
with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[agent]))as sim:
 overlays=load_scene_repair(sim,root/'scene_repairs/00337_three_floor_shell/repair.json',scene)
 for label,f,xr,yr in regions:
  ids=np.flatnonzero(mask&(a['floor']==f)&(p[:,0]>=xr[0])&(p[:,0]<xr[1])&(p[:,1]>=yr[0])&(p[:,1]<yr[1]));pts=p[ids]
  def visibility(pos,yaw=None,quat=None):
   d=pts-np.asarray(pos);rho=np.linalg.norm(d[:,:2],axis=1)
   if quat is not None:
    x,y,z,w=quat;r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);q=d@r;valid=(q[:,2]>.15)&(q[:,2]<=5)&(abs(q[:,0])<=q[:,2])&(abs(q[:,1])<=.75*q[:,2])
   elif yaw is not None:
    z=d[:,0]*np.cos(yaw)+d[:,1]*np.sin(yaw);x=d[:,0]*np.sin(yaw)-d[:,1]*np.cos(yaw);valid=(z>.15)&(z<=5)&(abs(x)<=z)&(abs(d[:,2])<=.75*z)
   else:valid=(rho>.15)&(rho<=5*np.sqrt(2))&(abs(d[:,2])<=.75*np.minimum(5,rho))
   good=np.zeros(len(pts),bool);h=hab(pos)
   for k in np.flatnonzero(valid):
    v=hab(pts[k])-h;length=np.linalg.norm(v);res=sim.cast_ray(habitat_sim.geo.Ray(h,v/length),float(length+.04),0.)
    good[k]=res.has_hits()and abs(float(res.hits[0].ray_distance)-length)<=.04
   return good
  hist=[]
  for ci,fi in sorted(set((int(source[k]),int(wi[source[k],k]))for k in ids)):
   frame=frames[ci][fi];v=visibility(frame['position'],quat=frame['orientation_xyzw']);hist.append({'case':names[ci]['case'],'frame':fi,'relative_s':frame['capture_wall_s']-frames[ci][0]['capture_wall_s'],'position':frame['position'],'orientation_xyzw':frame['orientation_xyzw'],'unique_missing_surface_m2':round(v.sum()*.01,3)})
  hist.sort(key=lambda h:h['unique_missing_surface_m2'],reverse=True)
  scorecache={};scored=[];scoreunion=np.zeros(len(ids),bool)
  for e in scores:
   key=tuple(e['position'])
   if key not in scorecache:scorecache[key]=visibility(key)
   v=scorecache[key]
   if not v.any():continue
   scoreunion|=v;scored.append({**e,'optimistic_visible_missing_m2':round(v.sum()*.01,3)})
  selected=[];selectedunion=np.zeros(len(ids),bool);executedunion=np.zeros(len(ids),bool);tasks={};exectasks={int(e['task_id'])for e in events if e['event']=='observation_executing'}
  for e in events:
   if e['event']!='observation_selected':continue
   v=visibility(e['goal'],yaw=e['yaw'])
   if not v.any():continue
   selectedunion|=v;tasks[int(e['task_id'])]=v
   # Same task may be frozen to a different final goal: validate actual executing goal separately.
   selected.append({**e,'relative_s':e['wall_s']-t0,'offline_visible_missing_m2':round(v.sum()*.01,3)})
  executed=[]
  for e in events:
   if e['event']!='observation_executing':continue
   preceding=[s for s in events if s['event']=='observation_selected'and s['task_id']==e['task_id']and s['wall_s']<=e['wall_s']]
   if not preceding:continue
   chosen=preceding[-1];v=visibility(e['goal'],yaw=chosen['yaw'])
   if not v.any():continue
   executedunion|=v;executed.append({**e,'relative_s':e['wall_s']-t0,'offline_visible_missing_m2':round(v.sum()*.01,3),'yaw_source':'latest preceding same-task selected; execution event omits yaw'})
  relevant={int(e['task_id'])for e in selected+executed};chain=[{**e,'relative_s':e['wall_s']-t0}for e in events if int(e.get('task_id',-1))in relevant and e['event'].startswith('observation_')]
  row={'region':label,'floor':f,'x':xr,'y':yr,'B_area_m2':round(len(ids)*.01,3),'best_historical_views':hist[:10],'historical_distinct_witness_poses':len(hist),'scored_optimistic_unknown_yaw_union_m2':round(scoreunion.sum()*.01,3),'selected_known_yaw_union_m2':round(selectedunion.sum()*.01,3),'executed_goal_potential_union_m2':round(executedunion.sum()*.01,3),'complete_score_records_with_optimistic_visibility':scored,'selected_visible_events':selected,'executed_visible_events':executed,'related_task_chain':chain};result.append(row);masksout[label+'_reference_indices']=ids;masksout[label+'_selected_visible']=selectedunion;masksout[label+'_executed_visible']=executedunion
  print(label,'B',row['B_area_m2'],'best',hist[0]if hist else None,'scores',len(scored),'selected',len(selected),'executed',len(executed),'areas',row['selected_known_yaw_union_m2'],row['executed_goal_potential_union_m2'],flush=True)
np.savez_compressed(out/'three_region_surface_masks.npz',**masksout)
(out/'three_region_opportunity_audit.json').write_text(json.dumps({'regions':result,'limits':['Scoring rows omit yaw and stable candidate identity; their visibility is an optimistic unknown-yaw bound, not proof of valid oriented viewpoints.','Selected-goal yaw has recorded identity; executing-goal yaw uses latest preceding selection of that task, not a full frozen yaw payload audit.','All visibility is offline geometry; none implies online ESDF or feasible planning at that time.','Historical best is best among saved per-surface witness poses, not all possible historical frames or global optimum.','Light logging has missing/interleaved score rows; no rows cannot prove no candidates.','The source surface sets are disjoint 2m tiles within interior-height bands.']},indent=2))
