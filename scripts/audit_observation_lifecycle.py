#!/usr/bin/env python3
"""Read-only task identity, actual arrival and integrated-depth outcome audit."""
import argparse,bisect,collections,json,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('case',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();events=json.loads((a.case/'execution_events.json').read_text());frames=[json.loads(l)for l in(a.case/'capture_trace.jsonl').open()];t0=frames[0]['capture_wall_s'];stamps=[f['capture_wall_s']for f in frames];arrivals={};settled=collections.Counter();errors=[];executing=[];active=None;cancelled=[];switches={e['id']:e for e in events if e['event']=='server_switch'};accepts={e['id']:e for e in events if e['event']=='server_accept'};candidate_by_traj={e['trajectory_id']:e for e in events if e['event']=='observation_candidate'};losses=[]
for e in events:
 kind=e['event']
 if kind=='observation_executing':
  tid=e['task_id'];tr=e['trajectory_id'];executing.append(e)
  if tr not in candidate_by_traj or candidate_by_traj[tr]['task_id']!=tid:errors.append({'reason':'missing_final_candidate_identity','event':e})
  if tr not in switches or abs(switches[tr]['wall_s']-e['wall_s'])>1:errors.append({'reason':'not_associated_with_real_switch','event':e})
  if tr not in accepts:errors.append({'reason':'no_server_acceptance','event':e})
  active=tid if e.get("phase",2)!=4 else None
 elif kind=='observation_arrived':
  tid=e['task_id'];arrivals[tid]=e
  if active!=tid:errors.append({'reason':'arrival_for_nonexecuting_task','event':e})
  if math.dist(e['actual'],e['goal'])>=.45 or abs(e['wall_s']-e['capture_stamp'])>.501:errors.append({'reason':'invalid_actual_arrival','event':e})
 elif kind=='observation_settled':
  tid=e['task_id'];settled[tid]+=1;arr=arrivals.get(tid)
  if active!=tid or not arr:errors.append({'reason':'settlement_without_active_arrival','event':e})
  if e['capture_stamp']-e['arrival_stamp']<.499999 or e['integrated_stamp']<=e['arrival_stamp'] or e['epoch']%2 or e['epoch']<=e['arrival_epoch']:errors.append({'reason':'settlement_without_later_integrated_data','event':e})
  if math.dist(e['actual'],e['goal'])>=.45 or abs(math.atan2(math.sin(e['actual_yaw']-e['goal_yaw']),math.cos(e['actual_yaw']-e['goal_yaw'])))>=.350001:errors.append({'reason':'not_at_real_observation_pose','event':e})
  if settled[tid]>1:errors.append({'reason':'duplicate_settlement','event':e})
  active=None
 elif kind=='observation_cancelled':
  cancelled.append(e)
  if active==e['task_id']:active=None
# Actual fixed-third-floor occupancy intervals, not merely height.
import numpy as np
ref=Path(__file__).resolve().parents[1]/'outputs/room_wall_guided_trial/adaptive_merge/L3/segmentation.npz';intervals=[]
if ref.exists():
 s=np.load(ref);xy=np.array([f['position'][:2]for f in frames]);ij=np.floor((xy-s['origin'])/float(s['resolution'])).astype(int);labels=s['labels'];valid=(ij[:,0]>=0)&(ij[:,0]<labels.shape[1])&(ij[:,1]>=0)&(ij[:,1]<labels.shape[0]);inside=np.zeros(len(frames),dtype=bool);inside[valid]=labels[ij[valid,1],ij[valid,0]]>=0;z=np.array([f['position'][2]for f in frames]);inside&=(z>5.745)&(z<7.795);edges=np.diff(np.r_[False,inside,False].astype(int))
 for b,en in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
  intervals.append({'first_s':frames[b]['capture_wall_s']-t0,'last_s':frames[en-1]['capture_wall_s']-t0,'frames':int(en-b),'observed_span_s':frames[en-1]['capture_wall_s']-frames[b]['capture_wall_s']})
r={'case':a.case.name,'checks_passed':not errors,'errors':errors,'event_counts':dict(collections.Counter(e['event']for e in events if e['event'].startswith('observation_'))),'settled_success':sum(e['event']=='observation_settled'and e['predicted']>0 and not e['failed']for e in events),'settled_empty_baseline':sum(e['event']=='observation_settled'and e['predicted']==0 for e in events),'settled_failure':sum(e['event']=='observation_settled'and e['failed']for e in events),'cancel_reasons':dict(collections.Counter(e['reason']for e in cancelled)),'third_floor_intervals':intervals,'cross_floor_execution_count':sum(e['cross_floor_intent']for e in executing),'executing_tasks':executing,'arrival_events':[e for e in events if e['event']=='observation_arrived'],'settlement_events':[e for e in events if e['event']=='observation_settled'],'cancellation_events':cancelled,'limits':['A cancellation record with final-target replacement is a reason category, not proof that the score comparison was optimal.','Actual arrival plus later integrated depth validates lifecycle ordering, not GT observation completeness.','Fixed third-floor intervals use offline reference segmentation only.']}
a.output.write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items()if k not in ['executing_tasks','arrival_events','settlement_events','cancellation_events','limits']},indent=2))
