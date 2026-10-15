#!/usr/bin/env python3
"""Read-only full acceptance audit of search batches, handovers and map growth."""
import argparse,bisect,collections,hashlib,json,math
from pathlib import Path
import rosbag
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();out=a.trial
root=Path('/workspace/shared');manifest=json.loads((out/'manifest.json').read_text());frozen=root/'runtime/consistency_validation_20261015'/(manifest['variant']+'_manifest.json')
identity=json.loads(frozen.read_text());events=json.loads((out/'execution_events.json').read_text());frames=[json.loads(l)for l in(out/'capture_trace.jsonl').open()];t0=frames[0]['capture_wall_s']
assert manifest['source_sha256']==identity['source_sha256']
assert all(hashlib.sha256((out/'source_snapshot'/k).read_bytes()).hexdigest()==v for k,v in identity['source_sha256'].items())
assert (out/'config.yaml').read_bytes()==(root/'runtime/consistency_validation_20261015/R0/config.yaml').read_bytes()
accepted={e['id']:e for e in events if e['event']=='server_accept'};switched={e['id']for e in events if e['event']=='server_switch'};published=collections.Counter(e['id']for e in events if e['event']=='published');command_switches=[];coverage=[];prev=None
with rosbag.Bag(str(out/'trial.bag'))as bag:
 for topic,m,t in bag.read_messages(topics=['/planning/pos_cmd','/voxel_mapping/map_coverage']):
  if topic=='/voxel_mapping/map_coverage':coverage.append((t.to_sec(),float(m.data)));continue
  c={'id':m.trajectory_id,'stamp':m.header.stamp.to_sec(),'position':[m.position.x,m.position.y,m.position.z],'velocity':[m.velocity.x,m.velocity.y,m.velocity.z]}
  if prev and c['id']!=prev['id']and c['id']in switched:
   command_switches.append({'id':c['id'],'relative_s':c['stamp']-t0,'command_step_m':math.dist(c['position'],prev['position']),'velocity_step_mps':math.dist(c['velocity'],prev['velocity']),'dt_s':c['stamp']-prev['stamp'],'accepted_before_command':c['id']in accepted and accepted[c['id']]['wall_s']<=c['stamp']})
  prev=c
# Join by callback sequence: planning_start precedes the plan counter increment.
rounds=[];cur=None
for e in events:
 if e['event']=='planning_start':cur={'start':e,'candidates':[]};rounds.append(cur)
 elif cur and e['event']=='plan_begin':cur['plan']=e['plan']
 elif cur and e['event']=='search_candidate_begin':cur['candidates'].append({'begin':e})
 elif cur and e['event']=='search_candidate_end'and cur['candidates']:cur['candidates'][-1]['end']=e
 elif cur and e['event']=='planning_end':cur['end']=e;cur['compute_s']=e['ros_s']-cur['start']['ros_s']
for r in rounds:
 r['pre_candidate_s']=r['candidates'][0]['begin']['ros_s']-r['start']['ros_s']if r['candidates']else r.get('compute_s')
 r['candidate_attempts']=len(r['candidates']);r['search_interval_s']=sum(c['end']['ros_s']-c['begin']['ros_s']for c in r['candidates']if'end'in c)
ends=[c['end']for r in rounds for c in r['candidates']if'end'in c];incomplete=[e for e in ends if e['termination']in [3,4]]
no_false_local_fail=all(r.get('end',{}).get('result')!=0 for r in rounds if any(c.get('end',{}).get('termination')in [3,4]for c in r['candidates']))
ts=[v[0]for v in coverage];pending=[]
for e in events:
 if e['event']!='search_round_pending':continue
 end=e['wall_s']+10;complete=end<=frames[-1]['capture_wall_s'];window=[f for f in frames if e['wall_s']<=f['capture_wall_s']<=end];ia=bisect.bisect_right(ts,e['wall_s'])-1;ib=bisect.bisect_right(ts,end)-1
 pending.append({'relative_s':e['wall_s']-t0,'full_10s_observed':complete,'max_displacement_next10s_m':max((math.dist(f['position'],window[0]['position'])for f in window),default=0),'coverage_gain_next10s_m3':coverage[ib][1]-coverage[ia][1]if ia>=0 and ib>=ia else None})
window_rounds=[r for r in rounds if 150<=r['start']['wall_s']-t0<=220]
r={'identity_and_config_match':True,'checks':{'unique_published_ids':max(published.values(),default=0)<=1,'no_normal_replan_truncation':not any(e['event']=='stop_request'and e.get('kind')==0 for e in events),'every_command_switch_had_prior_acceptance':all(s['accepted_before_command']for s in command_switches),'all_accepts_inside_original_time_window':all(.02<=e['start_time']-e['ros_s']<=3 for e in accepted.values()),'unfinished_search_not_returned_as_local_FAIL':no_false_local_fail},'published_count':sum(published.values()),'server_accepts':len(accepted),'actual_switches':len(switched),'command_switches':len(command_switches),'server_rejections':dict(collections.Counter(e['reason']for e in events if e['event']=='server_reject')),'stop_reasons':dict(collections.Counter(e['reason']for e in events if e['event']=='emergency_stop')),'max_command_step_at_switch_m':max((s['command_step_m']for s in command_switches),default=0),'max_velocity_step_at_switch_mps':max((s['velocity_step_mps']for s in command_switches),default=0),'top_command_steps':sorted(command_switches,key=lambda s:s['command_step_m'],reverse=True)[:8],'planning_results':dict(collections.Counter(r['end']['result']for r in rounds if'end'in r)),'search_results':dict(collections.Counter(str((e['result'],e['termination']))for e in ends)),'incomplete_candidate_searches':len(incomplete),'pending_recovery':pending,'coverage_samples':len(coverage),'all_rounds':rounds,'window150_220':{'rounds':len(window_rounds),'max_compute_s':max((r.get('compute_s',0)for r in window_rounds),default=None),'max_candidate_attempts':max((r['candidate_attempts']for r in window_rounds),default=None),'max_search_interval_s':max((r['search_interval_s']for r in window_rounds),default=None)},'limits':['Adjacent command steps include normal displacement over dt and do not independently establish physical safety.','No historical same-map replay.','Local search result checks do not audit all legacy global cost/cooldown decisions.','If early-stopped, volume is not comparable to full504 results.']}
(out/'search_round_full_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items()if k not in ['all_rounds','pending_recovery','top_command_steps','limits']},indent=2))
