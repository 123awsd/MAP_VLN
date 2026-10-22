#!/usr/bin/env python3
"""Evaluate actual injected failures and fresh-state recovery before full trials."""
import argparse,json,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();out=a.trial
events=[]
for line in (out/'decision_trace.jsonl').open():
 try:r=json.loads(line)
 except ValueError:continue
 if r['event']!='candidate':events.append(r)
checks={}
injected=[(i,e)for i,e in enumerate(events)if e['event']=='injected_plan_failure']
checks['injected_failure_preserves_safe_old_trajectory']=bool(injected) and all(any(e['event']=='active_preserved' and e.get('reason')=='plan_failed' for e in events[i+1:i+5])for i,_ in injected)
checks['no_normal_replan_truncation']=not any(e['event']=='stop_request' and e.get('kind')==0 for e in events)
stops=[e for e in events if e['event']=='emergency_stop'];recoveries=[e for e in events if e['event']=='fresh_recovery']
checks['both_collision_entrances_exercised']=all(any(e.get('reason')==cause for e in stops)for cause in ['safetyCallback','SceneGeometryFeedback'])
checks['recovery_uses_post_stop_capture']=bool(recoveries) and all(e['stamp']>e['stop_stamp'] for e in recoveries)
aligned=[]
for i,e in enumerate(events):
 if e['event']=='fresh_recovery':
  following=next((v for v in events[i+1:] if v['event']=='planning_start'),None)
  if following:aligned.append(math.dist(following['actual'],following['predicted_start']))
checks['recovery_start_is_actual_position']=bool(aligned) and max(aligned)<1e-7
accepted={e['id']:e for e in events if e['event']=='server_accept'};switched=[e for e in events if e['event']=='server_switch']
checks['switch_after_server_accept_and_scheduled_time']=bool(switched) and all(e['id']in accepted and e['ros_s']>=accepted[e['id']]['start_time'] for e in switched)
rows=[json.loads(l)for l in (out/'capture_trace.jsonl').open()];held=[]
for previous,current in zip(rows,rows[1:]):
 if (current.get('executed_command')or{}).get('execution_hold'):held.append(math.dist(previous['position'],current['position']))
checks['explicit_hold_never_rewinds_to_old_odometry']=bool(held) and max(held)<1e-7
report={'passed':all(checks.values()),'checks':checks,'injected_failures':len(injected),'post_stop_recoveries':len(recoveries),'max_recovery_start_error_m':max(aligned,default=None),'held_capture_count':len(held),'max_held_step_m':max(held,default=None)}
(out/'execution_gate.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
if not report['passed']:raise SystemExit(1)
