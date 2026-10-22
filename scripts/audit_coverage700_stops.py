"""Join existing trajectory IDs and deadlines; do not infer collision safety."""
import json,bisect,collections
from pathlib import Path
root=Path(__file__).resolve().parents[1];case=root/'outputs/coverage_lifecycle_700_r1_20261016';events=json.loads((case/'execution_events.json').read_text());cap=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];t0=cap[0]['capture_wall_s'];tr={e['id']:e for e in events if e['event']=='candidate_finalized'};active=None;rows=[];preserved=[]
for e in events:
 if e['event']=='server_switch':active=e['id']
 elif e['event']=='active_preserved':preserved.append(e)
 elif e['event']=='emergency_stop':
  curve=tr.get(active,{});remaining=curve.get('start_time',0)+curve.get('duration',0)-e['wall_s']if curve else None;rows.append({'relative_s':e['wall_s']-t0,'reason':e.get('reason'),'active_trajectory':active,'remaining_original_time_s':remaining,'time_domain':'ros start_time compared with wall event; this experiment ROS uses wall time','note':'Positive remaining time alone does not establish safe resume: measured tracking/freshness/collision checks also required.'});active=None
buckets=collections.defaultdict(list)
for row in rows:buckets[row['reason']].append(row)
out={'stops':rows,'by_reason':{k:{'count':len(v),'positive_original_time_gt40ms':sum(r['remaining_original_time_s']is not None and r['remaining_original_time_s']>.04 for r in v),'expired_original_time':sum(r['remaining_original_time_s']is not None and r['remaining_original_time_s']<=0 for r in v),'no_known_active_curve':sum(r['remaining_original_time_s']is None for r in v)}for k,v in buckets.items()},'active_preserved_events':preserved,'limits':['Server/FSM event streams are asynchronous; last switch ID is inferred, no full switch-time state snapshot.','Expired old curves cannot safely be extended by rescheduling alone.','Cannot label a stop unnecessary without online collision/freshness/tracking predicate evidence.']};dest=root/'outputs/consistency_validation_20261015/coverage700_stop_audit.json';dest.write_text(json.dumps(out,indent=2));print(json.dumps(out['by_reason'],indent=2))
