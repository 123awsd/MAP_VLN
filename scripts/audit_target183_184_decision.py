#!/usr/bin/env python3
"""Read-only decision reconstruction using saved ROS logs and wire audit."""
import hashlib,json,re
from pathlib import Path
root=Path(__file__).resolve().parents[1];case=root/'outputs/consistency_A3_search_round_504_r1_20261016';out=root/'outputs/consistency_validation_20261015'
text=re.sub(r'\x1b\[[0-9;]*m','',(case/'run.log').read_text(errors='replace'))
pattern=re.compile(r'\[(?:INFO|WARN|ERROR|DEBUG|FATAL)\]\s+\[(\d+\.\d+)\]:');marks=list(pattern.finditer(text));records=[]
for i,m in enumerate(marks):
 t=float(m[1]);body=text[m.end():marks[i+1].start() if i+1<len(marks) else len(text)].strip()
 if 1792092047.4397<=t<=1792092051.025147:records.append({'ros_s':t,'message':body})
score=re.compile(r'\[ObservationDecision\] xyz=\(([^)]+)\) gain=(\d+) confidence=([\d.]+) travel=([\d.]+) onward=([\d.]+) utility=([\d.]+)');plans={}
for plan,lo,hi in [(222,1792092047.4397,1792092048.293432),(223,1792092050.19224,1792092051.025147)]:
 rows=[]
 for r in records:
  if not lo<=r['ros_s']<=hi:continue
  m=score.search(r['message'])
  if m:rows.append({'ros_s':r['ros_s'],'position_rounded':[float(x) for x in m[1].split(',')],'gain':int(m[2]),**dict(zip(['confidence','travel','onward','utility'],map(float,m.groups()[2:])) )})
 plans[plan]={'complete_score_rows':rows,'top5':sorted(rows,key=lambda r:r['utility'],reverse=True)[:5],'high_rows':sum(r['position_rounded'][2]>5.745 for r in rows)}
source=(case/'source_snapshot/exploration_manager.cpp').read_text();r0=(root/'runtime/consistency_validation_20261015/R0/exploration_manager.cpp').read_text()
blocks={}
for name,begin,end in [('observation_outcome','  using ObservationKey=', '  // Start planning'),('floor_priority','void ExplorationManager::applyFloorPriority(', 'ExplorationManager::~ExplorationManager()'),('final_observation_selection','  // Final action selection uses', '  // Local planner')]:
 a=source[source.index(begin):source.index(end,source.index(begin))];b=r0[r0.index(begin):r0.index(end,r0.index(begin))]
 blocks[name]={'identical_to_R0':a==b,'run_source_sha256':hashlib.sha256(a.encode()).hexdigest(),'R0_sha256':hashlib.sha256(b.encode()).hexdigest()}
wire=json.loads((out/'target183_184_wire_audit.json').read_text());manifest=json.loads((case/'manifest.json').read_text());assert all(hashlib.sha256((case/'source_snapshot'/k).read_bytes()).hexdigest()==v for k,v in manifest['source_sha256'].items())
r={'plans':plans,'records':sorted(records,key=lambda r:r['ros_s']),'R0_source_comparison':blocks,'source_snapshot_matches_run_manifest':True,'target_mapping':{'183':{'plan':222,'hgrid_next_cell':28,'hgrid_center':0,'SOP_frontier_indices':[33,40],'final_observation_id':None,'final_frontier_owner':None},'184':{'plan':223,'hgrid_next_cell':48,'hgrid_center':1,'SOP_frontier_indices':[17],'final_observation_id':None,'final_frontier_owner':None}},'facts':{'predicted_start_inside_arrival_gate':wire['predicted_distance_to_goal183_m']<.45,'actual_feedback_outside_arrival_gate':wire['feedback_distance_to_goal183_m']>.45,'low_goal_is_real_descending_local_segment':wire['wire']['184']['end_position'][2]<5.595,'high_candidates_still_scored':plans[223]['high_rows']>0},'limits':['Scores retained only when complete rows survive stdout/stderr interleaving; missing rows are not inferred.','Stable candidate IDs and final-goal ownership disabled by light tracing; HGrid/SOP indices are per-update, not stable identities.','No occupancy/ESDF or unknown-voxel-set replay; cannot isolate the effect of one failed-observation memory on the winner.','No algorithm changes or simulation performed for this audit.']}
(out/'target183_184_decision_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps({'source_comparison':blocks,'facts':r['facts'],'complete_score_rows':{k:len(v['complete_score_rows'])for k,v in plans.items()}},indent=2))
