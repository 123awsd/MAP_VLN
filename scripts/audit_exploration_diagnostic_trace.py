#!/usr/bin/env python3
"""Check diagnostic evidence completeness without inferring missing outcomes."""
import argparse,json,collections
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args()
f=a.trial/'decision_trace.jsonl';events=[];invalid=[]
for number,line in enumerate(f.read_text().splitlines(),1):
 try:events.append(json.loads(line))
 except ValueError:invalid.append(number)
counts=collections.Counter(e['event'] for e in events);plans=collections.defaultdict(list)
for e in events:plans[e['plan']].append(e)
report={'invalid_json_lines':invalid,'event_counts':dict(counts),'plans':len(plans),'search_exit_reasons':dict(collections.Counter(e['reason'] for e in events if e['event']=='search_result')),'candidate_exit_reasons':dict(collections.Counter(e['reason'] for e in events if e['event']=='candidate_result')),'limits':['Final-candidate diagnostics cannot establish why an earlier frontier cluster generated no candidate.','Per-height-band four-slot map ring, rate limited; pinned bathroom evidence is asynchronous published geometry. Snapshot capture pauses planning; not a timing benchmark.','Legacy search result semantics unchanged; reason is diagnostic only.','Current simulator odometry timestamps are bridge publication timestamps, not guaranteed actual capture timestamps.']}
manifest=json.loads((a.trial/'trial_manifest.json').read_text()) if (a.trial/'trial_manifest.json').exists() else {}
if manifest.get('variant') in ['search','combined','decision_revision']:report['limits'][2]='A* timeout and resource exhaustion return separate codes in this search ablation; candidate verification is budget limited.'
snapshot=a.trial/'source_snapshot'
if (snapshot/'astar.cpp').exists():
 source=(snapshot/'astar.cpp').read_text()
 if 'finishDiagnostic(SEARCH_TIMEOUT,' in source:
  report['limits'][2]='A* distinguishes timeout/resource exits. Check each caller separately; typed A* alone does not establish typed candidate-cost handling.'
bridge=snapshot/'bridge_snapshot/scripts/file_bridge_node.py'
if bridge.exists() and 'state["capture_stamp"]' in bridge.read_text():
 report['limits'][3]='Odometry/depth use frame capture_stamp from the frozen simulator; verify where that timestamp is acquired. Feedback velocity is sample displacement, not continuous dynamics.'
report['missing_plan_begin_for_selection']=[e['plan'] for e in events if e['event']=='goal_selected' and not any(v['event']=='plan_begin' for v in plans[e['plan']])]
(a.trial/'diagnostic_trace_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
