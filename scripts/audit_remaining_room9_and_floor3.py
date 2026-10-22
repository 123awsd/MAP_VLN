#!/usr/bin/env python3
"""Existing data only: room9 ROI unknown voxels, score chain and late L3 choice."""
import bisect,collections,json,math,re
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];case=root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016';dest=root/'outputs/consistency_validation_20261015'
frames=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];t0=frames[0]['capture_wall_s'];events=json.loads((case/'execution_events.json').read_text());full=json.loads((case/'search_round_full_audit.json').read_text());rounds=full['all_rounds'];starts=[r['start']['wall_s']for r in rounds]
def inside(p):return -1.3<p[0]<-.1 and 9.7<p[1]<11.7 and 2.8<p[2]<5.4
def near(p):return -2.3<p[0]<.9 and 8.7<p[1]<12.7 and 2.8<p[2]<5.4
def keys(pts):return {tuple(x)for x in np.rint((pts-.05)/.1).astype(int)}
grid=np.load(case/'observed_grid_final.npz');known={};filtered={}
for typ in ['free','occupied']:
 pts=grid[typ];v=pts[(pts[:,0]>-1.3)&(pts[:,0]<-.1)&(pts[:,1]>9.7)&(pts[:,1]<11.7)&(pts[:,2]>2.8)&(pts[:,2]<5.4)];filtered[typ]=v;known[typ]=keys(v)
allroi={(x,y,z)for x in range(-13,-1)for y in range(97,117)for z in range(28,54)};current=known['free']|known['occupied'];unknown=allroi-current
refs={}
for name in ['consistency_R0_light_504_r1_20261015','consistency_R0_light_504_r2_20261015','consistency_A3_motion_prefix_504_r1_20261016','consistency_A3_search_round_504_r1_20261016']:
 p=root/'outputs'/name/'observed_grid_final.npz'
 if not p.exists():continue
 g=np.load(p);sets={}
 for typ in ['free','occupied']:
  q=g[typ];q=q[(q[:,0]>-1.3)&(q[:,0]<-.1)&(q[:,1]>9.7)&(q[:,1]<11.7)&(q[:,2]>2.8)&(q[:,2]<5.4)];sets[typ]=keys(q)
 refs[name]={'free_voxels':len(sets['free']),'occupied_voxels':len(sets['occupied']),'historical_free_current_unknown_voxels':len(sets['free']&unknown),'historical_known_current_unknown_voxels':len((sets['free']|sets['occupied'])&unknown)}
# Parse only intact ROS records: stdout/stderr interleaving must not synthesize scores.
text=re.sub(r'\x1b\[[0-9;]*m','',(case/'run.log').read_text(errors='replace'));pattern=re.compile(r'\[(?:INFO|WARN|ERROR|DEBUG|FATAL)\]\s+\[(\d+\.\d+)\]:');marks=list(pattern.finditer(text));records=[];scores=[]
sp=re.compile(r'\[ObservationDecision\] xyz=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\) gain=(\d+) confidence=([\d.]+) travel=([\d.]+) onward=([\d.]+) utility=([\d.]+)')
for i,m in enumerate(marks):
 stamp=float(m[1]);body=text[m.end():marks[i+1].start()if i+1<len(marks)else len(text)].strip();idx=bisect.bisect_right(starts,stamp)-1;r=rounds[idx]if idx>=0 else None;plan=r.get('plan')if r and stamp<=r.get('end',{}).get('wall_s',r['start']['wall_s'])else None
 records.append({'relative_s':stamp-t0,'plan':plan,'message':body})
 q=sp.search(body)
 if q:scores.append({'relative_s':stamp-t0,'plan':plan,'position':[float(q[j])for j in range(1,4)],'gain':int(q[4]),**dict(zip(['confidence','travel','onward','utility'],map(float,q.groups()[4:])))})
room_scores=[s for s in scores if inside(s['position'])];near_scores=[s for s in scores if near(s['position'])];highscores=[s for s in scores if s['position'][2]>5.745];highselect=[e for e in events if e['event']=='observation_selected'and e['goal'][2]>5.745];firsthigh=highselect[0]['wall_s']if highselect else None
perplan=collections.defaultdict(list)
for s in scores:perplan[s['plan']].append(s)
selected={e['plan']:e for e in events if e['event']=='observation_selected'}
roomcomparisons=[]
for s in room_scores:
 e=selected.get(s['plan']);win=max(perplan[s['plan']],key=lambda x:x['utility'])if perplan[s['plan']]else None
 roomcomparisons.append({'candidate':s,'selected_event':e,'largest_complete_logged_score':win})
late=[]
for plan,rows in perplan.items():
 hs=[s for s in rows if s['position'][2]>5.745];e=selected.get(plan)
 if not hs or not e:continue
 late.append({'plan':plan,'relative_s':e['wall_s']-t0,'selected_goal':e['goal'],'best_complete_high_score':max(hs,key=lambda s:s['utility']),'best_complete_score':max(rows,key=lambda s:s['utility'])})
# Duration categories are elapsed planner calls, not causal delay attribution.
pre=[r for r in rounds if firsthigh and r['start']['wall_s']<firsthigh];pending=[r for r in pre if r.get('end',{}).get('result')==3]
room_events=[e for e in events if e.get('goal')and inside(e['goal'])];near_events=[e for e in events if e.get('goal')and near(e['goal'])]
report={'case':case.name,'room9':{'ROI':{'x':[-1.3,-.1],'y':[9.7,11.7],'z':[2.8,5.4],'resolution_m':.1},'prism_total_voxels':len(allroi),'known_free_voxels':len(known['free']),'known_occupied_voxels':len(known['occupied']),'unknown_voxels':len(unknown),'unknown_prism_m3':len(unknown)*.001,'historical_comparison':refs,'complete_scored_rows':room_scores,'score_comparisons':roomcomparisons,'nearby_score_rows':near_scores,'inside_goal_event_counts':dict(collections.Counter(e['event']for e in room_events)),'inside_goal_events':room_events,'nearby_goal_events':near_events,'unknown_centers':[[.05+.1*v for v in k]for k in sorted(unknown)]},'floor3':{'first_complete_high_score':min(highscores,key=lambda s:s['relative_s'])if highscores else None,'first_high_selected':highselect[0]if highselect else None,'first_high_published':next((e for e in events if e['event']=='published'and e['goal'][2]>5.745),None),'first_high_executing':next((e for e in events if e['event']=='observation_executing'and e['goal'][2]>5.745),None),'high_score_vs_selected_by_plan':late,'before_first_high_selection':{'rounds':len(pre),'pending_rounds':len(pending),'pending_compute_total_s':sum(r.get('compute_s',0)for r in pending),'all_compute_total_s':sum(r.get('compute_s',0)for r in pre),'stops':dict(collections.Counter(e['reason']for e in events if e['event']=='emergency_stop'and e['wall_s']<firsthigh))if firsthigh else {}},'floor_priority_records':[r for r in records if 'Floor priority' in r['message']or 'Online floor switch committed' in r['message']]},'limits':['Unknown prism voxels include wall/occlusion/unobserved volume and are not guaranteed navigable or useful room interior.','Historical FREE is observed evidence under historical map/execution, not current safe reachability.','Light candidate trace omits generation/filter/unique candidate IDs; incomplete ROS score rows are excluded.','Selection means final scoring choice; publication, server acceptance and execution are separately linked events.','Stop counts and total compute cannot be added to infer a counterfactual earlier floor3 arrival.']}
(dest/'room9_and_late_floor3_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps({'room9_prism':{k:v for k,v in report['room9'].items()if k in ['prism_total_voxels','known_free_voxels','known_occupied_voxels','unknown_voxels','unknown_prism_m3','historical_comparison','inside_goal_event_counts']},'room9_complete_score_rows':len(room_scores),'highfirst':report['floor3']['first_complete_high_score'],'high_selected_first':report['floor3']['first_high_selected'],'high_score_plans':len(late),'pre_high':report['floor3']['before_first_high_selection']},indent=2))
