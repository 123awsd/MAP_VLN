import json,collections
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'outputs/consistency_validation_20261015/global_observability';p=out/'three_region_opportunity_audit.json';data=json.loads(p.read_text());events=json.loads((root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016/execution_events.json').read_text());selected={e['plan']:e for e in events if e['event']=='observation_selected'}
for r in data['regions']:
 scores=r['complete_score_records_with_optimistic_visibility'];r['score_summary']={'row_count':len(scores),'distinct_positions':len({tuple(s['position'])for s in scores}),'distinct_plans':len({s['plan']for s in scores if s['plan']is not None}),'maximum_single_position_optimistic_area_m2':max((s['optimistic_visible_missing_m2']for s in scores),default=0),'first_score_s':min((s['relative_s']for s in scores),default=None),'last_score_s':max((s['relative_s']for s in scores),default=None)}
 best=sorted(scores,key=lambda s:(s['optimistic_visible_missing_m2'],s['utility']),reverse=True)[:15]
 for s in best:
  e=selected.get(s['plan']);s['actual_selected_goal_in_same_plan']=e
 r['top_geometry_score_examples']=best
 r['cancellation_reasons']=dict(collections.Counter(e['reason']for e in r['related_task_chain']if e['event']=='observation_cancelled'))
p.write_text(json.dumps(data,indent=2))
print(json.dumps([{k:r[k]for k in ['region','B_area_m2','scored_optimistic_unknown_yaw_union_m2','selected_known_yaw_union_m2','executed_goal_potential_union_m2','score_summary','cancellation_reasons','top_geometry_score_examples']}for r in data['regions']],indent=2))
