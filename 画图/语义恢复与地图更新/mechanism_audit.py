"""Executable mechanism fixture; illustration data, not an experimental run."""
from pathlib import Path
import sys, json, hashlib
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / 'PRE_MAP_VLN'))
from stage2.viewpoint_recovery import ViewpointRecovery
from stage2.semantic_region_search import SemanticRegionBelief
from stage2.semantic_recovery import validate_hypotheses
from stage2.semantic_fusion import OnlineSemanticFusion
from stage2.mission_executor import MissionState

def audit():
    task='find_backpack'
    graph={'tasks':[{'id':task,'active_initially':True,'prerequisites':[],'success_outcome':'found'}],'conditional_rules':[]}
    state=MissionState(graph)
    pool=[{'id':f'old_{i}','location_hypothesis_id':'old_location','candidate_angle_rad':i*.6,'terminal_cost':.1} for i in range(3)]
    recovery=ViewpointRecovery();events=[]
    for c in pool:
        e=recovery.decide(task,c['id'],False,pool);events.append(e)
        assert task in state.available()
    assert events[0]['retry'] and events[1]['retry'] and events[-1]['location_exhausted']
    assert not recovery.filtered_candidates({task:pool})[task]
    scene={'rooms':[{'id':'study','centroid_xy_m':[2,2],'objects':[{'id':'old_location','label':'backpack','probability':.8}]},{'id':'lounge','centroid_xy_m':[8,2],'objects':[{'id':'table','label':'table','probability':.9},{'id':'sofa','label':'sofa','probability':.9}]}]}
    proposed={'target_label':'backpack','hypotheses':[{'room_id':'lounge','anchor_object_id':'table','relevance':'high','relation':'under'},{'room_id':'lounge','anchor_object_id':'sofa','relevance':'medium','relation':'near'},{'room_id':'study','anchor_object_id':'old_location','relevance':'high'}]}
    hypotheses=validate_hypotheses(proposed,scene,{'old_location'},3)
    assert {h['anchor_object_id'] for h in hypotheses['hypotheses']}=={'table','sofa'}
    candidates=[]
    for anchor in ('table','sofa'):
        for i in range(2):
            candidates.append({'id':f'{anchor}_{i}','location_hypothesis_id':anchor,'semantic_region_id':anchor,'semantic_prior_score':.8 if anchor=='table' else .6,'base_geometry_cost':.1,'visible_region_sample_ids':[i*2,i*2+1],'region_sample_count':4,'view_quality':1.})
    belief=SemanticRegionBelief();belief.register(candidates)
    negative=belief.observe(candidates[0],False)
    repeat=belief.observe(candidates[0],False)
    assert repeat['new_belief']==negative['new_belief']
    second=belief.observe(candidates[1],False);belief.apply(candidates)
    assert second['new_belief']<negative['new_belief']<negative['old_belief']
    positive=belief.observe(candidates[2],True);assert positive['new_belief']==1
    assert scene['rooms'][1]['objects'][0]['probability']==.9
    state.finish_task(task,'found');assert state.result_status()=='complete'
    fusion=OnlineSemanticFusion();item={'label':'backpack','center':[8,2,.4],'size':[.3,.2,.4],'score':.9,'confirmed':False}
    assert not fusion.update([item],0)
    assert not fusion.materialize_scene_graph(scene)[1]
    fused=fusion.update([{**item,'center':[8.1,2,.4]}],1);assert len(fused)==1
    updated,added=fusion.materialize_scene_graph(scene);assert len(added)==1
    assert all(o['id']!=added[0] for r in scene['rooms'] for o in r['objects'])
    # Validate the guarded exits displayed by the abstract state diagram.
    positive_decision=ViewpointRecovery().decide(task,pool[0]['id'],True,pool)
    assert not positive_decision['retry'] and positive_decision['reason']=='target_found'
    cap_decision=ViewpointRecovery(maximum_attempts=1).decide(task,pool[0]['id'],False,pool)
    assert not cap_decision['retry'] and cap_decision['remaining_candidate_ids']
    other={'id':'alternative_0','location_hypothesis_id':'alternative_location','candidate_angle_rad':0.}
    location_policy=ViewpointRecovery(maximum_attempts_per_location=1,maximum_locations=2)
    change_decision=location_policy.decide(task,pool[0]['id'],False,pool+[other])
    assert change_decision['retry'] and change_decision['location_exhausted']
    assert location_policy.filtered_candidates({task:pool+[other]})[task]==[other]
    negative_state=MissionState(graph);negative_state.finish_task(task,'not_found')
    assert negative_state.status[task]=='completed'
    assert negative_state.result_status()=='completed_with_unresolved_tasks'
    absence_graph={'tasks':[{**graph['tasks'][0],'success_outcome':'not_found'}],'conditional_rules':[]}
    absence_state=MissionState(absence_graph);absence_state.finish_task(task,'not_found')
    assert absence_state.result_status()=='complete'
    sources=['stage2/viewpoint_recovery.py','stage2/semantic_region_search.py','stage2/semantic_recovery.py','stage2/semantic_fusion.py','stage2/mission_executor.py','scripts/run_stage2_habitat.py']
    result={'scope':'Synthetic mechanism fixture; no live VLM, detector, trajectory or experiment. Same task across recovery locations. Hypotheses supplied offline and validated by production code.','local_retry':events,'validated_hypotheses':hypotheses,'negative_updates':[negative,repeat,second],'positive_update':positive,'confirmed_tracks':fused,'materialized_novel_ids':added,'final_task_status':state.status,'source_sha256':{s:hashlib.sha256((ROOT/'PRE_MAP_VLN'/s).read_bytes()).hexdigest() for s in sources},'timing':'Region beliefs and task state change online. Novel tracks accumulate online; new scene-graph instances are materialized after execution.','not_shown':'Optional room-frontier fallback, known-target instance confidence updates, and task-level terminal failure.'}
    result['abstract_rule_checks']={'positive_exit':positive_decision,'budget_can_prevent_retry_despite_remaining_views':cap_decision,
        'local_limit_allows_other_location':change_decision,'terminal_negative_task_state':negative_state.status,
        'negative_find_task_result':negative_state.result_status(),'negative_absence_task_result':absence_state.result_status()}
    result['not_shown']='Optional room-frontier details, known-target confidence updates, conditional-rule activation, and semantic hypothesis contents.'
    (OUT/'mechanism_evidence.json').write_text(json.dumps(result,indent=2)+'\n');return result
