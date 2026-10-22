import json,collections,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root/'outputs/consistency_validation_20261015/minimal_candidate_audit';inv=json.loads((out/'evidence_inventory.json').read_text());targets={};examples=[]
for r in inv['runs'][1:]:
 rows=json.loads((out/(r['case']+'_ROI_events.json')).read_text());chosen=[]
 for bounds in [(2,4,4,6),(-2,0,10,12)]:
  sc=[e for e in rows if e['stage']=='scored'and e.get('candidate_id',0)>0 and bounds[0]<=e['xyz'][0]<bounds[1] and bounds[2]<=e['xyz'][1]<bounds[3]]
  if sc:chosen.append(max(sc,key=lambda e:e.get('gain',0)))
 targets[r['case']]={e['candidate_id']for e in chosen};examples.extend({'case':r['case'],'scored':e}for e in chosen)
chains=[]
for name,ids in targets.items():
 found=collections.defaultdict(list)
 for line in(root/'outputs'/name/'decision_trace.jsonl').open():
  # Filter before JSON to avoid parsing most records; every target has unique numeric ID within run.
  if not any(('"candidate_id":'+str(i)+',')in line or ('"candidate_id":'+str(i)+'}')in line for i in ids):continue
  e=json.loads(line)
  if e.get('candidate_id')in ids:found[e['candidate_id']].append(e)
 chains.extend({'case':name,'candidate_id':cid,'events':es}for cid,es in found.items())
(out/'historical_C_identity_examples.json').write_text(json.dumps({'examples':examples,'chains':chains},indent=2))
case=root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016';frames=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];t0=frames[0]['capture_wall_s'];errs=[]
for f in frames:
 x,y,z,w=f['orientation_xyzw'];forward=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)]);yaw=f['executed_command_yaw'];ang=math.degrees(math.acos(np.clip(forward@np.array([math.cos(yaw),math.sin(yaw),0]),-1,1)))
 if ang>.01:errs.append({'relative_s':f['capture_wall_s']-t0,'angle_deg':ang,'command_yaw':yaw,'optical_forward':forward.tolist()})
# Saved reference identity checks, no raycasting or online planner interaction.
g=root/'outputs/consistency_validation_20261015/global_observability';a=np.load(g/'surface_witnesses.npz');b=np.load(g/'mesh_verified_opportunities.npz');geom=np.load(g/'actual_geometry_visibility.npz')['visible'];m=np.load(g/'three_region_surface_masks.npz');area={}
for k in ['floor2_ordinary','floor2_room9_near']:
 ids=m[k+'_reference_indices'];assert np.all(a['distance_m'][ids]>.08);assert np.all(b['verified'][ids]);assert not np.any(geom[ids]);area[k]={'reference_samples':len(ids),'area_m2':float(round(a['weight_m2'][ids].sum(),5)),'selected_potential_m2':round(float((m[k+'_selected_visible']*.01).sum()),5)}
metrics=[]
for r in inv['runs'][1:]:
 n=r['candidate_stages'].get('scored',0);metrics.append({'case':r['case'],'full_trace_bytes':r['file_bytes'],'bytes_per_504s_second':r['file_bytes']/504,'scored_records':n,'estimated_500B_scored_summary_bytes':n*500,'illustrative_full_363_voxel_int32_payload_bytes':n*363*12})
(out/'minimal_audit_checks.json').write_text(json.dumps({'reference_set_checks':area,'optical_angle_outliers_gt_0_01deg':errs,'overhead_estimates':metrics,'overhead_limits':'Storage arithmetic only, not measured runtime overhead; 363 voxels is one illustrative candidate, not global average. Payload estimate excludes headers/compression/hash and generation/filter events.'},indent=2))
print(json.dumps({'historical_examples':examples,'chain_lengths':[(c['case'],c['candidate_id'],len(c['events']))for c in chains],'references':area,'outliers':errs,'overhead':metrics},indent=2))
