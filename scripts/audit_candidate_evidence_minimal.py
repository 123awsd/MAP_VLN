"""Read-only log inventory. Never reconstruct online maps from GT."""
import json,collections,hashlib,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];dest=root/'outputs/consistency_validation_20261015/minimal_candidate_audit';dest.mkdir(exist_ok=True);names=['consistency_A3_observation_lifecycle_504_final_20261016','consistency_R0_504_r2_20261015','consistency_R0_504_r3_20261015'];reports=[]
for name in names:
 path=root/'outputs'/name/'decision_trace.jsonl';counts=collections.Counter();stages=collections.Counter();fields=collections.defaultdict(collections.Counter);examples={};roi=[];ids=collections.defaultdict(list);latest209=[];malformed=0;dropped=0;lines=0
 for line in path.open():
  lines+=1
  try:e=json.loads(line)
  except json.JSONDecodeError:malformed+=1;continue
  event=e.get('event');counts[event]+=1
  if event=='records_dropped':dropped+=e.get('count',0)
  if name==names[0] and e.get('plan')==209:latest209.append(e)
  if event!='candidate':continue
  stage=e.get('stage');stages[stage]+=1;fields[stage].update(e.keys());examples.setdefault(stage,e)
  q=e.get('xyz',[])
  if len(q)!=3:continue
  if (2<=q[0]<4 and 4<=q[1]<6 and 2.745<=q[2]<4.795)or(-2<=q[0]<0 and 10<=q[1]<12 and 2.745<=q[2]<4.795):
   if stage in ['visibility','gain','scored','selected','quota_admitted','travel_rejected','onward_rejected','score_gain_rejected','near_suppressed']:
    roi.append(e)
  if stage=='scored' and ((2<=q[0]<4 and 4<=q[1]<6)or(-2<=q[0]<0 and 10<=q[1]<12)):
   ids[str(e.get('candidate_id'))].append(e)
 report={'case':name,'file_bytes':path.stat().st_size,'lines':lines,'malformed_lines':malformed,'dropped_records_reported':dropped,'events':dict(counts),'candidate_stages':dict(stages),'candidate_fields_by_stage':{k:dict(v)for k,v in fields.items()},'one_example_per_stage':examples,'ROI_records_count':len(roi),'scored_candidate_ids_in_ROIs':list(ids)};reports.append(report)
 (dest/(name+'_ROI_events.json')).write_text(json.dumps(roi,indent=2))
 if latest209:(dest/'plan209_events.json').write_text(json.dumps(latest209,indent=2))
 print(name,'lines',lines,'candidate',counts['candidate'],'stages',dict(stages),flush=True)
latest=root/'outputs'/names[0];snapshot=latest/'source_snapshot';manifest=json.loads((latest/'manifest.json').read_text());hashes={n:{'actual':hashlib.sha256((snapshot/n).read_bytes()).hexdigest(),'recorded':manifest['source_sha256'].get(n)}for n in ['exploration_manager.cpp','frontier_finder.cpp','exploration_diagnostics.h','exploration_fsm.cpp','run_habitat_falcon.py']};assert all(v['actual']==v['recorded']for v in hashes.values())
frames=[json.loads(l)for l in(latest/'capture_trace.jsonl').open()];errors=[];orth=[]
for f in frames:
 x,y,z,w=f['orientation_xyzw'];R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);yaw=f.get('executed_command_yaw')
 if yaw is not None:
  expected=np.array([math.cos(yaw),math.sin(yaw),0]);errors.append(math.degrees(math.acos(np.clip(R[:,2]@expected,-1,1))))
 orth.append(float(np.linalg.norm(R.T@R-np.eye(3))))
proof={'runs':reports,'source_hash_checks':hashes,'trace_mode':manifest.get('trace_mode'),'optical_axis_check':{'frames':len(frames),'frames_with_command_yaw':len(errors),'median_angle_deg':float(np.median(errors)),'max_angle_deg':max(errors),'p95_angle_deg':float(np.percentile(errors,95)),'max_rotation_orthogonality_error':max(orth),'expected_forward_falcon':'[cos(yaw),sin(yaw),0]','optical_axis':'R_falcon_optical[:,2]','limits':'Checks same-frame recorded executed yaw vs actual camera orientation; does not prove unselected candidate orientation.'}}
(dest/'evidence_inventory.json').write_text(json.dumps(proof,indent=2))
