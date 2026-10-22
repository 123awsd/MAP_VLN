"""Index actual local experiments; missing metrics remain unknown."""
from pathlib import Path
import json,numpy as np
root=Path(__file__).resolve().parents[1];records=[]
for p in sorted((root/'outputs').glob('exploration_goal_*_504')):
 if not (p/'trial_manifest.json').exists():continue
 config={}
 for line in (p/'config.yaml').read_text().splitlines():
  if ':' in line:
   key,value=line.strip().split(':',1)
   if key in ['cluster_min','min_visib_num','min_unique_gain','execution_clearance','entry_refine','entry_observe','consistent_path_clearance','unified_local_reachability']:config[key]=value.strip()
 record={'trial':p.name,'path':str(p.resolve()),'configuration':config,'finished':(p/'result.json').exists(),'assessment':json.loads((p/'assessment.json').read_text()) if (p/'assessment.json').exists() else None,'result':json.loads((p/'result.json').read_text()) if (p/'result.json').exists() else None,'segmentation_exists':(p/'postprocess/room_wall_guided_trial/adaptive_merge/three_floor_cloud_overlay_wide.png').exists(),'video_encoding_finished': '四个无字30fps视频完成' in (p/'postprocess.log').read_text(errors='replace') if (p/'postprocess.log').exists() else False,'goal_achieved':False}
 if (p/'live_trajectory.jsonl').exists():
  rows=[json.loads(s) for s in (p/'live_trajectory.jsonl').open()];q=np.array([s['position'] for s in rows]).reshape(-1,3);record['bathrooms']={}
  for key,b in [('room2_neighbor',[-1.4,-.15,4.3,6.2]),('room9_upper_right',[-1.3,-.1,9.7,11.7])]:
   x0,x1,y0,y1=b;inside=(q[:,0]>x0)&(q[:,0]<x1)&(q[:,1]>y0)&(q[:,1]<y1)&(q[:,2]>2.8)&(q[:,2]<5.4);ids=np.flatnonzero(inside);record['bathrooms'][key]={'trajectory_samples':len(ids),'exit_seen_after_last_entry':bool(len(ids) and ids[-1]<len(rows)-1)}
 records.append(record)
(root/'outputs/exploration_goal_audit/trial_registry.json').write_text(json.dumps({'note':'offline ROI checks only; no global non-regression proof; file existence is not artifact validation','trials':records},indent=2));print('indexed',len(records),'trials')
