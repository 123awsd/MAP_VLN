import argparse,json,bisect,math,sys
from pathlib import Path
import numpy as np
import rosbag
root=Path('/workspace/shared');out=root/'outputs/consistency_validation_20261015';names=['coverage_lifecycle_700_r1_20261016','consistency_R0_light_504_r1_20261015','consistency_R0_light_504_r2_20261015','consistency_A3_observation_lifecycle_504_final_20261016'];report=[]
parser=argparse.ArgumentParser();parser.add_argument('cases',nargs='*');parser.add_argument('--output');args=parser.parse_args()
if args.cases:names=args.cases
for name in names:
 case=root/'outputs'/name;frames=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];t0=frames[0]['capture_wall_s'];t=np.array([f['capture_wall_s']-t0 for f in frames]);q=np.array([f['position']for f in frames]);steps=np.linalg.norm(np.diff(q,axis=0),axis=1);dt=np.diff(t);coverage=[]
 with rosbag.Bag(str(case/'trial.bag'))as b:
  for _,m,bt in b.read_messages(topics=['/voxel_mapping/map_coverage']):coverage.append((bt.to_sec()-t0,m.data))
 times=[v[0]for v in coverage];rows=[]
 for cp in sorted(set([300.,503.]+([504.]if t[-1]>=504 else [])+[float(t[-1])])):
  ix=bisect.bisect_right(t,cp);ind=bisect.bisect_right(times,cp)-1;slow=steps[:max(0,ix-1)]<.01;edges=np.diff(np.r_[False,slow,False].astype(int));durations=[float(t[e]-t[b])for b,e in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1))];cov=coverage[ind]if ind>=0 else None
  rows.append({'elapsed_capture_wall_s':cp,'latest_coverage_sample':cov,'coverage_sample_lag_s':cp-cov[0]if cov else None,'path_m':float(steps[:max(0,ix-1)].sum()),'translation_time_s':float(dt[:max(0,ix-1)][steps[:max(0,ix-1)]>=.01].sum()),'longest_adjacent_subcm_s':max(durations,default=0),'blocked_frames':sum(f.get('preview_motion_blocked',False)for f in frames[:ix]),'floor_volumes_at_checkpoint':'not recorded for historical checkpoints; final grids are not substitutes'})
 report.append({'case':name,'capture_duration_s':float(t[-1]),'checkpoints':rows})
(out/(args.output or ('cost_buffers700_timepoints.json'if args.cases else 'coverage700_timepoint_comparison.json'))).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
