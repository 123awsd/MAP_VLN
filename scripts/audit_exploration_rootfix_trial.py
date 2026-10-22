#!/usr/bin/env python3
"""Offline audit of timing fixes and executed trajectory, with evaluation-only ROIs."""
import argparse,json,re
from pathlib import Path
import numpy as np
from exploration_sample_times import sample_times
p=argparse.ArgumentParser();p.add_argument('trial');a=p.parse_args();out=Path(a.trial)
rows=[json.loads(s) for s in (out/'live_trajectory.jsonl').read_text().splitlines() if s.strip()]
xyz=np.array([r['position'] for r in rows]);t,timing_method=sample_times(rows,out);steps=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
log=(out/'run.log').read_text(errors='replace');log=re.sub(r'\x1b\[[0-9;]*m','',log)
plans=[(float(x),float(y)) for x,y in re.findall(r'Exploration planner total time: ([\d.]+)ms, replan duration: ([\d.]+)ms',log)]
first_times=[float(x) for x in re.findall(r'\[TrajServer\] t_cur: ([-\d.e+]+)',log)]
result={'trial':out.name,'observed_sequence_duration_s':float(t[-1]),'samples':len(rows),'path_from_live_samples_m':float(steps.sum()),'max_height_m':float(xyz[:,2].max()),'max_sample_step_m':float(steps.max()),'blocked_samples':sum(bool(r.get('preview_motion_blocked')) for r in rows),'planning':{'calls':len(plans),'over_budget':sum(x>y for x,y in plans),'max_ms':max((x for x,y in plans),default=None)},'timing':{'rejected_dynamic_deadlines':log.count('Dynamic plan missed handoff'),'static_retimed':log.count('Measured-state trajectory starts at publication'),'first_command_times_count':len(first_times),'max_first_trajectory_time_s':max(first_times,default=None),'first_times_over_0_1s':sum(v>.1 for v in first_times),'invalid_time_messages':log.count('Invalid time:')},'observation_outcomes':len(re.findall(r'\[ObservationOutcome\]',log)),'incomplete_search_retained':log.count('Search budget exhausted; frontier remains eligible'),'fine_search_rescues':log.count('[LocalReachability] rescued')}
# Stationary position may include intentional yaw rotation; do not call every pause a planning stall.
result['sample_timing_method']=timing_method
still=steps<.01;edges=np.diff(np.r_[False,still,False].astype(int));pauses=[]
for b,e in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
    if t[e]-t[b]>=2:pauses.append({'start_s':float(t[b]),'duration_s':float(t[e]-t[b]),'position':xyz[b].tolist(),'blocked_samples':sum(bool(r.get('preview_motion_blocked')) for r in rows[b:e+1])})
result['longest_position_pauses']=sorted(pauses,key=lambda r:-r['duration_s'])[:8]
for name,(x0,x1,y0,y1) in {'bathroom_next_to_room2':(-1.4,-.15,4.3,6.2),'bathroom_room9_upper_right':(-1.3,-.1,9.7,11.7)}.items():
    inside=(xyz[:,0]>x0)&(xyz[:,0]<x1)&(xyz[:,1]>y0)&(xyz[:,1]<y1)&(xyz[:,2]>2.8)&(xyz[:,2]<5.4)
    edges=np.diff(np.r_[False,inside,False].astype(int));runs=[]
    for b,e in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
        runs.append({'entry_s':float(t[b]),'last_inside_s':float(t[e-1]),'exited':bool(e<len(rows))})
    result[name]={'trajectory_samples':int(inside.sum()),'entry_runs':runs}
for filename in ['assessment.json','floor_volumes.json']:
    if (out/filename).exists():result[filename]=json.loads((out/filename).read_text())
(out/'rootfix_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
