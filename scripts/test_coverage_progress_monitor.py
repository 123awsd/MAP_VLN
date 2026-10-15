"""Supervisor evidence contracts; no simulator, ROS, GPU or new trials."""
import copy,json
from pathlib import Path
from coverage_progress_monitor import classify_no_progress,checkpoint_evidence

def frames(move=False,blocked=False):
 return [{'capture_wall_s':i/5,'position':[i/500 if move else 0,0,1],'executed_command':{'velocity':[0,0,0] if not blocked else [1,0,0]},'preview_motion_blocked':blocked,'blocked_trajectory_id':17 if blocked else None}for i in range(301)]
cov=[(float(i),100.)for i in range(61)];passed=[]
def check(name,condition):
 assert condition,name
 passed.append(name)
check('hold with fresh depth/coverage is stalled incomplete',classify_no_progress(frames(),cov)['classification']=='stalled_incomplete')
check('moving transit without map gain continues',classify_no_progress(frames(move=True),cov)is None)
check('new observation while stationary continues',classify_no_progress(frames(),[(t,v+t*.1)for t,v in cov])is None)
check('repeated physical obstruction is bounded',classify_no_progress(frames(blocked=True),cov)['blocked_trajectory_ids']==[17])
check('stale coverage cannot prove stall',classify_no_progress(frames(),cov[:20])is None)
check('counter reset cannot prove stall',classify_no_progress(frames(),[(t,v-t*.1)for t,v in cov])is None)
check('missing captures cannot prove continuous stall',classify_no_progress(frames()[::100],cov)is None)
check('short windows continue',classify_no_progress(frames()[:100],cov)is None)
r=checkpoint_evidence(504,505,{'capture_wall_s':1000},{'snapshot_wall_times':{'free':998,'occupied':999}})
check('checkpoint records asynchronous ages',r['occupancy_component_lag_from_capture_s']=={'free':2,'occupied':1} and r['snapshot_is_consistent_map']is False)
root=Path(__file__).resolve().parents[1];out=root/'outputs/consistency_validation_20261015/coverage_monitor_regression.json';out.write_text(json.dumps({'passed':passed,'scope':'Experiment supervisor pure evidence functions; no long-run exploration or saturation validation.'},indent=2));print(json.dumps({'passed':len(passed),'output':str(out)}))
