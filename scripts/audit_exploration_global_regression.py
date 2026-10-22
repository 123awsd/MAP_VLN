from pathlib import Path
import json,re,numpy as np
root=Path(__file__).resolve().parents[1];names=['exploration_threshold_double_half_504','exploration_structural_execution_504','exploration_execution_contract_504','exploration_execution_contract_v2_504']
for name in names:
 p=root/'outputs'/name;rows=[json.loads(l) for l in (p/'live_trajectory.jsonl').read_text().splitlines()];q=np.array([r['position'] for r in rows]);t=np.arange(len(rows))*.2;dt=np.full(len(rows)-1,.2)
 yaw=[]
 for r in rows:
  if r.get('executed_command_yaw') is not None:yaw.append(r['executed_command_yaw'])
  else:
   x,y,z,w=r['body_orientation_xyzw'];yaw.append(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 angle=np.abs(np.arctan2(np.sin(np.diff(yaw)),np.cos(np.diff(yaw))));distance=np.linalg.norm(np.diff(q,axis=0),axis=1);still=distance<.01
 log=re.sub(r'\x1b\[[0-9;]*m','',(p/'run.log').read_text(errors='replace'))
 ratios=[float(x) for x in re.findall(r'lengthen ratio: ([\d.]+)',log)]
 planned=[float(x) for x in re.findall(r'Exploration planner total time: ([\d.]+)ms',log)]
 handoffs=[(float(a),float(b)) for a,b in re.findall(r'global_and_local_ms=([\d.]+) local_ms=([\d.]+)',log)]
 releases=re.findall(r'\[GoalLifecycle\] release complete=(\d) gain=(\d+) no_progress=(\d) occupied=(\d)',log)
 row={'trial':name,'path_m':float(distance.sum()),'translation_s':float(dt[~still].sum()),'stationary_turn_s':float(dt[still&(angle>.01)].sum()),'stationary_no_turn_s':float(dt[still&(angle<=.01)].sum()),'moving_mean_mps':float(distance[~still].sum()/dt[~still].sum()),'yaw_total_rad':float(angle.sum()),'legacy_compressions':len(ratios),'median_time_scale':float(np.median(ratios)) if ratios else None,'global_calls':log.count('[ExplorationManager] Hgrid update map time (total,'),'legacy_global_planning_sum_s':sum(planned)/1000,'handoff_call_s':sum(a for a,b in handoffs)/1000,'local_rebuild_s':sum(b for a,b in handoffs)/1000,'handoff_calls':len(handoffs),'selected_goals':log.count('[GoalLifecycle] selected='),'release_gain0_not_reached':sum(c=='0' and g=='0' for c,g,n,o in releases),'release_completed':sum(c=='1' for c,g,n,o in releases),'release_unfinished_gain_1_63':sum(c=='1' and 0<int(g)<64 for c,g,n,o in releases),'failed20sec':sum(n=='1' for c,g,n,o in releases)}
 print(json.dumps(row,ensure_ascii=False))
