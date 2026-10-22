#!/usr/bin/env python3
"""Read-only wire trajectory reconstruction; no occupancy/map replay."""
import json,math
from pathlib import Path
import numpy as np
import rosbag
root=Path('/workspace/shared');p=root/'outputs/consistency_A3_search_round_504_r1_20261016';frames=[json.loads(l)for l in(p/'capture_trace.jsonl').open()];t0=frames[0]['capture_wall_s'];es=json.loads((p/'execution_events.json').read_text())
def evaluate(k,pts,t):
 d=3;t=max(k[d],min(t,k[len(pts)]));span=max(d,min(int(np.searchsorted(k,t,side='left'))-1,len(pts)-1));v=[np.array(pts[j],dtype=float)for j in range(span-d,span+1)]
 for r in range(1,d+1):
  for j in range(d,r-1,-1):
   i=span-d+j;den=k[i+d-r+1]-k[i];alpha=(t-k[i])/den if den else 0.;v[j]=(1-alpha)*v[j-1]+alpha*v[j]
 return v[d]
wire={}
with rosbag.Bag(str(p/'trial.bag'))as bag:
 for _,m,t in bag.read_messages(topics=['/planning/bspline']):
  if m.traj_id not in [183,184]:continue
  k=np.array(m.knots);pts=np.array([[v.x,v.y,v.z]for v in m.pos_pts]);duration=float(k[len(pts)]-k[3]);samples=[evaluate(k,pts,k[3]+x)for x in np.linspace(0,duration,201)];ys=np.array(m.yaw_pts);yk=(np.arange(len(ys)+4)-3)*m.yaw_dt
  wire[m.traj_id]={'id':m.traj_id,'start_time':m.start_time.to_sec(),'relative_start_s':m.start_time.to_sec()-t0,'duration_s':duration,'end_time':m.start_time.to_sec()+duration,'relative_end_s':m.start_time.to_sec()+duration-t0,'start_position':samples[0].tolist(),'end_position':samples[-1].tolist(),'start_yaw':float(evaluate(yk,ys,0)),'end_yaw':float(evaluate(yk,ys,yk[len(ys)])),'sampled_path_m':sum(float(np.linalg.norm(a-b))for a,b in zip(samples,samples[1:])),'min_z':min(float(v[2])for v in samples),'max_z':max(float(v[2])for v in samples),'path_samples':[v.tolist()for v in samples]}
plan_start=next(e for e in es if e['event']=='planning_start'and abs(e['ros_s']-1792092050.192109)<.001);p183=next(e for e in es if e['event']=='published'and e['id']==183);p184=next(e for e in es if e['event']=='published'and e['id']==184)
actual_frame=max((f for f in frames if f['capture_wall_s']<=plan_start['ros_s']),key=lambda f:f['capture_wall_s'])
r={'wire':wire,'plan223_start':plan_start,'goal183':p183['goal'],'goal184':p184['goal'],'predicted_distance_to_goal183_m':math.dist(plan_start['predicted_start'],p183['goal']),'feedback_distance_to_goal183_m':math.dist(plan_start['actual'],p183['goal']),'latest_capture_distance_to_goal183_m':math.dist(actual_frame['position'],p183['goal']),'latest_capture':{k:actual_frame[k]for k in ['capture_wall_s','position','sequence']},'183_remaining_at_plan223_start_s':wire[183]['end_time']-plan_start['ros_s'],'183_remaining_at_publish184_s':wire[183]['end_time']-p184['ros_s'],'gap_183_end_to_184_start_s':wire[184]['start_time']-wire[183]['end_time'],'capture_near_switch':[{'relative_s':f['capture_wall_s']-t0,'position':f['position']}for f in frames if 399.5<=f['capture_wall_s']-t0<=411],'limits':['Trajectory sampling is wire reconstruction, not map safety replay.','Published goal and local trajectory endpoint are distinct quantities.']}
out=root/'outputs/consistency_validation_20261015/target183_184_wire_audit.json';out.write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items()if k not in ['wire','capture_near_switch','latest_capture','limits','plan223_start']}));print({i:{k:v for k,v in d.items()if k!='path_samples'}for i,d in wire.items()})
