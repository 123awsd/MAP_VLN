#!/usr/bin/env python3
"""Offline ROS-bag execution audit. Never provides exploration targets."""
import argparse,bisect,json,math
from pathlib import Path
import numpy as np
import rosbag
from exploration_sample_times import sample_times
p=argparse.ArgumentParser();p.add_argument('trial');a=p.parse_args();out=Path(a.trial)
rows=[json.loads(l) for l in (out/'live_trajectory.jsonl').read_text().splitlines()]
commands=[];odometry=[];stops=[];frames=[];feedback=[];trajectories=[]
with rosbag.Bag(str(out/'trial.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/planning/pos_cmd','/uav_simulator/odometry','/planning/replan','/habitat/rgb','/preview/motion_blocked','/planning/bspline']):
  ts=t.to_sec()
  if topic=='/planning/pos_cmd':commands.append((ts,int(m.trajectory_id),[m.position.x,m.position.y,m.position.z],[m.velocity.x,m.velocity.y,m.velocity.z]))
  elif topic=='/uav_simulator/odometry':odometry.append((ts,[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z]))
  elif topic=='/planning/replan':stops.append((ts,m.data))
  elif topic=='/habitat/rgb':frames.append((ts,int(m.header.seq)))
  elif topic=='/preview/motion_blocked':feedback.append((ts,int(m.data)))
  else:trajectories.append((ts,int(m.traj_id),m.start_time.to_sec()))
ct=[r[0] for r in commands];ot=[r[0] for r in odometry];t0=frames[0][0]
def q(values):
 return {k:float(np.percentile(values,v)) for k,v in [('p50',50),('p95',95),('max',100)]} if values else {}
errors=[];late_errors=[];zero_vel_errors=[]
for t,seq in frames:
 i=bisect.bisect_right(ct,t)-1;j=bisect.bisect_right(ot,t)-1
 if i<0 or j<0:continue
 c,o=commands[i],odometry[j];error=float(np.linalg.norm(np.array(c[2])-o[1]));errors.append(error)
 if t-t0>=300:
  late_errors.append(error)
  if np.linalg.norm(c[3])<1e-5:zero_vel_errors.append(error)
stop_rows=[]
for t,kind in stops:
 if kind!=1:continue
 j=bisect.bisect_right(ot,t)-1;i=bisect.bisect_left(ct,t+.10)
 if j<0 or i>=len(commands):continue
 c,o=commands[i],odometry[j]
 stop_rows.append({'time_s':t-t0,'trajectory_id':c[1],'command_position':c[2],'actual_position':o[1], 'post_stop_velocity':float(np.linalg.norm(c[3])),'post_stop_error_m':float(np.linalg.norm(np.array(c[2])-o[1]))})
positions=np.array([r['position'] for r in rows]);delta=np.linalg.norm(np.diff(positions,axis=0),axis=1)
# Sequence times are simulator sample times; ROS frame timestamps are close but asynchronous.
stationary=np.r_[False,delta<1e-6];edges=np.diff(np.r_[False,stationary,False].astype(int));runs=[]
sample_seconds,sample_timing_method=sample_times(rows,out)
def sample_time(row):
 return sample_seconds[row_index[row['sequence']]]
row_index={row['sequence']:i for i,row in enumerate(rows)}
for start,end in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
 duration=sample_time(rows[end-1])-sample_time(rows[max(0,start-1)])
 if duration>=5:runs.append({'first_sequence_s':sample_time(rows[start]),'last_sequence_s':sample_time(rows[end-1]),'duration_s':duration,'samples':int(end-start),'position':rows[start]['position']})
result={'trial':out.name,'alignment':'Last ROS command and odometry before received RGB; approximate within frame/bridge latency. Stop checks use first command >=100ms after stop and last actual pose before stop.', 'command_actual_error_m':q(errors),'after_300s_command_actual_error_m':q(late_errors),'after_300s_zero_velocity_command_error_m':q(zero_vel_errors),'after_300s_zero_velocity_command_samples':len(zero_vel_errors),'stops':len(stop_rows),'post_stop_error_m':q([r['post_stop_error_m'] for r in stop_rows]),'worst_stop_examples':sorted(stop_rows,key=lambda r:r['post_stop_error_m'],reverse=True)[:5],'stationary_runs':runs,'feedback_count':len(feedback),'published_trajectories':len(trajectories),'late_publication_count':sum(t>start for t,_,start in trajectories),'first_feedback_events':[(t-t0,i) for t,i in feedback[:10]],'odom_velocity_max':max(float(np.linalg.norm(r[2])) for r in odometry)}
result['sample_timing_method']=sample_timing_method
(out/'execution_contract_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
