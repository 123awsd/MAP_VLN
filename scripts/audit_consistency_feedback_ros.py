#!/usr/bin/env python3
"""Check capture/ROS transport correspondence using real ROS bag messages."""
import argparse,json
from pathlib import Path
import rosbag
import numpy as np
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();out=a.trial
rows=[json.loads(l) for l in (out/'capture_trace.jsonl').open()];reference={round(r['capture_wall_s'],6):r for r in rows};errors=[];missing=0;repeated=0;bad_repeats=0;seen={};images=[]
with rosbag.Bag(str(out/'trial.bag')) as bag:
 for topic,m,_ in bag.read_messages(topics=['/uav_simulator/odometry','/uav_simulator/depth_image']):
  stamp=round(m.header.stamp.to_sec(),6);r=reference.get(stamp)
  if r is None:missing+=1;continue
  if topic.endswith('depth_image'):images.append(stamp);continue
  v=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z];pos=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
  errors.append(float(np.linalg.norm(np.array(v)-r['velocity_world'])))
  payload=(tuple(pos),tuple(v))
  if stamp in seen:repeated+=1;bad_repeats+=seen[stamp]!=payload
  seen[stamp]=payload
checks={'all_transport_stamps_are_capture_stamps':missing==0,'velocity_transport_matches_capture_difference':bool(errors) and max(errors)<1e-8,'duplicate_stamp_payload_unchanged':bad_repeats==0,'depth_not_republished_as_new_frame':len(images)==len(set(images))}
report={'checks':checks,'passed':all(checks.values()),'odom_unique_capture_stamps':len(seen),'repeated_odom_messages':repeated,'unmatched_messages':missing,'max_velocity_transport_error':max(errors,default=None)}
(out/'feedback_ros_test.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
if not report['passed']:raise SystemExit(1)
