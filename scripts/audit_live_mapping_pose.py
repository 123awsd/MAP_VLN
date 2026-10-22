#!/usr/bin/env python3
"""Compare actual MapServer poses to exact-stamp optical sensor poses.

This is a bounded live-window audit, not an audit of the entire exploration.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--duration',type=float,default=60)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
source={};mapped={}
def pose(m):
 t=m.transform.translation;q=m.transform.rotation
 source[m.header.stamp.to_nsec()]=([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
def map_pose(m):
 t=m.pose.pose.position;q=m.pose.pose.orientation
 mapped[m.header.stamp.to_nsec()]=([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
rospy.init_node('mapping_pose_window_audit',anonymous=True)
rospy.Subscriber('/uav_simulator/sensor_pose',TransformStamped,pose,queue_size=100)
rospy.Subscriber('/voxel_mapping/interpolated_pose',Odometry,map_pose,queue_size=100)
started=time.monotonic()
while time.monotonic()-started<a.duration and not rospy.is_shutdown():time.sleep(.1)
matched=sorted(source.keys()&mapped.keys())
position_errors=[];angle_errors=[]
for s in matched:
 position_errors.append(float(np.linalg.norm(np.array(source[s][0])-mapped[s][0])))
 q1=np.array(source[s][1]);q2=np.array(mapped[s][1]);dot=abs(np.dot(q1,q2)/(np.linalg.norm(q1)*np.linalg.norm(q2)))
 angle_errors.append(float(np.rad2deg(2*np.arccos(np.clip(dot,0,1)))))
r={'scope':'live observation window only','elapsed_s':time.monotonic()-started,'source_pose_stamps':len(source),'mapped_pose_stamps':len(mapped),'exact_matches':len(matched),'mapped_without_exact_source':len(mapped.keys()-source.keys()),'source_without_mapped_pose':len(source.keys()-mapped.keys()),'max_position_error_m':max(position_errors)if matched else None,'max_optical_orientation_error_deg':max(angle_errors)if matched else None,'full_run_sync_proven':False}
a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
