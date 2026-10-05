#!/usr/bin/env python3
import json, os, sys
import rospy, math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

mission = json.loads(open(sys.argv[1]).read())
candidates = json.loads(open(sys.argv[2]).read())
yaw_lines=[]
with open(sys.argv[3]) as f:
    lines=[x.split() for x in f if x.strip()]
for n,row in enumerate(lines):
    if row[0]=='YAW': yaw_lines.append([float(x) for x in row[1:]])
points=[]
point_segments=[]
for seg_index,seg in enumerate(mission.get('segments',[])):
    t=seg.get('validated_trajectory') or {}
    seg_points=t.get('points_xyz_m') or seg.get('points_xyz_m',[])
    points.extend(seg_points)
    point_segments.extend([seg_index]*len(seg_points))
if not points: raise SystemExit('mission has no validated trajectory points')
rospy.init_node('final_trajectory_animation', anonymous=False)
path_pub=rospy.Publisher('/drone_room/animated_trajectory',Path,queue_size=1,latch=True)
marker_pub=rospy.Publisher('/drone_room/drone_marker',Marker,queue_size=1)
view_pub=rospy.Publisher('/drone_room/animated_viewpoints',MarkerArray,queue_size=1,latch=True)
tf_pub=tf2_ros.TransformBroadcaster()
rate=rospy.Rate(float(os.environ.get('RVIZ_ANIMATION_FPS','30')))
speed=max(0.05,float(os.environ.get('RVIZ_ANIMATION_SPEED','1.0')))
step_accumulator=0.0
yaw_rate_limit=float(os.environ.get('RVIZ_YAW_RATE_RPS','1.0'))
current_yaw=None
path=Path(); path.header.frame_id='map'; i=0
initial_hold_seconds=max(0.0,float(os.environ.get('RVIZ_INITIAL_HOLD_SECONDS','60')))
if initial_hold_seconds > 0:
    rospy.loginfo('Holding trajectory at start for %.1f seconds while RViz renders the point cloud', initial_hold_seconds)
    rospy.sleep(initial_hold_seconds)
by_task={k:v for k,v in candidates.get('by_task',{}).items()}
# These filters affect the video display only, leaving planner artifacts intact.
executed_candidate_ids={v.get('candidate_id') for v in mission.get('visits',[])}
reached_execution_ids=set()
hidden_candidate_ids={
    'recover_laptop_on_reading_table__L1_boxer_reviewed_002__r1.00__006',
}
for task, task_candidates in by_task.items():
    visible=[c for c in task_candidates
             if c.get('id') not in hidden_candidate_ids
             and (task != 'search_laptop_on_lounge_table'
                  or c.get('id') in executed_candidate_ids)]
    # Prefer the execution reference when regenerated IDs describe the same pose.
    visible.sort(key=lambda c: c.get('id') not in executed_candidate_ids)
    unique={}
    for c in visible:
        pose_key=tuple(round(float(c.get('pose',{}).get(k,0)),6)
                       for k in ('x','y','z','yaw'))
        unique.setdefault(pose_key,c)
    by_task[task]=list(unique.values())
def viewpoint_markers(visit, now):
    task=visit.get('task_id',''); selected=visit.get('candidate_id','')
    out=MarkerArray()
    marker_id=0
    # Show the display-filtered planning replay candidates. The active task is
    # highlighted; unvisited candidates remain visible until reached.
    for candidate_task, task_candidates in by_task.items():
      for c in task_candidates:
        q=c.get('pose',{}); x,y,z=float(q.get('x',0)),float(q.get('y',0)),float(q.get('z',0)); yaw=float(q.get('yaw',0))
        hf=math.radians(float(c.get('horizontal_fov_deg',90.0)))/2; vf=math.radians(float(c.get('vertical_fov_deg',70.0)))/2; depth=0.32
        fx,fy=math.cos(yaw),math.sin(yaw); rx,ry=-fy,fx; center=(x+fx*depth,y+fy*depth,z)
        corners=[(center[0]+rx*math.tan(hf)*depth*a,center[1]+ry*math.tan(hf)*depth*a,center[2]+math.tan(vf)*depth*b) for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]]
        m=Marker(); m.header.frame_id='map'; m.header.stamp=now; m.ns='candidate_viewpoints'; m.id=marker_id; marker_id += 1; m.type=Marker.LINE_LIST; m.action=Marker.ADD; m.scale.x=0.012
        selected_pose=visit.get('pose',{})
        pose_dist=sum((float(c.get('pose',{}).get(k,0))-float(selected_pose.get(k,0)))**2 for k in ('x','y','z'))
        # Preview candidates may be regenerated with different IDs. In that
        # case retain the selected execution pose by nearest geometric pose.
        same_task=candidate_task==task
        # Keep the selected candidate highlighted until the drone actually
        # reaches its pose; changing task index alone must not fade it early.
        hi=same_task and (c.get('id')==selected or pose_dist < 0.01) and c.get('id') not in reached_execution_ids
        active=same_task
        m.color.r,m.color.g,m.color.b=(0.0,0.9,1.0) if hi else (0.55,0.55,0.55); m.color.a=0.95 if hi else (0.52 if active else 0.24)
        from geometry_msgs.msg import Point
        origin=Point(x=x,y=y,z=z)
        for q in corners: m.points.extend([origin,Point(x=q[0],y=q[1],z=q[2])])
        for a,b in zip(corners,corners[1:]+corners[:1]): m.points.extend([Point(x=a[0],y=a[1],z=a[2]),Point(x=b[0],y=b[1],z=b[2])])
        out.markers.append(m)
    return out
def current_frustum(p, yaw, now):
    m=Marker(); m.header.frame_id='map'; m.header.stamp=now; m.ns='current_camera_frustum'; m.id=0; m.type=Marker.LINE_LIST; m.action=Marker.ADD; m.scale.x=0.025; m.color.r=1.0; m.color.g=0.85; m.color.b=0.0; m.color.a=0.9
    hf=math.radians(90)/2; vf=math.radians(70)/2; d=0.32; x,y,z=map(float,p); cx,sy=math.cos(yaw),math.sin(yaw); right=(-sy,cx,0); front=(cx,sy,0)
    c=(x+front[0]*d,y+front[1]*d,z); corners=[]
    for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]: corners.append((c[0]+front[0]*0+right[0]*math.tan(hf)*d*a,c[1]+right[1]*math.tan(hf)*d*a,c[2]+math.tan(vf)*d*b))
    for q in corners: m.points.extend([__import__('geometry_msgs.msg',fromlist=['Point']).Point(x=x,y=y,z=z),__import__('geometry_msgs.msg',fromlist=['Point']).Point(x=q[0],y=q[1],z=q[2])])
    for a,b in zip(corners,corners[1:]+corners[:1]): m.points.extend([__import__('geometry_msgs.msg',fromlist=['Point']).Point(x=a[0],y=a[1],z=a[2]),__import__('geometry_msgs.msg',fromlist=['Point']).Point(x=b[0],y=b[1],z=b[2])])
    return m
visits=mission.get('visits',[]); seg_idx=0
while not rospy.is_shutdown():
    p=points[min(i,len(points)-1)]; now=rospy.Time.now(); path.header.stamp=now
    pose=PoseStamped(); pose.header=path.header
    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,p); pose.pose.orientation.w=1.0
    path.poses.append(pose); path_pub.publish(path)
    tr=TransformStamped(); tr.header.stamp=now; tr.header.frame_id='map'; tr.child_frame_id='drone_visual'
    tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,p); tr.transform.rotation.w=1.0; tf_pub.sendTransform(tr)
    m=Marker(); m.header=path.header; m.ns='animated_drone'; m.id=0; m.type=Marker.SPHERE; m.action=Marker.ADD; m.pose=pose.pose; m.scale.x=m.scale.y=m.scale.z=0.28; m.color.r=1.0; m.color.g=0.85; m.color.a=1.0; marker_pub.publish(m)
    if visits:
        # Use the actual segment boundary from mission_plan, rather than
        # estimating phase from the global point-count ratio.
        seg_idx=point_segments[min(i,len(point_segments)-1)]
        visit=visits[min(seg_idx,len(visits)-1)]
        nxt=points[min(i+1,len(points)-1)]
        local_start=sum(len((s.get('validated_trajectory') or {}).get('points_xyz_m',[])) for s in mission.get('segments',[])[:seg_idx])
        local_len=max(1,len((mission['segments'][seg_idx].get('validated_trajectory') or {}).get('points_xyz_m',[])))
        if seg_idx < len(yaw_lines) and yaw_lines[seg_idx]:
            profile=yaw_lines[seg_idx]; u=min(1.0,max(0.0,(i-local_start)/max(1,local_len-1))); desired=profile[min(len(profile)-1,int(round(u*(len(profile)-1))))]
        else:
            desired=math.atan2(float(nxt[1])-float(p[1]), float(nxt[0])-float(p[0])) if i+1<len(points) and abs(float(nxt[0])-float(p[0]))+abs(float(nxt[1])-float(p[1]))>1e-5 else float(visit.get('pose',{}).get('yaw',0))
        if current_yaw is None: current_yaw=desired
        delta=(desired-current_yaw+math.pi)%(2*math.pi)-math.pi
        dt=1.0/max(1.0,float(os.environ.get('RVIZ_ANIMATION_FPS','30')))
        current_yaw += max(-yaw_rate_limit*dt,min(yaw_rate_limit*dt,delta))
        # Mark an execution candidate reached only when the animated pose
        # is physically at its target, rather than at a segment transition.
        selected_pose=visit.get('pose',{})
        selected_dist=math.sqrt(sum((float(p[j])-float(selected_pose.get(k,0)))**2 for j,k in enumerate(('x','y','z'))))
        selected_yaw=float(selected_pose.get('yaw',0.0))
        yaw_error=abs((current_yaw-selected_yaw+math.pi)%(2*math.pi)-math.pi)
        # Fade only after the animated camera is visually coincident with the
        # execution candidate: centimetric position and small yaw tolerance.
        if selected_dist <= 0.03 and yaw_error <= math.radians(5.0) and visit.get('candidate_id'):
            reached_execution_ids.add(visit.get('candidate_id'))
        va=viewpoint_markers(visit, now); va.markers.append(current_frustum(p, current_yaw, now)); view_pub.publish(va)
    step_accumulator += speed
    advance = int(step_accumulator)
    step_accumulator -= advance
    next_i = i + advance
    if next_i >= len(points):
        # Clear the latched path before replaying from the start.  Otherwise
        # RViz keeps the previous cycle's tail while the new cycle begins.
        path.poses = []
        path.header.stamp = rospy.Time.now()
        path_pub.publish(path)
        i = 0
        seg_idx = 0
        current_yaw = None
        reached_execution_ids.clear()
        rospy.sleep(0.15)
    else:
        i = next_i
        if visits and point_segments:
            seg_idx = point_segments[min(i,len(point_segments)-1)]
    rate.sleep()
