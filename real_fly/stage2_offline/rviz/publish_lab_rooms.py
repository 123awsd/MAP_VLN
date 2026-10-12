#!/usr/bin/env python3
"""Display approved room boundaries without changing map/planning artifacts."""
import json, sys, math, os
from pathlib import Path
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

scene=json.load(open(sys.argv[1]))
mission=json.load(open(sys.argv[2]))
# Display-only stairwell outline traced against the 0.8--1.8 m PCD wall slice.
# The archived room segmentation and planner geometry are left untouched.
for room in scene['rooms']:
    if room['id']=='L1_R01' and os.environ.get('RVIZ_ROOM_OUTLINE_FROM_SCENE','0')!='1':
        room['polygon_xy_m']=[
            [-2.20,-1.15],[1.20,-1.15],[.95,-.85],[1.43,-.40],
            [1.90,-.45],[1.90,.95],[1.45,.95],[1.17,1.55],
            [1.35,1.85],[.85,1.85],[.82,2.15],[-.35,2.15],
            [-.35,1.85],[-1.90,1.90],[-1.90,.75],[-2.20,.75],
            [-2.20,.22],[-2.50,.18],[-2.50,-.90],[-2.20,-.90]]
rospy.init_node('lab_room_preview')
pub=rospy.Publisher('/lab_rooms/map', MarkerArray, queue_size=1, latch=True)
positions_path=Path(os.environ.get('RVIZ_ROOM_LABEL_POSITIONS','/tmp/room_label_positions.json'))
positions=json.loads(positions_path.read_text()) if positions_path.is_file() else {}
labels={}
colors=[(.4,.75,1),(.9,.65,.35),(.65,.85,.5),(.85,.6,.9),(.5,.85,.8)]
out=MarkerArray()
def marker(ns, kind, color):
    m=Marker(); m.header.frame_id='map'; m.ns=ns; m.id=len(out.markers); m.type=kind; m.action=Marker.ADD; m.pose.orientation.w=1
    m.color.r,m.color.g,m.color.b=color; m.color.a=1; out.markers.append(m); return m
def triangulate(poly):
    vertices=list(poly)
    if sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(vertices,vertices[1:]+vertices[:1])) < 0:
        vertices.reverse()
    triangles=[]
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    while len(vertices)>3:
        for i,b in enumerate(vertices):
            a=vertices[i-1]; c=vertices[(i+1)%len(vertices)]
            if cross(a,b,c)<=1e-9: continue
            if any(cross(a,b,p)>=-1e-9 and cross(b,c,p)>=-1e-9 and cross(c,a,p)>=-1e-9
                   for j,p in enumerate(vertices) if j not in {i,(i-1)%len(vertices),(i+1)%len(vertices)}): continue
            triangles.extend([a,b,c]); vertices.pop(i); break
        else: raise ValueError('Room polygon cannot be triangulated')
    return triangles+vertices
for i, room in enumerate(scene['rooms']):
    polygon=room.get('polygon_xy_m',[])
    if not polygon: continue
    col=colors[i%len(colors)]
    fill=marker('room_fill',Marker.TRIANGLE_LIST,col); fill.scale.x=fill.scale.y=fill.scale.z=1; fill.color.a=.16
    fill.points=[Point(x=float(x),y=float(y),z=-.02) for x,y in triangulate(polygon)]
    m=marker('room_boundary',Marker.LINE_STRIP,col)
    m.scale.x=float(os.environ.get('RVIZ_ROOM_BOUNDARY_WIDTH','0.025'))
    m.color.a=float(os.environ.get('RVIZ_ROOM_BOUNDARY_ALPHA','0.4'))
    m.points=[Point(x=float(x),y=float(y),z=0) for x,y in polygon+[polygon[0]]]
    label=marker('room_name',Marker.TEXT_VIEW_FACING,col); label.scale.z=.32
    x,y=room.get('centroid_xy_m',[sum(p[0] for p in polygon)/len(polygon),sum(p[1] for p in polygon)/len(polygon)])
    saved=positions.get(room['id'],{})
    label.pose.position.x=float(saved.get('x',x)); label.pose.position.y=float(saved.get('y',y)); label.pose.position.z=.1
    labels[room['id']]=label
    label.text=room.get('name',room['id']).replace('_',' ')
pub.publish(out)
server=InteractiveMarkerServer('lab_room_labels')
menu=MenuHandler()
def save_positions(feedback):
    try:
        positions_path.parent.mkdir(parents=True,exist_ok=True)
        temporary=positions_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(positions,ensure_ascii=False,indent=2))
        temporary.replace(positions_path)
        rospy.loginfo('Saved room label positions to %s',positions_path)
    except OSError as exc:
        rospy.logerr('Cannot save room label positions: %s',exc)
menu.insert('Save all label positions',callback=save_positions)
def moved(feedback):
    if feedback.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE,InteractiveMarkerFeedback.MOUSE_UP): return
    label=labels[feedback.marker_name]
    label.pose.position.x=feedback.pose.position.x
    label.pose.position.y=feedback.pose.position.y
    positions[feedback.marker_name]={'x':feedback.pose.position.x,'y':feedback.pose.position.y}
    feedback.pose.position.z=.1
    server.setPose(feedback.marker_name,feedback.pose)
    server.applyChanges()
    pub.publish(out)
for room_id,label in labels.items():
    handle=InteractiveMarker(); handle.header.frame_id='map';handle.name=room_id
    handle.description='Drag on XY plane; right-click to save'
    handle.pose.position=label.pose.position;handle.pose.orientation.w=1;handle.scale=.5
    control=InteractiveMarkerControl();control.name='move_label'
    control.orientation.w=math.sqrt(.5);control.orientation.y=math.sqrt(.5)
    control.interaction_mode=InteractiveMarkerControl.MOVE_PLANE;control.always_visible=True
    grip=Marker();grip.type=Marker.SPHERE;grip.pose.orientation.w=1
    grip.scale.x=grip.scale.y=.12;grip.scale.z=.04
    grip.color.r=grip.color.g=grip.color.b=1;grip.color.a=.8
    control.markers.append(grip);handle.controls.append(control)
    server.insert(handle,moved);menu.apply(server,room_id)
server.applyChanges()
rospy.loginfo('Published %d room markers',len(out.markers))
rospy.spin()
