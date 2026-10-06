#!/usr/bin/env python3
"""Display approved room boundaries without changing map/planning artifacts."""
import json, sys, math
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

scene=json.load(open(sys.argv[1]))
mission=json.load(open(sys.argv[2]))
rospy.init_node('lab_room_preview')
pub=rospy.Publisher('/lab_rooms/map', MarkerArray, queue_size=1, latch=True)
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
    m=marker('room_boundary',Marker.LINE_STRIP,col); m.scale.x=.07
    m.points=[Point(x=float(x),y=float(y),z=0) for x,y in polygon+[polygon[0]]]
    label=marker('room_name',Marker.TEXT_VIEW_FACING,col); label.scale.z=1.0
    x,y=room.get('centroid_xy_m',[sum(p[0] for p in polygon)/len(polygon),sum(p[1] for p in polygon)/len(polygon)])
    label.pose.position.x=x; label.pose.position.y=y; label.pose.position.z=.1
    label.text=room.get('name',room['id']).replace('_',' ')
seen=set()
for visit in mission.get('visits',[]):
    task=visit.get('task_id')
    if task in seen: continue
    seen.add(task); q=visit['pose']
    m=marker('task_position',Marker.SPHERE,(1,.85,.15)); m.scale.x=m.scale.y=.45; m.scale.z=.1
    m.pose.position.x=q['x'];m.pose.position.y=q['y'];m.pose.position.z=.08
pub.publish(out)
rospy.loginfo('Published %d room and task markers',len(out.markers))
rospy.spin()
