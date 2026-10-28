#!/usr/bin/env python3
"""RViz display of the paper fixture and exact MINCO samples; no control output."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rospy
from rviz import bindings as rviz
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from python_qt_binding.QtWidgets import QApplication,QDockWidget
from python_qt_binding.QtCore import QTimer

p=argparse.ArgumentParser();p.add_argument('--publish-only',action='store_true');p.add_argument('--quit-after',type=float,default=0)
a=p.parse_args();root=Path(__file__).resolve().parents[1]
e=json.loads((root/'mechanism_evidence.json').read_text());d=e['input'];cycle=e['initial']
rospy.init_node('paper_joint_preview',anonymous=True,disable_signals=True)
static=rospy.Publisher('/paper_joint/static',MarkerArray,queue_size=1,latch=True)
vehicle=rospy.Publisher('/paper_joint/vehicle',MarkerArray,queue_size=1)
colors={'A':'159D77','B':'2485D2','C':'9064C8','D':'F08042'}
pale={'A':'D9F4E7','B':'DCEEFF','C':'EEE4FA','D':'FFE6CF'}
scene=MarkerArray();seq=0

def marker(kind,xyz=(0,0,0),scale=(1,1,1),color='536D7D',alpha=1.,ns='scene'):
 global seq
 m=Marker();m.header.frame_id='map';m.ns=ns;m.id=seq;seq+=1;m.type=kind;m.action=Marker.ADD
 m.pose.position.x,m.pose.position.y,m.pose.position.z=xyz;m.pose.orientation.w=1.
 m.scale.x,m.scale.y,m.scale.z=scale
 m.color.r,m.color.g,m.color.b=[int(color[i:i+2],16)/255 for i in (0,2,4)];m.color.a=alpha
 return m

def cube(box,z,h,color):
 x1,y1,x2,y2=box;m=marker(Marker.CUBE,((x1+x2)/2,(y1+y2)/2,z),(x2-x1,y2-y1,h),color)
 scene.markers.append(m)

def label(s,xyz,color,size=.28):
 m=marker(Marker.TEXT_VIEW_FACING,xyz,(0,0,size),color);m.text=s;scene.markers.append(m)

def lines(points,color,width=.035,kind=Marker.LINE_STRIP,alpha=1.):
 m=marker(kind,scale=(width,0,0),color=color,alpha=alpha)
 m.points=[Point(*[float(q) for q in x]) for x in points];scene.markers.append(m)

cube([0,0,12,9],-.03,.06,'F5F5F0')
for r in d['rooms']:
 t=r['task'];cube(r['box'],.015,.025,pale[t]);x,y=r['anchor']
 anchor_color=colors[t] if t in cycle['available'] else '98A8AF'
 scene.markers.append(marker(Marker.CYLINDER,(x,y,.82),(.33,.33,.055),anchor_color))
 label(t,(x,y,1.06),anchor_color,.30)
 x1,y1,x2,y2=r['box'];label(r['name'],((x1+x2)/2,y2-.38,.12),colors[t],.28)
for b in [[0,0,12,.2],[0,8.8,12,9],[0,0,.2,9],[11.8,0,12,9]]+d['walls']:
 cube(b,.30,.60,'A5B6C2')
for f in d['furniture']:
 b=f['box'];x1,y1,x2,y2=b
 if f['kind']=='table':
  cube(b,.65,.12,'E9C789')
  for x in [x1+.08,x2-.16]:
   for y in [y1+.08,y2-.16]:cube([x,y,x+.08,y+.08],.30,.60,'C3A472')
 elif f['kind']=='sofa':
  cube(b,.31,.55,'E7C68C');cube([x1,y2-.18,x2,y2],.56,.35,'D3AE6A')
 else:cube(b,.34,.65,'E9C789')
chosen={v['candidate_id'] for v in cycle['plan']['visits']}
for v in d['views']:
 selected=v['id'] in chosen;t=v['task'];x,y=v['xy'];theta=math.atan2(v['aim'][1]-y,v['aim'][0]-x)
 origin=np.array([x,y,.9]);front=np.array([math.cos(theta),math.sin(theta),0]);side=np.array([-math.sin(theta),math.cos(theta),0])
 depth=.45 if selected else .35
 corners=[origin+depth*front+side*depth*.62*a+np.array([0,0,depth*.33*b]) for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]]
 pairs=[]
 for c in corners:pairs.extend([origin,c])
 for aa,bb in zip(corners,corners[1:]+corners[:1]):pairs.extend([aa,bb])
 lines(pairs,colors[t] if selected else '9EAEB9',.022 if selected else .012,Marker.LINE_LIST,1 if selected else .55)
 if selected:scene.markers.append(marker(Marker.SPHERE,origin,(.105,.105,.105),colors[t]))
for phase in cycle['minco']:lines(phase['points'],'497894',.045)
start=np.array(cycle['start'][:3])
label('Start',(start[0]-.4,start[1],.7),'26343D',.24)
static.publish(scene)

if not a.publish_only:
 app=QApplication([])
 frame=rviz.VisualizationFrame();frame.setSplashPath('');frame.initialize()
 reader=rviz.YamlConfigReader();config=rviz.Config();reader.readFile(config,str(Path(__file__).with_name('scene.rviz')));frame.load(config)
 frame.getManager().startUpdate()
 frame.setMenuBar(None);frame.setStatusBar(None);frame.setHideButtonVisibility(False)
 for dock in frame.findChildren(QDockWidget):dock.hide()
 frame.resize(1500,1000);frame.show()
started=time.monotonic()
allpoints=np.vstack([ph['points'] for ph in cycle['minco']])
times=np.concatenate([np.asarray(ph['times'])+sum(pp['duration'] for pp in cycle['minco'][:i]) for i,ph in enumerate(cycle['minco'])])

def tick():
 global seq
 elapsed=time.monotonic()-started
 if elapsed<2:static.publish(scene)
 # Hold the single playback vehicle at Start for the native paper capture.
 flight_t=max(0,elapsed-12)*2.0%times[-1]
 pos=np.array([np.interp(flight_t,times,allpoints[:,i]) for i in range(3)])
 seq=0;out=MarkerArray()
 out.markers.append(marker(Marker.SPHERE,pos,(.22,.15,.09),'344D5C',ns='vehicle'))
 arms=marker(Marker.LINE_LIST,scale=(.025,0,0),color='344D5C',ns='vehicle')
 for dx,dy in [(-.17,-.14),(-.17,.14),(.17,-.14),(.17,.14)]:
  tip=pos+np.array([dx,dy,.02]);arms.points.extend([Point(*pos),Point(*tip)])
  out.markers.append(marker(Marker.CYLINDER,tip,(.16,.16,.018),'70838E',ns='vehicle'))
 out.markers.append(arms);vehicle.publish(out)
 if a.quit_after and elapsed>a.quit_after:
  if a.publish_only:rospy.signal_shutdown('preview complete')
  else:app.quit()
if a.publish_only:
 rate=rospy.Rate(30)
 while not rospy.is_shutdown():tick();rate.sleep()
else:
 timer=QTimer();timer.timeout.connect(tick);timer.start(33)
 app.exec_();rospy.signal_shutdown('preview closed')
