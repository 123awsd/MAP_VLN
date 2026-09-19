"""Sample the original bag and capture real RViz panels at identical timestamps."""
import argparse, copy, importlib.util, os, sys, time
from pathlib import Path
import cv2
import numpy as np
import rosbag, rospy, yaml
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from python_qt_binding import QtWidgets, QtCore
from rviz import bindings as rviz

p=argparse.ArgumentParser()
p.add_argument('--frames',type=int,default=201)
p.add_argument('--start',type=float,default=10)
p.add_argument('--end',type=float,default=419)
p.add_argument('--fps',type=float,default=4)
p.add_argument('--lossless',action='store_true')
p.add_argument('--dual-fixed',action='store_true')
p.add_argument('--fixed-view',choices=('side','top'))
p.add_argument('--switch-settle',type=float,default=.35)
p.add_argument('--top-settle',type=float)
p.add_argument('--name',default='exploration_real_rviz_three_views')
a=p.parse_args()
base=Path('/workspace/shared/outputs')
out=base/'video_previews'/a.name
out.mkdir(parents=True,exist_ok=True)
app=QtWidgets.QApplication(sys.argv[:1])
frame=rviz.VisualizationFrame(); frame.setSplashPath(''); frame.initialize()
rospy.init_node('rviz_capture_controller',anonymous=True,disable_signals=True)
rospy.set_param('/use_sim_time',True)
root=Path('/workspace/falcon_ws/src/pre_map_bridge')
cfg=yaml.safe_load((root/'config/stage1.rviz').read_text())
cfg['Panels']=[]
ds=cfg['Visualization Manager']['Displays']
ds[:]=[d for d in ds if d['Class']!='rviz/Image']
for d in ds:
    if d['Class']=='rviz/PointCloud2': d['Queue Size']=1
    if d.get('Topic')=='/pre_map_vln/roofless_map': d['Size (Pixels)']=2
hist=copy.deepcopy(next(d for d in ds if d.get('Topic')=='/pre_map_vln/roofless_map'))
hist.update({'Name':'Explored other floors','Topic':'/preview/history','Color Transformer':'FlatColor','Color':'110; 145; 175','Alpha':0.18,'Size (Pixels)':2})
ds.insert(0,hist)
cfg['Visualization Manager']['Global Options']['Frame Rate']=15
cfg['Window Geometry']={'Width':1120,'Height':520,'X':40,'Y':80,'Hide Left Dock':True,'Hide Right Dock':True}
config_path=out/'capture.rviz'; config_path.write_text(yaml.safe_dump(cfg))
config=rviz.Config();rviz.YamlConfigReader().readFile(config,str(config_path));frame.load(config)
frame.setMenuBar(None);frame.setStatusBar(None);frame.setHideButtonVisibility(False)
for w in frame.findChildren(QtWidgets.QDockWidget):w.hide()
for w in frame.findChildren(QtWidgets.QToolBar):w.hide()
frame.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
frame.resize(1120,520);frame.move(300,350);frame.show();frame.raise_();frame.activateWindow()
def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:app.processEvents();time.sleep(.005)
pump(6.0)
panel=next(w for w in frame.findChildren(QtWidgets.QWidget) if w.metaObject().className()=='rviz::RenderPanel' and w.isVisible() and w.width()>800)
top_frame=top_panel=None
if a.dual_fixed:
    top_config=rviz.Config();rviz.YamlConfigReader().readFile(top_config,str(config_path))
    top_frame=rviz.VisualizationFrame();top_frame.setSplashPath('');top_frame.initialize();top_frame.load(top_config)
    top_frame.setMenuBar(None);top_frame.setStatusBar(None);top_frame.setHideButtonVisibility(False)
    for w in top_frame.findChildren(QtWidgets.QDockWidget):w.hide()
    for w in top_frame.findChildren(QtWidgets.QToolBar):w.hide()
    top_frame.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
    frame.move(0,0);top_frame.resize(1120,520);top_frame.move(1200,0);top_frame.show();top_frame.raise_()
    pump(2.0)
    top_panel=next(w for w in top_frame.findChildren(QtWidgets.QWidget) if w.metaObject().className()=='rviz::RenderPanel' and w.isVisible() and w.width()>800)
spec=importlib.util.spec_from_file_location('roofless',str(root/'scripts/roofless_visualization_node.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
rospy.set_param('~track_floor_from_odom',False)
renderer=module.RooflessVisualization()
history=rospy.Publisher('/preview/history',PointCloud2,queue_size=1,latch=True)
clock=rospy.Publisher('/clock',Clock,queue_size=1)
pubs={}
topics=['/voxel_mapping/occupancy_grid_occupied','/habitat/rgb','/uav_simulator/odometry','/uav_simulator/sensor_pose','/pre_map_vln/agent_path','/planning/bspline','/planning_vis/frontier_pcl']
def cloud(points,header):
    msg=PointCloud2();msg.header=header;msg.height=1;msg.width=len(points)
    msg.fields=[PointField(n,4*i,PointField.FLOAT32,1) for i,n in enumerate(('x','y','z'))]
    msg.point_step=12;msg.row_step=12*len(points);msg.is_dense=True;msg.data=np.asarray(points,dtype='<f4').tobytes();return msg
def xyz(msg):
    fields={f.name:f.offset for f in msg.fields}
    return np.column_stack([np.ndarray((msg.width*msg.height,),dtype='>f4' if msg.is_bigendian else '<f4',buffer=msg.data,offset=fields[n],strides=(msg.point_step,)) for n in ('x','y','z')])
def set_view(target_frame,pitch,top=False):
    vm=target_frame.getManager().getViewManager()
    vm.setCurrentViewControllerType('rviz/TopDownOrtho' if top else 'rviz/Orbit')
    view=vm.getCurrent()
    if top:
        for key,value in [('Scale',20.0),('X',1.0),('Y',7.6),('Angle',0.0)]:view.subProp(key).setValue(value)
    else:
        for key,value in [('Distance',27.0),('Pitch',pitch),('Yaw',4.08)]:view.subProp(key).setValue(value)
        for key,value in [('X',1.0),('Y',7.6),('Z',5.0)]:view.subProp('Focal Point').subProp(key).setValue(value)
def capture(target_frame,target_panel,pitch,top=False):
    if not a.dual_fixed:
        set_view(target_frame,pitch,top)
        # A controller switch needs multiple complete RViz render periods.
        pump(a.top_settle if top and a.top_settle is not None else a.switch_settle)
    q=app.primaryScreen().grabWindow(int(target_panel.winId())).toImage().convertToFormat(4)
    ptr=q.bits();ptr.setsize(q.byteCount())
    img=np.frombuffer(ptr,np.uint8).reshape(q.height(),q.bytesPerLine())[:,:q.width()*4].reshape(q.height(),q.width(),4)
    return cv2.cvtColor(img,cv2.COLOR_BGRA2BGR)
if a.dual_fixed:
    set_view(frame,.24,False);set_view(top_frame,0,True);pump(1.0)
elif a.fixed_view:
    set_view(frame,.24,a.fixed_view=='top');pump(1.0)
video_path=out/('preview_lossless.avi' if a.lossless else 'preview.mp4')
writer=cv2.VideoWriter(str(video_path),cv2.VideoWriter_fourcc(*('FFV1' if a.lossless else 'mp4v')),a.fps,(1920,1080))
assert writer.isOpened()
bagpath=base/'bags/trimmed_lz4/hm3d_stage1_3d_00337_online_floor_priority_historyfix_trim420.bag'
with rosbag.Bag(str(bagpath)) as bag:
    origin=bag.get_start_time(); targets=np.linspace(a.start,a.end,a.frames)+origin
    messages=iter(bag.read_messages(topics=topics,start_time=rospy.Time.from_sec(max(origin,targets[0]-2))))
    pending=next(messages,None);latest={}
    for i,target in enumerate(targets):
        while pending is not None and pending[2].to_sec()<=target:
            latest[pending[0]]=pending[1];pending=next(messages,None)
        clock.publish(Clock(clock=rospy.Time.from_sec(target)))
        for topic,msg in latest.items():
            if topic not in pubs:pubs[topic]=rospy.Publisher(topic,type(msg),queue_size=1,latch=True)
            if topic not in ('/voxel_mapping/occupancy_grid_occupied','/habitat/rgb'):pubs[topic].publish(msg)
        if '/voxel_mapping/occupancy_grid_occupied' in latest:
            msg=latest['/voxel_mapping/occupancy_grid_occupied'];points=xyz(msg)
            bases=np.array([-.2,2.6,5.4]); floorids=np.clip(np.searchsorted(bases,points[:,2],side='right')-1,0,2)
            z=latest['/uav_simulator/odometry'].pose.pose.position.z
            active=int(np.clip(np.searchsorted(bases+.4,z,side='right')-1,0,2))
            relative=points[:,2]-bases[floorids]
            histpts=points[(floorids!=active)&(relative>.25)&(relative<2.2)]
            history.publish(cloud(histpts[::max(1,int(np.ceil(len(histpts)/45000)))],msg.header))
            cur=points[(floorids==active)&(relative<2.65)]
            cur=cur[::max(1,int(np.ceil(len(cur)/80000)))]
            renderer.floor_z=float(bases[active]);renderer.cloud_callback(cloud(cur,msg.header))
        pump(.08 if a.fixed_view else (.12 if not a.dual_fixed else .10))
        if a.fixed_view=='side':
            side=capture(frame,panel,.24);top=None
        elif a.fixed_view=='top':
            side=None;top=capture(frame,panel,0,True)
        else:
            side=capture(frame,panel,.24)
            top=capture(top_frame if a.dual_fixed else frame,top_panel if a.dual_fixed else panel,0,True)
        canvas=np.full((1080,1920,3),(35,24,15),np.uint8)
        if side is not None:canvas[42:552,800:1920]=cv2.resize(side,(1120,510))
        if top is not None:canvas[570:1080,800:1920]=cv2.resize(top,(1120,510))
        if '/habitat/rgb' in latest:
            m=latest['/habitat/rgb'];rgb=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
            if m.encoding=='rgb8':rgb=rgb[:,:,::-1]
            rgb=cv2.resize(rgb,(784,588));canvas[246:834,8:792]=rgb
        for label,loc in [('First-person RGB',(22,35)),('RViz - side view',(815,30)),('RViz - exact top view',(815,566))]:cv2.putText(canvas,label,loc,cv2.FONT_HERSHEY_SIMPLEX,.7,(240,240,240),1,cv2.LINE_AA)
        cv2.putText(canvas,'Bag time: %.1f s  |  preview speed ~%.1fx'%(target-origin,(a.end-a.start)/max(1,a.frames-1)*a.fps),(20,1025),cv2.FONT_HERSHEY_SIMPLEX,.6,(220,220,220),1,cv2.LINE_AA)
        writer.write(canvas)
        if i in (0,a.frames//2,a.frames-1):cv2.imwrite(str(out/('frame_%03d.jpg'%i)),canvas)
        print('frame %d/%d t=%.1f'%(i+1,a.frames,target-origin),flush=True)
writer.release();print('SAVED',video_path,flush=True)
os._exit(0)
