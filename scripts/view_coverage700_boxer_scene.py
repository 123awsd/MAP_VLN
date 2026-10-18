#!/usr/bin/env python3
"""Inspect measured Boxer predictions in the native Habitat scene (no controller)."""
import argparse,csv,itertools,json,math,time
from pathlib import Path
import numpy as np
import cv2
import habitat_sim
import magnum as mn
from habitat_scene_repairs import load_scene_repair
R=Path(__file__).resolve().parents[1];BASE=R/'outputs/boxer/coverage700_sam31_20261018'
S=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
ORIGIN=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092])
def to_h(points):return (np.asarray(points)-[0,0,1])@S+ORIGIN

def main():
 global BASE
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run-dir',type=Path,default=BASE);ap.add_argument('--preview',action='store_true');ap.add_argument('--object',default='O002');ap.add_argument('--width',type=int,default=1280);ap.add_argument('--height',type=int,default=720);args=ap.parse_args();BASE=args.run_dir
 out=BASE/'scene_viewer';out.mkdir(exist_ok=True);summary=json.loads((BASE/'fusion_fixed/boxer_summary.json').read_text());objects=summary['objects'];support=json.loads((BASE/'detections/episode/fusion_support_fixed.json').read_text());raw=list(csv.DictReader((BASE/'detections/episode/boxer_3dbbs.csv').open()));raw=[v for v in raw if float(v['prob'])>=.45]
 trace={round(v['capture_wall_s'],6):v for v in map(json.loads,(R/'outputs/coverage_lifecycle_700_r1_20261016/capture_trace.jsonl').read_text().splitlines())}
 selected=next((i for i,v in enumerate(objects)if v['id']==args.object),1)
 pos=np.zeros(3);yaw=0.;pitch=0.;mode=0;show_labels=True;drag=None
 def jump(i):
  nonlocal selected,pos,yaw,pitch
  selected=i%len(objects);obj=objects[selected];members=[raw[k]for k in support[selected]['filtered_detection_indices']];member=max(members,key=lambda v:float(v['prob']));f=trace[round(int(member['time_ns'])*1e-9,6)];pos=np.array(f['position'],float);delta=np.array(obj['center_xyz'])-pos;yaw=math.atan2(delta[1],delta[0]);pitch=float(np.clip(math.atan2(delta[2],np.linalg.norm(delta[:2])),-1.35,1.35))
 jump(selected)
 cfg=habitat_sim.SimulatorConfiguration();scene=R/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb';cfg.scene_id=str(scene);cfg.scene_dataset_config_file=str(R/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json');cfg.enable_physics=True
 sensor=habitat_sim.CameraSensorSpec();sensor.uuid='rgb';sensor.sensor_type=habitat_sim.SensorType.COLOR;sensor.resolution=[args.height,args.width];sensor.position=mn.Vector3(0,0,0);sensor.hfov=90
 agent=habitat_sim.agent.AgentConfiguration();agent.sensor_specifications=[sensor]
 signs=np.array(list(itertools.product((-.5,.5),repeat=3)));edges=[(i,j)for i in range(8)for j in range(i+1,8)if np.sum(signs[i]!=signs[j])==1]
 window='Habitat - measured Boxer 3D inspection'
 def mouse(event,x,y,flags,param):
  nonlocal drag,yaw,pitch
  if event==cv2.EVENT_LBUTTONDOWN:drag=(x,y)
  elif event==cv2.EVENT_LBUTTONUP:drag=None
  elif event==cv2.EVENT_MOUSEMOVE and drag is not None:
   yaw-=(x-drag[0])*.004;pitch=float(np.clip(pitch-(y-drag[1])*.004,-1.4,1.4));drag=(x,y)
 with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[agent]))as sim:
  repairs=load_scene_repair(sim,R/'scene_repairs/00337_three_floor_shell/repair.json',scene)
  debug=sim.get_debug_line_render();debug.set_line_width(2.5)
  if not args.preview:
   cv2.namedWindow(window,cv2.WINDOW_NORMAL);cv2.resizeWindow(window,args.width,args.height);cv2.setMouseCallback(window,mouse)
  def render():
   obj=objects[selected];forward=np.array([math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),math.sin(pitch)]);cp=mn.Vector3(*to_h(pos));target=mn.Vector3(*to_h(pos+forward));node=sim.agents[0].scene_node;node.translation=cp;node.rotation=mn.Quaternion.from_matrix(mn.Matrix4.look_at(cp,target,mn.Vector3(0,1,0)).rotation())
   visible=[]
   for i,v in enumerate(objects):
    if mode==2 or (mode==1 and i!=selected) or (mode==0 and v['floor']!=obj['floor']):continue
    corners=to_h(v['corners_xyz']);color=mn.Color4(1.,.85,0.,1.)if i==selected else mn.Color4(0.,.95,.95,1.)
    for a,b in edges:debug.draw_transformed_line(mn.Vector3(*corners[a]),mn.Vector3(*corners[b]),color)
    visible.append(i)
   rgb=sim.get_sensor_observations()['rgb'][...,:3];frame=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
   if show_labels:
    rot=np.array(node.rotation.to_matrix());focal=args.width/2
    for i in visible:
     v=objects[i];q=(to_h(v['center_xyz'])-to_h(pos))@rot;z=-q[2]
     if z<=.15:continue
     u=int(args.width/2+focal*q[0]/z);w=int(args.height/2-focal*q[1]/z)
     if 0<=u<args.width and 85<w<args.height:
      label=f"{v['id']} {v['label']}";cv2.putText(frame,label,(u,w),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),3,cv2.LINE_AA);cv2.putText(frame,label,(u,w),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,230,255)if i==selected else(255,255,0),1,cv2.LINE_AA)
   cv2.rectangle(frame,(0,0),(args.width,85),(25,25,25),-1)
   lines=[f"{selected+1}/{len(objects)}  {obj['id']} {obj['label']} | floor {obj['floor']} | {obj['support_count']} capture frames | predicted box",
          'W/S forward/back  A/D strafe  Q/E down/up  Drag mouse or I/J/K/L look',
          'N/P next/previous object  1/2/3 floor  B boxes(floor/selected/off)  T labels  R reset  C screenshot  ESC quit']
   for k,line in enumerate(lines):cv2.putText(frame,line,(12,23+25*k),cv2.FONT_HERSHEY_SIMPLEX,.48,(235,235,235),1,cv2.LINE_AA)
   return frame,forward
  frame,forward=render();cv2.imwrite(str(out/'initial_view.png'),frame)
  (out/'viewer_status.json').write_text(json.dumps({'status':'preview_complete'if args.preview else'window_ready','boxes':len(objects),'source':'fusion_fixed','scene':str(scene),'repairs':len(repairs),'selected':objects[selected]['id'],'initial_camera_falcon':pos.tolist(),'window':window,'inspection_only':True,'planner_commands_sent':False,'camera_mode':'free flight inspection, movement not collision checked'},indent=2));print('VIEWER_READY',flush=True)
  if args.preview:return
  while True:
   frame,forward=render();cv2.imshow(window,frame);key=cv2.waitKeyEx(20)
   if key==27 or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:break
   if key<0:continue
   right=np.array([math.sin(yaw),-math.cos(yaw),0.]);speed=.18
   if key in (ord('w'),ord('W')):pos+=speed*forward
   elif key in (ord('s'),ord('S')):pos-=speed*forward
   elif key in (ord('a'),ord('A')):pos-=speed*right
   elif key in (ord('d'),ord('D')):pos+=speed*right
   elif key in (ord('q'),ord('Q')):pos[2]-=speed
   elif key in (ord('e'),ord('E')):pos[2]+=speed
   elif key==ord('j'):yaw+=.08
   elif key==ord('l'):yaw-=.08
   elif key==ord('i'):pitch=min(1.4,pitch+.08)
   elif key==ord('k'):pitch=max(-1.4,pitch-.08)
   elif key in (ord('n'),ord('N')):jump(selected+1)
   elif key in (ord('p'),ord('P')):jump(selected-1)
   elif key in (ord('r'),ord('R')):jump(selected)
   elif key in (ord('b'),ord('B')):mode=(mode+1)%3
   elif key in (ord('t'),ord('T')):show_labels=not show_labels
   elif key in (ord('1'),ord('2'),ord('3')):
    floor=int(chr(key));jump(next(i for i,v in enumerate(objects)if v['floor']==floor))
   elif key in (ord('c'),ord('C')):cv2.imwrite(str(out/f"{objects[selected]['id']}_{time.time_ns()}.png"),frame)
  cv2.destroyAllWindows();(out/'viewer_closed.json').write_text(json.dumps({'closed_at':time.time(),'planner_commands_sent':False}))
if __name__=='__main__':main()
