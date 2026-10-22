import json,collections,math
from pathlib import Path
import rosbag
root=Path('/workspace/shared');case=root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016';out=root/'outputs/consistency_validation_20261015/minimal_candidate_audit';frames=[json.loads(l)for l in(case/'capture_trace.jsonl').open()];t0=frames[0]['capture_wall_s'];events=json.loads((case/'execution_events.json').read_text());e209=[e for e in events if e.get('plan')==209];low=min(e['wall_s']for e in e209)-1;high=max(e['wall_s']for e in e209)+1;target=[2.07,5.42,3.68];ns=collections.Counter();matches=[];schemas={};sensor=[]
with rosbag.Bag(str(case/'trial.bag'))as bag:
 topics={k:{'type':v.msg_type,'count':v.message_count}for k,v in bag.get_type_and_topic_info().topics.items()}
 for topic,m,t in bag.read_messages(topics=['/planning_vis/frontier','/uav_simulator/sensor_pose']):
  if topic.endswith('sensor_pose'):
   if len(sensor)<2:sensor.append({'header_stamp':m.header.stamp.to_sec(),'frame_id':m.header.frame_id,'child_frame_id':m.child_frame_id,'translation':[m.transform.translation.x,m.transform.translation.y,m.transform.translation.z],'quaternion':[m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w]})
   continue
  ns[(m.ns,m.type)]+=1;schemas.setdefault(m.ns,{'type':m.type,'point_count_example':len(m.points),'frame_id':m.header.frame_id,'text_example':m.text[:100]})
  if not(low<=t.to_sec()<=high):continue
  points=[[p.x,p.y,p.z]for p in m.points]
  for i,p in enumerate(points):
   if sum((p[j]-target[j])**2 for j in range(3))>.035**2:continue
   rec={'bag_relative_s':t.to_sec()-t0,'header_relative_s':m.header.stamp.to_sec()-t0,'namespace':m.ns,'marker_id':m.id,'type':m.type,'point_index':i,'point':p,'nearby_points':points[max(0,i-1):i+2]}
   if m.type==5 and i%2==0 and i+1<len(points):
    end=points[i+1];rec['line_yaw']=math.atan2(end[1]-p[1],end[0]-p[0]);rec['line_length_m']=sum((end[j]-p[j])**2 for j in range(3))**.5
   matches.append(rec)
report={'topics':topics,'marker_namespace_counts':{str(k):v for k,v in ns.items()},'marker_schemas':schemas,'plan209_window_relative_s':[low-t0,high-t0],'plan209_near_candidate_marker_points':matches,'sensor_pose_examples':sensor,'limits':['Marker IDs are visualization slots, not stable candidate IDs.','Visualization is asynchronous; timestamp proximity alone does not prove same plan or map.']};(out/'bag_inventory.json').write_text(json.dumps(report,indent=2));print(json.dumps({'marker_schemas':schemas,'plan209_matches':matches},indent=2))
