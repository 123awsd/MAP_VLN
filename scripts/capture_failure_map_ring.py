#!/usr/bin/env python3
"""Live bounded diagnostic supplement; published maps are asynchronous, not atomic."""
import argparse,json,time,threading
from pathlib import Path
import rospy,numpy as np
from sensor_msgs.msg import PointCloud2
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();out=a.trial/'published_failure_ring';out.mkdir(exist_ok=True)
rospy.init_node('failure_map_ring',anonymous=True);latest={};lock=threading.Lock()
def receive(m,key):
 names=['x','y','z']+(['intensity'] if key=='esdf' else []);fields={f.name:f.offset for f in m.fields};dt=np.dtype({'names':names,'formats':['<f4']*len(names),'offsets':[fields[n] for n in names],'itemsize':m.point_step});raw=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);q=np.column_stack([raw[n] for n in names]);
 with lock:latest[key]=(q,time.time(),m.header.stamp.to_sec())
subs=[rospy.Subscriber('/voxel_mapping/'+topic,PointCloud2,receive,callback_args=key,queue_size=1,buff_size=128*1024*1024) for key,topic in [('free','occupancy_grid_free'),('occupied','occupancy_grid_occupied'),('esdf','esdf')]]
trace=a.trial/'decision_trace.jsonl';offset=0;last={};counts={}
while not rospy.is_shutdown() and not(a.trial/'result.json').exists():
 if not trace.exists():time.sleep(.2);continue
 with trace.open() as f:f.seek(offset);lines=f.readlines();offset=f.tell()
 for line in lines:
  try:e=json.loads(line)
  except ValueError:continue
  if e['event']!='trajectory_rejected':continue
  point=e['point'];band=int(np.floor(point[2]/3));now=time.time()
  if now-last.get(band,0)<10:continue
  with lock:snapshot=dict(latest)
  if len(snapshot)!=3:continue
  slot=counts.get(band,0)%4;dest=out/('band_%d_slot_%d.npz'%(band,slot));tmp=dest.with_suffix('.tmp.npz');np.savez_compressed(str(tmp),**{k:v[0] for k,v in snapshot.items()});tmp.replace(dest)
  meta={'event':e,'saved_wall_s':now,'map_wall_s':{k:v[1] for k,v in snapshot.items()},'map_header_stamp':{k:v[2] for k,v in snapshot.items()},'limit':'Asynchronously published geometry supplement, NOT exact atomic failure map. Per-height-band four-slot ring; at least10s between saves.'};dest.with_suffix('.json').write_text(json.dumps(meta,indent=2));counts[band]=counts.get(band,0)+1;last[band]=now
 time.sleep(.2)
