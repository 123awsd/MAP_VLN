#!/usr/bin/env python3
"""Capture observed FREE+OCCUPIED grid volume by fixed height bands (ROS)."""
import argparse,json,time,threading,os
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

p=argparse.ArgumentParser();p.add_argument('trial');a=p.parse_args();out=Path(a.trial)
rospy.init_node('floor_volume_capture',anonymous=True)
latest={};lock=threading.Lock()
def receive(msg,key):
    fields={f.name:f.offset for f in msg.fields}
    dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':msg.point_step})
    # PCL grids are unorganized, with one point per 0.1m cell in this scene.
    points=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
    q=np.column_stack([points[k] for k in ['x','y','z']]).copy()
    with lock:latest[key]=(q,time.time())
def coverage(msg):
    with lock:latest['coverage']=(float(msg.data),time.time())
subs=[rospy.Subscriber('/voxel_mapping/occupancy_grid_'+k,PointCloud2,receive,callback_args=k,queue_size=1,buff_size=64*1024*1024) for k in ['free','occupied']]
subs.append(rospy.Subscriber('/voxel_mapping/map_coverage',Float32,coverage,queue_size=1))
def save():
    with lock:snapshot=dict(latest)
    if not all(k in snapshot for k in ['free','occupied']):return
    bands=[(-1.5,2.595),(2.595,5.595),(5.595,8.05)]
    rows=[]
    for f,(lo,hi) in enumerate(bands,1):
        row={'floor':f,'z_band_m':[lo,hi]}
        for key in ['free','occupied']:
            q=snapshot[key][0];mask=(q[:,0]>=-8.5)&(q[:,0]<10.5)&(q[:,1]>=-2.5)&(q[:,1]<18.5)&(q[:,2]>=lo)&(q[:,2]<hi)
            row[key+'_m3']=float(np.count_nonzero(mask)*.001)
        row['known_m3']=row['free_m3']+row['occupied_m3'];rows.append(row)
    result={'method':'Observed grid FREE+OCCUPIED; 0.1m cells; fixed disjoint height bands, not ground-truth completeness','floors':rows,'known_sum_m3':sum(r['known_m3'] for r in rows),'map_coverage_latest':snapshot.get('coverage'),'snapshot_wall_times':{k:snapshot[k][1] for k in ['free','occupied']}}
    target=out/'floor_volumes.json';tmp=target.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2));tmp.replace(target)
started=time.monotonic()
while not rospy.is_shutdown() and time.monotonic()-started<float(os.environ.get("EXPLORATION_RESOURCE_DURATION", "1000")):
    save()
    if (out/'result.json').exists():break
    time.sleep(2)
save()

# Save observed cells once for offline comparison against fixed room regions.
with lock: final=dict(latest)
if all(k in final for k in ['free','occupied']):
    np.savez_compressed(str(out/'observed_grid_final.npz'),free=final['free'][0],occupied=final['occupied'][0])
