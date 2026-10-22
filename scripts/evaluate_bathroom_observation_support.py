"""Offline surface reference assembled from existing actual observations, no GT."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
p=argparse.ArgumentParser();p.add_argument('trials',nargs='+');p.add_argument('--output',default='outputs/exploration_goal_audit/bathroom_surface_support.json');a=p.parse_args();clouds={}
for name in a.trials:
 f=Path('outputs')/name/'original_style_video/geometry.npz'
 if f.exists():clouds[name]=np.load(f)['p1']
records=[]
for label,b in [('next_to_room2',[-1.4,-.15,4.3,6.2]),('room9_upper_right',[-1.3,-.1,9.7,11.7])]:
 x0,x1,y0,y1=b;sets={}
 for name,q in clouds.items():
  m=(q[:,0]>x0)&(q[:,0]<x1)&(q[:,1]>y0)&(q[:,1]<y1)&(q[:,2]>2.8)&(q[:,2]<4.8);cells=np.unique(np.floor(q[m]/.1).astype(int),axis=0);sets[name]=cells*.1+.05
 joined=np.concatenate(list(sets.values()));reference=np.unique(np.floor(joined/.1).astype(int),axis=0)*.1+.05
 for name,q in sets.items():
  if len(q):dist,_=cKDTree(q).query(reference);ratio=float((dist<=.18).mean())
  else:ratio=0.
  records.append({'region':label,'trial':name,'observed_surface_voxels':len(q),'multi_run_reference_voxels':len(reference),'reference_supported_within_18cm':ratio})
Path(a.output).write_text(json.dumps({'method':'evaluation only; union of measured surfaces from specified runs; not ground truth, not completeness percentage, reference cannot include unseen surfaces','records':records},indent=2));print(records)
