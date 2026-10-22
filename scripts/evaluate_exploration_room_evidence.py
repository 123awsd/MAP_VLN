"""Offline fixed-region evidence comparison; never provides planner targets."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('trials',nargs='+');p.add_argument('--output',default='outputs/exploration_goal_audit/room_evidence.json');args=p.parse_args()
reference=Path('outputs/room_wall_guided_trial/adaptive_merge');records=[];reference_cells={}
for name in args.trials:
 root=Path('outputs')/name;geo=root/'original_style_video/geometry.npz'
 if not geo.exists():continue
 g=np.load(geo);rows=[json.loads(s) for s in (root/'live_trajectory.jsonl').open()]
 if name=='exploration_yaw_consistent_trial':
  g={f'p{f-1}':np.load(f'outputs/room_apf_latest_trial/L{f}_input.npy') for f in [1,2,3]}
  rows=[r for r in rows if r['sequence']*.2<=503.9297003272]
 xyz=np.array([s['position'] for s in rows]);record={'trial':name,'rooms':[]}
 for floor,ground in enumerate([-.155,2.595,5.595],1):
  d=np.load(reference/f'L{floor}/segmentation.npz');labels=d['labels'];origin=d['origin'];res=float(d['resolution']);q=g[f'p{floor-1}'];q=q[(q[:,2]>ground+.15)&(q[:,2]<ground+2.2)]
  def assign(a):
   xy=np.floor((a[:,:2]-origin)/res).astype(int);ok=(xy[:,0]>=0)&(xy[:,0]<labels.shape[1])&(xy[:,1]>=0)&(xy[:,1]<labels.shape[0]);out=np.full(len(a),-1);out[ok]=labels[xy[ok,1],xy[ok,0]];return out
  pl=assign(q);tl=assign(xyz);tl[(xyz[:,2]<=ground+.2)|(xyz[:,2]>=ground+2.65)]=-1
  for lab in np.unique(labels[labels>=0]):
   points=q[pl==lab];cells=np.unique(np.floor(points/.1).astype(int),axis=0);ids=np.flatnonzero(tl==lab)
   key=(floor,int(lab))
   cellset={tuple(v) for v in cells.tolist()}
   if name=='exploration_yaw_consistent_trial':reference_cells[key]=cellset
   baseline_cells=reference_cells.get(key,set())
   tolerance_match=sum(any((v[0]+dx,v[1]+dy,v[2]+dz) in cellset for dx in [-1,0,1] for dy in [-1,0,1] for dz in [-1,0,1]) for v in baseline_cells)
   record['rooms'].append({'baseline_surface_reobserved_within_neighbor_voxel':tolerance_match/len(baseline_cells) if baseline_cells else None,'floor':floor,'reference_region':int(lab),'observed_surface_voxels_10cm':len(cells),'trajectory_samples':len(ids),'first_entry_sequence_s':float(rows[ids[0]]['sequence']*.2) if len(ids) else None})
 # The unlabeled bathroom is an evaluation-only ROI, not an algorithm input.
 for label,bounds in [('bathroom_next_to_room2',[-1.4,-.15,4.3,6.2]),('bathroom_room9_upper_right',[-1.3,-.1,9.7,11.7])]:
  x0,x1,y0,y1=bounds;inside=(xyz[:,0]>x0)&(xyz[:,0]<x1)&(xyz[:,1]>y0)&(xyz[:,1]<y1)&(xyz[:,2]>2.8)&(xyz[:,2]<5.4);q=g['p1'];m=(q[:,0]>x0)&(q[:,0]<x1)&(q[:,1]>y0)&(q[:,1]<y1)&(q[:,2]>2.8)&(q[:,2]<4.8);ids=np.flatnonzero(inside)
  steps=np.linalg.norm(np.diff(xyz,axis=0),axis=1);entry_runs=[];edges=np.diff(np.r_[False,inside,False].astype(int))
  for start,end in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
   entry_runs.append({'entry_s':float(rows[start]['sequence']*.2),'last_inside_s':float(rows[end-1]['sequence']*.2),'exited_before_run_end':bool(end<len(rows)),'blocked_samples':sum(bool(rows[i].get('preview_motion_blocked',False)) for i in range(start,end))})
  record[label]={'entry_runs':entry_runs,'trajectory_samples':len(ids),'surface_voxels_10cm':len(np.unique(np.floor(q[m]/.1).astype(int),axis=0)),'first_entry_sequence_s':float(rows[ids[0]]['sequence']*.2) if len(ids) else None}
 records.append(record)
Path(args.output).write_text(json.dumps({'method':'fixed baseline regions; 10cm observed surface support and actual trajectory evidence, not completeness percentage; ROI only offline','records':records},indent=2))
for r in records:print(r['trial'],r['bathroom_next_to_room2'],r['bathroom_room9_upper_right'])
