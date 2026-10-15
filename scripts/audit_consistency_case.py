#!/usr/bin/env python3
"""Offline metrics from actual capture times; reference ROIs never guide planning."""
import argparse,collections,json,math,re
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();out=a.trial
rows=[json.loads(l) for l in (out/'capture_trace.jsonl').open()];q=np.array([r['position'] for r in rows]);t=np.array([r['capture_wall_s'] for r in rows]);t-=t[0];dt=np.diff(t);steps=np.linalg.norm(np.diff(q,axis=0),axis=1)
trace=collections.Counter();stages=collections.Counter();pauses=[];events=[];region_stages=collections.Counter();region_candidates={};latest_stages={}
for l in (out/'decision_trace.jsonl').open():
 try:r=json.loads(l)
 except ValueError:continue
 trace[r['event']]+=1
 if r['event'] not in ['candidate','search_result','cost_without_path']:events.append(r)
 if r['event']=='candidate':
  stages[r['stage']]+=1;xyz=r['xyz'];cid=r.get('candidate_id',0)
  if cid:latest_stages[cid]=r['stage']
  if -1.4<xyz[0]<-.15 and 4.3<xyz[1]<6.2 and 2.8<xyz[2]<5.4:
   region_stages[r['stage']]+=1
   if cid:region_candidates.setdefault(cid,[]).append(r)
still=steps<.01;edges=np.diff(np.r_[False,still,False].astype(int))
for b,e in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):pauses.append({'start_s':float(t[b]),'duration_s':float(t[e]-t[b]),'position':q[b].tolist()})
regions={}
for name,(x0,x1,y0,y1) in {'bathroom_next_to_room2':(-1.4,-.15,4.3,6.2),'bathroom_room9_upper_right':(-1.3,-.1,9.7,11.7)}.items():
 inside=(q[:,0]>x0)&(q[:,0]<x1)&(q[:,1]>y0)&(q[:,1]<y1)&(q[:,2]>2.8)&(q[:,2]<5.4)
 zband=(q[:,2]>2.8)&(q[:,2]<5.4)
 dx=np.maximum(np.maximum(x0-q[:,0],q[:,0]-x1),0);dy=np.maximum(np.maximum(y0-q[:,1],q[:,1]-y1),0)
 regions[name]={'inside_frames':int(inside.sum()),'first_inside_s':float(t[inside][0]) if inside.any() else None,'last_inside_s':float(t[inside][-1]) if inside.any() else None,'nearest_reference_rectangle_m':float(np.sqrt(dx[zband]**2+dy[zband]**2).min()) if zband.any() else None,'reference_rectangle':[x0,x1,y0,y1],'reference_z_band':[2.8,5.4]}
 if (out/'observed_grid_final.npz').exists():
  grid=np.load(out/'observed_grid_final.npz');counts={}
  for typ in ['free','occupied']:
   pts=grid[typ];mask=(pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>2.8)&(pts[:,2]<5.4);counts[typ+'_m3']=float(mask.sum()*.001)
  regions[name]['known_volume']=counts
feedback={}
if all('velocity_world' in r for r in rows):
 measured=np.array([r['velocity_world'] for r in rows]);expected=np.diff(q,axis=0)/dt[:,None];err=np.linalg.norm(measured[1:]-expected,axis=1)
 feedback={'velocity_displacement_max_error_mps':float(err.max()),'capture_times_strictly_increasing':bool(np.all(dt>0)),'sequence_strictly_increasing':all(b['sequence']>a['sequence'] for a,b in zip(rows,rows[1:])),'actual_max_speed_mps':float(np.linalg.norm(measured,axis=1).max())}
yaw_residual=[];command_errors=[]
for r in rows:
 command=r.get('executed_command')
 if command and not command.get('execution_hold'):
  x,y,z,w=r['body_orientation_xyzw'];actual_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));yaw_residual.append(abs(math.atan2(math.sin(actual_yaw-command['yaw']),math.cos(actual_yaw-command['yaw']))))
  command_errors.append(math.dist(command['position'],r['position']))
floor3=q[:,2]>5.745
reference=Path(__file__).resolve().parents[1]/'outputs/room_wall_guided_trial/adaptive_merge'
fixed_areas={}
if (reference/'L3/segmentation.npz').exists():
 s=np.load(reference/'L3/segmentation.npz');ij=np.floor((q[:,:2]-s['origin'])/float(s['resolution'])).astype(int);labels=s['labels'];valid=(ij[:,0]>=0)&(ij[:,0]<labels.shape[1])&(ij[:,1]>=0)&(ij[:,1]<labels.shape[0]);inside3=np.zeros(len(q),dtype=bool);inside3[valid]=labels[ij[valid,1],ij[valid,0]]>=0;inside3&=(q[:,2]>5.745)&(q[:,2]<7.795)
 fixed_areas['third_floor_first_reference_area_entry_s']=float(t[inside3][0]) if inside3.any() else None
 fixed_areas['third_floor_reference_inside_frames']=int(inside3.sum())
 if (out/'observed_grid_final.npz').exists():
  grid=np.load(out/'observed_grid_final.npz');area_rows=[]
  for floor,ground in enumerate([-.155,2.595,5.595],1):
   s=np.load(reference/f'L{floor}/segmentation.npz');labels=s['labels'];counts={int(k):{'free_m3':0.,'occupied_m3':0.} for k in np.unique(labels[labels>=0])}
   for typ in ['free','occupied']:
    pts=grid[typ];pts=pts[(pts[:,2]>ground+.15)&(pts[:,2]<ground+2.2)];ij=np.floor((pts[:,:2]-s['origin'])/float(s['resolution'])).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<labels.shape[1])&(ij[:,1]>=0)&(ij[:,1]<labels.shape[0]);lab=labels[ij[ok,1],ij[ok,0]]
    for k,c in zip(*np.unique(lab[lab>=0],return_counts=True)):counts[int(k)][typ+'_m3']=float(c*.001)
   area_rows.extend({'floor':floor,'reference_region':k,**v}for k,v in counts.items())
  fixed_areas['known_volume_by_reference_region']=area_rows
result={'trial':out.name,'capture_duration_s':float(t[-1]),'frames':len(rows),'path_m':float(steps.sum()),'translation_time_s':float(dt[~still].sum()),'longest_adjacent_subcm_pause':max(pauses,key=lambda r:r['duration_s']),'blocked_frames':sum(r.get('preview_motion_blocked',False) for r in rows),'first_height_above_5_745_s':float(t[floor3][0]) if floor3.any() else None,'regions':regions,'feedback':feedback,'events':trace,'candidate_stages':stages,'room2_candidate_stages':region_stages,'room2_unique_generated_or_retained_ids':len(region_candidates),'room2_terminal_stages':dict(collections.Counter(latest_stages[c] for c in region_candidates)),'limits':['Fixed rectangles are offline historical references, not GT room completeness.','Height-band entry alone is not proof of containment or complete third-floor coverage.','Small adjacent steps do not distinguish intentional yaw observation from planning delay.']}
result['fixed_reference_areas']=fixed_areas
result['capture_interval_p95_s']=float(np.percentile(dt,95))
result['actual_sample_speed_p95_mps']=float(np.percentile(steps/dt,95))
result['yaw_policy_counts']=dict(collections.Counter(r.get('yaw_policy','unrecorded')for r in rows))
result['nonhold_command_yaw_residual_deg']={'median':float(np.rad2deg(np.median(yaw_residual))),'p95':float(np.rad2deg(np.percentile(yaw_residual,95)))} if yaw_residual else None
result['nonhold_command_actual_position_error_m']={'median':float(np.median(command_errors)),'p95':float(np.percentile(command_errors,95)),'max':max(command_errors)} if command_errors else None
if (out/'floor_volumes.json').exists():result['floor_volumes']=json.loads((out/'floor_volumes.json').read_text())
(out/'consistency_audit.json').write_text(json.dumps(result,indent=2,ensure_ascii=False));(out/'execution_events.json').write_text(json.dumps(events,indent=2));(out/'room2_candidate_lifecycles.json').write_text(json.dumps(region_candidates,indent=2))
print(json.dumps({k:result[k] for k in ['trial','path_m','translation_time_s','blocked_frames','regions','room2_candidate_stages']},ensure_ascii=False))
