import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
root=Path(__file__).resolve().parents[1];out=root/'outputs/consistency_validation_20261015/global_observability';a=np.load(out/'surface_witnesses.npz');p=a['points'];floor=a['floor'];d=a['distance_m'];seen=a['depth_seen'];v=np.load(out/'mesh_verified_opportunities.npz')['verified'];geom=np.load(out/'actual_geometry_visibility.npz')['visible'];grid=np.load(root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016/observed_grid_final.npz');print(grid.files,flush=True)
occ=grid['occupied'];md=cKDTree(occ).query(p,workers=2)[0];area=lambda m:round(float(m.sum()*.01),3);rows=[]
for threshold in [.05,.08,.12]:
 for f in [1,2,3,0]:
  m=(d>threshold)&((floor==f)if f else True);raw=m&seen[0];potential=m&~geom&v;unknown=m&~geom&~v
  rows.append({'threshold_m':threshold,'floor':f,'reference_area_est_m2':area((floor==f)if f else np.ones(len(p),bool)),'unmatched_area_est_m2':area(m),'actual_geometric_visibility_without_depth_witness_m2':area(m&geom&~seen[0]),'current_raw_depth_witness_m2':area(raw),'raw_witness_with_final_occupied_within_12cm_m2':area(raw&(md<=.12)),'raw_witness_without_final_occupied_within_12cm_m2':area(raw&(md>.12)),'historical_mesh_checked_opportunity_m2':area(potential),'unresolved_m2':area(unknown)})
events=json.loads((root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016/execution_events.json').read_text());tasks={int(e['task_id']):e['goal'] for e in events if e['event']=='observation_executing'};captures=[json.loads(l) for l in (root/'outputs/consistency_A3_observation_lifecycle_504_final_20261016/capture_trace.jsonl').open()];t0=captures[0]['capture_wall_s']
regions=[];tiles=np.floor(p[:,:2]/2).astype(int)
for f,tx,ty in np.unique(np.column_stack((floor,tiles)),axis=0):
 base=(floor==f)&(tiles[:,0]==tx)&(tiles[:,1]==ty);m=base&(d>.08)
 if m.sum()<50:continue
 regions.append({'floor':int(f),'x_range_m':[int(tx*2),int(tx*2+2)],'y_range_m':[int(ty*2),int(ty*2+2)],'missing_area_m2':area(m),'actual_geometry_without_depth_m2':area(m&geom&~seen[0]),'actual_depth_witness_m2':area(m&seen[0]),'historical_mesh_checked_opportunity_m2':area(m&~geom&v),'unresolved_m2':area(m&~geom&~v),'raw_witness_map_gap_m2':area(m&seen[0]&(md>.12))})
for r in regions:
 def inside(q):
  return r['x_range_m'][0]<=q[0]<r['x_range_m'][1] and r['y_range_m'][0]<=q[1]<r['y_range_m'][1] and [-.005,2.745,5.745][r['floor']-1]<=q[2]<[2.045,4.795,7.795][r['floor']-1]
 ids={tid for tid,q in tasks.items() if inside(q)};r['executed_goal_task_ids_in_tile']=sorted(ids);r['selected_goal_events_in_tile']=sum(e['event']=='observation_selected' and inside(e['goal']) for e in events if 'goal' in e);r['cancellation_reasons_for_these_tasks']=[{'task_id':e['task_id'],'reason':e['reason']} for e in events if e['event']=='observation_cancelled' and int(e['task_id'])in ids];hits=[c['capture_wall_s']-t0 for c in captures if inside(c['position'])];r['actual_camera_frames_in_tile']=len(hits);r['first_camera_time_s']=min(hits) if hits else None
regions.sort(key=lambda r:r['historical_mesh_checked_opportunity_m2'],reverse=True)
result={'method':'uniform area sample thinning, 100 samples/m2; depth first-surface agreement; historical witnesses independently checked with repaired collision scene','scope':'three interior-height wall/object bands, fixed XY crop; floors/ceilings and outside crop not included','sample_count':len(p),'area_weight_per_sample_m2':.01,'threshold_floor_table':rows,'regions_2m_tiles_ranked_by_verified_opportunity':regions,'limits':['Not full-building exhaustive observability. Unresolved must not be treated as inaccessible or algorithm omission.','Missing reference surface is distinct from unknown free-space volume.','No historical online ESDF snapshots or counterfactual reranking.','Raw measured but sparse exported cloud absent can be export downsampling, not mapping failure. Final occupied proximity is a separate diagnostic.','Sampling uncertainty approximately sqrt(n)*0.01 m2 for a category, before geometric matching systematics.','Regional tiles are fixed 2x2m bins, not semantic rooms or connected components.']}
(out/'global_observability_summary.json').write_text(json.dumps(result,indent=2));np.savez_compressed(out/'final_map_proximity.npz',occupied_distance_m=md);print(json.dumps(rows,indent=2));print(json.dumps(regions[:12],indent=2))
