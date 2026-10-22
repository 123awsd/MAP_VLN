"""Offline GT area matching only; no import into planner or simulator."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('--export-family',choices=['coverage_surface_exports','coverage_surface_pose_exports'],default='coverage_surface_exports');a=p.parse_args();root=Path(__file__).resolve().parents[1];base=root/'outputs/consistency_validation_20261015';dest=base/a.export_family/a.case;m=json.loads((dest/'manifest.json').read_text());ref=np.load(base/'global_observability/surface_witnesses.npz');points=ref['points'];floor=ref['floor'];first=np.full((3,len(points)),np.inf,np.float32);thresholds=[.05,.08,.12];duration=m['capture_duration_s'];checkpoints=[t for t in [300.,503.,504.,700.]if t<=duration]+[float(duration)]
# Query every reference sample against all raw endpoints accumulated through each checkpoint.
chunks=[np.load(f)for f in sorted(dest.glob('chunk*.npz'))];result=[];roi=np.load(base/'global_observability/three_region_surface_masks.npz')
for cp in sorted(set(checkpoints)):
 observed=np.concatenate([c['points'][c['elapsed_s']<=cp]for c in chunks]);tree=cKDTree(observed);d=tree.query(points,workers=2)[0];row={'elapsed_capture_wall_s':cp,'depth_endpoints':len(observed),'surface_by_threshold':[]}
 for radius in thresholds:
  matched=d<=radius;areas={str(f):round(float(ref['weight_m2'][(floor==f)&matched].sum()),3)for f in [1,2,3]};regions={key:round(float(ref['weight_m2'][roi[key+'_reference_indices']][matched[roi[key+'_reference_indices']]].sum()),3)for key in ['floor2_ordinary','floor2_room9_near']};row['surface_by_threshold'].append({'radius_m':radius,'matched_reference_area_est_m2':round(float(ref['weight_m2'][matched].sum()),3),'by_floor_m2':areas,'previous_missing_B_now_matched_m2':regions})
 result.append(row);print(a.case,cp,row['surface_by_threshold'][1],flush=True)
(base/(a.case+('_pose'if a.export_family=='coverage_surface_pose_exports'else '')+'_surface_checkpoints.json')).write_text(json.dumps({'case':a.case,'method':m,'checkpoints':result,'limits':['Area sampling and endpoint stride4/.4s are approximate; use identical protocol for comparisons.','Reference points cover interior-height slices only, not full building or all observable surfaces.','Matching an endpoint does not guarantee final occupancy retained it.','Current B surface sets depend on prior candidate run; their overlap is evaluation only, not online objective.']},indent=2))
