from pathlib import Path
import json,numpy as np
from repair_depth_validity import invalidate_safety_shell_depth as mask
R=Path(__file__).resolve().parents[1]
d=np.array([[1.,2.,3.,np.nan,0]],np.float32)
out,b=mask(d,[0,0,0],np.eye(3),[([-100,-100,2],[100,100,2.2])]);assert np.array_equal(out,[[1,0,0,0,0]])
out,b=mask(np.array([[3.]],np.float32),[0,0,0],np.eye(3),[([1,1,2],[2,2,3])],cx=0,cy=0);assert out[0,0]==3
cfg=json.loads((R/'scene_repairs/00337_three_floor_shell/repair_collision_only.json').read_text());S=np.array([[0,0,-1],[-1,0,0],[0,1,0]]);origin=np.array(cfg['reference_initial_sensor_habitat_xyz'])
for item in cfg['objects']:
 vertices=np.array([[float(v)for v in l.split()[1:]]for l in (R/'scene_repairs/00337_three_floor_shell'/item['mesh']).read_text().splitlines()if l.startswith('v ')])
 f=(vertices-origin)@S.T+[0,0,1];assert np.allclose(f.min(0),item['falcon_min'],atol=1e-6);assert np.allclose(f.max(0),item['falcon_max'],atol=1e-6)
root=R/'outputs/coverage700_wall_room_preview/room305_provenance';audit=json.loads((root/'audit.json').read_text());a=np.load(root/'depth_witnesses.npz');frames={round(f['capture_wall_s'],6):f for f in map(json.loads,(R/'outputs/coverage_lifecycle_700_r1_20261016/capture_trace.jsonl').read_text().splitlines())}
from scipy.spatial.transform import Rotation
rows=[];shells=[(v['falcon_min'],v['falcon_max'])for v in cfg['objects']]
for v in audit['best']:
 f=frames[round(v['stamp'],6)];i=v['index'];depth=a[f'depth{i}'];out,b=mask(depth,f['position'],Rotation.from_quat(f['orientation_xyzw']).as_matrix(),shells);roi=a[f'mask{i}'];assert np.array_equal(out[~b],depth.astype(np.float32)[~b]);assert not out[b].any();rows.append({'index':i,'roi_pixels':int(roi.sum()),'invalidated_roi_pixels':int((b&roi).sum()),'invalidated_frame_pixels':int(b.sum())})
O=R/'outputs/coverage700_wall_room_preview/room305_root_cause';(O/'depth_validity_regression.json').write_text(json.dumps({'synthetic_checks':'passed','all_14_mesh_bounds_verified':True,'frames':rows,'renderer_regression':'not run; GPU unavailable'},indent=2));print(rows)
