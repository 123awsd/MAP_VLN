"""Native Habitat top-down navigability and illustrative long-task route.

The static map comes from PathFinder.get_topdown_view(), not online mapping.
Navmesh shortest paths serve only to keep the explanation inside the scene.
"""
from pathlib import Path
import json
import numpy as np
from PIL import Image
import habitat_sim

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
cfg=json.loads((ROOT/'PRE_MAP_VLN/outputs/stage2_3d/tasks/00166_cushion_recovery_20261024/generated_stage1_config.json').read_text())
manifest=json.loads((OUT/'view_manifest.json').read_text())
views={v['name']:v for v in manifest['views']}
selected=['start','kitchen_b','chair_b','retry_c','sofa_a','dining_a']
R=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
origin=np.array(cfg['initial_sensor_habitat_xyz'])
pf=habitat_sim.PathFinder();assert pf.load_nav_mesh(str(Path(cfg['scene']).with_suffix('.navmesh')))
lower,upper=map(np.asarray,pf.get_bounds())
resolution=.04;height=float(cfg['initial_agent_habitat_xyz'][1])
mask=np.asarray(pf.get_topdown_view(resolution,height),dtype=bool)
Image.fromarray(np.where(mask,255,43).astype(np.uint8)).save(OUT/'assets/habitat_topdown.png')
np.save(OUT/'assets/habitat_navigability.npy',mask)
def to_h(q):return origin+R.T@(np.array(q)-[0,0,1])
def to_f(q):return (R@(np.array(q)-origin)+[0,0,1]).tolist()
def official(q):
    T=np.array(json.loads((OUT.parent/'assets/hm3d_official/geometry.json').read_text())['falcon_xy_to_official_pixel'])
    return (T@[*q[:2],1]).tolist()
def grid(q):
    p=to_h(q)
    return [(p[0]-lower[0])/resolution,(p[2]-lower[2])/resolution]
stages=[];snapped=[];offsets=[]
for name in selected:
    v=views[name];ground=list(v['pose_falcon']);ground[2]=0
    h=to_h(ground);snap=np.asarray(pf.snap_point(h));snapped.append(snap)
    offsets.append(float(np.linalg.norm((h-snap)[[0,2]])))
    stages.append(dict(**v,official_pixel=official(v['pose_falcon']),grid_pixel=grid(v['pose_falcon'])))
routes=[];lengths=[]
for i in range(len(stages)-1):
    path=habitat_sim.ShortestPath();path.requested_start=snapped[i];path.requested_end=snapped[i+1]
    assert pf.find_path(path),f'No path: {selected[i]} -> {selected[i+1]}'
    points=[to_f(p) for p in path.points]
    routes.append(dict(falcon=points,official_pixel=[official(p) for p in points],grid_pixel=[grid(p) for p in points]))
    lengths.append(float(path.geodesic_distance))
out=dict(scene=cfg['scene'],instruction='Leave the bathroom through the bedroom and head to the kitchen island. Next, enter the other bedroom and look for a cushion, starting with the armchair by the window. After finding the cushion, return to the dining area and stop beside the table.',
    map=dict(api='habitat_sim.PathFinder.get_topdown_view',resolution_m=resolution,height_habitat=height,shape=list(mask.shape),lower_habitat=lower.tolist(),role='Static navigability reference; not an online occupancy estimate'),
    stages=stages,routes=routes,route_length_m=sum(lengths),segment_lengths_m=lengths,
    endpoint_ground_snap_offsets_m=offsets,sequence_role='Authored mechanism illustration. Route uses Habitat navmesh shortest paths, not the paper planner.',
    qwen_role='Illustrative candidate, not a recorded Qwen inference.')
(OUT/'episode_geometry.json').write_text(json.dumps(out,indent=2)+'\n')
print('Map shape:',mask.shape,'route length:',sum(lengths),'endpoint offsets:',offsets)
