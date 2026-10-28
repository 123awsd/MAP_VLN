"""Read-only extraction of the original Habitat navigation mesh for thumbnails.

No planner is run. Routes and the four-stage sequence are explanatory.
"""
from pathlib import Path
import json
import math
import numpy as np
import habitat_sim

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
CFG=ROOT/'PRE_MAP_VLN/outputs/stage2_3d/tasks/00166_cushion_recovery_20261024/generated_stage1_config.json'
cfg=json.loads(CFG.read_text())
source=Path(cfg['scene']).with_suffix('.navmesh')
pf=habitat_sim.PathFinder()
assert pf.load_nav_mesh(str(source))
origin=np.asarray(cfg['initial_sensor_habitat_xyz'])
rot=np.asarray([[0,0,-1],[-1,0,0],[0,1,0]])
verts=np.asarray(pf.build_navmesh_vertices())
indices=np.asarray(pf.build_navmesh_vertex_indices()).reshape(-1,3)
f=(verts-origin)@rot.T+np.asarray([0,0,1])

def project(p):
    x,y,z=p
    d=math.hypot(24,5.5)
    return [.5+(y+4.6)/13.4,.5+((x+2.05)*24/d-(z-.35)*5.5/d)/13.4]

faces=[]
for face in indices:
    p=f[face]
    if np.max(p[:,2])>.42 or np.min(p[:,2])<-.30:continue
    if np.min(p[:,0]) < -7.65 or np.max(p[:,0]) > 3.35:continue
    if np.min(p[:,1]) < -10.7 or np.max(p[:,1]) > 1.5:continue
    faces.append([project([a,b,0]) for a,b,_ in p])

old=json.loads((OUT.parent/'multiroom_redraw/assets/projection.json').read_text())
poses=old['poses_falcon']
poses['qpre']=[-2.65,-5.25,1.0]
# Preserve the previous, navmesh-guided entry route. The returning segment is
# adjusted within the actual doorway so its direction remains visually distinct.
approach=old['route_falcon']['approach']+old['route_falcon']['local'][1:3]
local=[poses['q1'],[-3.7,-7.40,.93],poses['q2']]
recovery=old['route_falcon']['recovery'][:6]+[[-2.28,-5.44,.99],poses['qpre']]
verify=[poses['qpre'],[-3.05,-5.25,.96],[-3.55,-5.28,.92],poses['q3']]
routes=dict(approach=approach,local=local,recovery=recovery,verify=verify)

# Check only the ground projection against the source navmesh, not UAV clearance.
distances=[]
for line in routes.values():
    for a,b in zip(line,line[1:]):
        for t in np.linspace(0,1,20):
            p=np.array(a)*(1-t)+np.array(b)*t
            p[2]=0
            h=origin+rot.T@(p-np.array([0,0,1]))
            snapped=np.asarray(pf.snap_point(h))
            distances.append(float(np.linalg.norm((snapped-h)[[0,2]])))

out=dict(navmesh_source=str(source),triangles=faces,
    poses={k:project(v) for k,v in poses.items()},
    route={k:[project(v) for v in r] for k,r in routes.items()},
    route_falcon=routes,ground_projection_max_offset_m=max(distances),
    note='Illustrative route; navmesh check is a ground-projection sanity check, not flight validation.')
(OUT/'assets/sequence_geometry.json').write_text(json.dumps(out,indent=2)+'\n')
print('Triangles:',len(faces),'Max ground offset:',max(distances))
