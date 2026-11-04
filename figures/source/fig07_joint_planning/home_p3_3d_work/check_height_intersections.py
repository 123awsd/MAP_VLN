"""Check every displayed route edge against the original GLB triangles."""
from pathlib import Path
import json
import numpy as np
import trimesh
H=Path(__file__).resolve().parent
D=json.loads((H/'candidate_results_height_limited.json').read_text())
m=trimesh.load('/home/uav/下载/home_-_p3.glb',force='scene').to_geometry()
# Convert glTF coordinates to Blender coordinates and metres.
v=np.asarray(m.vertices).copy(); m.vertices=np.column_stack((v[:,0],-v[:,2],v[:,1]))*.0254
origins=[]; directions=[]; lengths=[]; labels=[]
for s,visit in zip(D['plan']['segments'],D['plan']['visits']):
 p=visit['pose']; pts=np.array([s['from_xyz_m']]+s['points_xyz_m']+[[p['x'],p['y'],p['z']]])
 for i,(a,b) in enumerate(zip(pts,pts[1:])):
  delta=b-a; length=np.linalg.norm(delta)
  if length<1e-9: continue
  origins.append(a); directions.append(delta/length); lengths.append(length); labels.append((s['to_task_id'],i))
locations,rays,faces=m.ray.intersects_location(np.array(origins),np.array(directions),multiple_hits=True)
hits=[]
for loc,r,face in zip(locations,rays,faces):
 distance=float(np.linalg.norm(loc-origins[r]))
 if distance<=lengths[r]+1e-6: hits.append({'task':labels[r][0],'edge':labels[r][1],'point_m':loc.tolist(),'face':int(face),'distance_m':distance})
out={'route_edges':len(origins),'triangle_intersections':hits,'limitations':['Tests route centreline intersections, not vehicle radius clearance.','Includes all GLB triangles, including any decorative, glass or non-collision meshes.']}
(H/'mesh_intersection_height_limited.json').write_text(json.dumps(out,indent=2),encoding='utf8')
print('route edges',len(origins),'mesh hits',len(hits),'by task', {t:sum(x['task']==t for x in hits) for t in 'ABCD'},flush=True)
for hit in hits[:12]: print(hit,flush=True)
