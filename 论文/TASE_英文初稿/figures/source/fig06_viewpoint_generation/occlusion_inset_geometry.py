import bpy,json,math
from pathlib import Path
from mathutils import Vector
out=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(out/'algorithm_scene.blend'))
d=json.loads((out/'polished_projection.json').read_text())
obstacles=[o for o in bpy.data.objects if o.type=='MESH' and (o.name.startswith('Seat ') or o.name.startswith('Table '))]
bpy.context.view_layer.update();deps=bpy.context.evaluated_depsgraph_get()
def ray(p,t):
 start=Vector(p)+Vector((0,0,.0001));end=Vector(t)+Vector((0,0,.0001));best=(end-start).length;first=None;name=None
 for o in obstacles:
  inv=o.matrix_world.inverted();a=inv@start;b=inv@end;delta=b-a
  hit,loc,*_=o.evaluated_get(deps).ray_cast(a,delta.normalized(),distance=delta.length-.00001)
  if hit:
   loc=o.matrix_world@loc;dist=(loc-start).length
   if dist<best:best=dist;first=list(loc);name=o.name
 return {'visible':first is None,'hit':first,'blocker':name}
partial=[a for a in d['audit'] if not a['collision'] and 0<a['visible_samples']<9]
# Prefer a partial view rather than an almost entirely occluded one.
a=next(a for a in partial if a['azimuth_deg']==45);p=a['position']
samples=[]
for x in [-.8,0,.8]:
 for y in [-.4,0,.4]:
  t=[x,y,1.];samples.append({'target':t,**ray(p,t)})
grid=[]
for i in range(61):
 for j in range(31):
  t=[-1+2*i/60,-.5+j/30,1.];grid.append({'target':t,'visible':ray(p,t)['visible']})
backs=[]
for o in obstacles:
 if 'Chair back' not in o.name:continue
 bb=[Vector(v) for v in o.bound_box];lo=[min(v[k] for v in bb) for k in range(3)];hi=[max(v[k] for v in bb) for k in range(3)]
 z=(o.matrix_world.inverted()@Vector((o.location.x,o.location.y,1))).z
 if not lo[2]<=z<=hi[2]:continue
 backs.append([list(o.matrix_world@Vector((x,y,z))) for x,y in [(lo[0],lo[1]),(hi[0],lo[1]),(hi[0],hi[1]),(lo[0],hi[1])]])
# Exact projected source marker already recorded in the selected audit.
from bpy_extras.object_utils import world_to_camera_view
v=world_to_camera_view(bpy.context.scene,bpy.context.scene.camera,Vector(p))
r={'position':p,'projection':[v.x*2000,(1-v.y)*1400],'samples':samples,'grid':grid,'chair_backs':backs,'visible_count':sum(t['visible'] for t in samples),'azimuth_deg':a['azimuth_deg']}
(out/'occlusion_inset_geometry.json').write_text(json.dumps(r))
print('INSET',r['visible_count'],'/9 visible at',p)
