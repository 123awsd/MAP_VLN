import bpy,json,math
from pathlib import Path
from mathutils import Vector
out=Path(__file__).resolve().parent
report=json.loads((out/'collision_drone_geometry.json').read_text());center=Vector(report['candidate']['position'])
def material(name,color,metal,rough):
 m=bpy.data.materials.new(name);m.use_nodes=True;bs=m.node_tree.nodes['Principled BSDF'];bs.inputs['Base Color'].default_value=(*color,1);bs.inputs['Metallic'].default_value=metal;bs.inputs['Roughness'].default_value=rough;return m
for filename in ['collision_step_drone.blend','collision_explanation.blend']:
 bpy.ops.wm.open_mainfile(filepath=str(out/filename))
 mats=[material('Photo carbon frame',(.018,.023,.027),.25,.38),material('Photo brushed aluminium',(.53,.57,.59),.72,.3),material('Photo smoke brown rotor',(.16,.125,.085),.32,.3),material('Photo dark sensor glass',(.012,.045,.052),.55,.18),material('Photo black equipment',(.035,.04,.045),.15,.48)]
 for obj in bpy.data.objects:
  if obj.type!='MESH' or not obj.name.startswith('User STEP'):continue
  old=list(obj.data.materials);red={i for i,m in enumerate(old) if m and 'collision orange' in m.name.lower()};offset=len(old)
  for m in mats:obj.data.materials.append(m)
  for f in obj.data.polygons:
   if f.material_index in red:continue
   q=obj.matrix_world@f.center-center;r=math.hypot(q.x,q.y);n=(obj.matrix_world.to_3x3()@f.normal).normalized()
   if r>.095:
    index=2 if abs(n.z)>.7 else 0
   elif q.z>.055:
    index=1 if abs(n.z)<.6 or r>.038 else 3
   elif q.z>.005:index=1
   elif q.z<-.05:index=4
   else:index=0
   f.material_index=offset+index
 bpy.ops.wm.save_as_mainfile(filepath=str(out/filename))
 if filename=='collision_explanation.blend':
  bpy.context.scene.render.filepath=str(out/'collision_explanation.png');bpy.ops.render.render(write_still=True)
