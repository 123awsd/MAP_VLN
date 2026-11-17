import bpy,json,math
from pathlib import Path
from mathutils import Vector
out=Path('/home/uav/map_VLN/画图/候选视角生成_新版');p=Vector(json.loads((out/'collision_drone_geometry.json').read_text())['candidate']['position'])
for filename in ['collision_step_drone.blend','collision_explanation.blend']:
 bpy.ops.wm.open_mainfile(filepath=str(out/filename))
 m=bpy.data.materials.new('Lidar continuous blue housing');m.use_nodes=True;bs=m.node_tree.nodes['Principled BSDF'];bs.inputs['Base Color'].default_value=(.012,.13,.20,1);bs.inputs['Metallic'].default_value=.35;bs.inputs['Roughness'].default_value=.25
 for obj in bpy.data.objects:
  if obj.type!='MESH' or not obj.name.startswith('User STEP'):continue
  scale=obj.get('figure_scale_multiplier',1.0);obj.data.materials.append(m);idx=len(obj.data.materials)-1
  for face in obj.data.polygons:
   q=(obj.matrix_world@face.center-p)/scale;r=math.hypot(q.x,q.y)
   # Color the central sensor housing consistently on sloping and vertical faces.
   # The outside protective cage remains metallic.
   if q.z>.055 and r<.038:face.material_index=idx
 bpy.ops.wm.save_as_mainfile(filepath=str(out/filename))
 if filename=='collision_explanation.blend':
  bpy.context.scene.render.filepath=str(out/'collision_explanation.png');bpy.ops.render.render(write_still=True)
