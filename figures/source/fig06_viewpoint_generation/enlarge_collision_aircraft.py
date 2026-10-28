import bpy,json
from pathlib import Path
from mathutils import Vector,Matrix
from mathutils.bvhtree import BVHTree
out=Path('/home/uav/map_VLN/画图/候选视角生成_新版');report=json.loads((out/'collision_drone_geometry.json').read_text());p=Vector(report['candidate']['position'])
bpy.ops.wm.open_mainfile(filepath=str(out/'collision_explanation.blend'))
for o in list(bpy.data.objects):
 if o.name.startswith('Physical overlap '):bpy.data.objects.remove(o,do_unlink=True)
chairs=[o for o in bpy.data.objects if o.type=='MESH' and o.name.startswith(report['chair']+' ')]
trees=[BVHTree.FromPolygons([o.matrix_world@v.co for v in o.data.vertices],[list(f.vertices) for f in o.data.polygons]) for o in chairs]
turn=Matrix.Translation(p)@Matrix.Scale(1.8,4)@Matrix.Translation(-p)
for o in [o for o in bpy.data.objects if o.type=='MESH' and o.name.startswith('User STEP')]:
 world=o.matrix_world.copy();o.parent=None;o.matrix_world=turn@world
 patch=next((i for i,m in enumerate(o.data.materials) if m and 'collision orange' in m.name.lower()),None)
 if patch is not None:
  default=next((i for i,m in enumerate(o.data.materials) if m and m.name.startswith('Photo smoke brown rotor')),0)
  for face in o.data.polygons:
   if face.material_index==patch:face.material_index=default
   w=o.matrix_world@face.center
   if any(tree.find_nearest(w,.018)[0] is not None for tree in trees):face.material_index=patch
 o['figure_scale_multiplier']=1.8
bpy.context.view_layer.update();s=bpy.context.scene
bpy.ops.wm.save_as_mainfile(filepath=str(out/'collision_explanation.blend'));s.render.filepath=str(out/'collision_explanation.png');bpy.ops.render.render(write_still=True)
