import bpy
from pathlib import Path
from mathutils import Vector,Matrix
out=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(out/'candidate_selection_scene.blend'))
for o in list(bpy.data.objects):
 if o.name.startswith('Table '):bpy.data.objects.remove(o,do_unlink=True)
before=set(bpy.data.objects);bpy.ops.import_scene.gltf(filepath=str(out/'assets/edelweiss_extending_dining_table_ash_and_white.glb'));new=set(bpy.data.objects)-before;bpy.context.view_layer.update()
meshes=[o for o in new if o.type=='MESH' and 'FLOOR' not in o.name.upper()]
vs=[o.matrix_world@Vector(v) for o in meshes for v in o.bound_box];lo=Vector([min(v[i] for v in vs) for i in range(3)]);hi=Vector([max(v[i] for v in vs) for i in range(3)]);center=(lo+hi)/2
for i,o in enumerate(meshes):
 world=o.matrix_world.copy();o.data=o.data.copy();o.parent=None;o.matrix_world=Matrix.Identity(4)
 for v in o.data.vertices:
  w=world@v.co;v.co=Vector(((w.x-center.x)*2/(hi.x-lo.x),(w.y-center.y)/(hi.y-lo.y),(w.z-lo.z)/(hi.z-lo.z)))
 o.name=f'Table Edelweiss {i}'
for o in new:
 if o.type=='MESH' and o not in meshes:bpy.data.objects.remove(o,do_unlink=True)
bpy.ops.wm.save_as_mainfile(filepath=str(out/'candidate_selection_scene.blend'))
