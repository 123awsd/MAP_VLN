import bpy,json
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
out=Path(__file__).resolve().parent
report=json.loads((out/'collision_drone_geometry.json').read_text());p=Vector(report['candidate']['position'])
bpy.ops.wm.open_mainfile(filepath=str(out/'algorithm_scene.blend'));s=bpy.context.scene;q=world_to_camera_view(s,s.camera,p)
(out/'collision_callout.json').write_text(json.dumps({'projection':[q.x*s.render.resolution_x,(1-q.y)*s.render.resolution_y]}))
bpy.ops.wm.open_mainfile(filepath=str(out/'collision_step_drone.blend'));s=bpy.context.scene
chair=bpy.data.objects[report['chair']+' Chair back'];bpy.context.view_layer.update()
# Keep the actual furniture and aircraft geometry, expose the intersection from the side.
normal=(chair.matrix_world.to_3x3()@Vector((0,1,0))).normalized();side=(chair.matrix_world.to_3x3()@Vector((1,0,0))).normalized()
focus=p+Vector((0,0,-.01));s.camera.location=focus+side*1.7+normal*.7+Vector((0,0,.75));s.camera.rotation_euler=(focus-s.camera.location).to_track_quat('-Z','Y').to_euler();s.camera.data.ortho_scale=.72
for o in bpy.data.objects:
 if o.type=='MESH' and o.name.startswith(report['chair']+' '):
  o.data=o.data.copy()
  for i,mat in enumerate(list(o.data.materials)):
   m=mat.copy();o.data.materials[i]=m
 if o.name.startswith('Physical overlap '):
  for m in o.data.materials:
   bs=m.node_tree.nodes.get('Principled BSDF')
   if bs:
    bs.inputs['Base Color'].default_value=(.9,.008,.015,1);bs.inputs['Emission Color'].default_value=(.6,.002,.005,1);bs.inputs['Emission Strength'].default_value=.35
# The downloaded chair contains open shells; Boolean intersections can create
# spurious full-height caps. Highlight only aircraft faces near real chair surfaces.
for obj in list(bpy.data.objects):
 if obj.name.startswith('Physical overlap '):bpy.data.objects.remove(obj,do_unlink=True)
from mathutils.bvhtree import BVHTree
from mathutils import Matrix
chair_objects=[o for o in s.objects if o.type=='MESH' and o.name.startswith(report['chair']+' ')]
trees=[]
for obj in chair_objects:
 vs=[obj.matrix_world@v.co for v in obj.data.vertices]
 trees.append(BVHTree.FromPolygons(vs,[list(f.vertices) for f in obj.data.polygons]))
patch=bpy.data.materials.get('Physical collision orange')
for obj in [o for o in s.objects if o.type=='MESH' and o.name.startswith('User STEP')]:
 obj.data.materials.append(patch);index=len(obj.data.materials)-1
 for face in obj.data.polygons:
  w=obj.matrix_world@face.center
  if any((hit:=tree.find_nearest(w,.012))[0] is not None for tree in trees):face.material_index=index
bpy.context.view_layer.update()
points=[]
for obj in s.objects:
 if obj.type=='MESH' and not obj.hide_render and (obj.name.startswith(report['chair']+' ') or obj.name.startswith('User STEP')):
  points.extend(obj.matrix_world@Vector(v) for v in obj.bound_box)
basis=s.camera.rotation_euler.to_matrix();inv=basis.transposed();qs=[inv@v for v in points]
lo=Vector([min(v[i] for v in qs) for i in range(3)]);hi=Vector([max(v[i] for v in qs) for i in range(3)])
s.camera.location=basis@((lo+hi)/2)+basis@Vector((0,0,12));s.camera.data.ortho_scale=max(hi.x-lo.x,(hi.y-lo.y)*1400/900)*1.12
s.render.resolution_x=1400;s.render.resolution_y=900;s.cycles.samples=64;s.render.film_transparent=True;s.render.filepath=str(out/'collision_explanation.png')
bpy.ops.wm.save_as_mainfile(filepath=str(out/'collision_explanation.blend'));bpy.ops.render.render(write_still=True)
