import bpy,json,math
from pathlib import Path
from mathutils import Vector,Matrix
out=Path(__file__).resolve().parent
# User confirmed the second reference face as the aircraft front.
heading_correction=math.pi/2-math.atan2(.937,.35)
bpy.ops.wm.open_mainfile(filepath=str(out/'algorithm_scene.blend'))
s=bpy.context.scene
audit=json.loads((out/'polished_projection.json').read_text())['audit']
# Pick a near-miss rejection: the drone center is free, but its 0.30 m envelope overlaps furniture.
failed=[a for a in audit if a['collision'] and .08<a['clearance_m']<.30]
a=max(failed,key=lambda a:a['clearance_m']);p=Vector(a['position'])
parts=[o for o in bpy.data.objects if o.type=='MESH' and o.name.startswith('Seat ')]
nearest=min(parts,key=lambda o:(o.location-p).length)
prefix=nearest.name.split(' Chair')[0]
for o in bpy.data.objects:
 if o.name.startswith('Seat ') and not o.name.startswith(prefix+' '):o.hide_render=True
 if o.name.startswith('Table ') or o.name in ['Full tabletop observation surface','Region perimeter']:o.hide_render=True
before=set(bpy.data.objects)
bpy.ops.import_scene.gltf(filepath=str(out/'solidworks_model/assembly_complete.glb'))
objects=set(bpy.data.objects)-before;meshes=[o for o in objects if o.type=='MESH']
bpy.context.view_layer.update()
verts=[o.matrix_world@Vector(c) for o in meshes for c in o.bound_box]
lo=Vector([min(v[i] for v in verts) for i in range(3)]);hi=Vector([max(v[i] for v in verts) for i in range(3)]);center=(lo+hi)/2
# Explicit schematic body width 0.40 m, safety envelope radius 0.30 m.
scale=1.0
native_width=max(hi.x-lo.x,hi.y-lo.y)
transform=Matrix.Translation(p)@Matrix.Scale(scale,4)@Matrix.Translation(-center)
roots=[o for o in objects if o.parent not in objects]
for o in roots:o.matrix_world=transform@o.matrix_world
for m in bpy.data.materials:
 if not m.use_nodes:continue
 for n in m.node_tree.nodes:
  if n.type=='BSDF_PRINCIPLED' and 'Emission Strength' in n.inputs:n.inputs['Emission Strength'].default_value=min(4,n.inputs['Emission Strength'].default_value)
red=bpy.data.materials.new('Safety envelope red');red.use_nodes=True;bs=red.node_tree.nodes['Principled BSDF'];bs.inputs['Base Color'].default_value=(.80,.025,.035,1);bs.inputs['Roughness'].default_value=.7
# Evaluate actual candidate positions without moving any global furniture.
# Rank depth-weighted penetration of drone mesh vertices into chair OBBs.
bpy.context.view_layer.update()
relative=[body.matrix_world@v.co-p for body in meshes for v in list(body.data.vertices)[::30]]
chairs=[o for o in parts if 'Chair back' in o.name or 'Chair seat' in o.name]
best=None
for candidate in [v for v in audit if v['collision'] and v['azimuth_deg']==0]:
 cp=Vector(candidate['position']);yaw=math.atan2(-cp.y,-cp.x);rot=Matrix.Rotation(yaw+heading_correction,3,'Z') @ Matrix.Rotation(math.pi,3,'X')
 score=0.;count=0;by_chair={}
 for chair in chairs:
  inv=chair.matrix_world.inverted();bb=[Vector(v) for v in chair.bound_box]
  lo=[min(v[i] for v in bb) for i in range(3)];hi=[max(v[i] for v in bb) for i in range(3)]
  subtotal=0.
  for v in relative:
   q=inv@(cp+rot@v)
   if all(lo[i]<q[i]<hi[i] for i in range(3)):
    depth=min(min(q[i]-lo[i],hi[i]-q[i]) for i in range(3));subtotal+=depth;count+=1
  score+=subtotal;by_chair[chair.name]=subtotal
 if best is None or score>best[0]:best=(score,candidate,yaw,count,by_chair)
score,a,yaw,count,by_chair=best
old_p=p.copy();p=Vector(a['position']);rotation=Matrix.Rotation(yaw+heading_correction,4,'Z') @ Matrix.Rotation(math.pi,4,'X')
move=Matrix.Translation(p)@rotation@Matrix.Translation(-old_p)
for obj in roots:obj.matrix_world=move@obj.matrix_world
nearest=bpy.data.objects[max(by_chair,key=by_chair.get)];prefix=nearest.name.split(' Chair')[0]
for obj in parts:obj.hide_render=not obj.name.startswith(prefix+' ')
shift=Vector((0,0,0));bpy.context.view_layer.update()
print('WORST CANDIDATE',a['azimuth_deg'],'penetrating vertices',count,'depth score',score)
# Color only the drone's actual mesh volume intersecting chair components.
patch=bpy.data.materials.new('Physical collision orange');patch.use_nodes=True
bs=patch.node_tree.nodes['Principled BSDF'];bs.inputs['Base Color'].default_value=(1.,.12,.005,1);bs.inputs['Roughness'].default_value=.65
for body in meshes:
 for chair in [o for o in parts if o.name.startswith(prefix+' ')]:
  duplicate=body.copy();duplicate.data=body.data.copy();bpy.context.collection.objects.link(duplicate);duplicate.name='Physical overlap '+body.name+' '+chair.name
  duplicate.parent=None;duplicate.matrix_world=body.matrix_world.copy()
  duplicate.data.materials.clear();duplicate.data.materials.append(patch)
  for modifier in list(duplicate.modifiers):duplicate.modifiers.remove(modifier)
  boolean=duplicate.modifiers.new('Actual body chair intersection','BOOLEAN');boolean.operation='INTERSECT';boolean.solver='EXACT';boolean.object=chair
  bpy.context.view_layer.objects.active=duplicate
  try:bpy.ops.object.modifier_apply(modifier=boolean.name)
  except RuntimeError:pass
  if len(duplicate.data.vertices):
   center=sum((v.co for v in duplicate.data.vertices),Vector())/len(duplicate.data.vertices)
   for v in duplicate.data.vertices:v.co=center+(v.co-center)*1.02
cam=s.camera;focus=p*.65+Vector((nearest.location.x,nearest.location.y,p.z))*.35;away=Vector((p.x-nearest.location.x,p.y-nearest.location.y,0)).normalized();offset=away*2+Vector((-away.y,away.x,0))*1.3+Vector((0,0,1.5));cam.location=focus+offset;cam.rotation_euler=(focus-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=1.25
s.render.resolution_x=1300;s.render.resolution_y=1200;s.cycles.samples=64;s.render.film_transparent=True
s.render.filepath=str(out/'collision_step_drone.png')
(out/'collision_drone_geometry.json').write_text(json.dumps({'candidate':a,'drone_width_m':native_width,'model_source':'desktop assembly STEP; native dimensions','demonstration':'unmodified global candidate; maximum depth-weighted vertex penetration into chair bounding boxes','penetrating_vertices':count,'penetration_depth_score':score,'yaw_rad':yaw,'model_heading_correction_rad':heading_correction,'drone_translation_m':list(shift),'chair':prefix},indent=2))
bpy.ops.wm.save_as_mainfile(filepath=str(out/'collision_step_drone.blend'))
bpy.ops.render.render(write_still=True)
