"""Render a lightly oblique camera view of the editable 3D MINCO draft."""
from pathlib import Path
import bpy,json
from mathutils import Vector
H=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(H/'start_room_astar_raw.blend'))
sc=bpy.context.scene
cam=sc.camera; target=Vector((-192,-240,20)); cam.location=target+Vector((20,-110,920)); cam.rotation_euler=(target-cam.location).to_track_quat('-Z','Y').to_euler(); cam.data.type='ORTHO'; cam.data.ortho_scale=490
# Draw the actual original MINCO future phases as 3D dashed curves. No path offset.
data=json.loads((H/'minco_start_room'/'astar_points.json').read_text())
material=bpy.data.materials.new('Previous plan - dashed gray'); material.diffuse_color=(.18,.20,.22,1); material.use_nodes=True
node=material.node_tree.nodes.get('Principled BSDF'); node.inputs['Base Color'].default_value=material.diffuse_color; node.inputs['Emission Color'].default_value=material.diffuse_color; node.inputs['Emission Strength'].default_value=.35
collection=bpy.data.collections.new('Previous plan | actual initial MINCO dashed'); sc.collection.children.link(collection)
for phase_index,phase in enumerate(data['initial']['phases'][1:],1):
 curve=bpy.data.curves.new('Previous MINCO phase '+str(phase_index),'CURVE'); curve.dimensions='3D'; curve.bevel_depth=1.05; curve.bevel_resolution=2
 dense=[]
 for aa,bb in zip(phase,phase[1:]):
  a,b=Vector(aa),Vector(bb); count=max(1,int((b-a).length/.02)); dense.extend([a.lerp(b,i/count) for i in range(count)])
 dense.append(Vector(phase[-1]))
 previous=dense[0]; distance=0.; part=[]; chunks=[]
 for point in dense:
  p=Vector(point); distance+=(p-previous).length; previous=p
  if distance%0.55<0.31: part.append(p/.0254)
  elif part:
   if len(part)>1: chunks.append(part)
   part=[]
 if len(part)>1: chunks.append(part)
 for segment in chunks:
  sp=curve.splines.new('POLY'); sp.points.add(len(segment)-1)
  for item,point in zip(sp.points,segment): item.co=(*point,1)
 ob=bpy.data.objects.new('Previous MINCO phase '+str(phase_index),curve); collection.objects.link(ob); curve.materials.append(material)
 ob['provenance']='initial A-star nodes, future portion only'; ob['display']='dashed, no geometric offset'
# Separate the Start label from the nearby A3 label in the figure.
if bpy.data.objects.get('Start label'): bpy.data.objects['Start label'].location.x+=12; bpy.data.objects['Start label'].location.y-=5
# Labels face the camera; anchor points, frusta and trajectories remain unchanged.
for obj in bpy.data.objects:
 if obj.type=='FONT': obj.rotation_euler=cam.rotation_euler
sc.render.resolution_x=1600; sc.render.resolution_y=1280; sc.render.resolution_percentage=100
sc.render.film_transparent=False; sc.eevee.taa_render_samples=64; sc.eevee.use_gtao=True; sc.eevee.gtao_distance=4; sc.eevee.gtao_factor=1.1
sc.view_settings.view_transform='Standard'; sc.view_settings.look='Medium High Contrast'; sc.view_settings.exposure=0; sc.view_settings.gamma=1
sc.world.color=(.72,.72,.72)
sc.render.image_settings.file_format='PNG'; sc.render.filepath=str(H/'start_room_astar_render.png')
bpy.ops.wm.save_as_mainfile(filepath=str(H/'start_room_astar_view.blend'))
bpy.ops.render.render(write_still=True)
