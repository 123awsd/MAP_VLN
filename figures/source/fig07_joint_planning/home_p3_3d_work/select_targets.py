"""Make a target-selection draft over the original GLB; dimensions are approximate GLB units."""
import bpy, json
from pathlib import Path
from mathutils import Vector
HERE=Path(__file__).resolve().parent
SRC=Path('/home/uav/下载/home_-_p3.glb')
# Bounds manually estimated from an orthographic inspection, not detection results.
targets=[
 ('A_bed_bedroom',(-383,-312,-168,-96,0,30)),
 ('B_desk_study',(-266,-238,-237,-149,0,38)),
 ('C_sink_bathroom',(-359,-328,-302,-279,0,36)),
 ('D_sofa_living',(-37,5,-218,-127,0,43)),
]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=str(SRC))
collection=bpy.data.collections.new('Fig7 | target boxes - draft')
bpy.context.scene.collection.children.link(collection)
mat=bpy.data.materials.new('Target outline - warm yellow'); mat.diffuse_color=(1.0,0.34,0.015,1); mat.use_nodes=True
bs=mat.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=mat.diffuse_color; bs.inputs['Emission Color'].default_value=mat.diffuse_color; bs.inputs['Emission Strength'].default_value=0.4
for name,(xmin,xmax,ymin,ymax,zmin,zmax) in targets:
 corners=[(x,y,z) for x in (xmin,xmax) for y in (ymin,ymax) for z in (zmin,zmax)]
 curve=bpy.data.curves.new(name,'CURVE'); curve.dimensions='3D'; curve.bevel_depth=0.6; curve.bevel_resolution=2
 for i,a in enumerate(corners):
  for j,b in enumerate(corners):
   if j>i and sum(a[k]!=b[k] for k in range(3))==1:
    sp=curve.splines.new('POLY'); sp.points.add(1); sp.points[0].co=(*a,1); sp.points[1].co=(*b,1)
 obj=bpy.data.objects.new(name,curve); collection.objects.link(obj); obj.data.materials.append(mat)
 obj['provenance']='Manually estimated from scene inspection; requires 3D bounding-box verification.'
with open(HERE/'target_selection_draft.json','w',encoding='utf8') as f: json.dump({'source':str(SRC),'units':'GLB units; scale to metres not yet verified','status':'manual provisional bounds; no candidates or collision-checked paths','targets':[{'id':a,'bbox_xyz':list(b)} for a,b in targets]},f,ensure_ascii=False,indent=2)
bpy.ops.object.camera_add(location=(-190,-240,700)); cam=bpy.context.object; cam.name='Target inspection camera'; cam.data.type='ORTHO'; cam.data.ortho_scale=453.6; cam.rotation_euler=(Vector((-190,-240,0))-cam.location).to_track_quat('-Z','Y').to_euler(); bpy.context.scene.camera=cam
bpy.ops.object.light_add(type='AREA',location=(-190,-240,500)); light=bpy.context.object; light.data.energy=18000; light.data.size=360; light.rotation_euler=(Vector((-190,-240,0))-light.location).to_track_quat('-Z','Y').to_euler()
sc=bpy.context.scene; sc.render.engine='BLENDER_EEVEE'; sc.render.resolution_x=1200; sc.render.resolution_y=960; sc.render.resolution_percentage=100; sc.world=bpy.data.worlds.new('World'); sc.world.color=(.55,.58,.62); sc.view_settings.look='AgX - Medium High Contrast'; sc.view_settings.exposure=-0.5; sc.render.filepath=str(HERE/'target_selection_draft.png')
bpy.ops.wm.save_as_mainfile(filepath=str(HERE/'target_selection_draft.blend'))
bpy.ops.render.render(write_still=True)
