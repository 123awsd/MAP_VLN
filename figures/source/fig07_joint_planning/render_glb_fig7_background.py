import bpy
from mathutils import Vector
from pathlib import Path
ROOT=Path('/home/uav/map_VLN'); src=Path('/home/uav/下载/home_-_p3.glb'); out=ROOT/'PRE_MAP_VLN/figures/source/fig07_joint_planning/home_p3_clean_background.png'
bpy.ops.wm.read_factory_settings(use_empty=True); bpy.ops.import_scene.gltf(filepath=str(src))
bpy.ops.object.camera_add(location=(-190,-240,700)); cam=bpy.context.object; cam.data.type='ORTHO'; cam.data.ortho_scale=420; cam.rotation_euler=(Vector((-190,-240,0))-cam.location).to_track_quat('-Z','Y').to_euler(); bpy.context.scene.camera=cam
bpy.ops.object.light_add(type='AREA',location=(-190,-240,500)); key=bpy.context.object; key.data.energy=18000; key.data.size=360; key.rotation_euler=(Vector((-190,-240,0))-key.location).to_track_quat('-Z','Y').to_euler()
sc=bpy.context.scene; sc.render.engine='BLENDER_EEVEE'; sc.render.resolution_x=1200; sc.render.resolution_y=900; sc.render.resolution_percentage=100; sc.render.image_settings.file_format='PNG'; sc.render.filepath=str(out); sc.world=bpy.data.worlds.new('World'); sc.world.color=(.55,.58,.62); sc.view_settings.look='AgX - Medium High Contrast'; sc.view_settings.exposure=-1.0
bpy.ops.render.render(write_still=True); print(out)
