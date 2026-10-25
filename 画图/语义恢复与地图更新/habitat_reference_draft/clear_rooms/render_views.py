"""Render authored cameras from the original HM3D mesh, on CPU in Blender."""
from pathlib import Path
import json, math, bpy
from mathutils import Vector

OUT=Path(__file__).resolve().parent
G=json.loads((OUT/'episode_geometry.json').read_text())
bpy.ops.wm.open_mainfile(filepath='/tmp/fig8_00337_original.blend')
s=bpy.context.scene
s.render.engine='CYCLES';s.cycles.device='CPU'
s.cycles.samples=8;s.cycles.use_denoising=False
s.render.threads_mode='FIXED';s.render.threads=8
s.render.resolution_x=1600;s.render.resolution_y=1000
s.render.resolution_percentage=100
s.render.film_transparent=False
s.render.image_settings.file_format='PNG';s.render.image_settings.color_mode='RGB'
s.world.color=(.05,.05,.05)
cam=s.camera;cam.data.type='PERSP';cam.data.sensor_fit='HORIZONTAL'
cam.data.angle=math.radians(74);cam.data.clip_start=.05;cam.data.clip_end=100
def coord(p):return Vector((p[1],-p[0],p[2]))
manifest=[]
for v in G['stages']:
    cam.location=coord(v['pose_falcon'])
    cam.rotation_euler=(coord(v['target_falcon'])-cam.location).to_track_quat('-Z','Y').to_euler()
    s.render.filepath=str(OUT/'assets'/f"{v['name']}.png")
    bpy.ops.render.render(write_still=True)
    manifest.append(dict(name=v['name'],position_falcon=v['pose_falcon'],
        look_at_falcon=v['target_falcon'],resolution=[1600,1000],hfov_degrees=74))
(OUT/'view_manifest.json').write_text(json.dumps(dict(scene=G['scene_id'],
    source='Original HM3D geometry and RGB textures, decoded with Magnum; CPU Blender Cycles rendering. No generated geometry or texture retouching.',
    poses='Authored for a mechanism illustration; not recorded agent observations.',views=manifest),indent=2)+'\n')
