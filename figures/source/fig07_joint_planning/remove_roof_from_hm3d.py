import bpy, bmesh, math
from mathutils import Vector
from pathlib import Path
SRC='/home/uav/map_VLN/素材处理后/第一阶段/目标框检测/场景00337_L2_HM3D原场景_Boxer叠加.glb'
OUT='/home/uav/map_VLN/素材处理后/第一阶段/目标框检测/场景00337_L2_去顶预览'
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=SRC)
# Remove horizontal roof/ceiling faces while preserving walls and furniture.
removed=0
for o in list(bpy.data.objects):
    if o.type != 'MESH': continue
    me=o.data
    bm=bmesh.new(); bm.from_mesh(me)
    faces=[]
    for f in bm.faces:
        c=o.matrix_world @ f.calc_center_median()
        n=o.matrix_world.to_3x3() @ f.normal
        # top-level ceiling/roof surfaces only; keep furniture tops below this height
        if c.z > 2.18 and abs(n.normalized().z) > 0.85:
            faces.append(f)
    if faces:
        removed += len(faces)
        bmesh.ops.delete(bm, geom=faces, context='FACES')
        bm.to_mesh(me); me.update()
    bm.free()
print('removed faces',removed)
# add floor plane under scene for coherent background
bpy.ops.mesh.primitive_plane_add(size=60, location=(1.9,3.5,-0.14))
floor=bpy.context.object; floor.name='Paper floor'
mat=bpy.data.materials.new('Paper floor material'); mat.diffuse_color=(0.88,0.90,0.91,1.0); floor.data.materials.append(mat)
# camera
bpy.ops.object.camera_add(location=(2.0,3.5,19.5))
cam=bpy.context.object; cam.name='Top-down camera'; cam.data.type='ORTHO'; cam.data.ortho_scale=18.9
cam.rotation_euler=(0,0,0)
def look_at(obj, target): obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()
look_at(cam,(2.0,3.5,0)); bpy.context.scene.camera=cam
# lighting
bpy.ops.object.light_add(type='AREA', location=(2,3,12)); key=bpy.context.object; key.data.energy=1600; key.data.shape='DISK'; key.data.size=12
look_at(key,(2,3,0))
bpy.ops.object.light_add(type='AREA', location=(-8,10,8)); fill=bpy.context.object; fill.data.energy=800; fill.data.size=8; look_at(fill,(2,3,0))
# render
scene=bpy.context.scene
scene.render.engine='BLENDER_EEVEE'
scene.render.resolution_x=1200; scene.render.resolution_y=900; scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'; scene.render.filepath=OUT+'.png'
scene.render.film_transparent=False
scene.world = bpy.data.worlds.new('World') if scene.world is None else scene.world
scene.world.color=(0.96,0.97,0.98)
scene.view_settings.look='AgX - Medium High Contrast'
bpy.ops.wm.save_as_mainfile(filepath=OUT+'.blend')
bpy.ops.render.render(write_still=True)
