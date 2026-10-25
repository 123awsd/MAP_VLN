"""CPU setup for original HM3D mesh; official PNG remains the figure background."""
from pathlib import Path
import bpy,bmesh,json,math
from mathutils import Matrix,Vector
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
cfg=json.loads((ROOT/'PRE_MAP_VLN/outputs/stage2_3d/prepared/00337_reference/generated_stage1_config.json').read_text())
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.gltf(filepath='/tmp/fig8_00337_decoded/scene.gltf')
hx,hy,hz=cfg['initial_sensor_habitat_xyz']
T=Matrix.Translation((hx,-hz,1-hy))@Matrix.Diagonal((-1,-1,1,1))@Matrix.Rotation(-math.pi/2,4,'X')
for ob in list(bpy.context.scene.objects):
    if ob.type=='MESH':ob.matrix_world=T@ob.matrix_world
bpy.context.view_layer.update()
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
for mat in bpy.data.materials:
    if not mat.use_nodes:continue
    nt=mat.node_tree
    tex=next((n for n in nt.nodes if n.type=='TEX_IMAGE'),None)
    if tex is None:continue
    tc=nt.nodes.new('ShaderNodeTexCoord');flip=nt.nodes.new('ShaderNodeVectorMath');flip.operation='MULTIPLY_ADD'
    flip.inputs[1].default_value=(1,-1,1);flip.inputs[2].default_value=(0,1,0)
    nt.links.new(tc.outputs['UV'],flip.inputs[0]);nt.links.new(flip.outputs[0],tex.inputs['Vector'])
    out=next(n for n in nt.nodes if n.type=='OUTPUT_MATERIAL')
    em=nt.nodes.new('ShaderNodeEmission');nt.links.new(tex.outputs['Color'],em.inputs['Color']);nt.links.new(em.outputs[0],out.inputs['Surface'])
s=bpy.context.scene;s.render.engine='CYCLES';s.cycles.device='CPU';s.cycles.samples=4;s.cycles.use_denoising=False
s.render.threads_mode='FIXED';s.render.threads=8
s.view_settings.view_transform='Standard';s.view_settings.look='None'
s.render.film_transparent=True;s.render.image_settings.file_format='PNG';s.render.image_settings.color_mode='RGBA'
bpy.ops.object.camera_add(location=(8,-1,25));cam=bpy.context.object;s.camera=cam
cam.rotation_euler=(0,0,math.pi);cam.data.type='ORTHO';cam.data.ortho_scale=20
s.render.resolution_x=s.render.resolution_y=1600;s.render.resolution_percentage=100
bpy.ops.wm.save_as_mainfile(filepath='/tmp/fig8_00337_original.blend')
# Keep the actual second-floor textures for calibration, never for the background.
for ob in list(s.objects):
    if ob.type!='MESH':continue
    zs=[v.co.z for v in ob.data.vertices]
    if not zs or max(zs)<2.35 or min(zs)>3.85:
        bpy.data.objects.remove(ob,do_unlink=True);continue
    bm=bmesh.new();bm.from_mesh(ob.data)
    for z,upper in [(2.35,False),(3.85,True)]:
        bmesh.ops.bisect_plane(bm,geom=list(bm.verts)+list(bm.edges)+list(bm.faces),
            plane_co=(0,0,z),plane_no=(0,0,1),dist=1e-5,clear_inner=not upper,clear_outer=upper)
    bm.to_mesh(ob.data);bm.free();ob.data.update()
s.render.filepath=str(OUT/'assets/calibration.png');bpy.ops.render.render(write_still=True)
