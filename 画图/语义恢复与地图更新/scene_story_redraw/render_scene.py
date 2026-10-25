"""Render a textured, cropped HM3D scene; export projection for SVG annotations.

The route is deliberately authored for a mechanism illustration, not an
algorithm output. Original camera images are separate reference observations.
Run: blender -b -t 8 --python render_scene.py -- --draft
"""
import bpy
import bmesh
import math
import json
import sys
from pathlib import Path
from mathutils import Matrix, Vector
from bpy_extras.object_utils import world_to_camera_view

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
DRAFT = '--draft' in sys.argv
SCENE = Path('/tmp/fig8_00166_decoded/scene.gltf')
SOURCE = '/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train/00166-RaYrxWt5pR1/RaYrxWt5pR1.basis.glb'
CONFIG = ROOT / 'PRE_MAP_VLN/outputs/stage2_3d/tasks/00166_cushion_recovery_20261024/generated_stage1_config.json'
cfg = json.loads(CONFIG.read_text())
hx, hy, hz = cfg['initial_sensor_habitat_xyz']
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.gltf(filepath=str(SCENE))
meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']

# Imported glTF -> Z-up Habitat -> editorial coordinates (FALCON y, -x, z).
# This is one rigid transform applied equally to geometry and annotation poses.
transform = Matrix.Translation((hx, -hz, 1-hy)) @ Matrix.Diagonal((-1,-1,1,1)) @ Matrix.Rotation(-math.pi/2,4,'X')
for ob in meshes:
    ob.matrix_world = transform @ ob.matrix_world
bpy.context.view_layer.update()
bpy.ops.object.select_all(action='DESELECT')
for ob in meshes: ob.select_set(True)
bpy.context.view_layer.objects.active = meshes[0]
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# Ceiling and peripheral rooms are removed only for the explanatory cutaway.
bounds = [(-10.5,-2.55),(1.30,7.12),(-.24,1.22)]
for ob in meshes:
    bm = bmesh.new(); bm.from_mesh(ob.data)
    for axis, (low, high) in enumerate(bounds):
        for value, upper in [(low,False),(high,True)]:
            co=[0,0,0];co[axis]=value
            normal=[0,0,0];normal[axis]=1
            bmesh.ops.bisect_plane(bm, geom=list(bm.verts)+list(bm.edges)+list(bm.faces),
                plane_co=co,plane_no=normal,dist=1e-5,
                clear_inner=not upper,clear_outer=upper)
    bm.to_mesh(ob.data);bm.free();ob.data.update()

scene=bpy.context.scene
scene.render.engine='CYCLES'
scene.cycles.samples=16 if DRAFT else 48
scene.cycles.use_denoising=False
scene.render.threads_mode='FIXED';scene.render.threads=8
scene.render.resolution_x=2200 if DRAFT else 3200
scene.render.resolution_y=1320 if DRAFT else 1920
scene.render.resolution_percentage=100
scene.render.film_transparent=True
scene.render.image_settings.file_format='PNG'
scene.render.image_settings.color_mode='RGBA'
scene.view_settings.view_transform='Standard'
scene.view_settings.look='Medium High Contrast'
scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(1,1,1,1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.8

# The scene's unlit photo textures preserve its observed colors.
# A gentle material-level desaturation makes the later vector overlay legible.
for material in bpy.data.materials:
    if not material.use_nodes: continue
    nt=material.node_tree
    for node in list(nt.nodes):
        if node.type != 'TEX_IMAGE': continue
        # Magnum decoded Basis pixels use a bottom-left origin; the external
        # PNG/glTF import needs the opposite V direction. Correct UV sampling.
        coords=nt.nodes.new('ShaderNodeTexCoord')
        flip=nt.nodes.new('ShaderNodeVectorMath');flip.operation='MULTIPLY_ADD'
        flip.inputs[1].default_value=(1,-1,1)
        flip.inputs[2].default_value=(0,1,0)
        nt.links.new(coords.outputs['UV'],flip.inputs[0])
        nt.links.new(flip.outputs[0],node.inputs['Vector'])
        links=list(node.outputs['Color'].links)
        if not links: continue
        hsv=nt.nodes.new('ShaderNodeHueSaturation')
        hsv.inputs['Saturation'].default_value=.68
        hsv.inputs['Value'].default_value=1.06
        nt.links.new(node.outputs['Color'],hsv.inputs['Color'])
        for link in links:
            socket=link.to_socket
            nt.links.remove(link)
            nt.links.new(hsv.outputs['Color'],socket)

center=Vector((-6.5,4.15,.35))
bpy.ops.object.camera_add()
camera=bpy.context.object
camera.name='Fig8 cutaway camera'
camera.location=center+Vector((1.7,-8.8,11.8))
camera.rotation_euler=(center-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO';camera.data.ortho_scale=10.1
scene.camera=camera
bpy.context.view_layer.update()

def project(p):
    q=world_to_camera_view(scene,camera,Vector((p[1],-p[0],p[2])))
    return [float(q.x),float(1-q.y)]

# Local retry is shortened to two observations. Route exits through the actual
# doorway, but its shape and sample spacing are authored for readability.
poses = {
    'q1':[-3.45,-7.55,.93],
    'q2':[-3.97,-7.13,.93],
    'q3':[-4.12,-5.29,.84],
    'qpre':[-2.71,-6.09,1.0],
}
anchors={
    'armchair':[-4.70,-7.55,.65],
    'sofa':[-4.62,-3.43,.58],
    'cushion':[-4.03,-3.43,.75],
    'bedroom':[-3.2,-9.25,.08],
    'living_room':[-4.4,-4.5,.08],
    'door':[-1.75,-7.05,.6],
}
route_local=[[-2.3,-8.1,.93],[-2.9,-7.9,.93],poses['q1'],[-3.7,-7.22,.93],poses['q2']]
route_recovery=[poses['q2'],[-3.5,-7.72,.98],[-2.6,-7.82,.98],[-1.8,-7.55,.98],
                [-1.78,-7.04,.98],[-2.2,-6.55,.98],poses['qpre'],[-3.2,-5.62,.93],poses['q3']]

def frustum(p, aim, length=1.1, angle=30):
    x,y,z=p; dx,dy=aim[0]-x,aim[1]-y
    norm=math.hypot(dx,dy);dx/=norm;dy/=norm
    w=length*math.tan(math.radians(angle))
    return [p,[x+length*dx-w*dy,y+length*dy+w*dx,z],
            [x+length*dx+w*dy,y+length*dy-w*dx,z]]

projection={
    'poses':{k:project(v) for k,v in poses.items()},
    'anchors':{k:project(v) for k,v in anchors.items()},
    'route_local':[project(v) for v in route_local],
    'route_recovery':[project(v) for v in route_recovery],
    'frusta':{key:[project(p) for p in frustum(value,anchors['armchair' if key in ('q1','q2') else 'sofa'],
                length={'q1':1.25,'q2':.88,'q3':1.95,'qpre':1.1}[key],angle=24)]
              for key,value in poses.items()},
}
# Region samples are a schematic visualization of new versus repeated coverage.
samples=[]
for i in range(6):
    for j in range(3):
        p=[-4.82+i*.18,-7.82+j*.20,.10]
        category='previous' if j<1 else 'new'
        samples.append({'uv':project(p),'category':category})
projection['coverage_samples']=samples

def obb(center, size):
    from itertools import product
    return [project([center[i]+s[i]*size[i]/2 for i in range(3)]) for s in product([-1,1],repeat=3)]
projection['target_box']=obb([-4.03,-3.43,.72],[.54,.26,.42])
projection['anchor_box']=obb([-4.70,-7.55,.48],[.70,.78,.96])
projection['bounds_falcon']={'x':[-7.12,-1.3],'y':[-10.5,-2.55],'z':[-.24,1.22]}
projection['source_scene']=SOURCE
projection['illustrative_route']=True
projection['poses_falcon']=poses
projection['route_falcon']={'local':route_local,'recovery':route_recovery}
(OUT/'assets/projection.json').write_text(json.dumps(projection,indent=2)+'\n')
scene.render.filepath=str(OUT/'assets/cutaway.png')
bpy.ops.render.render(write_still=True)
if not DRAFT:
    bpy.ops.file.pack_all()
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'fig8_textured_scene.blend'),compress=True)
print('FIG8_TEXTURED_SCENE_READY')
