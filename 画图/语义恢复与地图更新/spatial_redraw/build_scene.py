"""Blender-built, explicitly schematic geometry for the Fig. 8 design study.

Run: blender -b --python build_scene.py -- --draft
The scene, observation windows and camera projections share one coordinate frame.
These are illustrative renders, not sensor observations or experiment results.
"""
import bpy
import json
import math
import sys
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

OUT = Path(__file__).resolve().parent
DRAFT = '--draft' in sys.argv
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.render.engine = 'CYCLES'
scene.cycles.samples = 48 if DRAFT else 128
scene.cycles.use_denoising = False
scene.render.threads_mode = 'FIXED'
scene.render.threads = 8
scene.render.film_transparent = True
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGBA'
scene.view_settings.view_transform = 'Standard'
scene.view_settings.look = 'Medium High Contrast'
scene.view_settings.exposure = 0
scene.view_settings.gamma = 1
scene.world.use_nodes = True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value = (1, 1, 1, 1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value = .65


def rgb(hexstr):
    vals = [int(hexstr[i:i+2], 16)/255 for i in (1, 3, 5)]
    return tuple(v/12.92 if v <= .04045 else ((v+.055)/1.055)**2.4 for v in vals)


def material(name, color, rough=.65, alpha=1):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*rgb(color), 1)
    bsdf.inputs['Roughness'].default_value = rough
    bsdf.inputs['Specular IOR Level'].default_value = .2
    bsdf.inputs['Alpha'].default_value = alpha
    m.diffuse_color = (*rgb(color), alpha)
    m.blend_method = 'BLEND' if alpha < 1 else 'OPAQUE'
    return m


FLOOR = material('warm-white floor', '#F1EFEB')
WALL = material('architectural gray', '#D5DADD')
CAP = material('wall upper face', '#E4E7E8')
ANCHOR = material('anchor body', '#C6CCD0')
TOP = material('anchor top', '#E4E7E7')
METAL = material('anchor supports', '#9DA8B0')
PAST = material('previous views', '#8A99A2')
PAST_FAINT = material('past view surfaces', '#8FA0AC', alpha=.035)
TEAL = material('recovery view', '#287A91')
TEAL_FAINT = material('viewing volume', '#4B9CAA', alpha=.065)
REGION_OLD = material('old search footprint', '#CBD5D9', alpha=.34)
REGION_NEW = material('alternative footprint', '#7DAFB5', alpha=.13)
ORANGE = material('verified evidence', '#D27A45')
TARGET = material('untextured target', '#79868E')
GRID = material('floor guide lines', '#DFE1DF')


def box(name, center, size, mat, bevel=0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new('minute edge bevel', 'BEVEL')
        mod.width = bevel
        mod.segments = 3
        obj.modifiers.new('weighted corner normals', 'WEIGHTED_NORMAL')
    return obj


def line(name, pts, mat, radius=.009, cyclic=False):
    data = bpy.data.curves.new(name, 'CURVE')
    data.dimensions = '3D'
    data.bevel_depth = radius
    data.bevel_resolution = 3
    sp = data.splines.new('POLY')
    sp.points.add(len(pts)-1)
    for p, co in zip(sp.points, pts):
        p.co = (*co, 1)
    sp.use_cyclic_u = cyclic
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    data.materials.append(mat)
    return obj


def mesh(name, points, faces, mat):
    data = bpy.data.meshes.new(name)
    data.from_pydata(points, [], faces)
    data.update()
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    data.materials.append(mat)
    return obj


def dot(name, pos, mat, radius=.05):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8,
                                      radius=radius, location=pos)
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    for p in obj.data.polygons:
        p.use_smooth = True
    return obj


def footprint(name, x, y, w, d, mat, outline):
    pts = [(x-w/2,y-d/2,.025),(x+w/2,y-d/2,.025),
           (x+w/2,y+d/2,.025),(x-w/2,y+d/2,.025)]
    obj = mesh(name, pts, [(0,1,2,3)], mat)
    obj.visible_shadow = False
    line(name+' boundary', pts, outline, radius=.012, cyclic=True)


def bench(name, x, y, w=1.55, d=.78):
    box(name+' worktop', (x,y,.91), (w,d,.12), TOP, .018)
    box(name+' apron', (x,y,.79), (w-.12,d-.12,.13), ANCHOR, .008)
    for dx in (-w/2+.12,w/2-.12):
        for dy in (-d/2+.12,d/2-.12):
            box(name+' support', (x+dx,y+dy,.43), (.065,.065,.8), METAL, .007)


box('continuous map slab', (0,2.65,-.11), (9.7,5.5,.22), FLOOR, .035)
box('back boundary', (0,5.32,.49), (9.6,.13,.98), WALL, .008)
box('back coping', (0,5.32,.99), (9.63,.16,.04), CAP, .006)
box('left cutaway wall', (-4.77,2.65,.22), (.12,5.3,.44), WALL, .007)
box('right cutaway wall', (4.77,2.65,.22), (.12,5.3,.44), WALL, .007)
box('partition rear', (0,4.64,.43), (.13,1.35,.86), WALL, .007)
box('partition front', (0,.56,.20), (.13,1.12,.40), WALL, .007)
for x in [-3.6,-2.4,-1.2,0,1.2,2.4,3.6]:
    line('floor seam', [(x,.03,.006),(x,5.2,.006)], GRID, .003)
for y in [1.2,2.4,3.6,4.8]:
    line('floor seam', [(-4.7,y,.006),(4.7,y,.006)], GRID, .003)
footprint('initial search region', -2.65,3.25,2.65,2.12,REGION_OLD,PAST)
footprint('recovery search region', 2.5,3.25,2.65,2.12,REGION_NEW,TEAL)
bench('initial mapped anchor', -2.65,3.6)
bench('alternative mapped anchor', 2.5,3.6)

# A generic instance is intentionally used instead of a stylized category icon.
target = box('novel target instance', (2.62,3.44,1.18), (.28,.24,.42), TARGET, .025)
box('target top detail', (2.62,3.44,1.40), (.22,.19,.035), TARGET, .01)
bbox_min = Vector((2.42,3.26,.94))
bbox_max = Vector((2.82,3.62,1.48))
corners = [(x,y,z) for z in (bbox_min.z,bbox_max.z)
           for y in (bbox_min.y,bbox_max.y) for x in (bbox_min.x,bbox_max.x)]
for a,b in [(0,1),(0,2),(1,3),(2,3),(4,5),(4,6),(5,7),(6,7),(0,4),(1,5),(2,6),(3,7)]:
    line('verified 3D box', [corners[a],corners[b]], ORANGE, .009)

positions = {'q1':(-4.00,1.50,1.15), 'q2':(-2.55,.85,1.15),
             'q3':(-.84,1.70,1.15), 'q4':(1.85,1.25,1.15),
             'q5':(4.03,2.25,1.15)}
goals = {'old':(-2.65,3.45,1.15), 'new':(2.62,3.44,1.15)}
frustum_objects = []


def frustum(key, aim, mat, face, length, dashed=False):
    a = Vector(positions[key]); d = (Vector(aim)-a).normalized()
    right = d.cross(Vector((0,0,1))).normalized()
    up = right.cross(d).normalized()
    center = a+length*d
    hw = length*math.tan(math.radians(22))
    hh = hw * .7  # Same sensor aspect and FoV as the illustrative observation images.
    far = [center-hw*right-hh*up, center+hw*right-hh*up,
           center+hw*right+hh*up, center-hw*right+hh*up]
    objects=[]
    for i,p in enumerate(far):
        if dashed:
            for u in range(0,10,2):
                objects.append(line(key+' dashed ray', [a+(p-a)*(u/10),a+(p-a)*((u+1)/10)], mat, .008))
        else:
            objects.append(line(key+' ray', [a,p], mat, .009))
    objects.append(line(key+' image plane', far, mat, .009, cyclic=True))
    o=mesh(key+' viewing volume',[a]+far,[(0,1,2),(0,2,3),(0,3,4),(0,4,1)],face)
    o.visible_shadow=False
    objects.append(o)
    objects.append(dot(key+' observation pose', a, mat, .067))
    line(key+' vertical projection',[(*a[:2],.03),a], GRID, .006)
    frustum_objects.extend(objects)


frustum('q1', goals['old'], PAST, PAST_FAINT, .75)
frustum('q2', goals['old'], PAST, PAST_FAINT, .75)
frustum('q3', goals['old'], PAST, PAST_FAINT, 1.05)
frustum('q4', goals['new'], TEAL, TEAL_FAINT, 1.50)
frustum('q5', goals['new'], PAST, PAST_FAINT, .65)

# Soft lighting is used only for depth cues; no dramatic materials or shadows.
ld = bpy.data.lights.new('large softbox','AREA')
lo = bpy.data.objects.new('large softbox',ld)
bpy.context.collection.objects.link(lo)
lo.location=(-3,-4,11);ld.energy=950;ld.shape='DISK';ld.size=8


def camera(name, pos, aim, ortho=None):
    data=bpy.data.cameras.new(name)
    ob=bpy.data.objects.new(name,data)
    bpy.context.collection.objects.link(ob)
    ob.location=pos
    ob.rotation_euler=(Vector(aim)-ob.location).to_track_quat('-Z','Y').to_euler()
    if ortho:
        data.type='ORTHO';data.ortho_scale=ortho
    else:
        data.type='PERSP';data.sensor_fit='HORIZONTAL';data.sensor_width=36
        data.lens=36/(2*math.tan(math.radians(22)))
    data.clip_start=.05;data.clip_end=100
    return ob


cam=camera('orthographic editorial camera',(8,-11,14),(0,2.6,.25),ortho=11.5)
scene.camera=cam
scene.render.resolution_x=1600 if DRAFT else 2600
scene.render.resolution_y=1300 if DRAFT else 2112
scene.render.resolution_percentage=100
bpy.context.view_layer.update()


def project(camera, xyz):
    p=world_to_camera_view(scene,camera,Vector(xyz))
    return [p.x,1-p.y]


anchors={**positions, 'r0':(-2.65,4.72,.03), 'r1':(2.5,4.72,.03),
         'old_anchor':(-2.65,3.6,1.0), 'new_anchor':(2.5,3.6,1.0),
         'target':(2.62,3.44,1.55), 'bridge':(.08,1.55,.75),
         'near_old':(-3.3,2.8,.03), 'near_new':(3.0,2.8,.03)}
metadata={'scope':'Illustrative mechanism scene; not an experimental reconstruction.',
          'projection':'orthographic', 'positions_m':positions,
          'goals_m':goals, 'main_points':{k:project(cam,p) for k,p in anchors.items()},
          'render_size':[scene.render.resolution_x,scene.render.resolution_y],
          'camera_horizontal_fov_deg':44, 'camera_pitch_deg':0,
          'frustum_aspect':.7, 'display_frustum_lengths_are_not_sensing_range':True,
          'sequence_not_trajectory':True}
scene.render.filepath=str(OUT/'scene.png')
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'fig8_scene.blend'))
bpy.ops.render.render(write_still=True)

# Inset observations come from the same scene and the same declared viewpoints.
# Hide diagram overlays so these remain simple illustrative observation windows.
for ob in frustum_objects:
    ob.hide_render=True
for ob in bpy.data.objects:
    if any(k in ob.name for k in ['verified 3D box','search region','projection']):
        ob.hide_render=True
scene.render.resolution_x=600;scene.render.resolution_y=420
scene.cycles.samples=64
for key in ('q4','q5'):
    oc=camera('illustrative observation '+key,positions[key],(2.62,3.44,1.15))
    scene.camera=oc
    bpy.context.view_layer.update()
    projected=[project(oc,p) for p in corners]
    metadata[key+'_bbox']=[min(p[0] for p in projected),min(p[1] for p in projected),
                          max(p[0] for p in projected),max(p[1] for p in projected)]
    scene.render.filepath=str(OUT/(key+'_observation.png'))
    bpy.ops.render.render(write_still=True)
(OUT/'scene_geometry.json').write_text(json.dumps(metadata,indent=2)+'\n')
print('FIG8_SCENE_READY',OUT)
