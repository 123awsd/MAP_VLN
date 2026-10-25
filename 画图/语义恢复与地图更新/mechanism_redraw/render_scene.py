"""Reframe the approved full HM3D apartment for a spatial mechanism figure.

No mesh replacement or synthetic furniture. All overlays are exported in the
same orthographic camera. Coverage is an explicit explanatory construction.
Run with blender -b -t 8 --python this_file.py.
"""
import bpy
import json
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

OUT = Path(__file__).resolve().parent
SOURCE = OUT.parent / 'multiroom_redraw'
old = json.loads((SOURCE/'assets/projection.json').read_text())
bpy.ops.wm.open_mainfile(filepath=str(SOURCE/'fig8_textured_scene.blend'))
scene = bpy.context.scene
scene.render.resolution_x = 3200
scene.render.resolution_y = 2800
scene.render.resolution_percentage = 100
scene.cycles.samples = 32
scene.render.threads_mode = 'FIXED'
scene.render.threads = 8
scene.view_settings.look = 'None'
scene.view_settings.exposure = -0.12
for mat in bpy.data.materials:
    if not mat.use_nodes:
        continue
    for n in mat.node_tree.nodes:
        if n.type == 'HUE_SAT':
            n.inputs['Saturation'].default_value = .55
            n.inputs['Value'].default_value = 1.0
camera = scene.camera
center = Vector((-4.6,2.0,.25))
camera.location = center + Vector((0,-4,23))
camera.rotation_euler = (center-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.ortho_scale = 13.6
bpy.context.view_layer.update()

def project(p):
    q = world_to_camera_view(scene,camera,Vector((p[1],-p[0],p[2])))
    return [float(q.x),float(1-q.y)]

def quad(x0,x1,y0,y1,z=.12):
    return [project(p) for p in [(x0,y0,z),(x0,y1,z),(x1,y1,z),(x1,y0,z)]]

def roi(x0,x1,y0,y1):
    pts = quad(x0,x1,y0,y1,z=.4)
    u,v = zip(*pts)
    return [min(u),min(v),max(u)-min(u),max(v)-min(v)]

poses = old['poses_falcon']
anchors = {'armchair':[-4.70,-7.55,.50], 'cushion':[-4.03,-3.43,.75],
           'sofa':[-4.62,-3.43,.58], 'bedroom':[-4,-9,.1],
           'living_room':[-3.2,-4.8,.1]}
data = dict(
    source_scene=old['source_scene'],
    poses={k:project(v) for k,v in poses.items()},
    floor_poses={k:project([v[0],v[1],.12]) for k,v in poses.items()},
    anchors={k:project(v) for k,v in anchors.items()},
    route_recovery=[project(v) for v in old['route_falcon']['recovery']],
    route_local=[project(v) for v in old['route_falcon']['local'][2:]],
    # Rectangles express conceptual coverage of the same local search region.
    # They are not measurements of FoV, visibility, or algorithm confidence.
    covered=quad(-5.32,-4.05,-8.08,-7.57),
    newly_covered=quad(-5.32,-4.05,-7.57,-7.03),
    search_region=quad(-5.32,-4.05,-8.08,-7.03),
    active_bedroom=quad(-6.3,-1.8,-10.45,-6.95,1.0),
    active_living=quad(-7.3,-1.6,-6.94,-2.9,1.0),
    local_roi=roi(-5.95,-2.95,-8.72,-6.35),
    focus_roi=roi(-7.15,-1.20,-10.35,-2.75),
    source_pixel_size=[3200,2800],
    coverage_kind='Schematic new-versus-previous coverage; no numerical measurements',
    route_kind=old['route_design'],
)
(OUT/'assets/projection.json').write_text(json.dumps(data,indent=2)+'\n')
scene.render.filepath = str(OUT/'assets/apartment.png')
bpy.ops.render.render(write_still=True)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'fig8_scene.blend'),compress=True)
print('MECHANISM_SCENE_READY')
