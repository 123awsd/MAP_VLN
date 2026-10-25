"""Render-only assets for the BSC-Nav-inspired layout study.

Use existing HM3D mesh and authored route. Coverage samples are evaluated by
ray casting at illustrative poses; they are not output from the paper planner.
No publication PDF or replacement manuscript figure is produced.
"""
import bpy
import json
import math
from pathlib import Path
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from bpy_extras.object_utils import world_to_camera_view

OUT=Path(__file__).resolve().parent
(OUT/'assets').mkdir(exist_ok=True)
SOURCE=OUT.parent/'multiroom_redraw'
old=json.loads((SOURCE/'assets/projection.json').read_text())
bpy.ops.wm.open_mainfile(filepath=str(SOURCE/'fig8_textured_scene.blend'))
scene=bpy.context.scene
scene.render.resolution_x=2800
scene.render.resolution_y=2800
scene.render.resolution_percentage=100
scene.cycles.samples=24
scene.render.threads_mode='FIXED';scene.render.threads=8
scene.view_settings.look='None'
scene.view_settings.exposure=-.38
for mat in bpy.data.materials:
    if mat.use_nodes:
        for n in mat.node_tree.nodes:
            if n.type=='HUE_SAT':
                n.inputs['Saturation'].default_value=.78
                n.inputs['Value'].default_value=1
camera=scene.camera
center=Vector((-4.6,2.05,.35))
camera.location=center+Vector((0,-5.5,24))
camera.rotation_euler=(center-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.ortho_scale=13.4
bpy.context.view_layer.update()

def world(p):return Vector((p[1],-p[0],p[2]))
def project(p):
    q=world_to_camera_view(scene,camera,world(p))
    return [float(q.x),float(1-q.y)]

poses=old['poses_falcon']
anchor=[-4.70,-7.55,.60]
target=[-4.03,-3.43,.75]
verts=[];faces=[]
for ob in scene.objects:
    if ob.type!='MESH':continue
    offset=len(verts)
    verts.extend([ob.matrix_world@v.co for v in ob.data.vertices])
    faces.extend([tuple(offset+i for i in p.vertices) for p in ob.data.polygons])
tree=BVHTree.FromPolygons(verts,faces)

def visible(p,q):
    aim=world(anchor)-world(q)
    ray=world(p)-world(q)
    # Fixed camera pitch: horizontal heading faces the anchor, half-FoV=43 deg.
    angle=math.atan2(ray.y,ray.x)-math.atan2(aim.y,aim.x)
    angle=(angle+math.pi)%(2*math.pi)-math.pi
    elevation=abs(math.atan2(ray.z,math.hypot(ray.x,ray.y)))
    if abs(angle)>math.radians(43) or elevation>math.radians(39):return False
    distance=ray.length
    hit,normal,idx,d=tree.ray_cast(world(q),ray.normalized(),max(0,distance-.025))
    return hit is None

# A horizontal search slice is explicit. Visibility changes follow real occlusion.
cells=[]
step=.015
xmin,xmax=-5.32,-3.62
ymin,ymax=-8.18,-6.94
nx=math.ceil((xmax-xmin)/step);ny=math.ceil((ymax-ymin)/step)
for i in range(nx):
    for j in range(ny):
        p=[xmin+(i+.5)*step,ymin+(j+.5)*step,.30]
        first=visible(p,poses['q1']);second=visible(p,poses['q2'])
        corners=[project([p[0]+dx*step/2,p[1]+dy*step/2,p[2]]) for dx,dy in [(-1,-1),(-1,1),(1,1),(1,-1)]]
        cells.append(dict(i=i,j=j,xyz=p,uv=project(p),polygon=corners,first=first,second=second))

def poly(x0,x1,y0,y1,z=.12):
    return [project([x,y,z]) for x,y in [(x0,y0),(x0,y1),(x1,y1),(x1,y0)]]

data=dict(source_scene=old['source_scene'],poses={k:project(v) for k,v in poses.items()},
    anchors=dict(armchair=project(anchor),cushion=project(target),
        bedroom=project([-3.8,-9.2,.1]),living=project([-3.2,-4.5,.1])),
    route_local=[project(p) for p in old['route_falcon']['local'][2:]],
    route_recovery=[project(p) for p in old['route_falcon']['recovery']],
    cells=cells,grid=dict(nx=nx,ny=ny,step_m=step,height_m=.30),
    local_roi=[.20,.17,.145,.185],
    relevant=[poly(-6.4,-1.6,-10.6,-6.94,1),poly(-7.5,-1.5,-6.94,-2.9,1)],
    coverage_note='Ray-tested illustrative horizontal search slice, at authored poses and fixed pitch; not planner output or measured belief.',
    source_mesh_cut='Existing display mesh clipped at 1.3 m; not a full-sensor evaluation')
(OUT/'assets/projection.json').write_text(json.dumps(data,indent=2)+'\n')
print('Coverage counts',sum(c['first'] for c in cells),sum(c['second'] for c in cells),
      sum(c['second'] and not c['first'] for c in cells),flush=True)
scene.render.filepath=str(OUT/'assets/apartment.png')
bpy.ops.render.render(write_still=True)
print('DRAFT_ASSETS_READY')
