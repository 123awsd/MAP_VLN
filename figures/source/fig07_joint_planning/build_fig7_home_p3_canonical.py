import bpy, math
from mathutils import Vector
from pathlib import Path
ROOT=Path('/home/uav/map_VLN'); SRC=Path('/home/uav/下载/home_-_p3.glb'); OUT=ROOT/'PRE_MAP_VLN/figures/source/fig07_joint_planning'; BLEND=OUT/'fig7_home_p3_canonical.blend'; PNG=OUT/'fig7_home_p3_canonical.png'
import json
DATA=json.load(open(OUT/'example_inputs.json'))
def mp(x,y,z): return (-390 + (x-.2)/11.6*380, -410 + (y-.2)/8.6*330, z)
ROOM_ANCHORS={r['task']: mp(*r['anchor'],14) for r in DATA['rooms']}
VIEW_MAP={v['id']: mp(v['xy'][0],v['xy'][1],92) for v in DATA['views']}
START=mp(DATA['start'][0],DATA['start'][1],92)
bpy.ops.wm.read_factory_settings(use_empty=True); bpy.ops.import_scene.gltf(filepath=str(SRC))
def mat(name,c,rough=.6):
 m=bpy.data.materials.new(name); m.diffuse_color=(*c,1); m.use_nodes=True; bs=m.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=(*c,1); bs.inputs['Roughness'].default_value=rough; return m
teal=mat('selected trajectory',(0.02,.40,.48),.45); grey=mat('candidate guide',(.32,.37,.40),.75); orange=mat('activated task',(.9,.32,.10),.5); white=mat('drone marker',(1,.98,.9),.5)
def wire(name,a,b,ma,depth=1.1):
 cu=bpy.data.curves.new(name,'CURVE'); cu.dimensions='3D'; cu.bevel_depth=depth; cu.bevel_resolution=2; sp=cu.splines.new('POLY'); sp.points.add(1); sp.points[0].co=(*a,1); sp.points[1].co=(*b,1); o=bpy.data.objects.new(name,cu); bpy.context.collection.objects.link(o); o.data.materials.append(ma)
def dashed(a,b,ma,depth=1.0):
 a=Vector(a); b=Vector(b); L=(b-a).length; u=(b-a).normalized(); t=0
 while t<L: wire('A* guide',tuple(a+u*t),tuple(a+u*min(t+10,L)),ma,depth); t+=18
def frustum(anchor,cam,ma):
 ax,ay,az=anchor; cx,cy,cz=cam; dx,dy=ax-cx,ay-cy; L=max((dx*dx+dy*dy)**.5,1); px,py=-dy/L,dx/L; sx,sy=10*px,10*py
 base=[(cx-sx,cy-sy,cz),(cx+sx,cy+sy,cz),(cx+sx,cy+sy,cz-7),(cx-sx,cy-sy,cz-7)]
 for p,q in zip(base,base[1:]+base[:1]): wire('candidate view',p,q,ma,.75)
 for p in base: wire('candidate view',p,(ax,ay,az),ma,.75)
# canonical Fig. 7 anchors and admitted candidates, mapped into the unchanged GLB scene
anchors=ROOM_ANCHORS
candidates={task:[VIEW_MAP[f'{task}_0'],VIEW_MAP[f'{task}_1'],VIEW_MAP[f'{task}_2']] for task in ['A','B','C','D']}
for lab,cs in candidates.items():
 for i,c in enumerate(cs): frustum(anchors[lab],c,teal if i==0 else grey)
# prior order (dashed) and updated smooth route
old=[START,VIEW_MAP['A_0'],VIEW_MAP['C_0'],VIEW_MAP['B_0']]
new=[START,VIEW_MAP['A_0'],VIEW_MAP['D_0'],VIEW_MAP['C_0'],VIEW_MAP['B_0']]
for a,b in zip(old[:-1],old[1:]): dashed(a,b,grey,1.0)
cu=bpy.data.curves.new('updated smooth UAV trajectory','CURVE'); cu.dimensions='3D'; cu.bevel_depth=2.6; cu.bevel_resolution=6; sp=cu.splines.new('BEZIER'); sp.bezier_points.add(len(new)-1)
for bp,p in zip(sp.bezier_points,new): bp.co=p; bp.handle_left_type='AUTO'; bp.handle_right_type='AUTO'
o=bpy.data.objects.new('updated smooth UAV trajectory',cu); bpy.context.collection.objects.link(o); o.data.materials.append(teal)
# anchor markers at furniture locations
for lab,p in anchors.items():
 bpy.ops.mesh.primitive_uv_sphere_add(segments=20,ring_count=10,radius=4.0,location=p); bpy.context.object.name='anchor '+lab; bpy.context.object.data.materials.append(orange if lab=='D' else teal)
bpy.ops.mesh.primitive_uv_sphere_add(segments=20,ring_count=10,radius=5.0,location=START); bpy.context.object.data.materials.append(white)
# camera fits the main house cluster, not distant export outliers
bpy.ops.object.camera_add(location=(-190,-240,700)); cam=bpy.context.object; cam.data.type='ORTHO'; cam.data.ortho_scale=420; cam.rotation_euler=(Vector((-190,-240,0))-cam.location).to_track_quat('-Z','Y').to_euler(); bpy.context.scene.camera=cam
# lighting and color management
bpy.ops.object.light_add(type='AREA',location=(-190,-240,500)); key=bpy.context.object; key.data.energy=18000; key.data.size=360; key.rotation_euler=(Vector((-190,-240,0))-key.location).to_track_quat('-Z','Y').to_euler()
sc=bpy.context.scene; sc.render.engine='BLENDER_EEVEE'; sc.render.resolution_x=1200; sc.render.resolution_y=900; sc.render.resolution_percentage=100; sc.render.image_settings.file_format='PNG'; sc.render.filepath=str(PNG); sc.world=bpy.data.worlds.new('World'); sc.world.color=(.55,.58,.62); sc.view_settings.look='AgX - Medium High Contrast'; sc.view_settings.exposure=-1.0
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND)); bpy.ops.render.render(write_still=True); print(BLEND,PNG)
