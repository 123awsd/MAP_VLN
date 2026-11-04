import bpy, math
from mathutils import Vector
from pathlib import Path

ROOT=Path('/home/uav/map_VLN')
OUT=ROOT/'PRE_MAP_VLN/figures/source/fig07_joint_planning'
TABLE=ROOT/'画图/候选视角生成_新版/assets/edelweiss_extending_dining_table_ash_and_white.glb'
CHAIR=ROOT/'画图/候选视角生成_新版/assets/old_wooden_chair_low-poly.glb'
BLEND=OUT/'fig7_polyhaven_room_scene.blend'
PNG=OUT/'fig7_polyhaven_room_scene.png'

bpy.ops.wm.read_factory_settings(use_empty=True)
# materials
def mat(name,color,rough=0.7,metal=0):
    m=bpy.data.materials.new(name); m.diffuse_color=(*color,1); m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=(*color,1); bs.inputs['Roughness'].default_value=rough; bs.inputs['Metallic'].default_value=metal
    return m
wallmat=mat('warm white walls',(0.78,0.81,0.82),.9)
floormat=mat('light wood floor',(0.52,0.34,0.20),.75)
teal=mat('updated trajectory',(0.04,0.40,0.47),.45)
grey=mat('discrete guide',(0.35,0.39,0.41),.8)
orange=mat('newly active task',(0.90,0.34,0.12),.55)
white=mat('marker white',(0.98,0.98,0.96),.6)

def cube(name,loc,scale,material,bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(location=loc); o=bpy.context.object; o.name=name; o.scale=scale; bpy.ops.object.transform_apply(location=False,rotation=False,scale=True); o.data.materials.append(material)
    if bevel:
        mod=o.modifiers.new('soft edges','BEVEL'); mod.width=bevel; mod.segments=2
    return o
# floor
cube('floor',(0,0,-.12),(9.4,6.4,.12),floormat)
# perimeter walls and room dividers with openings
for name,loc,sc in [
 ('wall_north',(0,6.15,1.35),(9.4,.16,1.35)),('wall_south',(0,-6.15,1.35),(9.4,.16,1.35)),
 ('wall_west',(-9.25,0,1.35),(.16,6.0,1.35)),('wall_east',(9.25,0,1.35),(.16,6.0,1.35)),
 ('divider_v_top',(0,3.6,1.35),(.12,2.55,1.35)),('divider_v_bottom',(0,-3.6,1.35),(.12,2.55,1.35)),
 ('divider_h_left',(-5.7,0,1.35),(3.55,.12,1.35)),('divider_h_mid',(-.7,0,1.35),(.5,.12,1.35)),
 ('divider_h_right',(7.25,0,1.35),(1.75,.12,1.35)),
]: cube(name,loc,sc,wallmat,.04)
# import local Poly Haven assets; table source needs cm->m correction
def import_asset(path, loc, scale, rot=0):
    before=set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(path))
    new=[o for o in bpy.data.objects if o not in before]
    top=[o for o in new if o.parent is None]
    root=bpy.data.objects.new(path.stem+' instance',None); bpy.context.collection.objects.link(root); root.location=loc; root.rotation_euler[2]=rot
    for o in top:
        o.parent=root
        o.location=(0,0,0)
        o.rotation_euler=(0,0,0)
        o.scale=(scale,scale,scale)
    return root
# tables and chairs in each room
tablemat=mat('simple oak tabletop',(0.48,0.25,0.10),.75)
for x,y in [(-4.8,3.2),(4.8,3.2),(-4.8,-3.2),(4.8,-3.2)]:
    # a compact custom table keeps the scene readable at half-column scale
    cube('table top',(x,y,.78),(.75,.48,.07),tablemat,.05)
    for lx in (-.58,.58):
        for ly in (-.32,.32): cube('table leg',(x+lx,y+ly,.38),(.055,.055,.38),tablemat,.02)
    for dx,dy,r in [(-1.0,0,math.pi/2),(1.0,0,-math.pi/2),(0,-1.0,0)]: import_asset(CHAIR,(x+dx,y+dy,0),1.0,r)
# trajectory curves
def curve_poly(name,pts,material,bevel,dashed=False):
    for i in range(len(pts)-1):
        if dashed:
            a=Vector(pts[i]); b=Vector(pts[i+1]); L=(b-a).length; u=(b-a).normalized(); t=0
            while t<L:
                t2=min(t+.42,L); p0=a+u*t; p1=a+u*t2
                cu=bpy.data.curves.new(name,'CURVE'); cu.dimensions='3D'; cu.bevel_depth=bevel; cu.bevel_resolution=2
                sp=cu.splines.new('POLY'); sp.points.add(1); sp.points[0].co=(*p0,1); sp.points[1].co=(*p1,1)
                ob=bpy.data.objects.new(name,cu); bpy.context.collection.objects.link(ob); ob.data.materials.append(material); t+=.75
        else:
            cu=bpy.data.curves.new(name,'CURVE'); cu.dimensions='3D'; cu.bevel_depth=bevel; cu.bevel_resolution=3
            sp=cu.splines.new('POLY'); sp.points.add(1); sp.points[0].co=(*pts[i],1); sp.points[1].co=(*pts[i+1],1)
            ob=bpy.data.objects.new(name,cu); bpy.context.collection.objects.link(ob); ob.data.materials.append(material)
# points at floor height
old=[(4.8,3.2,.25),(4.8,-.8,.25),(0.8,-.8,.25),(-4.8,-.8,.25),(-4.8,3.2,.25)]
new=[(7.3,4.8,.28),(4.8,3.2,.28),(4.8,-3.2,.28),(1.0,-.8,.28),(-1.0,-.8,.28),(-4.8,-3.2,.28),(-4.8,3.2,.28)]
curve_poly('A-star guide',old,grey,.035,True)
# smooth bezier through new points
cu=bpy.data.curves.new('MINCO smooth trajectory','CURVE'); cu.dimensions='3D'; cu.bevel_depth=.075; cu.bevel_resolution=5
sp=cu.splines.new('BEZIER'); sp.bezier_points.add(len(new)-1)
for bp,p in zip(sp.bezier_points,new): bp.co=p; bp.handle_left_type='AUTO'; bp.handle_right_type='AUTO'
ob=bpy.data.objects.new('MINCO smooth trajectory',cu); bpy.context.collection.objects.link(ob); ob.data.materials.append(teal)
# markers
for label,p,material in [('A',(4.8,3.2,.35),grey),('B',(-4.8,3.2,.35),teal),('C',(-4.8,-3.2,.35),teal),('D',(4.8,-3.2,.35),orange)]:
    bpy.ops.mesh.primitive_cylinder_add(vertices=48,radius=.32,depth=.06,location=p); o=bpy.context.object; o.name='task '+label; o.data.materials.append(material)
# start marker
bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=.18, location=(7.3,4.8,.35)); bpy.context.object.data.materials.append(white)
# camera
bpy.ops.object.camera_add(location=(12,-15,18))
cam=bpy.context.object; cam.data.type='ORTHO'; cam.data.ortho_scale=18.3
def look(obj,target): obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()
look(cam,(0,0,0)); bpy.context.scene.camera=cam
# lighting
bpy.ops.object.light_add(type='AREA',location=(0,0,14)); key=bpy.context.object; key.data.energy=1800; key.data.size=12; look(key,(0,0,0))
bpy.ops.object.light_add(type='AREA',location=(-8,-8,8)); fill=bpy.context.object; fill.data.energy=700; fill.data.size=8; look(fill,(0,0,0))
scene=bpy.context.scene; scene.render.engine='BLENDER_EEVEE'; scene.render.resolution_x=1200; scene.render.resolution_y=900; scene.render.resolution_percentage=100; scene.render.image_settings.file_format='PNG'; scene.render.filepath=str(PNG)
scene.world=bpy.data.worlds.new('World'); scene.world.color=(0.92,0.94,0.95); scene.view_settings.look='AgX - Medium High Contrast'
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND)); bpy.ops.render.render(write_still=True)
print('saved',BLEND,PNG)
