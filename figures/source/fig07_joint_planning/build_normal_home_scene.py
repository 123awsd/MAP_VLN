import bpy, math
from mathutils import Vector
from pathlib import Path
ROOT=Path('/home/uav/map_VLN'); OUT=ROOT/'PRE_MAP_VLN/figures/source/fig07_joint_planning'
TABLE=ROOT/'画图/候选视角生成_新版/assets/edelweiss_extending_dining_table_ash_and_white.glb'; CHAIR=ROOT/'画图/候选视角生成_新版/assets/old_wooden_chair_low-poly.glb'
BLEND=OUT/'fig7_normal_home_scene.blend'; PNG=OUT/'fig7_normal_home_scene.png'
bpy.ops.wm.read_factory_settings(use_empty=True)
def mat(name,c,rough=.7):
 m=bpy.data.materials.new(name); m.diffuse_color=(*c,1); m.use_nodes=True; bs=m.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=(*c,1); bs.inputs['Roughness'].default_value=rough; return m
wall=mat('plaster',(0.79,.82,.83),.9); floor=mat('oak floor',(.42,.25,.12),.8); wood=mat('table wood',(.52,.29,.12),.72); fabric=mat('sofa fabric',(.55,.56,.53),.9); white=mat('door paint',(.93,.93,.90),.85); teal=mat('smooth trajectory',(.04,.40,.47),.45); grey=mat('A star guide',(.35,.39,.41),.8); orange=mat('active task',(.9,.34,.12),.55)
def cube(name,loc,sc,ma,bev=0):
 if len(loc)==2: loc=(*loc,sc[2])
 bpy.ops.mesh.primitive_cube_add(location=loc); o=bpy.context.object; o.name=name; o.scale=sc; bpy.ops.object.transform_apply(location=False,rotation=False,scale=True); o.data.materials.append(ma)
 if bev: mod=o.modifiers.new('soft','BEVEL'); mod.width=bev; mod.segments=2
 return o
cube('floor',(0,0,-.12),(9.4,6.4,.12),floor)
# outer walls
for name,loc,sc in [('north',(0,6.15,1.35),(9.4,.16,1.35)),('south',(0,-6.15,1.35),(9.4,.16,1.35)),('west',(-9.25,0,1.35),(.16,6,1.35)),('east',(9.25,0,1.35),(.16,6,1.35))]: cube(name,loc,sc,wall,.04)
# internal walls with 1m door gaps at x/y +/-5 and +/-3
for a,b in [(-6,-3.5),(-2.5,2.5),(3.5,6)]: cube('vertical wall',(0,(a+b)/2,1.35),(.12,(b-a)/2,1.35),wall,.03)
for a,b in [(-9,-5.5),(-4.5,4.5),(5.5,9)]: cube('horizontal wall',((a+b)/2,0,1.35),((b-a)/2,.12,1.35),wall,.03)
# openings only: no door panels
# compact furniture helpers
def import_asset(path,loc,scale,rot=0):
 before=set(bpy.data.objects); bpy.ops.import_scene.gltf(filepath=str(path)); new=[o for o in bpy.data.objects if o not in before]; top=[o for o in new if o.parent is None]
 root=bpy.data.objects.new(path.stem+' instance',None); bpy.context.collection.objects.link(root); root.location=loc; root.rotation_euler[2]=rot
 for o in top: o.parent=root; o.location=(0,0,0); o.rotation_euler=(0,0,0); o.scale=(scale,scale,scale)
 return root
def table(x,y):
 cube('dining table',(x,y,.78),(.85,.52,.07),wood,.05)
 for lx in (-.66,.66):
  for ly in (-.38,.38): cube('table leg',(x+lx,y+ly,.38),(.05,.05,.38),wood,.02)
def sofa(x,y,rot=0):
 o=cube('sofa',(x,y,.52),(1.25,.55,.40),fabric,.12); o.rotation_euler[2]=rot; cube('sofa back',(x-.85*math.sin(rot),y+.85*math.cos(rot),1.0),(.18,.62,.45),fabric,.08)
def shelf(x,y,w=1.4,d=.35,h=2.1):
 cabinet=mat('shelf wood',(.24,.13,.06),.8)
 cube('tall shelf',(x,y,h/2),(w/2,d/2,h/2),cabinet,.035)
 for z in (.35,.85,1.35,1.85): cube('shelf board',(x,y,z),(w/2+.03,d/2+.03,.035),wood,.015)
def cabinet(x,y,w=1.6,d=.55,h=.9):
 cabinetmat=mat('cabinet walnut',(.30,.17,.08),.8)
 cube('low cabinet',(x,y,h/2),(w/2,d/2,h/2),cabinetmat,.04)
 for xx in (x-w*.18,x+w*.18): bpy.ops.mesh.primitive_uv_sphere_add(segments=16,ring_count=8,radius=.045,location=(xx,y-d/2-.02,h*.62)); bpy.context.object.data.materials.append(white)
# one distinct anchor furniture per room
sofa(-4.8,3.2,0)
table(4.8,3.2)
cube('bed',(-4.8,-3.2,.38),(1.45,1.85,.38),fabric,.10); cube('bed head',(-4.8,-5.0,1.0),(1.5,.12,.65),wood,.04)
shelf(4.8,-3.2,1.5,.38,2.2)
# path through doors: Start -> A -> top-left B -> bottom-left C -> bottom-right D
old=[(5.8,4.7,3.1),(3.6,3.1,3.1),(0,3,3.1),(-3.6,3.1,3.1),(-5,0,3.1),(-3.6,-3.8,3.1),(0,-3,3.1),(3.6,-3.1,3.1)]
new=[(7.4,4.9,3.1),(3.6,3.1,3.1),(0,3,3.1),(-3.6,3.1,3.1),(-5,0,3.1),(-3.6,-3.8,3.1),(0,-3,3.1),(3.6,-3.1,3.1)]
def seg(a,b,ma,depth):
 cu=bpy.data.curves.new('path','CURVE'); cu.dimensions='3D'; cu.bevel_depth=depth; cu.bevel_resolution=3; sp=cu.splines.new('POLY'); sp.points.add(1); sp.points[0].co=(*a,1); sp.points[1].co=(*b,1); ob=bpy.data.objects.new('path',cu); bpy.context.collection.objects.link(ob); ob.data.materials.append(ma)
for a,b in zip(old[:-1],old[1:]):
 # dashed guide
 va=Vector(a); vb=Vector(b); L=(vb-va).length; u=(vb-va).normalized(); t=0
 while t<L: seg(tuple(va+u*t),tuple(va+u*min(t+.38,L)),grey,.035); t+=.72
# smooth curve
cu=bpy.data.curves.new('MINCO trajectory','CURVE'); cu.dimensions='3D'; cu.bevel_depth=.07; cu.bevel_resolution=5; sp=cu.splines.new('BEZIER'); sp.bezier_points.add(len(new)-1)
for bp,p in zip(sp.bezier_points,new): bp.co=p; bp.handle_left_type='AUTO'; bp.handle_right_type='AUTO'
ob=bpy.data.objects.new('MINCO trajectory',cu); bpy.context.collection.objects.link(ob); ob.data.materials.append(teal)
# candidate viewpoints around each furniture anchor; the first candidate is selected
def wire(a,b,ma,depth=.018):
 cu=bpy.data.curves.new('view wire','CURVE'); cu.dimensions='3D'; cu.bevel_depth=depth; cu.bevel_resolution=2; sp=cu.splines.new('POLY'); sp.points.add(1); sp.points[0].co=(*a,1); sp.points[1].co=(*b,1); ob=bpy.data.objects.new('view wire',cu); bpy.context.collection.objects.link(ob); ob.data.materials.append(ma)
def frustum(anchor,cam,ma):
 ax,ay,az=anchor; cx,cy,cz=cam; dx,dy=ax-cx,ay-cy; L=max((dx*dx+dy*dy)**.5,.1); px,py=-dy/L,dx/L; sx,sy=.22*px,.22*py
 base=[(cx-sx,cy-sy,cz),(cx+sx,cy+sy,cz),(cx+sx,cy+sy,cz-.12),(cx-sx,cy-sy,cz-.12)]
 for p,q in zip(base,base[1:]+base[:1]): wire(p,q,ma)
 for p in base: wire(p,(ax,ay,az),ma)
views={'A':((4.8,3.1,.8),[(3.6,3.1,3.1),(4.8,2.0,3.1),(6.0,3.1,3.1)]),'B':((-4.8,3.1,.8),[(-3.6,3.1,3.1),(-4.8,2.0,3.1),(-6.0,3.1,3.1)]),'C':((-5,-3.8,.8),[(-3.6,-3.8,3.1),(-5,-2.7,3.1),(-6.2,-3.8,3.1)]),'D':((4.8,-3.1,.8),[(3.6,-3.1,3.1),(4.8,-2.0,3.1),(6.0,-3.1,3.1)])}
for lab,(anchor,cands) in views.items():
 for idx,campos in enumerate(cands): frustum(anchor,campos,teal if idx==0 else grey)
for lab,p,ma in [('A',(4.8,3.1,.55),grey),('B',(-4.8,3.1,.55),teal),('C',(-5,-3.8,.55),teal),('D',(4.8,-3.1,.55),orange)]:
 bpy.ops.mesh.primitive_cylinder_add(vertices=40,radius=.27,depth=.07,location=p); bpy.context.object.name='task '+lab; bpy.context.object.data.materials.append(ma)
bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=12,radius=.16,location=(7.4,4.9,3.1)); bpy.context.object.data.materials.append(white)
# camera and lighting
bpy.ops.object.camera_add(location=(9,-10,24)); cam=bpy.context.object; cam.data.type='ORTHO'; cam.data.ortho_scale=18.4
def look(o,t): o.rotation_euler=(Vector(t)-o.location).to_track_quat('-Z','Y').to_euler()
look(cam,(0,0,0)); bpy.context.scene.camera=cam
bpy.ops.object.light_add(type='AREA',location=(0,0,14)); key=bpy.context.object; key.data.energy=1800; key.data.size=12; look(key,(0,0,0))
bpy.ops.object.light_add(type='AREA',location=(-8,-8,8)); fill=bpy.context.object; fill.data.energy=700; fill.data.size=8; look(fill,(0,0,0))
sc=bpy.context.scene; sc.render.engine='BLENDER_EEVEE'; sc.render.resolution_x=1200; sc.render.resolution_y=900; sc.render.resolution_percentage=100; sc.render.image_settings.file_format='PNG'; sc.render.filepath=str(PNG); sc.world=bpy.data.worlds.new('World'); sc.world.color=(.92,.94,.95); sc.view_settings.look='AgX - Medium High Contrast'
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND)); bpy.ops.render.render(write_still=True)
print(BLEND,PNG)
