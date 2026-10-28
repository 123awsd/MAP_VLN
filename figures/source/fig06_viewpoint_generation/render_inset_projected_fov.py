import bpy,json,math
from pathlib import Path
from mathutils import Vector
out=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(out/'algorithm_scene.blend'))
g=json.loads((out/'occlusion_inset_geometry.json').read_text())
s=bpy.context.scene;camera=s.camera;p=Vector(g['position']);target=Vector((0,0,1.01));forward=(target-p).normalized();side=forward.cross(Vector((0,0,1))).normalized();up=Vector((0,0,1))
blockers={r['blocker'] for r in g['samples'] if r['blocker']}
chair_prefixes={name.split(' Chair')[0] for name in blockers}
for o in bpy.data.objects:
 if o.name.startswith('Seat ') and not any(o.name.startswith(prefix+' ') for prefix in chair_prefixes):o.hide_render=True

def mat(name,color,alpha=1):
 m=bpy.data.materials.new(name);m.use_nodes=True;bs=m.node_tree.nodes['Principled BSDF'];bs.inputs['Base Color'].default_value=(*color,1);bs.inputs['Roughness'].default_value=.75;bs.inputs['Alpha'].default_value=alpha;return m
teal=mat('FoV edge',(.005,.24,.32));fill=mat('FoV volume',(.05,.50,.62),.055);amber=mat('Occluded surface',(.93,.51,.045),.8)
def line(name,pts,m,width=.009):
 c=bpy.data.curves.new(name,'CURVE');c.dimensions='3D';c.bevel_depth=width;c.bevel_resolution=3;sp=c.splines.new('POLY');sp.points.add(len(pts)-1)
 for v,co in zip(sp.points,pts):v.co=(*co,1)
 o=bpy.data.objects.new(name,c);bpy.context.collection.objects.link(o);o.data.materials.append(m)
# Compact frustum near the camera, dashed continuations into the scene.
end=p+forward*.36
corners=[end+side*x*(.36*math.tan(math.radians(69/2)))+up*z*(.36*math.tan(math.radians(42/2))) for x,z in [(-1,-1),(1,-1),(1,1),(-1,1)]]
for c in corners:line('Compact FoV boundary',[p,c],teal,.006)
line('Compact FoV frame',corners+[corners[0]],teal,.006)
# Four dashed continuations are collinear with the compact frustum edges.
# Use the same pinhole rays for the near and extended corners.
bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=12,radius=.035,location=p);bpy.context.object.data.materials.append(teal)
# A thin horizontal section makes the 3D visibility shadow legible on white.
bpy.context.view_layer.update();deps=bpy.context.evaluated_depsgraph_get();obstacles=[o for o in s.objects if o.type=='MESH' and o.name.startswith('Seat ') and not o.hide_render]
length=2.35;half=math.tan(math.radians(69/2));ends=[]
for i in range(241):
 direction=forward+side*(-half+2*half*i/240);depth=length
 for obj in obstacles:
  inv=obj.matrix_world.inverted();a=inv@p;b=inv@(p+direction*length);delta=b-a;hit,loc,*_=obj.evaluated_get(deps).ray_cast(a,delta.normalized(),distance=delta.length)
  if hit:depth=min(depth,(obj.matrix_world@loc-p).dot(forward))
 ends.append(p+direction*depth)
beam_corners=ends
m=bpy.data.materials.new('Illustrated visible field');m.use_nodes=True;n=m.node_tree.nodes;n.clear();o=n.new('ShaderNodeOutputMaterial');tr=n.new('ShaderNodeBsdfTransparent');em=n.new('ShaderNodeEmission');em.inputs['Color'].default_value=(.008,.24,.36,1);em.inputs['Strength'].default_value=.8;mix=n.new('ShaderNodeMixShader');mix.inputs[0].default_value=.60;m.node_tree.links.new(tr.outputs[0],mix.inputs[1]);m.node_tree.links.new(em.outputs[0],mix.inputs[2]);m.node_tree.links.new(mix.outputs[0],o.inputs['Surface'])
# A flat viewing field inside the original 3D furniture scene.
verts=[tuple(p+Vector((0,0,.025)))]+[tuple(q+Vector((0,0,.025))) for q in ends]
faces=[(0,i+1,i+2) for i in range(len(ends)-1)]
mesh=bpy.data.meshes.new('Flat visibility fan');mesh.from_pydata(verts,[],faces);mesh.update()
obj=bpy.data.objects.new('2D FoV in 3D scene',mesh);bpy.context.collection.objects.link(obj);obj.data.materials.append(m)
for q in (ends[0],ends[-1]):line('Field outer edge',[p+Vector((0,0,.028)),q+Vector((0,0,.028))],teal,.004)
beam_corners=ends
# Oblique overhead view, looking across the chair toward the table.
center=Vector((0,0,.9))*.6+p*.4
camera.location=center+Vector((4,-6,12.5))
camera.rotation_euler=(center-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.ortho_scale=3.7
s.render.resolution_x=1200;s.render.resolution_y=1100;s.render.resolution_percentage=100;s.cycles.samples=96;s.cycles.use_denoising=False;s.render.film_transparent=True
s.render.filepath=str(out/'occlusion_inset_3d.png')
bpy.context.view_layer.update()
# Fit complete table, blocking chairs and compact camera glyph with a margin.
points=[Vector(p)]+corners+beam_corners
for obj in s.objects:
 if obj.type=='MESH' and not obj.hide_render and (obj.name.startswith('Table ') or obj.name.startswith('Seat ')):
  points.extend(obj.matrix_world@Vector(v) for v in obj.bound_box)
basis=camera.rotation_euler.to_matrix();inv=basis.transposed();qs=[inv@v for v in points]
lo=Vector([min(v[i] for v in qs) for i in range(3)]);hi=Vector([max(v[i] for v in qs) for i in range(3)])
center=basis@((lo+hi)/2);camera.location=center+basis@Vector((0,0,12));camera.data.ortho_scale=max(hi.x-lo.x,(hi.y-lo.y)*1200/1100)*1.16
bpy.context.view_layer.update()
# Export an actual blocked ray in render coordinates for x-ray annotation.
from bpy_extras.object_utils import world_to_camera_view
blocked_rays=[r for r in g['samples'] if r.get('hit')]
if blocked_rays:
 ray=blocked_rays[len(blocked_rays)//2]
 def screen(v):
  q=world_to_camera_view(s,camera,Vector(v));return [q.x*1200,(1-q.y)*1100]
 visible=[v for v in g['samples'] if v['visible']]
 debug={'source':screen(p),'hit':screen(ray['hit']),'target':screen(ray['target']),'visible_target':screen(visible[len(visible)//2]['target']) if visible else None}
 (out/'inset_xray.json').write_text(json.dumps(debug))
bpy.ops.wm.save_as_mainfile(filepath=str(out/'occlusion_projected_fov.blend'))
bpy.ops.render.render(write_still=True)
