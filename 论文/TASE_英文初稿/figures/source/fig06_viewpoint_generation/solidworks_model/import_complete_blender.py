import bpy,json,math
from pathlib import Path
from mathutils import Vector,Matrix
out=Path(__file__).resolve().parent
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
bpy.ops.wm.stl_import(filepath=str(out/'assembly_complete.stl'))
o=bpy.context.object;o.name='User STEP aircraft - complete assembly'
o.data=o.data.copy();o.scale=(.001,)*3;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
# Align the dominant planar rotor-disk normal to world Z; the CAD assembly
# is saved at an arbitrary tilted orientation, not aligned with its axes.
lo=Vector([min(v.co[k] for v in o.data.vertices) for k in range(3)]);hi=Vector([max(v.co[k] for v in o.data.vertices) for k in range(3)]);center=(lo+hi)/2;dims=hi-lo
bins={};normals={}
o.data.calc_loop_triangles()
for tri in o.data.loop_triangles:
 if tri.area<.00002:continue
 n=tri.normal.copy()
 if n[max(range(3),key=lambda k:abs(n[k]))]<0:n=-n
 key=tuple(round(v,1) for v in n);bins[key]=bins.get(key,0)+tri.area;normals[key]=normals.get(key,Vector())+n*tri.area
key=max(bins,key=bins.get);normal=normals[key].normalized();rot=normal.rotation_difference(Vector((0,0,1))).to_matrix();up='dominant rotor plane'
for v in o.data.vertices:v.co=rot@(v.co-center)
m=bpy.data.materials.new('Aircraft graphite');m.use_nodes=True;p=m.node_tree.nodes['Principled BSDF'];p.inputs['Base Color'].default_value=(.14,.17,.20,1);p.inputs['Metallic'].default_value=.2;p.inputs['Roughness'].default_value=.55;o.data.materials.append(m)
for f in o.data.polygons:f.use_smooth=True
o.data.use_auto_smooth=True
bpy.ops.wm.save_as_mainfile(filepath=str(out/'assembly_complete.blend'))
bpy.ops.export_scene.gltf(filepath=str(out/'assembly_complete.glb'),export_format='GLB')
(out/'blender_import_report.json').write_text(json.dumps({'native_dimensions_m':list(dims),'normalized_up_axis':up,'triangles':len(o.data.polygons),'vertices':len(o.data.vertices)},indent=2))
