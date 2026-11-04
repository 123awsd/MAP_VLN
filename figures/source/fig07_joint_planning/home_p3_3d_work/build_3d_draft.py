"""Visualize project-computed candidate poses and A* grid paths in the GLB."""
import bpy,json,math
from pathlib import Path
from mathutils import Vector
H=Path(__file__).resolve().parent
D=json.loads((H/'candidate_results_draft.json').read_text())
if 'plan' not in D: raise RuntimeError('No computed mission plan to visualize')
bpy.ops.wm.open_mainfile(filepath=str(H/'target_selection_draft.blend'))
sc=bpy.context.scene; S=.0254
coll=bpy.data.collections.new('Fig7 | computed 3D planning draft'); sc.collection.children.link(coll)
def material(name,rgba):
 m=bpy.data.materials.new(name); m.diffuse_color=rgba; m.use_nodes=True
 n=m.node_tree.nodes.get('Principled BSDF'); n.inputs['Base Color'].default_value=rgba; n.inputs['Emission Color'].default_value=rgba; n.inputs['Emission Strength'].default_value=.7
 return m
faint=material('Candidate - light blue',(0.15,.57,.68,1)); selected=material('Selected - dark blue',(.01,.30,.45,1)); route=material('A-star path - blue',(.01,.48,.66,1)); originmat=material('Start',(.83,.12,.08,1))
def lines(name,segments,mat,radius):
 c=bpy.data.curves.new(name,'CURVE'); c.dimensions='3D'; c.bevel_depth=radius; c.bevel_resolution=2
 for segment in segments:
  sp=c.splines.new('POLY'); sp.points.add(len(segment)-1)
  for p,xyz in zip(sp.points,segment): p.co=(*xyz,1)
 obj=bpy.data.objects.new(name,c); coll.objects.link(obj); obj.data.materials.append(mat); return obj
def glb(v): return Vector(v)/S
chosen={v['candidate_id'] for v in D['plan']['visits']}
labelmat=material('View label - near black',(.02,.025,.03,1))
labels={}

for task,candidates in D['candidates'].items():
 for n,c in enumerate(candidates):
  p=c['pose']; apex=glb((p['x'],p['y'],p['z'])); yaw=p['yaw']; forward=Vector((math.cos(yaw),math.sin(yaw),0)); right=Vector((-math.sin(yaw),math.cos(yaw),0)); up=Vector((0,0,1))
  depth=12; half_w=depth*math.tan(math.radians(c['horizontal_fov_deg'])/2); half_h=depth*math.tan(math.radians(c['vertical_fov_deg'])/2)
  corners=[apex+depth*forward+sx*half_w*right+sz*half_h*up for sx,sz in [(-1,-1),(1,-1),(1,1),(-1,1)]]
  segments=[[apex,k] for k in corners]+[[corners[i],corners[(i+1)%4]] for i in range(4)]
  isselected=c['id'] in chosen; obj=lines(task+'_view_'+str(n+1)+('_selected' if isselected else '_candidate'),segments,selected if isselected else faint,.42 if isselected else .19)
  obj['candidate_id']=c['id']; obj['camera_apex']=str(tuple(apex)); obj['vertical_fov_deg']=c['vertical_fov_deg']; obj['horizontal_fov_deg']=c['horizontal_fov_deg']
  label=f'{task}{n+1}'; obj['review_number']=label
  labels[label]={'candidate_id':c['id'],'selected':isselected,'pose':c['pose']}
  # Label stands above the roofline for readable review; the frustum apex remains at the true pose.
  text=bpy.data.curves.new('Label_'+label,'FONT'); text.body=label; text.align_x='CENTER'; text.align_y='CENTER'; text.size=6; text.extrude=.04
  t=bpy.data.objects.new('Label_'+label,text); coll.objects.link(t); t.location=(apex.x+[-8,14,-14][n],apex.y+[8,10,-10][n],96); text.materials.append(labelmat)

for i,seg in enumerate(D['plan']['segments']):
 visit=D['plan']['visits'][i]; pose=visit['pose']; goal=[pose['x'],pose['y'],pose['z']]
 points=[seg['from_xyz_m']]+seg['points_xyz_m']+[goal]
 obj=lines('Astar_segment_'+str(i+1)+'_to_'+seg['to_task_id'],[[glb(v) for v in points]],route,.75)
 obj['path_type']='Project 3D A* voxel centers; endpoint connectors verified against draft voxel map, not MINCO'
start=glb(D['start_xyz_yaw'][:3]); bpy.ops.mesh.primitive_uv_sphere_add(segments=16,ring_count=8,radius=2,location=start); bpy.context.object.name='Start pose'; bpy.context.object.data.materials.append(originmat)
(H/'view_numbers.json').write_text(json.dumps(labels,indent=2),encoding='utf8')
# Top view is just a preview; curves and camera frusta remain actual 3D objects.
sc.render.filepath=str(H/'planning_3d_numbered.png'); bpy.ops.wm.save_as_mainfile(filepath=str(H/'planning_3d_numbered.blend')); bpy.ops.render.render(write_still=True)
