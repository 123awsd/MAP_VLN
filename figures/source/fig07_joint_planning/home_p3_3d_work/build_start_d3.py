"""Scene-native 3-D MINCO draft. Curves sample saved project MINCO polynomials."""
from pathlib import Path
import bpy,json,math
from mathutils import Vector
H=Path(__file__).resolve().parent
D=json.loads((H/'conditional_plans_start_d3.json').read_text()); M=json.loads((H/'start_d3'/'astar_points.json').read_text()); S=.0254
bpy.ops.wm.open_mainfile(filepath=str(H/'target_selection_draft.blend'))
sc=bpy.context.scene
coll=bpy.data.collections.new('Fig7 | conditional planning and MINCO'); sc.collection.children.link(coll)
def mat(name,rgba):
 m=bpy.data.materials.new(name); m.diffuse_color=rgba; m.use_nodes=True; n=m.node_tree.nodes.get('Principled BSDF'); n.inputs['Base Color'].default_value=rgba; n.inputs['Emission Color'].default_value=rgba; n.inputs['Emission Strength'].default_value=.6; return m
faint=mat('Candidate',(.13,.53,.63,1)); sel=mat('Selected updated',(.015,.29,.43,1)); reached=mat('Reached',(.31,.32,.35,1)); new=mat('Newly active',(.95,.29,.03,1)); pathmat=mat('Updated MINCO',(.0,.42,.58,1)); execmat=mat('Executed MINCO',(.37,.39,.40,1)); labmat=mat('Annotation',(.035,.04,.045,1)); startmat=mat('Start',(.8,.12,.08,1))
def lines(name,segments,material,width):
 curve=bpy.data.curves.new(name,'CURVE'); curve.dimensions='3D'; curve.bevel_depth=width; curve.bevel_resolution=2
 for points in segments:
  sp=curve.splines.new('POLY'); sp.points.add(len(points)-1)
  for p,v in zip(sp.points,points): p.co=(*v,1)
 obj=bpy.data.objects.new(name,curve); coll.objects.link(obj); curve.materials.append(material); return obj
def text(name,body,x,y,z,size,material):
 cu=bpy.data.curves.new(name,'FONT'); cu.body=body; cu.align_x='CENTER'; cu.align_y='CENTER'; cu.size=size; cu.extrude=.02; ob=bpy.data.objects.new(name,cu); coll.objects.link(ob); ob.location=(x,y,z); cu.materials.append(material)
chosen={v['candidate_id'] for v in D['updated_plan']['visits']}; reached_id=D['initial_plan']['visits'][0]['candidate_id']
for task,candidates in D['candidates'].items():
 for c in candidates:
  label=D['candidate_review_numbers'][c['id']]; n=int(label[1:])-1; p=c['pose']; apex=Vector((p['x'],p['y'],p['z']))/S; yaw=p['yaw']; f=Vector((math.cos(yaw),math.sin(yaw),0)); r=Vector((-math.sin(yaw),math.cos(yaw),0)); up=Vector((0,0,1)); depth=12; w=depth*math.tan(math.radians(c['horizontal_fov_deg'])/2); h=depth*math.tan(math.radians(c['vertical_fov_deg'])/2)
  corners=[apex+depth*f+sx*w*r+sz*h*up for sx,sz in [(-1,-1),(1,-1),(1,1),(-1,1)]]; segs=[[apex,k] for k in corners]+[[corners[i],corners[(i+1)%4]] for i in range(4)]
  active=c['id'] in chosen; done=c['id']==reached_id
  ob=lines('View '+label,segs,reached if done else (new if active and task=='D' else sel if active else faint),.45 if (active or done) else .18); ob['review_number']=label; ob['candidate_id']=c['id']
  text('Label '+label,label,apex.x+[-8,14,-14][n],apex.y+[8,10,-10][n],96,5.5,labmat)
for title,phases,material in [('Executed: Start to A',[M['initial']['phases'][0]],execmat),('Updated plan: A onward',M['updated']['phases'],pathmat)]:
 for i,phase in enumerate(phases):
  ob=lines(title+' '+str(i+1),[[Vector(p)/S for p in phase]],material,.8); ob['source']='Project 3D A-star voxel path and validated endpoint connectors'; ob['state']='executed prefix' if material==execmat else 'planned, not executed'
start=Vector(D['start_xyz_yaw'][:3])/S; bpy.ops.mesh.primitive_uv_sphere_add(segments=16,ring_count=8,radius=2,location=start); bpy.context.object.name='Start'; bpy.context.object.data.materials.append(startmat)
text('Start label','Start',start.x-9,start.y+7,96,6,labmat)
text('Not found event','A: not found',-343,-187,96,6,labmat)
text('Fallback activated','D: active',-16,-105,96,6,new)
sc.render.filepath=str(H/'start_d3_astar_raw.png'); bpy.ops.wm.save_as_mainfile(filepath=str(H/'start_d3_astar_raw.blend'))
