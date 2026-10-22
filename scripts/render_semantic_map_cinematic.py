"""Blender presentation scene from measured room masks and Boxer OBBs only."""
import bpy, numpy as np,json,math,sys
from pathlib import Path
from mathutils import Vector
R=Path(__file__).resolve().parents[1];O=R/'outputs/videos/semantic_map_cinematic_14s';O.mkdir(parents=True,exist_ok=True);(O/'frames').mkdir(exist_ok=True)
G=R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz';D=R/'outputs/coverage700_wall_room_preview/manual_room305_boundary';B=R/'outputs/boxer/coverage700_sam31_20261018/fusion_fixed/boxer_summary.json'
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
s=bpy.context.scene;s.render.engine='BLENDER_EEVEE';s.eevee.use_gtao=True;s.eevee.gtao_distance=3;s.eevee.gtao_factor=1.15;s.eevee.use_soft_shadows=True;s.eevee.taa_render_samples=16
s.render.resolution_x=1280;s.render.resolution_y=720;s.render.resolution_percentage=100;s.render.fps=24;s.frame_start=1;s.frame_end=336;s.render.image_settings.file_format='PNG';s.render.film_transparent=False
s.world.color=(.65,.65,.65);s.view_settings.view_transform='AgX'
s.render.image_settings.color_mode='RGB'
def mat(name,c,metal=0,alpha=1):
 m=bpy.data.materials.new(name);m.diffuse_color=(*c,alpha);m.use_nodes=True;p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(*c,1);p.inputs['Roughness'].default_value=.62;p.inputs['Metallic'].default_value=metal;p.inputs['Alpha'].default_value=alpha
 if alpha<1:m.blend_method='BLEND';m.use_screen_refraction=False;m.show_transparent_back=False
 return m
roomm=[mat('Room '+str(i),c)for i,c in enumerate([(.48,.64,.61),(.57,.67,.74),(.73,.62,.47),(.59,.57,.68),(.65,.70,.56),(.73,.57,.52)])]
wall=mat('Warm ceramic walls',(.81,.80,.75));edge=mat('Teal anodized object edges',(.025,.25,.24),.25);ground=mat('Studio ivory',(.83,.82,.78));pointmat=mat('Measured graphite points',(.17,.28,.30),0,.8)
allstructure=[];allboxes=[];cloudobs=[]
def mesh(name,v,faces,mats,idx=None):
 me=bpy.data.meshes.new(name);me.from_pydata(v,[],faces);me.update();ob=bpy.data.objects.new(name,me);bpy.context.collection.objects.link(ob)
 for m in mats:me.materials.append(m)
 if idx is not None:
  for p,k in zip(me.polygons,idx):p.material_index=k
 return ob
signs=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])
faces0=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
def addblock(v,faces,ids,x,y,dx,dy,z0,z1,mi):
 start=len(v);v.extend((signs*np.array([dx,dy,z1-z0])/2+np.array([x+dx/2,y+dy/2,(z0+z1)/2])).tolist());faces.extend([tuple(start+k for k in face)for face in faces0]);ids.extend([mi]*6)
a=np.load(G);objects=json.loads(B.read_text())['objects'];offsets=[-25,0,25]
for f in range(3):
 b=np.load(D/f'L{f+1}_constrained.npz');l=b['labels'];res=float(b['resolution']);org=b['origin_xy'];off=offsets[f];floorz=[0,2.75,5.75][f]
 v=[];faces=[];ids=[]
 for y in range(l.shape[0]):
  row=l[y];diff=np.r_[True,row[1:]!=row[:-1],True];cuts=np.where(diff)[0]
  for x,e in zip(cuts[:-1],cuts[1:]):
   k=int(row[x])
   if k:addblock(v,faces,ids,org[0]+x*res+off-1,org[1]+y*res-8,(e-x)*res,res,-.12,0,(k-1)%6)
 ob=mesh(f'Floor {f+1} measured room slabs',v,faces,roomm,ids);allstructure.append(ob)
 v=[];faces=[];ids=[];wm=b['measured_wall']
 for y in range(wm.shape[0]):
  dv=np.diff(np.r_[False,wm[y],False].astype(int))
  for x,e in zip(np.where(dv==1)[0],np.where(dv==-1)[0]):addblock(v,faces,ids,org[0]+x*res+off-1,org[1]+y*res-8,(e-x)*res,res,0,.55,0)
 ob=mesh(f'Floor {f+1} cutaway measured walls',v,faces,[wall]);allstructure.append(ob)
 # Actual measured points become small octahedra, spatially subsampled for display.
 pp=a[f'p{f}'];_,ix=np.unique(np.floor(pp/.11).astype(np.int32),axis=0,return_index=True);pp=pp[np.sort(ix)];pp=pp[::max(1,len(pp)//22000)];pp=pp.copy();pp[:,0]+=off-1;pp[:,1]-=8;pp[:,2]-=floorz
 verts=(pp[:,None,:]+np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])[None,:,:]*.035).reshape(-1,3)
 fs=np.array([[0,2,4],[2,1,4],[1,3,4],[3,0,4],[2,0,5],[1,2,5],[3,1,5],[0,3,5]])
 ff=(np.arange(len(pp))[:,None,None]*6+fs[None,:,:]).reshape(-1,3)
 ob=mesh(f'Floor {f+1} measured cloud',verts.tolist(),ff.tolist(),[pointmat]);cloudobs.append(ob)
 for o in objects:
  if o['floor']!=f+1:continue
  corners=np.array(o['corners_xyz']);corners[:,0]+=off-1;corners[:,1]-=8;corners[:,2]-=floorz
  curve=bpy.data.curves.new(o['id'],'CURVE');curve.dimensions='3D';curve.bevel_depth=.023;curve.bevel_resolution=2;curve.resolution_u=1
  for i in range(8):
   for j in range(i+1,8):
    if bin(i^j).count('1')!=1:continue
    sp=curve.splines.new('POLY');sp.points.add(1);sp.points[0].co=(*corners[i],1);sp.points[1].co=(*corners[j],1)
  ob=bpy.data.objects.new(o['id']+' '+o['label'],curve);bpy.context.collection.objects.link(ob);curve.materials.append(edge);allboxes.append(ob)
 # Real room IDs, flat typography on the room surface.
 for k in np.unique(l[l>0]):
  yy,xx=np.nonzero(l==k);i=np.argmin((xx-np.median(xx))**2+(yy-np.median(yy))**2)
  cu=bpy.data.curves.new('Room ID','FONT');cu.body=f'{f+1}{int(k):02d}';cu.align_x='CENTER';cu.size=.35;cu.extrude=.001
  ob=bpy.data.objects.new(cu.body,cu);bpy.context.collection.objects.link(ob);ob.location=(org[0]+(xx[i]+.5)*res+off-1,org[1]+(yy[i]+.5)*res-8,.035);cu.materials.append(edge);allstructure.append(ob)
bpy.ops.mesh.primitive_plane_add(size=250,location=(0,0,-.2));bpy.context.object.data.materials.append(ground)
for loc,power,size in [((0,-15,35),10000,22),((-25,8,25),8000,20),((30,15,30),10000,25)]:
 bpy.ops.object.light_add(type='AREA',location=loc);light=bpy.context.object;light.data.energy=power;light.data.shape='DISK';light.data.size=size;light.rotation_euler=(Vector((0,0,0))-light.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add();cam=bpy.context.object;s.camera=cam;cam.data.type='ORTHO';cam.data.lens=50
# Gentle continuous orbit, followed by a pull-back to the three-floor tabletop.
for fr,loc,target,scale in [(1,(18,-27,35),(0,1,.4),29),(72,(16,-28,36),(0,1,.4),30),(168,(13,-29,37),(0,1,.4),31),(240,(6,-40,47),(0,1,.4),58),(300,(0,-45,52),(0,1,.4),83),(336,(-1,-45,52),(0,1,.4),83)]:
 cam.location=loc;cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=scale;cam.keyframe_insert('location',frame=fr);cam.keyframe_insert('rotation_euler',frame=fr);cam.data.keyframe_insert('ortho_scale',frame=fr)
def alpha_keys(m,keys):
 m.blend_method='BLEND';p=m.node_tree.nodes.get('Principled BSDF')
 for fr,value in keys:p.inputs['Alpha'].default_value=value;p.inputs['Alpha'].keyframe_insert('default_value',frame=fr)
for m in roomm+[wall]:alpha_keys(m,[(1,0),(40,0),(100,1),(336,1)])
alpha_keys(pointmat,[(1,.8),(60,.8),(125,.06),(336,.06)])
alpha_keys(edge,[(1,0),(115,0),(165,1),(336,1)])
s.render.filepath=str(O/'frames/frame_');bpy.ops.wm.save_as_mainfile(filepath=str(O/'scene.blend'))
(O/'manifest.json').write_text(json.dumps({'cloud':str(G),'rooms':str(D),'boxes':str(B),'duration':14,'fps':24,'frames':336,'display_only':True,'wall_display_height_m':.55,'room_slab_display_thickness_m':.12,'floor_layout':'separate floors side by side, not physical building positions','manual_room_revision':True,'gt_used':False},indent=2))
if '--preview' in sys.argv:
 s.frame_set(170);s.render.filepath=str(O/'preview.png');bpy.ops.render.render(write_still=True)
