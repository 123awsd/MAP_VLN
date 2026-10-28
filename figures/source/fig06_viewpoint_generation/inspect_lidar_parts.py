import bpy,json,math
from pathlib import Path
from mathutils import Vector
p=Path('/home/uav/map_VLN/画图/候选视角生成_新版');bpy.ops.wm.open_mainfile(filepath=str(p/'collision_explanation.blend'));center=Vector(json.loads((p/'collision_drone_geometry.json').read_text())['candidate']['position']);o=next(o for o in bpy.data.objects if o.type=='MESH' and o.name.startswith('User STEP'));n=len(o.data.vertices);parent=list(range(n))
def root(a):
 while parent[a]!=a:parent[a]=parent[parent[a]];a=parent[a]
 return a
for e in o.data.edges:
 a,b=map(root,e.vertices);parent[b]=a
groups={}
for v in o.data.vertices:groups.setdefault(root(v.index),[]).append(v.index)
result=[]
for key,ids in groups.items():
 vs=[(o.matrix_world@o.data.vertices[i].co-center)/1.8 for i in ids];lo=[min(v[k] for v in vs) for k in range(3)];hi=[max(v[k] for v in vs) for k in range(3)]
 if hi[2]>.055 and max(abs(lo[0]),abs(hi[0]),abs(lo[1]),abs(hi[1]))<.08:result.append({'id':key,'n':len(ids),'lo':lo,'hi':hi})
Path('/tmp/lidar_parts.json').write_text(json.dumps(result));print(result)
