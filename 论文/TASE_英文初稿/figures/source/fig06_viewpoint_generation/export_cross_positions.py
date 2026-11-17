import bpy,json
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
p=Path('/home/uav/map_VLN/画图/候选视角生成_新版');result={}
for name,filename,point in [('collision','collision_explanation.blend',json.loads((p/'collision_drone_geometry.json').read_text())['candidate']['position']),('occlusion','occlusion_projected_fov.blend',None)]:
 bpy.ops.wm.open_mainfile(filepath=str(p/filename));s=bpy.context.scene;bpy.context.view_layer.update()
 if point is None:
  origin=Vector(json.loads((p/'occlusion_inset_geometry.json').read_text())['position']);direction=(Vector((0,0,1.01))-origin).normalized();point=origin+direction*.23
 q=world_to_camera_view(s,s.camera,Vector(point));result[name]=[q.x*s.render.resolution_x,(1-q.y)*s.render.resolution_y]
(p/'cross_positions.json').write_text(json.dumps(result))
