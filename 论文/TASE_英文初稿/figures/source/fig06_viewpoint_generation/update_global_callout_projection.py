import bpy,json
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
p=Path('/home/uav/map_VLN/画图/候选视角生成_新版');bpy.ops.wm.open_mainfile(filepath=str(p/'algorithm_scene.blend'));s=bpy.context.scene;bpy.context.view_layer.update()
for filename in ['occlusion_inset_geometry.json','collision_callout.json']:
 f=p/filename;d=json.loads(f.read_text());point=d['position'] if 'position' in d else json.loads((p/'collision_drone_geometry.json').read_text())['candidate']['position'];q=world_to_camera_view(s,s.camera,Vector(point));d['projection']=[q.x*2000,(1-q.y)*1400];f.write_text(json.dumps(d))
