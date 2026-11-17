import bpy
from pathlib import Path
out=Path(__file__).resolve().parent
for scene,image in [('algorithm_scene.blend','polished_scene.png'),('occlusion_projected_fov.blend','occlusion_inset_3d.png'),('collision_explanation.blend','collision_explanation.png')]:
 bpy.ops.wm.open_mainfile(filepath=str(out/scene))
 bpy.context.scene.render.filepath=str(out/image)
 bpy.ops.render.render(write_still=True)
