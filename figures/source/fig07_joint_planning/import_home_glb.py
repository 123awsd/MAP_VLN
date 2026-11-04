import bpy
from pathlib import Path

src = Path('/home/uav/下载/home_-_p3.glb')
out = Path('/home/uav/下载/home_-_p3_imported.blend')

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=str(src))
bpy.ops.wm.save_as_mainfile(filepath=str(out))
print(f'Imported: {src}')
print(f'Saved: {out}')
