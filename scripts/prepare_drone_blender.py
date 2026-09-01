"""Import the project drone GLB into a clean, editable Blender scene."""

from pathlib import Path

import bpy


project_root = Path(__file__).resolve().parents[1]
source_glb = project_root / "assets" / "drone" / "drone.glb"
output_blend = project_root / "assets" / "drone" / "drone_editable.blend"

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)

bpy.ops.import_scene.gltf(filepath=str(source_glb))
imported = list(bpy.context.selected_objects)
if not imported:
    raise RuntimeError(f"No objects imported from {source_glb}")

for obj in imported:
    obj.select_set(True)
bpy.context.view_layer.objects.active = imported[0]

bpy.ops.wm.save_as_mainfile(filepath=str(output_blend))
print(f"Imported {len(imported)} objects")
print(f"Saved {output_blend}")
