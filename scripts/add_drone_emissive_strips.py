"""Add editable emissive side strips to the drone and export a new GLB."""

from pathlib import Path
import math

import bpy
from mathutils import Vector


project_root = Path(__file__).resolve().parents[1]
asset_dir = project_root / "assets" / "drone"
output_blend = asset_dir / "drone_emissive_editable.blend"
output_glb = asset_dir / "drone_emissive.glb"

# Imported model geometry is Z-up in Blender. Rotor centers establish the real
# arm directions, avoiding any screen-space approximation.
body_center = Vector((0.0, 0.134, 0.0))
rotors = {
    "FL": Vector((0.2507, -0.1265, 0.0)),
    "FR": Vector((-0.2504, -0.1265, 0.0)),
    "BL": Vector((0.2507, 0.3946, 0.0)),
    "BR": Vector((-0.2504, 0.3946, 0.0)),
}

# Remove only previously generated strips when the script is rerun.
for obj in list(bpy.data.objects):
    if obj.name.startswith("LED_"):
        bpy.data.objects.remove(obj, do_unlink=True)

material = bpy.data.materials.get("LED_Cyan_Emissive")
if material is None:
    material = bpy.data.materials.new("LED_Cyan_Emissive")
material.use_nodes = True
material.diffuse_color = (0.0, 0.82, 1.0, 1.0)
principled = material.node_tree.nodes.get("Principled BSDF")
principled.inputs["Base Color"].default_value = (0.0, 0.42, 0.55, 1.0)
principled.inputs["Roughness"].default_value = 0.22
emission_input = principled.inputs.get("Emission Color") or principled.inputs.get("Emission")
if emission_input is not None:
    emission_input.default_value = (0.0, 0.82, 1.0, 1.0)
strength_input = principled.inputs.get("Emission Strength")
if strength_input is not None:
    strength_input.default_value = 8.0

strip_offset = 0.018
strip_width = 0.008
strip_height = 0.010
strip_z = 0.026
body_clearance = 0.100
rotor_clearance = 0.072

created = []
for arm_name, rotor_center in rotors.items():
    direction = rotor_center - body_center
    direction.z = 0.0
    direction.normalize()
    lateral = Vector((-direction.y, direction.x, 0.0))
    start = body_center + direction * body_clearance
    end = rotor_center - direction * rotor_clearance
    length = (end - start).length
    angle = math.atan2(direction.y, direction.x)

    for side_name, sign in (("L", -1.0), ("R", 1.0)):
        midpoint = (start + end) * 0.5 + lateral * (sign * strip_offset)
        midpoint.z = strip_z
        bpy.ops.mesh.primitive_cube_add(location=midpoint)
        strip = bpy.context.active_object
        strip.name = f"LED_{arm_name}_{side_name}"
        strip.dimensions = (length, strip_width, strip_height)
        strip.rotation_euler[2] = angle
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        strip.data.materials.append(material)
        created.append(strip)

# Keep the eight strips selected so they are immediately editable in Blender.
bpy.ops.object.select_all(action="DESELECT")
for strip in created:
    strip.select_set(True)
bpy.context.view_layer.objects.active = created[0]

bpy.ops.wm.save_as_mainfile(filepath=str(output_blend))
bpy.ops.export_scene.gltf(
    filepath=str(output_glb),
    export_format="GLB",
    export_apply=True,
)
print(f"Created {len(created)} editable emissive strips")
print(f"Saved {output_blend}")
print(f"Exported {output_glb}")
