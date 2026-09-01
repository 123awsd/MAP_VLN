"""Cut a narrow emissive band directly from the drone body's side surface."""

from pathlib import Path
import math

import bpy
import bmesh
from mathutils import Vector


project_root = Path(__file__).resolve().parents[1]
asset_dir = project_root / "assets" / "drone"
output_blend = asset_dir / "drone_surface_band_emissive_editable.blend"
output_glb = asset_dir / "drone_surface_band_emissive.glb"

for obj in list(bpy.data.objects):
    if obj.name.startswith("LED_"):
        bpy.data.objects.remove(obj, do_unlink=True)

body = bpy.data.objects.get("Body")
if body is None or body.type != "MESH":
    raise RuntimeError("Body mesh was not found")

material = bpy.data.materials.get("LED_Cyan_Emissive")
if material is None:
    material = bpy.data.materials.new("LED_Cyan_Emissive")
material.use_nodes = True
material.diffuse_color = (0.0, 0.82, 1.0, 1.0)
principled = material.node_tree.nodes.get("Principled BSDF")
principled.inputs["Base Color"].default_value = (0.0, 0.42, 0.55, 1.0)
principled.inputs["Roughness"].default_value = 0.20
emission = principled.inputs.get("Emission Color") or principled.inputs.get("Emission")
if emission is not None:
    emission.default_value = (0.0, 0.82, 1.0, 1.0)
strength = principled.inputs.get("Emission Strength")
if strength is not None:
    strength.default_value = 8.0

# Duplicate the actual body mesh, then intersect it with a thin horizontal slab.
# The resulting outer faces inherit the exact fuselage/arm curvature.
bpy.ops.object.select_all(action="DESELECT")
body.select_set(True)
bpy.context.view_layer.objects.active = body
bpy.ops.object.duplicate()
band = bpy.context.active_object
band.name = "LED_SURFACE_BAND"
band.data = band.data.copy()

bpy.ops.mesh.primitive_cube_add(location=(0.0, 0.134, 0.017))
cutter = bpy.context.active_object
cutter.name = "LED_BAND_CUTTER"
cutter.dimensions = (1.20, 1.20, 0.020)
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

bpy.context.view_layer.objects.active = band
band.select_set(True)
modifier = band.modifiers.new(name="Conforming side-band cut", type="BOOLEAN")
modifier.operation = "INTERSECT"
modifier.solver = "EXACT"
modifier.object = cutter
bpy.ops.object.modifier_apply(modifier=modifier.name)

bpy.data.objects.remove(cutter, do_unlink=True)

# Boolean intersection also creates horizontal cap faces. Remove them so only
# the narrow band on the drone's real vertical/oblique side skin can emit.
mesh_editor = bmesh.new()
mesh_editor.from_mesh(band.data)
mesh_editor.faces.ensure_lookup_table()
world_normal_matrix = band.matrix_world.to_3x3()
cap_faces = []
for face in mesh_editor.faces:
    world_normal = (world_normal_matrix @ face.normal).normalized()
    if abs(world_normal.z) > 0.22:
        cap_faces.append(face)
bmesh.ops.delete(mesh_editor, geom=cap_faces, context="FACES")
mesh_editor.to_mesh(band.data)
mesh_editor.free()
band.data.update()

band.data.materials.clear()
band.data.materials.append(material)
for polygon in band.data.polygons:
    polygon.material_index = 0

# Move the copied skin just 1 mm outward so it does not z-fight with the body.
displace = band.modifiers.new(name="LED surface clearance", type="DISPLACE")
displace.direction = "NORMAL"
displace.strength = 0.001
displace.mid_level = 0.0
bpy.context.view_layer.objects.active = band
bpy.ops.object.modifier_apply(modifier=displace.name)

# Add slim physical rails to both vertical sides of every arm. These sit at the
# same height as the conforming body band and remain individually editable.
body_center = Vector((0.0, 0.134, 0.0))
rotors = {
    "FL": Vector((0.2507, -0.1265, 0.0)),
    "FR": Vector((-0.2504, -0.1265, 0.0)),
    "BL": Vector((0.2507, 0.3946, 0.0)),
    "BR": Vector((-0.2504, 0.3946, 0.0)),
}
arm_lights = []
for arm_name, rotor_center in rotors.items():
    direction = rotor_center - body_center
    direction.z = 0.0
    direction.normalize()
    lateral = Vector((-direction.y, direction.x, 0.0))
    start = body_center + direction * 0.100
    end = rotor_center - direction * 0.072
    length = (end - start).length
    angle = math.atan2(direction.y, direction.x)
    for side_name, sign in (("L", -1.0), ("R", 1.0)):
        midpoint = (start + end) * 0.5 + lateral * (sign * 0.018)
        midpoint.z = 0.017
        bpy.ops.mesh.primitive_cube_add(location=midpoint)
        rail = bpy.context.active_object
        rail.name = f"LED_ARM_{arm_name}_{side_name}"
        rail.dimensions = (length, 0.0045, 0.018)
        rail.rotation_euler[2] = angle
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        rail.data.materials.append(material)
        arm_lights.append(rail)

bpy.ops.object.select_all(action="DESELECT")
band.select_set(True)
for rail in arm_lights:
    rail.select_set(True)
bpy.context.view_layer.objects.active = band

bpy.ops.wm.save_as_mainfile(filepath=str(output_blend))
bpy.ops.export_scene.gltf(
    filepath=str(output_glb),
    export_format="GLB",
    export_apply=True,
)
print(
    f"Band vertices={len(band.data.vertices)} polygons={len(band.data.polygons)} "
    f"arm_side_lights={len(arm_lights)}"
)
print(f"Saved {output_blend}")
print(f"Exported {output_glb}")
