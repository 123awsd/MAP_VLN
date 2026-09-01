"""Paint an emissive band onto existing body/arm side faces; add no geometry."""

from pathlib import Path

import bmesh
import bpy
from mathutils import Vector


project_root = Path(__file__).resolve().parents[1]
asset_dir = project_root / "assets" / "drone"
output_blend = asset_dir / "drone_side_painted_emissive_editable.blend"
output_glb = asset_dir / "drone_side_painted_emissive.glb"

body = bpy.data.objects.get("Body")
if body is None or body.type != "MESH":
    raise RuntimeError("Body mesh was not found")

# Ensure reruns never retain any previously added LED geometry.
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
principled.inputs["Roughness"].default_value = 0.20
emission = principled.inputs.get("Emission Color") or principled.inputs.get("Emission")
if emission is not None:
    emission.default_value = (0.0, 0.82, 1.0, 1.0)
strength = principled.inputs.get("Emission Strength")
if strength is not None:
    strength.default_value = 8.0

if material.name not in body.data.materials:
    body.data.materials.append(material)
emissive_index = body.data.materials.find(material.name)

# Use the arm's thin middle thickness as one shared horizontal band around the
# body, arms, and motor housings. Split only at the two material boundaries.
band_low = -0.025
band_high = 0.007
mesh_editor = bmesh.new()
mesh_editor.from_mesh(body.data)
for height in (band_low, band_high):
    geometry = list(mesh_editor.verts) + list(mesh_editor.edges) + list(mesh_editor.faces)
    bmesh.ops.bisect_plane(
        mesh_editor,
        geom=geometry,
        plane_co=Vector((0.0, 0.0, height)),
        plane_no=Vector((0.0, 0.0, 1.0)),
        clear_inner=False,
        clear_outer=False,
    )
mesh_editor.faces.ensure_lookup_table()
normal_matrix = body.matrix_world.to_3x3()
drone_center = Vector((0.0, 0.134, 0.0))
rotor_centers = (
    Vector((0.2507, -0.1265, 0.0)),
    Vector((-0.2504, -0.1265, 0.0)),
    Vector((0.2507, 0.3946, 0.0)),
    Vector((-0.2504, 0.3946, 0.0)),
)


def in_arm_corridor(point: Vector) -> bool:
    for rotor_center in rotor_centers:
        direction = rotor_center - drone_center
        arm_length = direction.length
        direction.normalize()
        relative = point - drone_center
        along = relative.dot(direction)
        lateral = (relative - direction * along).length
        if 0.18 <= along / arm_length <= 0.86 and lateral <= 0.055:
            return True
    return False


def in_motor_body(point: Vector) -> bool:
    flat_point = Vector((point.x, point.y, 0.0))
    return any((flat_point - rotor_center).length <= 0.067 for rotor_center in rotor_centers)


painted_body = 0
painted_arms = 0
painted_motors = 0
for face in mesh_editor.faces:
    center_world = body.matrix_world @ face.calc_center_median()
    normal_world = (normal_matrix @ face.normal).normalized()
    # Flat top/bottom faces have |normal.z| ~= 1. Keep them and the separate
    # Rotor_* blade objects untouched. Only paint the shared thin middle band.
    if not (band_low - 1e-5 <= center_world.z <= band_high + 1e-5):
        continue
    if abs(normal_world.z) >= 0.94:
        continue
    face.material_index = emissive_index
    if in_motor_body(center_world):
        painted_motors += 1
    elif in_arm_corridor(center_world):
        painted_arms += 1
    else:
        painted_body += 1

mesh_editor.to_mesh(body.data)
mesh_editor.free()
body.data.update()

bpy.ops.object.select_all(action="DESELECT")
body.select_set(True)
bpy.context.view_layer.objects.active = body

bpy.ops.wm.save_as_mainfile(filepath=str(output_blend))
bpy.ops.export_scene.gltf(
    filepath=str(output_glb),
    export_format="GLB",
    export_apply=True,
)
print(
    f"Painted shared arm-thickness side band: body={painted_body}, "
    f"arms={painted_arms}, motors={painted_motors}; "
    "extra LED objects=0"
)
print(f"Saved {output_blend}")
print(f"Exported {output_glb}")
