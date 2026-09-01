"""Create one editable emissive loop around the drone's outer side."""

from pathlib import Path

import bpy


project_root = Path(__file__).resolve().parents[1]
asset_dir = project_root / "assets" / "drone"
output_blend = asset_dir / "drone_perimeter_emissive_editable.blend"
output_glb = asset_dir / "drone_perimeter_emissive.glb"

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
emission = principled.inputs.get("Emission Color") or principled.inputs.get("Emission")
if emission is not None:
    emission.default_value = (0.0, 0.82, 1.0, 1.0)
strength = principled.inputs.get("Emission Strength")
if strength is not None:
    strength.default_value = 8.0

# Clockwise footprint around the frame/arm ends. The loop deliberately stays
# clear of the center and sits just inside the four rotor assemblies.
z = 0.026
points = [
    (-0.245, -0.145, z),
    (-0.115, -0.185, z),
    (0.115, -0.185, z),
    (0.245, -0.145, z),
    (0.282, -0.020, z),
    (0.282, 0.288, z),
    (0.245, 0.413, z),
    (0.115, 0.453, z),
    (-0.115, 0.453, z),
    (-0.245, 0.413, z),
    (-0.282, 0.288, z),
    (-0.282, -0.020, z),
]

curve = bpy.data.curves.new("LED_Perimeter_Path", type="CURVE")
curve.dimensions = "3D"
curve.resolution_u = 2
curve.bevel_depth = 0.006
curve.bevel_resolution = 3
curve.resolution_u = 2

spline = curve.splines.new("NURBS")
spline.points.add(len(points) - 1)
for point, coordinate in zip(spline.points, points):
    point.co = (*coordinate, 1.0)
spline.use_cyclic_u = True
spline.use_endpoint_u = False
spline.order_u = 3
spline.use_smooth = True

ring = bpy.data.objects.new("LED_PERIMETER", curve)
bpy.context.collection.objects.link(ring)
curve.materials.append(material)

bpy.ops.object.select_all(action="DESELECT")
ring.select_set(True)
bpy.context.view_layer.objects.active = ring

bpy.ops.wm.save_as_mainfile(filepath=str(output_blend))
bpy.ops.export_scene.gltf(
    filepath=str(output_glb),
    export_format="GLB",
    export_apply=True,
)
print(f"Saved {output_blend}")
print(f"Exported {output_glb}")
