"""Print imported drone object transforms and world-space bounds."""

import bpy
from mathutils import Vector


points = []
for obj in bpy.context.scene.objects:
    if obj.type != "MESH":
        continue
    corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    points.extend(corners)
    xs = [point.x for point in corners]
    ys = [point.y for point in corners]
    zs = [point.z for point in corners]
    print(
        f"OBJECT {obj.name!r} dims={tuple(round(v, 5) for v in obj.dimensions)} "
        f"bounds=({min(xs):.5f},{max(xs):.5f}) "
        f"({min(ys):.5f},{max(ys):.5f}) ({min(zs):.5f},{max(zs):.5f})"
    )

if points:
    print(
        "TOTAL_BOUNDS "
        f"x=({min(p.x for p in points):.5f},{max(p.x for p in points):.5f}) "
        f"y=({min(p.y for p in points):.5f},{max(p.y for p in points):.5f}) "
        f"z=({min(p.z for p in points):.5f},{max(p.z for p in points):.5f})"
    )
