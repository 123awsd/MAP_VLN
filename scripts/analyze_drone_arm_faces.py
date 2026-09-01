"""Inspect face-normal ranges in the four diagonal arm corridors."""

import math

import bpy
import numpy as np
from mathutils import Vector


body = bpy.data.objects["Body"]
center = Vector((0.0, 0.134, 0.0))
rotors = [
    Vector((0.2507, -0.1265, 0.0)),
    Vector((-0.2504, -0.1265, 0.0)),
    Vector((0.2507, 0.3946, 0.0)),
    Vector((-0.2504, 0.3946, 0.0)),
]


def distance_to_arm(point: Vector, rotor: Vector) -> tuple[float, float]:
    direction = rotor - center
    length = direction.length
    direction.normalize()
    relative = point - center
    along = relative.dot(direction)
    lateral = (relative - direction * along).length
    return along / length, lateral


normal_z = []
center_z = []
for polygon in body.data.polygons:
    world_center = body.matrix_world @ polygon.center
    in_arm = any(
        0.22 <= fraction <= 0.82 and lateral <= 0.050
        for fraction, lateral in (distance_to_arm(world_center, rotor) for rotor in rotors)
    )
    if not in_arm:
        continue
    world_normal = (body.matrix_world.to_3x3() @ polygon.normal).normalized()
    normal_z.append(abs(world_normal.z))
    center_z.append(world_center.z)

print(f"arm_faces={len(normal_z)}")
for name, values in (("abs_normal_z", normal_z), ("center_z", center_z)):
    data = np.asarray(values, dtype=np.float64)
    print(
        name,
        "min", round(float(data.min()), 4),
        "p25", round(float(np.quantile(data, 0.25)), 4),
        "p50", round(float(np.quantile(data, 0.50)), 4),
        "p75", round(float(np.quantile(data, 0.75)), 4),
        "max", round(float(data.max()), 4),
    )

vertical_z = [z for nz, z in zip(normal_z, center_z) if nz <= 0.22]
print(f"near_vertical_arm_faces={len(vertical_z)}")
if vertical_z:
    data = np.asarray(vertical_z, dtype=np.float64)
    print(
        "near_vertical_center_z",
        "min", round(float(data.min()), 4),
        "p25", round(float(np.quantile(data, 0.25)), 4),
        "p50", round(float(np.quantile(data, 0.50)), 4),
        "p75", round(float(np.quantile(data, 0.75)), 4),
        "max", round(float(data.max()), 4),
    )
