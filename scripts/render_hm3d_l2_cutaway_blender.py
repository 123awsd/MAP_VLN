#!/usr/bin/env python3
"""Blender headless renderer for a textured L2 cutaway from the official HM3D GLB.

Run with: blender --background --python this_file.py -- [options]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix, Vector


def arguments() -> argparse.Namespace:
    values = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--official-source", type=Path,
                        help="Original HM3D GLB recorded in metadata when --scene is a decoded temporary glTF")
    parser.add_argument("--generated-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--falcon-floor-z", type=float, default=2.60)
    parser.add_argument("--wall-height", type=float, default=1.55)
    parser.add_argument("--width", type=int, default=1800)
    parser.add_argument("--height", type=int, default=1500)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=30.0)
    parser.add_argument("--front-cut", type=float, default=0.20,
                        help="Fraction of scene depth removed on camera-facing side")
    parser.add_argument("--boxes", type=Path,
                        help="Optional Boxer CSV in FALCON coordinates to overlay on the GLB")
    parser.add_argument("--box-height-scale", type=float, default=1.0)
    parser.add_argument("--box-alpha", type=float, default=0.16)
    parser.add_argument("--export-glb", type=Path,
                        help="Optional path for the clipped scene plus Boxer geometry")
    parser.add_argument("--inspect", action="store_true")
    return parser.parse_args(values)


BOX_PALETTE = {
    "sleep": (0.61, 0.74, 0.88, 1.0),
    "opening": (0.95, 0.67, 0.65, 1.0),
    "storage": (0.78, 0.71, 0.90, 1.0),
    "seating": (0.66, 0.85, 0.72, 1.0),
    "surface": (0.96, 0.78, 0.56, 1.0),
    "fixture": (0.62, 0.85, 0.84, 1.0),
    "appliance": (0.95, 0.86, 0.57, 1.0),
    "decor": (0.85, 0.71, 0.80, 1.0),
    "other": (0.73, 0.80, 0.84, 1.0),
}

BOX_GROUPS = {
    "sleep": {"bed", "crib", "mattress", "pillow"},
    "opening": {"curtain", "door", "railing", "window"},
    "storage": {"cabinet", "dresser", "vanity"},
    "seating": {"armchair", "bench", "chair", "sofa"},
    "surface": {"coffee table", "countertop", "kitchen island", "table"},
    "fixture": {"bathtub", "heater", "sink", "toilet", "toilet paper"},
    "appliance": {"dishwasher", "microwave", "television"},
    "decor": {"fireplace", "lamp", "mirror", "picture", "rug"},
}


def box_color(label: str):
    label = label.strip().lower()
    for group, labels in BOX_GROUPS.items():
        if label in labels:
            return BOX_PALETTE[group]
    return BOX_PALETTE["other"]


def falcon_to_blender(point, initial_sensor_habitat):
    """Map FALCON XYZ to the Z-up coordinates used by the imported HM3D mesh."""
    x, y, z = point
    hx, hy, hz = initial_sensor_habitat
    return Vector((hx - y, -hz + x, hy + z - 1.0))


def add_boxer_geometry(box_csv: Path, initial_sensor_habitat, height_scale: float,
                       alpha: float) -> int:
    with box_csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    edge_pairs = ((0, 1), (1, 2), (2, 3), (3, 0),
                  (4, 5), (5, 6), (6, 7), (7, 4),
                  (0, 4), (1, 5), (2, 6), (3, 7))
    face_indices = ((0, 1, 2, 3), (4, 5, 6, 7),
                    (0, 1, 5, 4), (1, 2, 6, 5),
                    (2, 3, 7, 6), (3, 0, 4, 7))
    materials = {}
    edge_materials = {}
    for index, row in enumerate(rows):
        label = row["name"].strip().lower()
        color = box_color(label)
        group = next((key for key, value in BOX_PALETTE.items() if value == color), "other")
        if group not in materials:
            material = bpy.data.materials.new(f"Boxer_{group}")
            material.diffuse_color = (*color[:3], alpha)
            material.use_nodes = True
            principled = material.node_tree.nodes.get("Principled BSDF")
            principled.inputs["Base Color"].default_value = (*color[:3], 1.0)
            principled.inputs["Alpha"].default_value = alpha
            principled.inputs["Roughness"].default_value = 0.68
            if hasattr(material, "surface_render_method"):
                material.surface_render_method = "DITHERED"
            elif hasattr(material, "blend_method"):
                material.blend_method = "BLEND"
                material.show_transparent_back = True
            materials[group] = material
            edge_material = bpy.data.materials.new(f"BoxerEdge_{group}")
            edge_material.diffuse_color = (*color[:3], 0.88)
            edge_material.use_nodes = True
            edge_principled = edge_material.node_tree.nodes.get("Principled BSDF")
            edge_principled.inputs["Base Color"].default_value = (*color[:3], 1.0)
            edge_principled.inputs["Alpha"].default_value = 0.88
            edge_principled.inputs["Roughness"].default_value = 0.55
            if hasattr(edge_material, "surface_render_method"):
                edge_material.surface_render_method = "DITHERED"
            elif hasattr(edge_material, "blend_method"):
                edge_material.blend_method = "BLEND"
            edge_materials[group] = edge_material
        material = materials[group]
        edge_material = edge_materials[group]

        cx, cy, cz = (float(row[key]) for key in
                      ("tx_world_object", "ty_world_object", "tz_world_object"))
        sx, sy, sz = (float(row[key]) for key in ("scale_x", "scale_y", "scale_z"))
        yaw = 2.0 * math.atan2(float(row["qz_world_object"]), float(row["qw_world_object"]))
        z0 = cz - sz * 0.5
        z1 = z0 + sz * height_scale
        xy_local = ((-sx/2, -sy/2), (sx/2, -sy/2), (sx/2, sy/2), (-sx/2, sy/2))
        xy_world = []
        for lx, ly in xy_local:
            xy_world.append((cx + math.cos(yaw)*lx - math.sin(yaw)*ly,
                             cy + math.sin(yaw)*lx + math.cos(yaw)*ly))
        vertices = [falcon_to_blender((x, y, z), initial_sensor_habitat)
                    for z in (z0, z1) for x, y in xy_world]

        mesh = bpy.data.meshes.new(f"BoxerMesh_{index:03d}_{label}")
        mesh.from_pydata([tuple(v) for v in vertices], [], face_indices)
        mesh.materials.append(material)
        obj = bpy.data.objects.new(f"Boxer_{index:03d}_{label}", mesh)
        bpy.context.collection.objects.link(obj)

        curve = bpy.data.curves.new(f"BoxerEdges_{index:03d}_{label}", "CURVE")
        curve.dimensions = "3D"
        curve.resolution_u = 1
        curve.bevel_depth = 0.012
        curve.bevel_resolution = 1
        curve.materials.append(edge_material)
        for a, b in edge_pairs:
            spline = curve.splines.new("POLY")
            spline.points.add(1)
            spline.points[0].co = (*vertices[a], 1.0)
            spline.points[1].co = (*vertices[b], 1.0)
        edge_obj = bpy.data.objects.new(f"BoxerEdges_{index:03d}_{label}", curve)
        bpy.context.collection.objects.link(edge_obj)
    return len(rows)


def delete_outside_box(obj, z_min: float, z_max: float, front_y: float) -> None:
    """Geometrically bisect the mesh at the L2 slab and front cut planes."""
    mesh = obj.data
    bm = bmesh.new()
    bm.from_mesh(mesh)
    geom = list(bm.verts) + list(bm.edges) + list(bm.faces)
    bmesh.ops.bisect_plane(bm, geom=geom, plane_co=(0, 0, z_min),
                           plane_no=(0, 0, 1), dist=1e-5,
                           clear_inner=True, clear_outer=False)
    geom = list(bm.verts) + list(bm.edges) + list(bm.faces)
    bmesh.ops.bisect_plane(bm, geom=geom, plane_co=(0, 0, z_max),
                           plane_no=(0, 0, 1), dist=1e-5,
                           clear_inner=False, clear_outer=True)
    geom = list(bm.verts) + list(bm.edges) + list(bm.faces)
    bmesh.ops.bisect_plane(bm, geom=geom, plane_co=(0, front_y, 0),
                           plane_no=(0, 1, 0), dist=1e-5,
                           clear_inner=True, clear_outer=False)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()


def look_at(camera, target: Vector) -> None:
    camera.rotation_euler = ((target - camera.location).to_track_quat("-Z", "Y").to_euler())


def main() -> None:
    args = arguments()
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.gltf(filepath=str(args.scene.resolve()))
    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not meshes:
        raise RuntimeError("No mesh objects imported from HM3D GLB")

    corners = []
    for obj in meshes:
        corners.extend(obj.matrix_world @ Vector(corner) for corner in obj.bound_box)
    low = Vector((min(v.x for v in corners), min(v.y for v in corners), min(v.z for v in corners)))
    high = Vector((max(v.x for v in corners), max(v.y for v in corners), max(v.z for v in corners)))
    print("HM3D_BOUNDS", list(low), list(high), "MESHES", len(meshes))
    if args.inspect:
        return

    # The glTF arrives Y-up with upward Habitat motion represented by -Blender Y.
    # Rotate it once into normal Blender Z-up coordinates: new Z = -old Y.
    rotate_to_z_up = Matrix.Rotation(-math.pi / 2.0, 4, "X")
    for obj in meshes:
        obj.matrix_world = rotate_to_z_up @ obj.matrix_world
    bpy.ops.object.select_all(action="DESELECT")
    for obj in meshes:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.select_all(action="DESELECT")

    corners = []
    for obj in meshes:
        corners.extend(obj.matrix_world @ Vector(corner) for corner in obj.bound_box)
    low = Vector((min(v.x for v in corners), min(v.y for v in corners), min(v.z for v in corners)))
    high = Vector((max(v.x for v in corners), max(v.y for v in corners), max(v.z for v in corners)))

    generated = json.loads(args.generated_config.read_text())
    initial_sensor_y = float(generated["initial_sensor_habitat_xyz"][1])
    # Falcon initial sensor z is 1.0 m; Habitat Y is its vertical axis. After
    # the axis correction above, Blender Z equals Habitat Y.
    floor_blender_z = initial_sensor_y + (args.falcon_floor_z - 1.0)
    z_min = floor_blender_z - 0.12
    z_max = floor_blender_z + args.wall_height
    front_y = low.y + args.front_cut * (high.y - low.y)
    for obj in meshes:
        delete_outside_box(obj, z_min, z_max, front_y)

    box_count = 0
    if args.boxes:
        box_count = add_boxer_geometry(
            args.boxes.resolve(), generated["initial_sensor_habitat_xyz"],
            args.box_height_scale, args.box_alpha,
        )

    # Recompute visible bounds after clipping.
    visible = []
    for obj in meshes:
        for polygon in obj.data.polygons:
            visible.extend(obj.matrix_world @ obj.data.vertices[i].co
                           for i in polygon.vertices)
    low_v = Vector((min(v.x for v in visible), min(v.y for v in visible), min(v.z for v in visible)))
    high_v = Vector((max(v.x for v in visible), max(v.y for v in visible), max(v.z for v in visible)))
    center = (low_v + high_v) * 0.5
    radius = max(high_v.x - low_v.x, high_v.y - low_v.y) * 0.62

    bpy.ops.object.camera_add()
    camera = bpy.context.object
    angle = math.radians(args.azimuth)
    elev = math.radians(args.elevation)
    camera.location = center + Vector((radius * math.cos(angle), radius * math.sin(angle),
                                       radius * math.tan(elev)))
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = max(high_v.x - low_v.x, high_v.y - low_v.y) * 1.18
    camera.data.lens = 50
    look_at(camera, center)
    bpy.context.scene.camera = camera

    world = bpy.context.scene.world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (1, 1, 1, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.8
    bpy.ops.object.light_add(type="AREA", location=(center.x, center.y, high_v.z + radius))
    light = bpy.context.object
    light.data.energy = 1300
    light.data.size = radius * 1.3

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    if hasattr(scene, "eevee"):
        scene.eevee.use_gtao = True
        scene.eevee.gtao_distance = 3
        scene.eevee.gtao_factor = 1.15
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.look = "Medium High Contrast"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(args.output.resolve())
    bpy.ops.render.render(write_still=True)

    if args.export_glb:
        args.export_glb.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.export_scene.gltf(
            filepath=str(args.export_glb.resolve()), export_format="GLB",
            export_apply=True, export_cameras=False, export_lights=False,
        )

    report = {
        "format": "pre_map_vln.hm3d_l2_cutaway_blender.v1",
        "source_glb": str((args.official_source or args.scene).resolve()),
        "temporary_decoded_scene": args.official_source is not None,
        "coordinate_source": str(args.generated_config.resolve()),
        "source_type": "official HM3D textured mesh",
        "vertical_clip_blender_z": [z_min, z_max],
        "falcon_floor_z_m": args.falcon_floor_z,
        "retained_wall_height_m": args.wall_height,
        "camera_facing_cut_fraction": args.front_cut,
        "view": {"azimuth": args.azimuth, "elevation": args.elevation,
                 "projection": "orthographic"},
        "renderer": "Blender EEVEE",
        "uses_pointcloud": False,
        "uses_image_generation": False,
        "boxer_csv": str(args.boxes.resolve()) if args.boxes else None,
        "box_count": box_count,
        "box_height_scale": args.box_height_scale,
        "box_alpha": args.box_alpha,
        "exported_glb": str(args.export_glb.resolve()) if args.export_glb else None,
        "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
