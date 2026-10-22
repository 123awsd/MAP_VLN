#!/usr/bin/env python3
"""Render a reusable semantic-map video from existing measured outputs.

Run: blender -b --python scripts/render_semantic_map_animation.py -- \
  --config config/semantic_map_animation_9s.json --mode preview|video
The script only reads mapping and detection products. All geometry remains in
the input's XYZ coordinates; no per-floor display offset is used.
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[1]
ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
parser = argparse.ArgumentParser()
parser.add_argument("--config", type=Path, required=True)
parser.add_argument("--mode", choices=("preview", "video"), default="preview")
args = parser.parse_args(ARGS)
config_path = args.config.resolve()
cfg = json.loads(config_path.read_text())


def resolve_input(value):
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


inputs = {name: resolve_input(path) for name, path in cfg["inputs"].items()}
for name, path in inputs.items():
    if not path.exists():
        raise FileNotFoundError(f"{name}: {path}")
out = resolve_input(cfg["output_dir"])
out.mkdir(parents=True, exist_ok=True)
preview_dir = out / "previews"
preview_dir.mkdir(exist_ok=True)


def load_cloud(path):
    if path.suffix == ".npz":
        a = np.load(path)
        keys = sorted((k for k in a.files if k == "points" or k.startswith("p") and k[1:].isdigit()), key=lambda k: (k != "points", k))
        if not keys:
            raise ValueError("Cloud NPZ needs 'points' or p0/p1/... arrays")
        points = np.concatenate([np.asarray(a[k], dtype=np.float32)[:, :3] for k in keys])
    elif path.suffix == ".npy":
        points = np.asarray(np.load(path), dtype=np.float32)[:, :3]
    elif path.suffix in (".ply", ".pcd"):
        # Open3D is optional in Blender's Python; ASCII PLY/PCD is supported here.
        with path.open() as f:
            header = []
            while True:
                line = f.readline()
                if not line:
                    raise ValueError("Incomplete point-cloud header")
                header.append(line.strip())
                if line.strip() in ("end_header", "DATA ascii"):
                    break
            if path.suffix == ".ply":
                if "format ascii 1.0" not in header:
                    raise ValueError("Binary PLY: provide a .npz/.npy export")
                count = int(next(v.split()[2] for v in header if v.startswith("element vertex ")))
                points = np.loadtxt(f, max_rows=count, usecols=(0, 1, 2), dtype=np.float32)
            else:
                if header[-1] != "DATA ascii":
                    raise ValueError("Binary PCD: provide a .npz/.npy export")
                fields = next(v.split()[1:] for v in header if v.startswith("FIELDS "))
                ix = [fields.index(k) for k in ("x", "y", "z")]
                points = np.loadtxt(f, usecols=ix, dtype=np.float32)
    else:
        raise ValueError(f"Unsupported cloud format: {path.suffix}")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) > cfg["max_render_points"]:
        rng = np.random.default_rng(7)
        points = points[np.sort(rng.choice(len(points), cfg["max_render_points"], replace=False))]
    return points


def load_objects(path):
    if path.suffix == ".json":
        data = json.loads(path.read_text())
        rows = data["objects"] if isinstance(data, dict) else data
        objects = []
        for row in rows:
            corners = np.asarray(row.get("corners_xyz"), dtype=float)
            if corners.shape != (8, 3):
                raise ValueError("Object JSON needs corners_xyz of shape 8x3")
            objects.append((str(row.get("label", "object")), corners, str(row.get("id", ""))))
        return objects
    if path.suffix == ".csv":
        # Native Boxer fused CSV: quaternion, centre and scale in world XYZ.
        from mathutils import Quaternion
        objects = []
        with path.open(newline="") as f:
            for i, row in enumerate(csv.DictReader(f)):
                center = Vector(float(row[k + "_world_object"]) for k in ("tx", "ty", "tz"))
                scale = [float(row["scale_" + k]) for k in "xyz"]
                quat = Quaternion((float(row["qw_world_object"]), float(row["qx_world_object"]), float(row["qy_world_object"]), float(row["qz_world_object"])))
                corners = np.array([center + quat @ Vector((sx * scale[0] / 2, sy * scale[1] / 2, sz * scale[2] / 2)) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                objects.append((row["name"], corners, f"O{i + 1:03d}"))
        return objects
    raise ValueError(f"Unsupported object format: {path.suffix}")


def load_trajectory(path):
    if path.suffix == ".npy":
        a = np.asarray(np.load(path), dtype=float)
        if a.shape[1] < 3:
            raise ValueError("Trajectory needs XYZ columns")
        # Project export uses [time, x, y, z]; standalone XYZ is also valid.
        return a[:, 1:4] if a.shape[1] >= 4 else a[:, :3]
    if path.suffix == ".jsonl":
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        for key in ("position", "position_xyz", "xyz"):
            if key in rows[0]:
                return np.asarray([v[key] for v in rows], dtype=float)
        raise ValueError("Trajectory JSONL needs position/position_xyz/xyz")
    if path.suffix == ".csv":
        with path.open(newline="") as f:
            return np.asarray([[float(row[k]) for k in ("x", "y", "z")] for row in csv.DictReader(f)])
    raise ValueError(f"Unsupported trajectory format: {path.suffix}")


cloud = load_cloud(inputs["point_cloud"])
objects = load_objects(inputs["objects"])
trajectory = load_trajectory(inputs["trajectory"])
room_dir = inputs["rooms"]
room_files = sorted(room_dir.glob("L*_constrained.npz")) if room_dir.is_dir() else [room_dir]
if not room_files:
    raise ValueError("Room input needs L*_constrained.npz or one room NPZ")

# Validate all sources share one physical extent before constructing any scene.
cloud_min, cloud_max = cloud.min(axis=0), cloud.max(axis=0)
if np.any(trajectory.min(axis=0) < cloud_min - 3) or np.any(trajectory.max(axis=0) > cloud_max + 3):
    raise ValueError("Trajectory extent disagrees with cloud; check coordinates")

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE"
scene.render.resolution_x = int(cfg["width"])
scene.render.resolution_y = int(cfg["height"])
scene.render.resolution_percentage = 100
scene.render.fps = int(cfg["fps"])
scene.frame_start = 1
scene.frame_end = int(cfg["fps"] * cfg["seconds"])
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGB"
scene.render.film_transparent = False
scene.render.filepath = str(out / "frames" / "frame_")
scene.eevee.taa_render_samples = int(cfg["render_samples"])
scene.eevee.use_gtao = True
scene.eevee.gtao_distance = 3
scene.eevee.gtao_factor = 1.2
scene.view_settings.view_transform = "Standard"
scene.world.use_nodes = True
scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (*cfg["background_rgb"], 1)
scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.8


def material(name, color, opacity=1, emission=False):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*color, opacity)
    m.use_nodes = True
    p = m.node_tree.nodes.get("Principled BSDF")
    p.inputs["Base Color"].default_value = (*color, 1)
    p.inputs["Roughness"].default_value = .48
    p.inputs["Metallic"].default_value = .12
    if emission:
        p.inputs["Emission Color"].default_value = (*color, 1)
        p.inputs["Emission Strength"].default_value = .24
    p.inputs["Alpha"].default_value = opacity
    m.blend_method = "BLEND"
    m.show_transparent_back = False
    return m, p.inputs["Alpha"]


def key_alpha(input_node, sequence):
    for frame, alpha in sequence:
        input_node.default_value = alpha
        input_node.keyframe_insert("default_value", frame=frame)


def mesh_obj(name, vertices, faces, materials=None, indices=None):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    ob = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(ob)
    for mat in materials or []:
        mesh.materials.append(mat)
    if indices is not None:
        for face, index in zip(mesh.polygons, indices):
            face.material_index = index
    return ob


# One mesh with tetrahedral glyphs: no per-point Blender objects. Height is
# encoded as a vertex color so fading a single material preserves the gradient.
low_z, high_z = float(cloud_min[2]), float(cloud_max[2])
colors = [(0.42, 0.18, 0.78), (0.18, 0.47, 0.93), (0.04, 0.76, 0.79), (0.57, 0.84, 0.20), (0.98, 0.64, 0.12)]
glyph = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], dtype=np.float32) * float(cfg["point_radius_m"])
faces = np.array([[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]], dtype=np.int32)
verts = (cloud[:, None, :] + glyph[None, :, :]).reshape(-1, 3)
triangles = (faces[None, :, :] + 6 * np.arange(len(cloud), dtype=np.int32)[:, None, None]).reshape(-1, 3)
pmat, palpha = material("Height gradient measured cloud", (.8, .8, .8), 1)
pn = pmat.node_tree.nodes
attr = pn.new("ShaderNodeVertexColor")
attr.layer_name = "HeightGradient"
pmat.node_tree.links.new(attr.outputs["Color"], pn.get("Principled BSDF").inputs["Base Color"])
point_ob = mesh_obj("Measured cloud / one optimized mesh", verts.tolist(), triangles.tolist(), [pmat])
color_attr = point_ob.data.vertex_colors.new(name="HeightGradient")
height = np.clip((cloud[:, 2] - low_z) / max(high_z - low_z, 1e-6), 0, 1)
rgba = np.asarray([colors[min(4, int(x * 4.999))] + (1,) for x in height], dtype=np.float32)
for poly in point_ob.data.polygons:
    c = rgba[poly.index // 8]
    for loop in poly.loop_indices:
        color_attr.data[loop].color = c
key_alpha(palpha, [(1, 1), (150, 1), (210, 0), (270, 0)])

room_palette = [(.96, .33, .39), (.19, .77, .86), (.98, .77, .19), (.52, .35, .87), (.40, .79, .35), (.99, .52, .22), (.91, .39, .77)]
room_mats = []
for i, color in enumerate(room_palette):
    room_mats.append(material(f"Room color {i}", color, 0)[0])
room_alpha = float(cfg["room_opacity"])

for floor_index, path in enumerate(room_files):
    data = np.load(path)
    labels = np.asarray(data["labels"], dtype=np.int32)
    origin = np.asarray(data["origin_xy"], dtype=float)
    resolution = float(data["resolution"])
    z = float(cfg["floor_z_m"][floor_index])
    v, f, mi = [], [], []
    for y, row in enumerate(labels):
        changes = np.flatnonzero(np.r_[True, row[1:] != row[:-1], True])
        for x0, x1 in zip(changes[:-1], changes[1:]):
            label = int(row[x0])
            if label <= 0:
                continue
            xlo, xhi = origin[0] + x0 * resolution, origin[0] + x1 * resolution
            ylo, yhi = origin[1] + y * resolution, origin[1] + (y + 1) * resolution
            k = len(v)
            v.extend([(xlo, ylo, z), (xhi, ylo, z), (xhi, yhi, z), (xlo, yhi, z)])
            f.append((k, k + 1, k + 2, k + 3))
            mi.append((label - 1 + floor_index * 2) % len(room_mats))
    ob = mesh_obj(f"Floor {floor_index + 1} room polygons / source grid", v, f, room_mats, mi)
    if cfg.get("show_room_labels", False):
        for label in np.unique(labels[labels > 0]):
            yy, xx = np.nonzero(labels == label)
            q = np.argmin((xx - np.median(xx)) ** 2 + (yy - np.median(yy)) ** 2)
            curve = bpy.data.curves.new("Room ID", "FONT")
            curve.body = f"{floor_index + 1}{int(label):02d}"
            curve.size = .28
            curve.align_x = "CENTER"
            obj = bpy.data.objects.new(curve.body, curve)
            bpy.context.collection.objects.link(obj)
            obj.location = (origin[0] + (xx[q] + .5) * resolution, origin[1] + (yy[q] + .5) * resolution, z + .01)

for m in room_mats:
    key_alpha(m.node_tree.nodes.get("Principled BSDF").inputs["Alpha"], [(1, 0), (210, 0), (230, room_alpha), (270, room_alpha)])

class_palette = [(.94, .23, .36), (.06, .70, .89), (.98, .69, .12), (.53, .31, .91), (.26, .75, .32), (.97, .40, .68), (.18, .55, .96), (.96, .48, .18)]
categories = sorted({label for label, _, _ in objects})
class_mats = {label: material(f"Class / {label}", class_palette[i % len(class_palette)], 0, True)[0] for i, label in enumerate(categories)}

# JSON corners are in binary-sign order (see Boxer output). Build adjacency
# from the object-local shape, retaining world-space rotation and dimensions.
for label, corners, oid in objects:
    pairs = [(i, j) for i in range(8) for j in range(i + 1, 8)
             if bin(i ^ j).count("1") == 1]
    curve = bpy.data.curves.new(f"Box {oid}", "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = float(cfg["box_line_radius_m"])
    curve.bevel_resolution = 2
    for i, j in sorted(set(pairs)):
        spline = curve.splines.new("POLY")
        spline.points.add(1)
        spline.points[0].co = (*corners[i], 1)
        spline.points[1].co = (*corners[j], 1)
    ob = bpy.data.objects.new(f"{oid} {label}", curve)
    bpy.context.collection.objects.link(ob)
    curve.materials.append(class_mats[label])
    if cfg.get("show_object_labels", False):
        title = bpy.data.curves.new(f"Label {oid}", "FONT")
        title.body = label
        title.size = .22
        text = bpy.data.objects.new(f"Label {oid}", title)
        bpy.context.collection.objects.link(text)
        text.location = corners.mean(axis=0) + np.array([0, 0, .15])

for m in class_mats.values():
    key_alpha(m.node_tree.nodes.get("Principled BSDF").inputs["Alpha"], [(1, 0), (90, 0), (150, 1), (270, 1)])

# The recorded actual trajectory is a single 3D bevelled curve in source XYZ.
trajectory = trajectory[np.isfinite(trajectory).all(axis=1)]
if len(trajectory) > 4000:
    trajectory = trajectory[np.linspace(0, len(trajectory) - 1, 4000).astype(int)]
tmat, _ = material("Executed trajectory blue", (.04, .22, .91), 1, True)
curve = bpy.data.curves.new("Executed trajectory", "CURVE")
curve.dimensions = "3D"
curve.bevel_depth = float(cfg["trajectory_line_radius_m"])
curve.bevel_resolution = 2
spline = curve.splines.new("POLY")
spline.points.add(len(trajectory) - 1)
for point, xyz in zip(spline.points, trajectory):
    point.co = (*xyz, 1)
ob = bpy.data.objects.new("Recorded executed trajectory", curve)
bpy.context.collection.objects.link(ob)
curve.materials.append(tmat)

# Data-derived scene bounds. Orthographic fit uses the projected eight corners
# for every camera orientation, so the 3D extent remains uncropped.
all_points = np.concatenate((cloud, trajectory, np.concatenate([o[1] for o in objects])))
lo, hi = all_points.min(axis=0), all_points.max(axis=0)
center = Vector(((lo + hi) / 2).tolist())
corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
radius = float(np.linalg.norm(hi - lo))
bpy.ops.object.camera_add()
camera = bpy.context.object
camera.name = "Data-fitted orthographic camera"
scene.camera = camera
camera.data.type = "ORTHO"
aspect = cfg["width"] / cfg["height"]


def fit_camera(frame, azimuth, elevation, margin=1.19):
    az, el = math.radians(azimuth), math.radians(elevation)
    direction = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    camera.location = center + direction * radius * 2.3
    camera.rotation_euler = (center - camera.location).to_track_quat("-Z", "Y").to_euler()
    q = camera.rotation_euler.to_matrix()
    right, up = q.col[0], q.col[1]
    flat = np.asarray([Vector(v) - center for v in corners])
    projected_x = np.asarray([Vector(v).dot(right) for v in flat])
    projected_y = np.asarray([Vector(v).dot(up) for v in flat])
    h = np.ptp(projected_y)
    w = np.ptp(projected_x)
    camera.data.ortho_scale = max(w, h * aspect) * margin
    camera.keyframe_insert("location", frame=frame)
    camera.keyframe_insert("rotation_euler", frame=frame)
    camera.data.keyframe_insert("ortho_scale", frame=frame)


fit_camera(1, -67, 56)
fit_camera(90, -48, 54)
fit_camera(150, -37, 52)
fit_camera(210, -25, 50)
fit_camera(270, -8, 15, 1.24)
for action in (camera.animation_data.action, camera.data.animation_data.action):
    for fc in action.fcurves:
        for key in fc.keyframe_points:
            key.interpolation = "BEZIER"

# Broad studio light brings out depth without any simulator UI or grid.
for location, power, size in [((0, -12, 25), 2600, 18), ((-10, 12, 20), 1900, 15)]:
    bpy.ops.object.light_add(type="AREA", location=location)
    light = bpy.context.object
    light.data.energy = power
    light.data.shape = "DISK"
    light.data.size = size
    light.rotation_euler = (center - light.location).to_track_quat("-Z", "Y").to_euler()

project = out / "semantic_map.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(project))
metadata = {
    "source": {k: str(v) for k, v in inputs.items()},
    "point_count_rendered": len(cloud),
    "object_count": len(objects),
    "room_floor_count": len(room_files),
    "trajectory_count": len(trajectory),
    "bounds_min": lo.tolist(),
    "bounds_max": hi.tolist(),
    "coordinates_unchanged": True,
    "config": str(config_path),
}
(out / "source_manifest.json").write_text(json.dumps(metadata, indent=2))
print("SCENE_READY", json.dumps({k: v for k, v in metadata.items() if k != "source"}), flush=True)

if args.mode == "preview":
    for frame in (1, 105, 180, 270):
        scene.frame_set(frame)
        scene.render.filepath = str(preview_dir / f"frame_{frame:03d}.png")
        bpy.ops.render.render(write_still=True)
else:
    frames = out / "frames"
    frames.mkdir(exist_ok=True)
    scene.render.filepath = str(frames / "frame_")
    scene.render.image_settings.file_format = "PNG"
    bpy.ops.render.render(animation=True)
