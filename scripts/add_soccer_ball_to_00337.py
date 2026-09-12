#!/usr/bin/env python3
"""Create a derived 00337 GLB with a CC-BY soccer ball under the dining table."""
import argparse
import sys
import math
from pathlib import Path
import bpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--ball', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--x', type=float, default=3.78)
    ap.add_argument('--y', type=float, default=10.30)
    ap.add_argument('--z', type=float, default=2.47)
    ap.add_argument('--diameter', type=float, default=0.22)
    ap.add_argument('--rot-x-deg', type=float, default=0.0)
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else sys.argv[1:]
    args = ap.parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=str(args.source))
    scene_objs = set(bpy.context.scene.objects)
    bpy.ops.import_scene.gltf(filepath=str(args.ball))
    # Some downloaded props contain a helper/collision cube; do not merge it
    # into the visible scene asset.
    ball_objs = [o for o in bpy.context.scene.objects
                 if o not in scene_objs and o.name.lower() not in {'cube', 'collision'}]
    if not ball_objs:
        raise RuntimeError('soccer ball import produced no objects')
    # Parent all imported ball pieces so the asset can be positioned/scaled as one.
    root = bpy.data.objects.new('soccer_ball_under_table', None)
    bpy.context.collection.objects.link(root)
    for obj in ball_objs:
        obj.parent = root
    root.location = (args.x, args.y, args.z)
    root.rotation_euler[0] = math.radians(args.rot_x_deg)
    max_dim = max(max(float(obj.dimensions[i]) for obj in ball_objs) for i in range(3))
    if max_dim <= 0:
        raise RuntimeError(f'invalid imported ball dimensions: {tuple(root.dimensions)}')
    root.scale = (args.diameter / max_dim,) * 3
    # Preserve the source scene and export a self-contained binary glTF.
    bpy.ops.object.select_all(action='DESELECT')
    root.select_set(True)
    bpy.context.view_layer.objects.active = root
    bpy.ops.export_scene.gltf(
        filepath=str(args.output), export_format='GLB',
        use_selection=False, export_materials='EXPORT',
        export_cameras=False, export_lights=False,
    )
    print(f'exported {args.output} ball_location=({args.x:.3f},{args.y:.3f},{args.z:.3f}) diameter={args.diameter:.3f}')


if __name__ == '__main__':
    main()
