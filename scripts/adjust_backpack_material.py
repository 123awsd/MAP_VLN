#!/usr/bin/env python3
"""Create a lighter-brown Backpack GLB for readable Habitat RGB renders."""
import argparse
import sys
from pathlib import Path

import bpy


def set_base_color(material_name, rgba, roughness):
    material = bpy.data.materials.get(material_name)
    if material is None:
        raise RuntimeError(f'missing material: {material_name}')
    material.diffuse_color = rgba
    if material.use_nodes:
        principled = next(
            (node for node in material.node_tree.nodes if node.type == 'BSDF_PRINCIPLED'),
            None,
        )
        if principled is not None:
            principled.inputs['Base Color'].default_value = rgba
            principled.inputs['Roughness'].default_value = roughness


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=str(args.input.resolve()))
    helper = bpy.data.objects.get('Cube')
    if helper is not None:
        bpy.data.objects.remove(helper, do_unlink=True)

    # Moderate warm browns: visibly lighter in Habitat while remaining a
    # plausible leather backpack. Gold hardware is intentionally preserved.
    set_base_color('Brown', (0.34, 0.16, 0.075, 1.0), 0.72)
    set_base_color('DarkBrown', (0.19, 0.075, 0.032, 1.0), 0.78)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(args.output.resolve()), export_format='GLB',
        export_materials='EXPORT', export_cameras=False, export_lights=False,
    )
    print(args.output.resolve())


if __name__ == '__main__':
    main()
