#!/usr/bin/env python3
"""Render a color preview of a GLB from a camera aimed at a world-space target."""
import argparse
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def vec3(value):
    parts = [float(v) for v in value.split(',')]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('expected x,y,z')
    return parts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera', type=vec3, required=True)
    parser.add_argument('--target', type=vec3, required=True)
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=720)
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=str(args.scene.resolve()))

    camera_data = bpy.data.cameras.new('PreviewCamera')
    camera = bpy.data.objects.new('PreviewCamera', camera_data)
    bpy.context.collection.objects.link(camera)
    camera.location = args.camera
    camera.rotation_euler = (Vector(args.target) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera_data.lens = 34
    camera_data.sensor_width = 36

    scene = bpy.context.scene
    scene.camera = camera
    scene.render.engine = 'BLENDER_EEVEE'
    scene.eevee.use_gtao = True
    scene.eevee.gtao_distance = 3
    scene.eevee.gtao_factor = 1.4
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = str(args.output.resolve())
    scene.view_settings.look = 'AgX - Medium High Contrast'

    world = bpy.data.worlds.new('PreviewWorld')
    scene.world = world
    world.use_nodes = True
    background = world.node_tree.nodes['Background']
    background.inputs['Color'].default_value = (0.65, 0.70, 0.80, 1.0)
    background.inputs['Strength'].default_value = 0.8

    light_data = bpy.data.lights.new('PreviewKey', 'AREA')
    light = bpy.data.objects.new('PreviewKey', light_data)
    bpy.context.collection.objects.link(light)
    light.location = (3.0, 8.5, 5.5)
    light.rotation_euler = (Vector(args.target) - light.location).to_track_quat('-Z', 'Y').to_euler()
    light_data.energy = 900
    light_data.shape = 'DISK'
    light_data.size = 4.0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.render.render(write_still=True)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
