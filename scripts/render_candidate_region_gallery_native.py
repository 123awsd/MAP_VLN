#!/usr/bin/env python3
"""Render semantic candidate viewpoints as native Habitat 3-D geometry."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import habitat_sim
import magnum as mn
import numpy as np
from PIL import Image

S_H2F = np.asarray([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
F0 = np.asarray([0., 0., 1.])
# The visible meshes in the decorative drone GLB are offset from the GLB
# root.  This is the combined mesh-bounds centre in local model units; use it
# only to make the model's visual centre sit on the same world-space point as
# the frustum apex.
DRONE_VISUAL_CENTER_LOCAL = np.asarray([0.00007, -0.05541, -0.13408])
TASKS = (
    "check_microwave_kitchen_l2", "check_coffee_table_living_l2",
    "check_bag_under_bed_l1", "check_tv_living_l3",
)
# Per-task framing presets.  All panels remain native Habitat renders; these
# only choose a safe same-floor camera side so the RGB context and the full
# candidate fan are both visible.  The sofa entry reproduces the supplied
# native reference image.
CAMERA_PROFILES = {
    "check_microwave_kitchen_l2": {"anchor": "first", "pullback": 1.25,
                                    "height": "agent", "target": "candidates",
                                    "target_y_offset": -1.40},
    "check_coffee_table_living_l2": {"anchor": "median", "pullback": 2.35,
                                      "height": "reference", "target": "sensor"},
    "check_bag_under_bed_l1": {"anchor": "index3", "pullback": 1.00,
                                "height": "floor_mid", "target": "sensor",
                                "camera_xz": [2.0, -7.0],
                                "target_y_offset": -1.50},
    "check_tv_living_l3": {"anchor": "median", "pullback": 1.75,
                            "height": "floor_mid", "target": "sensor",
                            "target_y_offset": -0.65},
}
COLORS = {
    "support_surface": mn.Color4(0.96, 0.42, 0.08, 1.0),
    # A neutral slate reads cleanly over the RGB render and is suitable for a
    # publication figure without relying on several competing hues.
    "surrounding_region": mn.Color4(0.22, 0.27, 0.30, 1.0),
    "below_region": mn.Color4(0.06, 0.70, 0.38, 1.0),
    "instance_region": mn.Color4(0.04, 0.48, 0.94, 1.0),
}
BOX_EDGES = ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7))
# A small near rectangle and a larger far rectangle make the 3-D frustum
# unmistakable in a perspective render.  Point 0 is the actual camera/UAV
# origin, points 1--4 are the near rectangle, and points 5--8 are the far
# rectangle.  Keeping the apex explicit prevents accidentally using one of
# the near-plane corners as the UAV position.
FRUSTUM_EDGES = ((1,2),(2,3),(3,4),(4,1),
                 (5,6),(6,7),(7,8),(8,5),
                 (1,5),(2,6),(3,7),(4,8),
                 (0,1),(0,2),(0,3),(0,4))


def f2h(points, generated):
    points = np.asarray(points, float).reshape(-1, 3)
    initial = np.asarray(generated["initial_agent_habitat_xyz"], float)
    return initial + (S_H2F.T @ (points - F0).T).T


def object_f2h(points, generated):
    points = np.asarray(points, float).reshape(-1, 3)
    initial = np.asarray(generated["initial_sensor_habitat_xyz"], float)
    return initial + (S_H2F.T @ (points - F0).T).T


def oriented_box(obj):
    center, size = np.asarray(obj["center_xyz_m"], float), np.asarray(obj["size_xyz_m"], float)
    w, x, y, z = map(float, obj.get("orientation_wxyz", [1,0,0,0]))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    rot = np.asarray([[math.cos(yaw),-math.sin(yaw),0],
                      [math.sin(yaw), math.cos(yaw),0],[0,0,1]])
    signs = ((-1,-1,-1),(-1,-1,1),(-1,1,1),(-1,1,-1),
             (1,-1,-1),(1,-1,1),(1,1,1),(1,1,-1))
    return np.asarray([center + rot @ (.5 * size * s) for s in signs])


def scaled_box(points, center, scale):
    """Shrink an annotation box around its semantic object center."""
    points = np.asarray(points, float)
    center = np.asarray(center, float).reshape(1, 3)
    return center + float(scale) * (points - center)


def frustum(pose, near=.05, depth=.34, hfov=48., aspect=1.35):
    origin = np.asarray([pose[k] for k in ("x","y","z")], float)
    yaw = float(pose["yaw"])
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.])
    left = np.asarray([-math.sin(yaw), math.cos(yaw), 0.])
    up = np.asarray([0., 0., 1.])
    rectangles = []
    for distance in (near, depth):
        half_w = distance * math.tan(math.radians(hfov) / 2.)
        half_h = half_w / aspect
        center = origin + distance * forward
        rectangles.append([center + s*half_w*left + v*half_h*up
                           for s,v in ((1,1),(-1,1),(-1,-1),(1,-1))])
    return np.vstack([origin, *rectangles])


def diverse(values, maximum=8):
    if len(values) <= maximum:
        return values
    ordered = sorted(values, key=lambda x: float(x.get("terminal_cost", 0.)))
    chosen = [ordered[0]]
    while len(chosen) < maximum:
        rest = [x for x in ordered if x not in chosen]
        chosen.append(max(rest, key=lambda x: min(abs(math.atan2(
            math.sin(float(x["candidate_angle_rad"])-float(y["candidate_angle_rad"])),
            math.cos(float(x["candidate_angle_rad"])-float(y["candidate_angle_rad"]))))
            for y in chosen)))
    return chosen


def navmesh_camera(sim, target_h, poses_h, box_h, task_id=None):
    """Choose a third-person camera on the target floor's navmesh.

    The saved scene graph and candidates are in the FALCON/agent frame.  Snapping
    the candidate points themselves gives a reliable floor estimate even when a
    target object is near a stairwell (where snapping the object can select a
    different floor).
    """
    floor_samples = []
    for pose in poses_h:
        agent_point = np.asarray(pose, float)
        snapped = sim.pathfinder.snap_point(mn.Vector3(*agent_point))
        snapped = np.asarray(snapped, dtype=float)
        if np.all(np.isfinite(snapped)):
            floor_samples.append(snapped)
    if not floor_samples:
        raise RuntimeError("no navmesh floor near candidate poses")
    floor_samples = np.asarray(floor_samples)
    # The candidates for one task are floor-consistent.  Use the median level
    # and reject snaps that jumped to another storey.
    floor_y = float(np.median(floor_samples[:, 1]))
    same_floor = floor_samples[np.abs(floor_samples[:, 1] - floor_y) < 0.35]
    if len(same_floor) == 0:
        raise RuntimeError("candidate poses do not share a navigable floor")
    # Start from the task box, not from a fixed floor offset.  The best
    # candidate supplies a natural viewing direction; the distance scales with
    # the box footprint so a microwave and a bed receive sensible framing.
    target_h = np.asarray(target_h, dtype=float)
    box_h = np.asarray(box_h, dtype=float)
    profile = CAMERA_PROFILES.get(task_id, {"anchor": "first", "pullback": 1.0,
                                            "height": "agent", "target": "agent"})
    # The sofa reference image supplied for the paper was rendered with the
    # median candidate as the camera-side anchor and a longer pullback.  Keep
    # that native Habitat view exactly for the surrounding-region panel; the
    # other task families use the closest/best candidate so their object and
    # candidate fan stay readable.
    reference_view = profile.get("height") == "reference"
    anchor_spec = profile.get("anchor", "first")
    if anchor_spec == "median":
        anchor = same_floor[len(same_floor) // 2].copy()
    elif isinstance(anchor_spec, str) and anchor_spec.startswith("index"):
        try:
            anchor = same_floor[int(anchor_spec[5:]) % len(same_floor)].copy()
        except (TypeError, ValueError):
            anchor = same_floor[0].copy()
    else:
        anchor = same_floor[0].copy()
    direction = anchor[[0, 2]] - target_h[[0, 2]]
    if np.linalg.norm(direction) < 1e-3:
        direction = np.asarray([1.0, 0.0])
    direction /= max(np.linalg.norm(direction), 1e-6)
    footprint = float(np.linalg.norm(np.ptp(box_h[:, [0, 2]], axis=0)))
    # The sofa reference used 2.35 m plus a small object-size term; the close
    # third-person views use their task-specific base distance and a gentler
    # size term so all candidate frustums stay inside the image.
    pullback = float(profile.get("pullback", 1.0))
    pullback += (0.20 if reference_view else 0.10) * footprint
    desired = anchor.copy()
    desired[[0, 2]] += pullback * direction
    # Project the desired point onto the same-floor navmesh.  If the ideal
    # point is outside a room, use the nearest candidate-derived nav point on
    # that floor instead of silently crossing a wall or storey.
    absolute_xz = profile.get("camera_xz")
    if absolute_xz is not None and len(absolute_xz) == 2:
        desired[[0, 2]] = np.asarray(absolute_xz, dtype=float)
    desired_floor = mn.Vector3(float(desired[0]), floor_y, float(desired[2]))
    projected = np.asarray(sim.pathfinder.snap_point(desired_floor), dtype=float)
    # A few rooms in the HM3D navmesh have a small ramp/mesh height offset;
    # explicit camera presets may use those neighbouring walkable samples,
    # while the automatically selected candidate floor remains strict.
    floor_tolerance = 0.55 if absolute_xz is not None else 0.35
    if abs(float(projected[1]) - floor_y) >= floor_tolerance:
        projected = same_floor[0]
    camera = projected.copy()
    lateral = float(profile.get("lateral", 0.0))
    if abs(lateral) > 1e-6:
        # Positive lateral motion is camera-left in the final image.  Snap the
        # shifted point again so the camera remains on the same floor.
        left_vec = np.asarray([direction[1], -direction[0]])
        shifted = camera.copy()
        shifted[[0, 2]] += lateral * left_vec
        shifted_floor = mn.Vector3(float(shifted[0]), floor_y, float(shifted[2]))
        shifted = np.asarray(sim.pathfinder.snap_point(shifted_floor), dtype=float)
        if abs(float(shifted[1]) - floor_y) < floor_tolerance:
            camera = shifted
    # The saved scene graph and candidate pool share the FALCON/agent origin.
    # Keep the camera in that same frame and use the original third-person
    # clearance that produced the validated RGB reference views.
    # The reference sofa view sits 1.25 m above its floor.  For the other
    # panels, retain the closer agent-relative height that keeps small objects
    # (microwave/TV/bag) in frame.  (The candidate array used for this helper
    # is in the agent frame; the drawn UAV/frustum geometry is separately
    # lifted to the RGB sensor frame.)
    height_mode = profile.get("height", "agent")
    if height_mode in ("reference", "floor", "floor_mid", "floor_low"):
        height_offset = {"reference": 1.25, "floor": 1.05,
                         "floor_mid": 0.75, "floor_low": 0.60}[height_mode]
        camera[1] = float(floor_y) + height_offset
    else:
        camera[1] = float(poses_h[0, 1]) + 0.75
    return camera, floor_y


def write_tube_mesh(path, segments, color, radius, sides=8):
    vertices, faces = [], []
    for start, end in segments:
        start, end = np.asarray(start, float), np.asarray(end, float)
        axis = end - start
        axis /= max(np.linalg.norm(axis), 1e-9)
        helper = np.asarray([0., 1., 0.]) if abs(axis[1]) < .9 else np.asarray([1., 0., 0.])
        u = np.cross(axis, helper); u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        base = len(vertices)
        for point in (start, end):
            for i in range(sides):
                angle = 2.*math.pi*i/sides
                vertices.append(point + radius*(math.cos(angle)*u + math.sin(angle)*v))
        for i in range(sides):
            j = (i+1) % sides
            outward = ((base+i, base+j, base+sides+j),
                       (base+i, base+sides+j, base+sides+i))
            faces.extend(outward)
            faces.extend(tuple(reversed(face)) for face in outward)
    mesh_center = np.mean(np.asarray(vertices), axis=0)
    vertices = [point - mesh_center for point in vertices]
    mtl = path.with_suffix(".mtl")
    rgb = [float(color[i]) for i in range(3)]
    mtl.write_text(f"newmtl tubes\nKd {rgb[0]} {rgb[1]} {rgb[2]}\nKa 0 0 0\nKs 0.1 0.1 0.1\n")
    lines = [f"mtllib {mtl.name}", "usemtl tubes"]
    lines.extend(f"v {p[0]} {p[1]} {p[2]}" for p in vertices)
    lines.extend(f"f {a+1} {b+1} {c+1}" for a,b,c in faces)
    path.write_text("\n".join(lines)+"\n")
    return mesh_center


def add_mesh(sim, obj_path, handle, translation, scale=1.0):
    attributes = habitat_sim.attributes.ObjectAttributes()
    attributes.render_asset_handle = str(obj_path.resolve())
    attributes.collision_asset_handle = str(obj_path.resolve())
    attributes.is_collidable = False
    attributes.compute_COM_from_shape = False
    attributes.shader_type = "flat"
    attributes.scale = mn.Vector3(float(scale), float(scale), float(scale))
    template_id = sim.get_object_template_manager().register_template(attributes, handle)
    obj = sim.get_rigid_object_manager().add_object_by_template_id(template_id)
    obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
    obj.translation = mn.Vector3(*translation)
    visual_bounds = [str(node.cumulative_bb) for node in obj.visual_scene_nodes]
    print(f"native_asset handle={handle} template={template_id} object={obj.object_id} "
          f"visual_nodes={len(obj.visual_scene_nodes)} translation={obj.translation} "
          f"bounds={visual_bounds}", flush=True)
    return obj


def add_native_cylinders(sim, segments, prefix, radius):
    asset_manager = sim.get_asset_template_manager()
    handles = asset_manager.get_template_handles("cylinderSolid")
    if not handles:
        handles = asset_manager.get_template_handles("cylinder")
    if not handles:
        raise RuntimeError("Habitat has no built-in cylinder primitive")
    primitive = handles[0]
    template_manager = sim.get_object_template_manager()
    object_manager = sim.get_rigid_object_manager()
    for index, (start, end) in enumerate(segments):
        start, end = np.asarray(start, float), np.asarray(end, float)
        delta = end - start
        length = float(np.linalg.norm(delta))
        direction = delta / max(length, 1e-9)
        attributes = habitat_sim.attributes.ObjectAttributes()
        attributes.render_asset_handle = primitive
        attributes.collision_asset_handle = primitive
        attributes.scale = mn.Vector3(radius, length * .5, radius)
        attributes.is_collidable = False
        attributes.shader_type = "flat"
        template_id = template_manager.register_template(attributes, f"{prefix}_{index}")
        obj = object_manager.add_object_by_template_id(template_id)
        obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        obj.translation = mn.Vector3(*(0.5 * (start + end)))
        y_axis = np.asarray([0., 1., 0.])
        cosine = float(np.clip(np.dot(y_axis, direction), -1., 1.))
        if cosine < 1. - 1e-8:
            axis = np.cross(y_axis, direction)
            if np.linalg.norm(axis) < 1e-8:
                axis = np.asarray([1., 0., 0.])
            axis /= np.linalg.norm(axis)
            obj.rotation = mn.Quaternion.rotation(mn.Rad(math.acos(cosine)), mn.Vector3(*axis))


def render_panel(task_id, values, target_obj, anchor_obj, generated, scene, scene_config,
                 output_dir, panel_index, width, height, hfov,
                 show_target_box=True, show_drone_model=False,
                 drone_model=None, drone_scale=0.035, drone_backoff=0.08):
    # A below-region task may contain hypotheses around several beds.  A panel
    # is meant to explain candidate generation for one target instance, so keep
    # only the hypothesis associated with the best candidate.
    best = min(values, key=lambda x: float(x.get("terminal_cost", 0.)))
    hypothesis = best.get("location_hypothesis_id")
    if hypothesis is not None:
        values = [x for x in values if x.get("location_hypothesis_id") == hypothesis]
    candidates = diverse(values)
    target = np.asarray(target_obj["center_xyz_m"], float)
    poses = np.asarray([[c["pose"][k] for k in ("x","y","z")] for c in candidates])
    look_h = f2h([target], generated)[0]

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(scene)
    sim_cfg.scene_dataset_config_file = str(scene_config)
    sim_cfg.gpu_device_id = 0
    sim_cfg.enable_physics = True
    sensor = habitat_sim.CameraSensorSpec()
    sensor.uuid = "native_gallery_rgb"
    sensor.sensor_type = habitat_sim.SensorType.COLOR
    sensor.resolution = [height, width]
    sensor.hfov = hfov
    sensor.clear_color = mn.Color4(1., 1., 1., 1.)
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor]
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        sim.initialize_agent(0)
        family_color = COLORS[candidates[0]["region_type"]]
        # Keep the task/anchor boxes neutral, but use one high-contrast colour
        # for all candidate frustums so they remain legible over both the red
        # sofa and the light wood floor without introducing per-view colours.
        frustum_color = mn.Color4(1.0, 0.50, 0.02, 1.0)
        # Keep the figure in one restrained color family.  Per-candidate
        # debug colors are useful during development but distract in a paper.
        anchor_color = mn.Color4(0.52, 0.55, 0.60, 0.85)
        target_segments, anchor_segments = [], []
        frustum_segments = []
        candidate_origins_h = []
        # Keep the camera in the validated agent frame, but place the 3-D
        # annotation geometry at the RGB/depth sensor origin where the object
        # was measured.  Keep the target geometry available for camera framing
        # even when its visual box is intentionally hidden in the paper view.
        box_h = object_f2h(oriented_box(target_obj), generated)
        if show_target_box:
            target_segments.extend((box_h[a], box_h[b]) for a,b in BOX_EDGES)
        # The target (coffee table) is what the task asks us to verify.  The
        # sofa is the spatial reference used to generate a surrounding-region
        # candidate set, so show it as a quiet secondary outline.
        if anchor_obj is not None and anchor_obj.get("id") != target_obj.get("id"):
            # The anchor is contextual rather than the verification target;
            # keep its outline slightly inset so it does not dominate the RGB
            # scene or compete with the candidate frustums.
            anchor_box = scaled_box(
                oriented_box(anchor_obj), anchor_obj["center_xyz_m"], 0.86
            )
            anchor_h = object_f2h(anchor_box, generated)
            anchor_segments.extend((anchor_h[a], anchor_h[b]) for a,b in BOX_EDGES)
        # Candidate poses are sensor-frame positions (the stage-2 executor
        # places the agent one metre below its RGB/depth sensor).  Keep their
        # native Habitat elevation so the frustums sit at the true viewpoints.
        poses_h = f2h(poses, generated)
        sensor_offset_h = (np.asarray(generated["initial_sensor_habitat_xyz"], float)
                           - np.asarray(generated["initial_agent_habitat_xyz"], float))
        candidate_mean_h = poses_h.mean(axis=0) + sensor_offset_h
        # The native annotation geometry remains in the sensor frame above.
        # The supplied sofa reference aims at the sensor-frame target; the
        # other close third-person views use the agent-frame target, matching
        # the original RGB framing while leaving all overlays as native 3-D
        # world-space geometry.
        profile = CAMERA_PROFILES.get(task_id, {"target": "agent"})
        if profile.get("target") == "sensor":
            camera_target_h = look_h
        elif profile.get("target") == "combined":
            camera_target_h = 0.5 * (look_h + candidate_mean_h)
        elif profile.get("target") == "candidates":
            camera_target_h = candidate_mean_h
        else:
            camera_target_h = f2h([target], generated)[0]
        camera_target_h = np.asarray(camera_target_h, float).copy()
        camera_target_h[1] += float(profile.get("target_y_offset", 0.0))
        target_h = camera_target_h
        # Constrain the camera to a navigable point on the same floor.  This is
        # deliberately computed after the simulator loads its navmesh so a
        # camera can never be placed outside the building or between floors.
        camera_h, floor_y = navmesh_camera(sim, target_h, poses_h, box_h, task_id)
        print(f"{task_id}: camera_h={camera_h.round(3).tolist()} floor_y={floor_y:.3f}", flush=True)
        for candidate in candidates:
            points_h = object_f2h(frustum(candidate["pose"]), generated)
            frustum_segments.extend((points_h[a], points_h[b]) for a,b in FRUSTUM_EDGES)
            candidate_origins_h.append(points_h[0])
        if show_drone_model:
            if drone_model is None or not Path(drone_model).is_file():
                raise FileNotFoundError(f"drone model not found: {drone_model}")
            # The project GLB's nose points along its local -Z axis.  The
            # Falcon-to-Habitat transform maps that axis to the frustum's
            # forward direction, so applying the usual +Z-to--Z correction
            # would turn every drone 180 degrees away from its frustum.
            for index, (candidate, origin_h) in enumerate(
                zip(candidates, candidate_origins_h)
            ):
                drone = add_mesh(
                    sim, Path(drone_model), f"candidate_drone_{index}",
                    origin_h, scale=drone_scale,
                )
                drone.rotation = mn.Quaternion.rotation(
                    mn.Rad(float(candidate["pose"]["yaw"])), mn.Vector3.y_axis()
                )
                # Put the visible body centre (rather than the GLB root) on
                # the exact same world-space point as the frustum apex.
                visual_offset_h = np.asarray(
                    drone.rotation.transform_vector(
                        mn.Vector3(*(float(drone_scale) * DRONE_VISUAL_CENTER_LOCAL))
                    ), dtype=float
                )
                # Pull the decorative body slightly behind the camera origin
                # along the reverse viewing direction.  The frustum itself
                # remains anchored at the exact candidate pose.
                yaw = float(candidate["pose"]["yaw"])
                backward_f = -np.asarray([math.cos(yaw), math.sin(yaw), 0.])
                backward_h = S_H2F.T @ backward_f
                drone.translation = mn.Vector3(*(origin_h - visual_offset_h
                                                 + float(drone_backoff) * backward_h))
        node = sim.agents[0].scene_node
        node.translation = mn.Vector3(*camera_h)
        node.rotation = mn.Quaternion.from_matrix(mn.Matrix4.look_at(
            mn.Vector3(*camera_h), mn.Vector3(*camera_target_h), mn.Vector3(0.,1.,0.)
        ).rotation())
        debug = sim.get_debug_line_render()
        # A thin wireframe is legible at paper scale and avoids the starburst
        # effect caused by overlapping solid cone primitives.  The far
        # rectangle plus its four depth edges is still a native 3-D frustum.
        debug.set_line_width(1.7)
        for start, end in anchor_segments:
            debug.draw_transformed_line(mn.Vector3(*start), mn.Vector3(*end), anchor_color)
        if target_segments:
            debug.set_line_width(3.0)
            for start, end in target_segments:
                debug.draw_transformed_line(mn.Vector3(*start), mn.Vector3(*end), family_color)
        # One consistent slate stroke keeps all candidate poses in the same
        # visual language while remaining readable on the bright window
        # background.
        debug.set_line_width(2.2)
        for start, end in frustum_segments:
            debug.draw_transformed_line(mn.Vector3(*start), mn.Vector3(*end), frustum_color)
        sim.step_physics(1.0 / 60.0)
        rgba = np.asarray(sim.get_sensor_observations()["native_gallery_rgb"])
        return Image.fromarray(np.ascontiguousarray(rgba[..., :3])), candidates


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--candidates", type=Path, required=True)
    ap.add_argument("--generated-config", type=Path, required=True)
    ap.add_argument("--scene-graph", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--width", type=int, default=900)
    ap.add_argument("--height", type=int, default=650)
    ap.add_argument("--hfov", type=float, default=76.)
    target_box = ap.add_mutually_exclusive_group()
    target_box.add_argument("--show-target-box", dest="show_target_box", action="store_true",
                            help="draw the verification target box (default)")
    target_box.add_argument("--hide-target-box", dest="show_target_box", action="store_false",
                            help="hide the verification target box")
    ap.set_defaults(show_target_box=True)
    ap.add_argument("--show-drone-model", action="store_true",
                    help="place a small native drone model at every candidate pose")
    ap.add_argument("--drone-model", type=Path,
                    default=Path(__file__).resolve().parents[1] / "assets" / "drone" / "drone_surface_band_emissive.glb")
    ap.add_argument("--drone-scale", type=float, default=0.035)
    ap.add_argument("--drone-backoff", type=float, default=0.08,
                    help="move the decorative drone backwards from the frustum apex (metres)")
    ap.add_argument("--task", choices=TASKS, help="render one task panel only")
    args = ap.parse_args()
    pool = json.loads(args.candidates.read_text())["by_task"]
    generated = json.loads(args.generated_config.read_text())
    objects = {x["id"]: x for x in json.loads(args.scene_graph.read_text())["objects"]}
    scene = Path(generated["scene"]).resolve()
    scene_config = Path(generated["scene_config"]).resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    panels, report = [], []
    tasks = (args.task,) if args.task else TASKS
    for i, task in enumerate(tasks):
        target_obj = objects[pool[task][0]["object_id"]]
        anchor_obj = objects.get(pool[task][0].get("anchor_object_id"))
        panel, shown = render_panel(task, pool[task], target_obj, anchor_obj,
                                    generated, scene,
                                    scene_config, args.output_dir, i+1,
                                    args.width, args.height, args.hfov,
                                    show_target_box=args.show_target_box,
                                    show_drone_model=args.show_drone_model,
                                    drone_model=args.drone_model,
                                    drone_scale=args.drone_scale,
                                    drone_backoff=args.drone_backoff)
        path = args.output_dir / f"{i+1}_{shown[0]['region_type']}_native.png"
        panel.save(path)
        panels.append(panel)
        report.append({"task_id":task,"region_type":shown[0]["region_type"],
                       "candidate_count_shown":len(shown),"image":str(path.resolve())})
    gap = 20
    cols = 1 if len(panels) == 1 else 2
    rows = int(math.ceil(len(panels) / cols))
    gallery = Image.new("RGB", (cols*args.width+(cols-1)*gap,
                                 rows*args.height+(rows-1)*gap), "white")
    for i, panel in enumerate(panels):
        gallery.paste(panel, ((i%cols)*(args.width+gap),(i//cols)*(args.height+gap)))
    gallery_path = args.output_dir / "candidate_region_gallery_native_2x2.png"
    gallery.save(gallery_path)
    (args.output_dir/"native_manifest.json").write_text(json.dumps({
        "format":"pre_map_vln.native_candidate_gallery.v1",
        "geometry":"Habitat native world-space 3-D debug lines", "panels":report,
        "gallery":str(gallery_path.resolve())}, indent=2)+"\n")
    print(gallery_path)


if __name__ == "__main__":
    main()
