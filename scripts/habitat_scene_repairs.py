"""Optional, explicit scene overlays and geometry checks for Habitat previews."""
import itertools
import hashlib
import json
from pathlib import Path

import habitat_sim
import numpy as np


def load_scene_repair(sim, config_path, scene_path):
    config_path = Path(config_path).resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config["scene_filename"] != Path(scene_path).name:
        raise ValueError("Scene repair belongs to another scene")
    if config.get("source_glb_sha256"):
        digest = hashlib.sha256(Path(scene_path).read_bytes()).hexdigest()
        if digest != config["source_glb_sha256"]:
            raise ValueError("Scene geometry changed; repair bounds need revalidation")
    manager = sim.get_object_template_manager()
    objects = []
    for item in config["objects"]:
        mesh = (config_path.parent / item["mesh"]).resolve()
        if not mesh.is_file():
            raise FileNotFoundError(mesh)
        attributes = habitat_sim.attributes.ObjectAttributes()
        attributes.render_asset_handle = str(mesh)
        attributes.collision_asset_handle = str(mesh)
        attributes.is_collidable = True
        attributes.compute_COM_from_shape = False
        attributes.shader_type = "flat"
        template = manager.register_template(attributes, "scene_repair_" + item["name"])
        obj = sim.get_rigid_object_manager().add_object_by_template_id(template)
        obj.motion_type = habitat_sim.physics.MotionType.STATIC
        objects.append(obj)
    print(f"scene_repair={config_path} static_objects={len(objects)}", flush=True)
    return objects


def mesh_safe_fraction(sim, start, end, radius=0.12, margin=0.015, contacts=None):
    """Check a sampled swept sphere at the planner/camera centre, not a walking navmesh.

    This is a simulation execution check. It neither selects targets nor replaces
    the planner's trajectory with a different route. A rejected step stays at the
    last reachable point and reports its trajectory ID to the existing bridge.
    """
    start, end = np.asarray(start), np.asarray(end)
    delta = end - start
    distance = float(np.linalg.norm(delta))
    if distance < 1e-6:
        return 1.0
    direction = delta / distance
    offsets = [np.zeros(3)]
    for xyz in itertools.product((-1, 0, 1), repeat=3):
        vector = np.asarray(xyz, dtype=float)
        if np.any(vector):
            offsets.append(vector / np.linalg.norm(vector) * radius)
    safe_distance = distance
    for offset in offsets:
        result = sim.cast_ray(
            habitat_sim.geo.Ray((start + offset).astype(np.float32), direction.astype(np.float32)),
            distance + margin,
            0.0,
        )
        for hit in result.hits:
            d = float(hit.ray_distance)
            if 0 <= d <= distance + margin:
                if contacts is not None:
                    contacts.append(np.asarray(hit.point, dtype=float).tolist())
                safe_distance = min(safe_distance, max(0.0, d - margin))
    return min(1.0, safe_distance / distance)
