"""Load visual-only runtime objects into Habitat without leaking them into the pre-map."""
from __future__ import annotations

import json
from pathlib import Path

import habitat_sim
import magnum as mn
import numpy as np


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
FALCON_SENSOR_ORIGIN = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)


def add_runtime_objects(sim, config_path: Path | None, initial_sensor_h) -> list:
    if config_path is None:
        return []
    config_path = Path(config_path).resolve()
    payload = json.loads(config_path.read_text(encoding="utf-8"))
    records = payload.get("objects", [])
    template_manager = sim.get_object_template_manager()
    object_manager = sim.get_rigid_object_manager()
    initial_sensor_h = np.asarray(initial_sensor_h, dtype=np.float64)
    loaded = []
    for index, record in enumerate(records):
        asset = Path(record["asset"])
        if not asset.is_absolute():
            asset = (config_path.parent / asset).resolve()
        if not asset.is_file():
            raise FileNotFoundError(asset)
        scale = float(record.get("scale", 1.0))
        attrs = habitat_sim.attributes.ObjectAttributes()
        attrs.render_asset_handle = str(asset)
        attrs.collision_asset_handle = str(asset)
        attrs.is_collidable = bool(record.get("collidable", False))
        attrs.scale = mn.Vector3(scale, scale, scale)
        handle = str(record.get("id", f"runtime_object_{index}"))
        template_id = template_manager.register_template(attrs, handle)
        obj = object_manager.add_object_by_template_id(template_id)
        obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        position_f = np.asarray(record["position_xyz_f"], dtype=np.float64)
        position_h = initial_sensor_h + S_HABITAT_TO_FALCON.T @ (
            position_f - FALCON_SENSOR_ORIGIN
        )
        obj.translation = mn.Vector3(*position_h)
        yaw = float(record.get("yaw_rad", 0.0))
        obj.rotation = mn.Quaternion.rotation(mn.Rad(yaw), mn.Vector3.y_axis())
        loaded.append(obj)
        print(
            f"runtime_object id={handle} asset={asset} position_f={position_f.tolist()} "
            f"position_h={position_h.tolist()} scale={scale}", flush=True,
        )
    return loaded
