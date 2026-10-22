#!/usr/bin/env python3
"""Compare measured observation velocity with acquisition and receipt clocks."""
import argparse
import json
from pathlib import Path
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('log', type=Path)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
rows = [json.loads(line) for line in a.log.read_text().splitlines()]
if len(rows) < 2:
    raise ValueError('At least two received frames required')
seq = np.array([r['sequence'] for r in rows])
position = np.array([r['position'] for r in rows])
velocity = np.array([r['velocity_world'] for r in rows])
mono = np.array([r['capture_monotonic_s'] for r in rows])
wall = np.array([r['capture_wall_time_ns'] for r in rows], dtype=np.int64)
receipt = np.array([r['receipt_ros_time_ns'] for r in rows], dtype=np.int64)
dt_mono = np.diff(mono)
dt_wall = np.diff(wall) * 1e-9
dt_receipt = np.diff(receipt) * 1e-9
if (dt_mono <= 0).any() or (dt_wall <= 0).any() or (dt_receipt <= 0).any():
    raise ValueError('Non-monotonic timestamps: cannot derive velocities')
# Velocity is between consecutive acquired frames; skipped frames are excluded.
consecutive = np.diff(seq) == 1
movement = np.diff(position, axis=0)
error_mono = np.linalg.norm(movement / dt_mono[:, None] - velocity[1:], axis=1)
error_wall = np.linalg.norm(movement / dt_wall[:, None] - velocity[1:], axis=1)
error_receipt = np.linalg.norm(movement / dt_receipt[:, None] - velocity[1:], axis=1)
def stats(x):
    return {'min': float(np.min(x)), 'median': float(np.median(x)), 'max': float(np.max(x))} if len(x) else None
report = {
    'received_frames': len(rows),
    'skipped_acquired_frames': int(np.maximum(np.diff(seq)-1, 0).sum()),
    'capture_to_receipt_delay_s': stats((receipt-wall)*1e-9),
    'acquisition_monotonic_interval_s': stats(dt_mono),
    'receipt_interval_s': stats(dt_receipt),
    'reported_speed_mps': stats(np.linalg.norm(velocity, axis=1)),
    'acquisition_derived_speed_mps': stats(np.linalg.norm(movement/dt_wall[:,None], axis=1)),
    'receipt_derived_speed_mps': stats(np.linalg.norm(movement/dt_receipt[:,None], axis=1)),
    'consecutive_velocity_error_monotonic_mps': stats(error_mono[consecutive]),
    'consecutive_velocity_error_wall_mps': stats(error_wall[consecutive]),
    'consecutive_velocity_error_receipt_mps': stats(error_receipt[consecutive]),
    'limitations': ['Host wall clock and ROS receipt clock must share the same epoch; this run uses normal wall time.', 'Speed agreement does not prove acceleration feasibility or coverage.'],
}
a.output.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
