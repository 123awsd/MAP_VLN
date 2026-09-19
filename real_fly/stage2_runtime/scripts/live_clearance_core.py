"""Read-only safety experiment. Geometry results are NOT flight authorization."""
import numpy as np


def cloud_xyz(msg, max_points=200000):
    """Zero-copy PointCloud2 view, including row padding and byte order."""
    n = int(msg.width) * int(msg.height)
    if not 0 < n <= max_points:
        raise ValueError("empty or oversized cloud")
    if msg.row_step < msg.width * msg.point_step:
        raise ValueError("invalid row_step")
    if len(msg.data) < msg.row_step * msg.height:
        raise ValueError("truncated cloud")
    fields = {f.name: f for f in msg.fields}
    formats, offsets = [], []
    for key in ("x", "y", "z"):
        f = fields.get(key)
        if f is None or f.count != 1 or f.datatype not in (7, 8):
            raise ValueError("XYZ must be scalar float32/float64")
        fmt = (">" if msg.is_bigendian else "<") + ("f4" if f.datatype == 7 else "f8")
        if f.offset < 0 or f.offset + np.dtype(fmt).itemsize > msg.point_step:
            raise ValueError("invalid XYZ offset")
        formats.append(fmt)
        offsets.append(f.offset)
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=formats,
                         offsets=offsets, itemsize=msg.point_step))
    view = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    return np.stack([view[k].ravel() for k in ("x", "y", "z")], axis=1)


def assess(points, position, velocity, clearance=0.22, reaction=0.30,
           deceleration=1.0, jerk=2.0):
    """Capsule around a straight smooth braking path, without point thinning.

    T bounds peak acceleration/jerk for v(u)=v0*(1-3u^2+2u^3).
    Neither braking capability nor tracking accuracy is established here.
    No ray tracing: absence of points does not establish observed free space.
    """
    p, v = np.asarray(position, dtype=float), np.asarray(velocity, dtype=float)
    params = np.array([clearance, reaction, deceleration, jerk])
    if (p.shape != (3,) or v.shape != (3,) or not np.isfinite(p).all()
            or not np.isfinite(v).all() or not np.isfinite(params).all()
            or clearance <= 0 or reaction < 0 or deceleration <= 0 or jerk <= 0):
        raise ValueError("invalid pose, velocity or limits")
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("invalid or empty cloud")
    if not np.isfinite(points).all():
        raise ValueError("nonfinite cloud: cannot certify input")
    speed = float(np.linalg.norm(v))
    duration = max(1.5 * speed / deceleration, np.sqrt(6 * speed / jerk), 0.2)
    delta = v * (reaction + duration / 2)
    length = float(np.linalg.norm(delta))
    relative = points - p
    squared = np.einsum("ij,ij->i", relative, relative)
    nearest = float(np.sqrt(squared.min()))
    # All points outside this sphere are mathematically outside the capsule.
    local = relative[squared <= (length + clearance) ** 2]
    segment_distance = None
    if len(local):
        u = np.clip(np.sum(local * delta, axis=1) / max(length ** 2, 1e-20), 0, 1)
        distance = np.linalg.norm(local - u[:, None] * delta, axis=1)
        segment_distance = float(distance.min())
    near = nearest <= clearance
    predicted = segment_distance is not None and segment_distance <= clearance
    return dict(status="WOULD_STOP" if near or predicted else "NO_HIT_IN_RECEIVED_POINTS",
                reason="near_obstacle" if near else "braking_capsule" if predicted else "none",
                nearest_m=nearest, braking_clearance_m=segment_distance,
                braking_length_m=length, braking_duration_s=duration,
                input_points=len(points), local_points=len(local))
