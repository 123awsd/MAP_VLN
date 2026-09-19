"""Seal/verify the exact MINCO file and derive preview from its coefficients."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_trajectory(path):
    tokens = iter(Path(path).read_text().split())
    def expect(value):
        if next(tokens) != value:
            raise ValueError("bad MINCO record: " + value)
    def number():
        value = float(next(tokens))
        if not math.isfinite(value):
            raise ValueError("nonfinite MINCO value")
        return value
    expect("FULL_SMOOTH_TRAJECTORY_V1")
    expect("START")
    start = [number() for _ in range(4)]
    phases = []
    while True:
        token = next(tokens)
        if token == "END":
            break
        if token != "PHASE":
            raise ValueError("expected PHASE")
        dwell, yaw_start, yaw_end, yaw_dt = [number() for _ in range(4)]
        ny, npieces = int(next(tokens)), int(next(tokens))
        if dwell < 0 or yaw_dt <= 0 or not 2 <= ny <= 1000000 or not 1 <= npieces <= 100000:
            raise ValueError("invalid phase sizes")
        expect("YAW")
        yaw = [number() for _ in range(ny)]
        points, times, elapsed = [], [], 0.0
        for _ in range(npieces):
            expect("PIECE")
            duration, rows, cols = number(), int(next(tokens)), int(next(tokens))
            if duration <= 0 or rows != 3 or not 2 <= cols <= 16:
                raise ValueError("invalid piece")
            coeff = np.array([[number() for _ in range(cols)] for _ in range(rows)])
            ts = np.linspace(0, duration, max(2, int(math.ceil(duration / .02)) + 1))
            xyz = np.stack([np.polyval(row, ts) for row in coeff], axis=1)
            if not np.isfinite(xyz).all():
                raise ValueError("invalid evaluated trajectory")
            points.extend(xyz.tolist())
            times.extend((ts + elapsed).tolist())
            elapsed += duration
        expect("END_PHASE")
        if abs((ny - 1) * yaw_dt - elapsed) > 1e-5:
            raise ValueError("yaw timing does not match polynomial")
        phases.append(dict(points=points, times=times, duration=elapsed, dwell=dwell,
                           yaw=yaw, yaw_dt=yaw_dt, start_yaw=yaw_start, target_yaw=yaw_end))
    if next(tokens, None) is not None or not phases:
        raise ValueError("trailing data or empty trajectory")
    if np.linalg.norm(np.array(phases[0]["points"][0]) - start[:3]) > 1e-5:
        raise ValueError("START differs from polynomial")
    for first, second in zip(phases, phases[1:]):
        if np.linalg.norm(np.array(first["points"][-1])-second["points"][0]) > 1e-5:
            raise ValueError("discontinuous phases")
    return start, phases


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["seal", "verify"])
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--map", type=Path, required=True)
    p.add_argument("--mission", type=Path)
    p.add_argument("--clearance", type=float, default=.25)
    args = p.parse_args()
    d = args.directory.resolve()
    names = ["final_minco.txt", "collision.pcd", "planner.yaml", "full_smooth_route.txt", "execution_bundle.json"]
    manifest_path = d / "final_minco_manifest.json"
    if args.mode == "seal":
        start, phases = read_trajectory(d / "final_minco.txt")
        bundle = json.loads((d / "execution_bundle.json").read_text())
        if bundle["map_sha256"] != digest(args.map):
            raise ValueError("map/bundle mismatch")
        if not bundle.get("safety", {}).get("start_pose_explicitly_approved"):
            raise ValueError("explicit hover start approval required")
        if np.linalg.norm(np.array(start[:3])-bundle["start_xyz_yaw"][:3]) > 1e-5:
            raise ValueError("saved start/bundle mismatch")
        preview = json.loads(args.mission.read_text())
        if len(preview["segments"]) != len(phases):
            raise ValueError("MINCO observation phases differ from mission")
        for segment, phase in zip(preview["segments"], phases):
            segment["points_xyz_m"] = phase["points"]
            segment["validated_trajectory"] = dict(points_xyz_m=phase["points"],
                times_sec=phase["times"], source="saved_MINCO_coefficients")
        preview["trajectory_source"] = "final_minco.txt (exact coefficients sampled at 20 ms)"
        (d / "final_minco_preview.json").write_text(json.dumps(preview, ensure_ascii=False))
        names.append("final_minco_preview.json")
        manifest = dict(format="pre_map_vln.saved_minco.v1", map_sha256=digest(args.map),
            files={name: digest(d / name) for name in names}, start_xyz_yaw=start,
            collision_clearance=args.clearance, phases=len(phases),
            duration_s=sum(x["duration"] + x["dwell"] for x in phases))
        temp = manifest_path.with_suffix(".tmp")
        temp.write_text(json.dumps(manifest, indent=2)+"\n")
        temp.replace(manifest_path)
    else:
        manifest = json.loads(manifest_path.read_text())
        if manifest["format"] != "pre_map_vln.saved_minco.v1" or manifest["map_sha256"] != digest(args.map):
            raise ValueError("saved MINCO map mismatch")
        if not math.isfinite(manifest["collision_clearance"]) or manifest["collision_clearance"] <= 0:
            raise ValueError("invalid certified clearance")
        if not set(names).issubset(manifest["files"]):
            raise ValueError("missing mandatory artifact hashes")
        for name, expected in manifest["files"].items():
            if Path(name).name != name or digest(d / name) != expected:
                raise ValueError("artifact mismatch: " + name)
        read_trajectory(d / "final_minco.txt")
    print(json.dumps(dict(status="verified", phases=manifest["phases"],
                          duration_s=manifest["duration_s"])))


if __name__ == "__main__":
    main()
