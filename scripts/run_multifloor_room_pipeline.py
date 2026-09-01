#!/usr/bin/env python3
"""Run the reusable FALCON -> per-floor grid -> OccuSG preparation chain.

The floor estimate uses only the recorded FALCON trajectory and occupied PCD.
Habitat truth is intentionally not used by this preparation step.
"""
from __future__ import annotations

import argparse, csv, json, shutil, subprocess
from pathlib import Path
import numpy as np


def read_pcd_xyz(path: Path) -> np.ndarray:
    lines = path.read_bytes().splitlines()
    data = next(i for i, line in enumerate(lines) if line.startswith(b"DATA"))
    if lines[data].split()[1].lower() != b"ascii":
        raise ValueError("only ASCII PCD is supported by the generic offline pipeline")
    return np.loadtxt(lines[data + 1:], dtype=np.float32, usecols=(0, 1, 2))


def infer_levels(run_dir: Path, maximum: int) -> list[dict]:
    traj = []
    with (run_dir / "bag_export/trajectory.csv").open() as f:
        traj = [float(row["z"]) for row in csv.DictReader(f)]
    z = np.asarray(traj, dtype=np.float32)
    edges = np.arange(np.floor(z.min() / .1) * .1, np.ceil(z.max() / .1) * .1 + .15, .1)
    hist, _ = np.histogram(z, edges)
    smooth = np.convolve(hist, np.ones(5), mode="same")
    peaks = []
    for i in np.argsort(smooth)[::-1]:
        if smooth[i] < max(.01 * len(z), .12 * smooth.max()): break
        mode = float((edges[i] + edges[i + 1]) / 2)
        if all(abs(mode - old) >= 1.8 for old in peaks): peaks.append(mode)
    peaks.sort()
    points = read_pcd_xyz(run_dir / "bag_export/map_occupied.pcd")
    pz = np.round(points[:, 2] / .1) * .1
    levels = []
    values, counts = np.unique(pz, return_counts=True)
    support_limit = .20 * counts.max()
    for mode in peaks:
        candidates = np.flatnonzero((values >= mode - 1.6) & (values <= mode - .35) & (counts >= support_limit))
        if len(candidates):
            idx = candidates[np.argmax(counts[candidates])]
            floor = float(values[idx])
            if all(abs(floor - item["floor_z_m"]) >= 1.5 for item in levels):
                levels.append({"floor": len(levels) + 1, "floor_z_m": floor, "flight_z_m": mode})

    # A UAV can cross a floor or inspect it briefly without producing a strong
    # trajectory histogram peak.  Recover such levels from horizontal support
    # in the already-built occupied map.  This remains online-map-only: no
    # Habitat mesh, navmesh, or truth floor metadata is consulted.
    if len(levels) < maximum:
        support_limit = .05 * counts.max()
        floor_support = []
        for i in range(1, len(values) - 1):
            if counts[i] < support_limit or counts[i] < counts[i - 1] or counts[i] < counts[i + 1]:
                continue
            floor_z = float(values[i])
            # A floor must be below an observed flight band; this removes the
            # upper ceiling/support surface from consideration.
            if floor_z > float(z.max()) - .8:
                continue
            if any(abs(floor_z - item["floor_z_m"]) < 1.5 for item in levels):
                continue
            # Require trajectory evidence above the candidate floor.  Furniture
            # tops without a corresponding higher flight band are rejected.
            upper = z[z >= floor_z + .6]
            if len(upper) < max(20, .002 * len(z)):
                continue
            flight_z = float(np.median(upper))
            floor_support.append((int(counts[i]), floor_z, flight_z))
        for _, floor_z, flight_z in sorted(floor_support, reverse=True):
            if all(abs(floor_z - item["floor_z_m"]) >= 1.5 for item in levels):
                levels.append({"floor": len(levels) + 1, "floor_z_m": floor_z, "flight_z_m": flight_z,
                               "detection_source": "trajectory+occupied_horizontal_support"})
                if len(levels) >= maximum:
                    break
        levels.sort(key=lambda item: item["floor_z_m"])
        for number, item in enumerate(levels, start=1):
            item["floor"] = number
    if not levels: raise RuntimeError("could not infer FALCON floor levels")
    return levels[:maximum]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("boxes", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("--max-floor", type=int, default=3)
    ap.add_argument("--policy", type=Path, default=None)
    ap.add_argument("--auto-policy", action="store_true",
                    help="use a conservative generic label policy when no Qwen policy is available")
    ap.add_argument("--resolution", type=float, default=.05)
    ap.add_argument(
        "--occusg-output-group", default="",
        help="optional path below outputs/occusg used to group one pipeline run",
    )
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    episode = args.run_dir / "episode"
    levels = infer_levels(args.run_dir, args.max_floor)
    args.output.mkdir(parents=True, exist_ok=True)
    all_rows = list(csv.DictReader(args.boxes.open(newline="", encoding="utf-8")))
    policy = args.policy
    if policy is None and args.auto_policy:
        structural = {"wall", "column", "pillar", "door", "door frame", "stairs", "stair", "stair railing", "stairs railing"}
        removable = {"bed", "sofa", "couch", "chair", "table", "desk", "cabinet", "dresser", "nightstand", "wardrobe", "television", "tv", "lamp", "refrigerator", "oven", "microwave", "sink", "toilet", "bathtub", "shower", "countertop", "kitchen island", "mattress", "pillow"}
        instances = []
        for i, row in enumerate(all_rows):
            label = row["name"].strip().lower()
            role = "structural_boundary" if label in structural else ("interior_object" if label in removable else "uncertain")
            instances.append({"instance_id": f"boxer_{i}", "label": label,
                "center_xyz_m": [float(row[k]) for k in ("tx_world_object", "ty_world_object", "tz_world_object")],
                "size_xyz_m": [float(row[k]) for k in ("scale_x", "scale_y", "scale_z")],
                "detection_confidence": float(row["prob"]), "role": role,
                "confidence": .90 if role != "uncertain" else .0,
                "remove_from_structure_map": role == "interior_object",
                "keep_in_navigation_map": True, "reason": "generic offline label policy"})
        policy = args.output / "structure_policy.json"
        policy.write_text(json.dumps({"format":"pre_map_vln.structure_policy.v1", "minimum_remove_confidence":.65, "instances":instances}, ensure_ascii=False, indent=2) + "\n")
    (args.output / "floors.json").write_text(json.dumps(levels, indent=2) + "\n")
    for item in levels:
        floor = item["floor"]
        floor_rows = [row for row in all_rows if item["floor_z_m"] - .25 <= float(row["tz_world_object"]) <= item["floor_z_m"] + 1.85]
        floor_boxes = args.output / f"L{floor}_boxes.csv"
        with floor_boxes.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=all_rows[0].keys()); writer.writeheader(); writer.writerows(floor_rows)
        floor_policy = None
        if policy:
            source = json.loads(policy.read_text(encoding="utf-8"))
            source_by_label = {str(x["label"]): x for x in source.get("instances", [])}
            decisions = []
            for i, row in enumerate(floor_rows):
                old = source_by_label.get(str(row["name"]).strip().lower(), {})
                # Match by geometry/label rather than a global boxer index after floor filtering.
                candidates = [x for x in source.get("instances", []) if x.get("label") == row["name"].strip().lower() and all(abs(float(a)-float(b)) < 1e-5 for a,b in zip(x.get("center_xyz_m", []), [float(row[k]) for k in ("tx_world_object","ty_world_object","tz_world_object")]))]
                old = candidates[0] if candidates else old
                decisions.append({**old, "instance_id": f"boxer_{i}"})
            floor_policy = args.output / f"L{floor}_structure_policy.json"
            floor_policy.write_text(json.dumps({**source, "instances": decisions}, ensure_ascii=False, indent=2) + "\n")
        common = [str(root / "scripts/build_occusg_grid.py"), str(episode), str(args.output / f"L{floor}_navigation_grid"),
                  "--resolution", str(args.resolution), "--floor-z", str(item["floor_z_m"]),
                  "--camera-z-min", str(item["flight_z_m"] - .8), "--camera-z-max", str(item["flight_z_m"] + .8)]
        subprocess.run([str(root / ".envs/habitat/bin/python")] + common, check=True)
        if policy:
            structure = [str(root / ".envs/habitat/bin/python"), str(root / "scripts/build_occusg_grid.py"), str(episode), str(args.output / f"L{floor}_structure_grid"),
                         "--resolution", str(args.resolution), "--floor-z", str(item["floor_z_m"]),
                         "--camera-z-min", str(item["flight_z_m"] - .8), "--camera-z-max", str(item["flight_z_m"] + .8),
                         "--object-boxes", str(floor_boxes), "--structure-policy", str(floor_policy)]
            subprocess.run(structure, check=True)
            for role in ("navigation", "structure"):
                prefix = args.output / f"L{floor}_{role}_grid"
                run_name = f"{args.output.name}_L{floor}_{role}"
                for suffix in (".npy", ".json"):
                    shutil.copy2(prefix.with_suffix(suffix), root / "runtime/occusg" / f"{run_name}_grid{suffix}")
                output_rel = (
                    f"{args.occusg_output_group}/L{floor}_{role}"
                    if args.occusg_output_group else run_name
                )
                subprocess.run([
                    str(root / "scripts/run_occusg.sh"), run_name, "1.8", output_rel,
                ], check=True)
                regions = root / "outputs/occusg" / output_rel / "regions.json"
                if role == "structure":
                    subprocess.run([str(root / ".envs/habitat/bin/python"), str(root / "scripts/fuse_rooms_boxes.py"), str(regions), str(floor_boxes), str(args.output / f"L{floor}_scene_graph.json"), "--grid-meta", str(prefix.with_suffix('.json'))], check=True)
    print(json.dumps({"levels": levels, "output": str(args.output)}, indent=2))

if __name__ == "__main__": main()
