#!/usr/bin/env python3
"""Check the external pieces that Git intentionally cannot carry."""
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = {
    "boxernet_hw960in2x6d768-c88128f8.ckpt": "d5a30b348a8f5b0e5990ff3aa0e8f473ce77d860da22586322e7f47abc83ca6f",
    "dinov3_vits16plus_pretrain_lvd1689m-4057cbaa.pth": "4057cbaaad8c16657adb09d6815f28d4164eeba30532fde23f0d17313124caea",
    "owlv2-base-patch16-ensemble.pt": "14aa78ffe7b13e5b3ebf55845bc9a07e339a095cfd88f4c4e8f726b38ce1ebbf",
}


def locked_repositories(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    current = None
    for line in path.read_text().splitlines():
        if line.startswith("  ") and not line.startswith("    ") and line.rstrip().endswith(":"):
            current = line.strip()[:-1]
        elif current and line.strip().startswith("commit:"):
            result[current] = line.split(":", 1)[1].strip()
    return result


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(command: list[str]) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except FileNotFoundError:
        return subprocess.CompletedProcess(command, 127, f"command not found: {command[0]}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", help="also verify one HM3D train scene, e.g. 00166-RaYrxWt5pR1")
    parser.add_argument(
        "--prepared",
        help="also verify outputs/stage2_3d/prepared/<name> for the quick Stage2 path",
    )
    parser.add_argument("--skip-weight-hash", action="store_true", help="only check checkpoint names and sizes")
    args = parser.parse_args()
    failures: list[str] = []
    warnings: list[str] = []

    for command in ("git", "docker", "conda"):
        if shutil.which(command) is None:
            failures.append(f"missing command: {command}")

    directory_names = {
        "falcon": "FALCON", "boxer": "boxer", "occusg": "OccuSG",
        "habitat_lab": "habitat-lab", "open3d": "Open3D", "nlopt": "nlopt",
        "open_nav": "Open-Nav", "vln_zero": "VLN-Zero", "spatial_x": "Spatial-X",
    }
    for name, expected in locked_repositories(ROOT / "third_party.lock.yaml").items():
        checkout = ROOT / "third_party" / directory_names[name]
        probe = run(["git", "-C", str(checkout), "rev-parse", "HEAD"])
        actual = probe.stdout.strip() if probe.returncode == 0 else "missing"
        if actual != expected:
            failures.append(f"third_party/{directory_names[name]}: expected {expected}, got {actual}")

    boxer_runner = ROOT / "third_party/boxer/run_boxer.py"
    if boxer_runner.is_file():
        boxer_source = boxer_runner.read_text(encoding="utf-8")
        if 'dataset_type = "habitat"' not in boxer_source or "HabitatLoader" not in boxer_source:
            failures.append("Boxer Habitat loader patch is not applied")

    import_checks = {
        "habitat": [str(ROOT / ".envs/habitat/bin/python"), "-c", "import habitat,habitat_sim,numpy; assert habitat.__version__=='0.3.3'; assert habitat_sim.__version__=='0.3.3'; assert numpy.__version__=='1.26.4'"],
        "boxer": [str(ROOT / ".envs/boxer/bin/python"), "-c", "import importlib.metadata,torch,numpy,cv2,dill,yaml; assert torch.__version__.startswith('2.13.0'); assert numpy.__version__=='2.5.2'; assert importlib.metadata.version('opencv-python')=='5.0.0.93'; assert dill.__version__=='0.4.1'; assert yaml.__version__=='6.0.3'"],
    }
    for name, command in import_checks.items():
        if not Path(command[0]).is_file():
            failures.append(f"missing .envs/{name} Python environment")
            continue
        probe = run(command)
        if probe.returncode:
            failures.append(f"{name} import/version check failed: {probe.stdout.strip()}")

    checkpoint_dir = ROOT / "third_party/boxer/ckpts"
    for name, expected in WEIGHTS.items():
        path = checkpoint_dir / name
        if not path.is_file() or path.stat().st_size < 1024:
            failures.append(f"missing Boxer checkpoint: {path}")
        elif not args.skip_weight_hash and sha256(path) != expected:
            failures.append(f"Boxer checkpoint hash mismatch: {path}")

    docker_command = ["docker"]
    probe = run(docker_command + ["info"])
    if probe.returncode:
        sudo = run(["sudo", "-n", "docker", "info"])
        if sudo.returncode:
            failures.append("Docker daemon is not accessible by docker or sudo -n docker")
        else:
            docker_command = ["sudo", "-n", "docker"]
    for image in ("pre-map-vln/falcon-noetic:local", "pre-map-vln/occusg-humble:local"):
        if run(docker_command + ["image", "inspect", image]).returncode:
            failures.append(f"missing Docker image: {image}")

    if args.scene_id:
        if "-" not in args.scene_id:
            failures.append("--scene-id must include the numeric id and HM3D token")
        else:
            token = args.scene_id.split("-", 1)[1]
            scene_root = Path(os.environ.get("PRE_MAP_VLN_HM3D_TRAIN_ROOT", "/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train"))
            scene_config = Path(os.environ.get("PRE_MAP_VLN_HM3D_SCENE_CONFIG", "/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/hm3d_annotated_basis.scene_dataset_config.json"))
            scene = scene_root / args.scene_id / f"{token}.basis.glb"
            for path in (scene, scene_config):
                if not path.is_file():
                    failures.append(f"missing HM3D asset: {path}")

    if args.prepared:
        prepared = ROOT / "outputs/stage2_3d/prepared" / args.prepared
        manifest_path = prepared / "manifest.json"
        required = [
            manifest_path,
            prepared / "scene_graph.json",
            prepared / "generated_stage1_config.json",
            prepared / "voxel_snapshot/metadata.json",
            prepared / "voxel_snapshot/voxel_map.npz",
        ]
        for path in required:
            if not path.is_file():
                failures.append(f"missing prepared Stage2 asset: {path}")
        if manifest_path.is_file():
            try:
                import json
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                if manifest.get("format") != "pre_map_vln.prepared_multifloor_stage2.v1":
                    failures.append(f"unsupported prepared manifest format: {manifest_path}")
                prepared_scene_id = manifest.get("scene_id")
                if args.scene_id and prepared_scene_id != args.scene_id:
                    failures.append(
                        f"prepared scene {prepared_scene_id} differs from --scene-id {args.scene_id}"
                    )
            except (OSError, ValueError) as error:
                failures.append(f"invalid prepared Stage2 manifest: {error}")

    falcon_status = run(["git", "-C", str(ROOT / "third_party/FALCON"), "status", "--porcelain"]).stdout.strip()
    if falcon_status:
        warnings.append("third_party/FALCON has local changes; clean clones reproduce through patches/falcon/*.patch")

    for warning in warnings:
        print(f"WARN: {warning}")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        print(f"Reproducibility check failed: {len(failures)} issue(s).")
        return 1
    print("PASS: dependencies, host environments, Boxer weights and Docker images are reproducible.")
    if not args.scene_id:
        print("NOTE: pass --scene-id to verify scene-specific HM3D assets too.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
