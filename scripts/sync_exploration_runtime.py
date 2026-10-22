#!/usr/bin/env python3
"""Synchronize the accepted exploration sources and ROS bridge before launch."""
import os
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
checkpoint = Path(os.environ.get("EXPLORATION_CHECKPOINT_DIR", str(root / "runtime/falcon_selected_yaw_checkpoint")))
workspace = Path("/workspace/falcon_ws")
falcon = workspace / "src/FALCON"
records = {}
changed = False
copied_sources = set()
sources = list(sorted(checkpoint.iterdir()))
# The700 snapshot omitted this upstream dependency; restore its verified bytes on rollback.
if checkpoint.name == "A3_observation_lifecycle" and not (checkpoint / "hierarchical_grid.cpp").exists():
    dependency = checkpoint.parent / "shared_dependencies/hierarchical_grid.cpp"
    expected = "2fca25380ba13bd1551ca5b767b8f079ec5890a65cc82a29adcf524765b08896"
    if hashlib.sha256(dependency.read_bytes()).hexdigest() != expected:
        raise RuntimeError("Verified700 HGrid rollback dependency changed")
    sources.append(dependency)
for source in sources:
    if source.suffix not in {".cpp", ".h"}:
        continue
    if source.name in {"execution_constraints.h", "exploration_diagnostics.h"}:
        target = falcon / "falcon_planner/pathfinding/include/pathfinding" / source.name
        target.parent.mkdir(parents=True, exist_ok=True)
        matches = [target]
    else:
        matches = list(falcon.rglob(source.name))
    if len(matches) != 1:
        raise RuntimeError(f"Expected one destination for {source.name}: {matches}")
    destination = matches[0]
    if not destination.exists() or source.read_bytes() != destination.read_bytes():
        shutil.copy2(source, destination)
        changed = True
        copied_sources.add(str(destination))
    records[str(destination)] = hashlib.sha256(source.read_bytes()).hexdigest()
bridge = checkpoint / "bridge_snapshot"
if not bridge.is_dir():
    bridge = root / "ros_ws/src/pre_map_bridge"
for source in sorted(bridge.rglob("*")):
    if not source.is_file() or "__pycache__" in source.parts or source.suffix == ".pyc":
        continue
    destination = workspace / "src/pre_map_bridge" / source.relative_to(bridge)
    if not destination.exists() or source.read_bytes() != destination.read_bytes():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        changed = True
        copied_sources.add(str(destination))
    records[str(destination)] = hashlib.sha256(source.read_bytes()).hexdigest()
build_state = workspace / ".exploration_runtime_build.json"
try:
    previous_build = json.loads(build_state.read_text())
except (OSError, ValueError):
    previous_build = {}
# copy2 preserves historical timestamps. A changed source with an older mtime
# can otherwise be skipped by make, leaving a stale binary with a fresh hash audit.
changed = changed or previous_build != records
if changed:
    for filename in records:
        if Path(filename).suffix in {".cpp", ".h"} and (
                filename in copied_sources or previous_build.get(filename) != records[filename]):
            os.utime(filename, None)
    subprocess.run(["bash", "-c", "source /opt/ros/noetic/setup.bash && catkin_make -j2"], cwd=workspace, check=True)
    build_state.write_text(json.dumps(records, sort_keys=True))
for filename, expected in records.items():
    if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != expected:
        raise RuntimeError(f"Runtime source mismatch: {filename}")
report = {"checkpoint": str(checkpoint), "rebuilt": changed, "source_sha256": records,
          "config_sha256": hashlib.sha256((checkpoint / "config.yaml").read_bytes()).hexdigest()}
report["binary_sha256"] = {}
for relative in ["devel/lib/exploration_manager/exploration_node", "devel/lib/fast_planner/traj_server",
                 "devel/lib/libpathfinding.so", "devel/lib/libfast_planner.so", "devel/lib/libvoxel_mapping.so"]:
    binary = workspace / relative
    if binary.exists():
        report["binary_sha256"][relative] = hashlib.sha256(binary.read_bytes()).hexdigest()
if len(sys.argv) > 1:
    (Path(sys.argv[1]) / "runtime_source_audit.json").write_text(json.dumps(report, indent=2))
print("Exploration sources and ROS bridge synchronized and verified.", flush=True)
