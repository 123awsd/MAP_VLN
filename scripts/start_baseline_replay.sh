#!/usr/bin/env bash
set -euo pipefail
root=/home/uav/map_VLN/PRE_MAP_VLN
name=exploration_baseline_replay_504_20261015
out=/data/map_VLN/PRE_MAP_VLN/outputs/$name
container=doublehalf-v2-repeat
mkdir -p "$out/bridge"
[[ ! -e "$out/bridge/state.json" && ! -e "$out/result.json" ]] || exit 2
ln -s "$out" "$root/outputs/$name" 2>/dev/null || true
cp "$root/runtime/falcon_baseline_replay/config.yaml" "$out/config.yaml"
cp -r "$root/runtime/falcon_baseline_replay" "$out/source_snapshot"
python3 - "$root" "$out" <<'PY'
from pathlib import Path
import sys,json,hashlib
root,out=map(Path,sys.argv[1:]);base=root/'outputs/exploration_baseline_restore_504'
s=(base/'trial.launch').read_text().replace('/workspace/shared/outputs/exploration_baseline_restore_504',str(out))
# Recording disabled by user: keep structured evidence, not full sensor/map bags.
s='\n'.join(line for line in s.splitlines() if '<node pkg="rosbag"' not in line)+'\n'
(out/'trial.launch').write_text(s)
s=(base/'run_simulator.sh').read_text().replace('PRE_MAP_VLN/outputs/exploration_baseline_restore_504/source_snapshot/run_habitat_falcon.py',str(out/'source_snapshot/run_habitat_falcon.py')).replace('PRE_MAP_VLN/outputs/exploration_baseline_restore_504',str(out))
(out/'run_simulator.sh').write_text(s)
m=json.loads((out/'source_snapshot/replay_manifest.json').read_text());m['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (out/'source_snapshot').iterdir() if p.is_file()};m['output']=str(out);(out/'trial_manifest.json').write_text(json.dumps(m,indent=2))
PY
# Planner wrapper exits after result; floor collector also exits after result.
docker exec -d -e EXPLORATION_CHECKPOINT_DIR="$out/source_snapshot" "$container" bash -lc "bash /workspace/shared/runtime/falcon_selected_yaw_checkpoint/run_planner.sh '$out' 11355 > '$out/planner_start.log' 2>&1"
for i in $(seq 1 240); do
 if [[ -f "$out/run.log" ]] && grep -q 'File bridge ready\|process\[trial_recorder\|started with pid' "$out/run.log"; then break; fi
 sleep 2
done
docker exec -d "$container" bash -lc "source /opt/ros/noetic/setup.bash; source /workspace/falcon_ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11355; python3 /workspace/shared/scripts/capture_exploration_floor_volumes.py '$out' > '$out/floor_capture.log' 2>&1"
nohup bash "$out/run_simulator.sh" > "$out/habitat.log" 2>&1 < /dev/null &
echo "$!" > "$out/simulator.pid"
echo "Started $out"
