#!/usr/bin/env bash
# Run inside the dedicated ROS container. Each trial must have a fresh bridge.
set -euo pipefail
trial_dir="${1:?trial directory under /workspace/shared/outputs required}"
master_port="${2:-11314}"
set +u
source /opt/ros/noetic/setup.bash
source /workspace/falcon_ws/devel/setup.bash
set -u
export ROS_MASTER_URI="http://127.0.0.1:${master_port}"
export ROS_HOSTNAME=127.0.0.1
if [[ -e "$trial_dir/result.json" || -e "$trial_dir/trial.bag" || -e "$trial_dir/trial.bag.active" || -e "$trial_dir/bridge/command.json" ]]; then
  echo 'Trial already contains execution data; use a fresh directory.' >&2
  exit 2
fi
roslaunch -p "$master_port" "$trial_dir/trial.launch" > "$trial_dir/run.log" 2>&1 &
planner_pid=$!
cleanup() {
  kill -INT "$planner_pid" 2>/dev/null || true
  wait "$planner_pid" || true
}
trap cleanup EXIT
python3 - "$trial_dir" <<'PY'
import json
import sys
import time
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped

out = Path(sys.argv[1])
rospy.init_node('realtime_trial_trigger', anonymous=True)
publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
started, triggered, sequence = time.monotonic(), False, -1
with (out/'live_trajectory.jsonl').open('w') as stream:
    while time.monotonic()-started < 720:
        try:
            state = json.loads((out/'bridge/state.json').read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            time.sleep(.1)
            continue
        if state['sequence'] != sequence:
            sequence = state['sequence']
            stream.write(json.dumps(state)+'\n')
            stream.flush()
        if not triggered and sequence >= 15 and publisher.get_num_connections():
            message = PoseStamped()
            message.header.frame_id = 'world'
            message.header.stamp = rospy.Time.now()
            message.pose.orientation.w = 1
            publisher.publish(message)
            triggered = True
            print('Triggered exploration at sequence', sequence, flush=True)
        if (out/'result.json').exists():
            break
        time.sleep(.1)
print('Monitor exited, triggered=', triggered, flush=True)
PY
