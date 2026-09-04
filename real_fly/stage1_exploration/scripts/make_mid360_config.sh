#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --model mid360|mid360s --lidar-ip IP --host-ip IP --output /absolute/path/under/stage1.json"
}

model="mid360"
lidar_ip=""
host_ip=""
output=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; model="${2,,}"; shift 2 ;;
    --lidar-ip) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; lidar_ip="$2"; shift 2 ;;
    --host-ip) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; host_ip="$2"; shift 2 ;;
    --output) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; output="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$lidar_ip" && -n "$host_ip" && -n "$output" ]] || { usage >&2; exit 2; }
[[ "$model" == "mid360" || "$model" == "mid360s" ]] || {
  echo "Unsupported model: $model (expected mid360 or mid360s)." >&2
  exit 2
}
STAGE1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "$output" in
  "$STAGE1_ROOT"/*) ;;
  *) echo "Output must be under $STAGE1_ROOT; no external file will be written." >&2; exit 2 ;;
esac
[[ ! -e "$output" ]] || { echo "Refusing to overwrite existing file: $output" >&2; exit 2; }

python3 - "$model" "$lidar_ip" "$host_ip" "$output" <<'PY'
import ipaddress
import json
import os
import sys

model, lidar_ip, host_ip, output = sys.argv[1:]
for label, value in (("lidar", lidar_ip), ("host", host_ip)):
    try:
        ipaddress.ip_address(value)
    except ValueError:
        raise SystemExit("invalid %s IPv4/IPv6 address: %s" % (label, value))

network = {
        "lidar_net_info": {
            "cmd_data_port": 56100,
            "push_msg_port": 56200,
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500,
        },
}
if model == "mid360s":
    network["host_net_info"] = [{
            "host_ip": host_ip,
            "cmd_data_port": 56101,
            "push_msg_port": 56201,
            "point_data_port": 56301,
            "imu_data_port": 56401,
            "log_data_port": 56501,
        }]
    section = "Mid360s"
else:
    network["host_net_info"] = {
            "cmd_data_ip": host_ip,
            "cmd_data_port": 56101,
            "push_msg_ip": host_ip,
            "push_msg_port": 56201,
            "point_data_ip": host_ip,
            "point_data_port": 56301,
            "imu_data_ip": host_ip,
            "imu_data_port": 56401,
            "log_data_ip": "",
            "log_data_port": 56501,
        }
    section = "MID360"

cfg = {
    "lidar_summary_info": {"lidar_type": 8},
    section: network,
    "lidar_configs": [{
        "ip": lidar_ip,
        "pcl_data_type": 1,
        "pattern_mode": 0,
        "extrinsic_parameter": {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "x": 0.0, "y": 0.0, "z": 0.0,
        },
    }],
}
os.makedirs(os.path.dirname(output), exist_ok=True)
with open(output, "x", encoding="utf-8") as f:
    json.dump(cfg, f, indent=2)
    f.write("\n")
print(output)
PY
