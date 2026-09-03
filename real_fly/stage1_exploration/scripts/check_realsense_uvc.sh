#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: $0"
  echo "Read-only RealSense UVC enumeration and short frame check."
  exit 0
fi

echo "D435/D435i UVC read-only check"
nodes=()
for dev in /dev/video*; do
  [[ -e "$dev" ]] || continue
  props="$(udevadm info --query=property --name="$dev" 2>/dev/null || true)"
  if echo "$props" | /usr/bin/grep -Eiq 'ID_MODEL=.*RealSense|ID_VENDOR_ID=8086|ID_MODEL_ID=0b07'; then
    nodes+=("$dev")
  fi
done

if [[ "${#nodes[@]}" -eq 0 ]]; then
  echo "No RealSense UVC node found." >&2
  exit 1
fi
printf 'nodes:'
printf ' %s' "${nodes[@]}"
printf '\n'

if command -v v4l2-ctl >/dev/null 2>&1; then
  for dev in "${nodes[@]}"; do
    echo "--- $dev formats ---"
    v4l2-ctl --device="$dev" --list-formats-ext 2>&1 | head -80 || true
  done
else
  echo "v4l2-ctl is not installed; format listing is unavailable."
fi

if command -v gst-launch-1.0 >/dev/null 2>&1; then
  passed=0
  for dev in "${nodes[@]}"; do
    echo "--- $dev short frame read ---"
    if gst-launch-1.0 -q v4l2src device="$dev" num-buffers=30 ! videoconvert ! fakesink sync=false; then
      echo "UVC frames: OK ($dev)"
      passed=$((passed + 1))
    else
      echo "UVC frames: FAIL ($dev)" >&2
    fi
  done
  [[ "$passed" -gt 0 ]] || exit 1
else
  echo "gst-launch-1.0 is not installed; frame read was not attempted." >&2
  exit 1
fi

if command -v rs-enumerate-devices >/dev/null 2>&1; then
  echo "--- librealsense enumeration ---"
  rs-enumerate-devices 2>&1 | head -120
else
  echo "librealsense tools are not installed; depth units/IMU/intrinsics remain unverified."
fi
