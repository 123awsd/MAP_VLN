#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --output /absolute/path/under/stage1.tsv [--duration-sec N] [--interval-sec N]"
  echo "Read-only CPU/memory/thermal/disk/tegrastats sampling; output is never overwritten."
}

output=""
duration=60
interval=5
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; output="$2"; shift 2 ;;
    --duration-sec) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; duration="$2"; shift 2 ;;
    --interval-sec) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; interval="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
[[ "$output" = /* ]] || { echo "Output must be an absolute path." >&2; exit 2; }
case "$output" in
  "$STAGE1_ROOT"/*) ;;
  *) echo "Output must be under $STAGE1_ROOT; no external file will be written." >&2; exit 2 ;;
esac
[[ ! -e "$output" ]] || { echo "Refusing to overwrite existing output: $output" >&2; exit 2; }
[[ "$duration" =~ ^[1-9][0-9]*$ && "$interval" =~ ^[1-9][0-9]*$ ]] || {
  echo "Duration and interval must be positive integer seconds." >&2
  exit 2
}

mkdir -p "$(dirname "$output")"
printf 'timestamp_utc\tload1\tmem_used_mb\tmem_total_mb\tdisk_used_bytes\tdisk_available_bytes\tthermal_c\ttegrastats\n' > "$output"

thermal_snapshot() {
  local first=1 zone temp
  for zone in /sys/class/thermal/thermal_zone*/temp; do
    [[ -r "$zone" ]] || continue
    temp="$(/bin/cat "$zone" 2>/dev/null || true)"
    [[ "$temp" =~ ^[0-9]+$ ]] || continue
    [[ "$first" -eq 0 ]] && printf ';'
    printf '%s=%.3f' "$(basename "$(dirname "$zone")")" "$(awk -v value="$temp" 'BEGIN { printf "%.3f", value / 1000.0 }')"
    first=0
  done
  [[ "$first" -eq 1 ]] && printf 'n/a'
}

started="$SECONDS"
while (( SECONDS - started <= duration )); do
  timestamp="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  load1="$(awk '{print $1}' /proc/loadavg)"
  mem_total_kb="$(awk '/^MemTotal:/ {print $2; exit}' /proc/meminfo)"
  mem_available_kb="$(awk '/^MemAvailable:/ {print $2; exit}' /proc/meminfo)"
  mem_used_mb="$(awk -v total="$mem_total_kb" -v available="$mem_available_kb" 'BEGIN { printf "%.1f", (total - available) / 1024.0 }')"
  mem_total_mb="$(awk -v total="$mem_total_kb" 'BEGIN { printf "%.1f", total / 1024.0 }')"
  disk="$(df -P "$STAGE1_ROOT" | awk 'NR == 2 {print $3 "\t" $4}')"
  disk_used="$(printf '%s\n' "$disk" | awk '{print $1}')"
  disk_available="$(printf '%s\n' "$disk" | awk '{print $2}')"
  thermal="$(thermal_snapshot)"
  tegrastats_value="n/a"
  if command -v tegrastats >/dev/null 2>&1; then
    tegrastats_value="$(timeout 3 tegrastats --interval 1000 --count 1 2>&1 || true)"
    tegrastats_value="$(printf '%s' "$tegrastats_value" | tr '\n\t' '  ' | sed 's/[[:space:]]\+/ /g')"
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$timestamp" "$load1" "$mem_used_mb" "$mem_total_mb" "$disk_used" \
    "$disk_available" "$thermal" "$tegrastats_value" >> "$output"
  (( SECONDS - started >= duration )) && break
  sleep "$interval"
done
echo "Resource samples written: $output"
