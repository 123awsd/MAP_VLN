#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --interface IFACE [--peer-ip IP] [--probe]"
  echo "Read-only: reports link/address/routes/neighbors; --probe only pings the explicit peer."
}

iface=""
peer_ip=""
probe=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --interface) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; iface="$2"; shift 2 ;;
    --peer-ip) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; peer_ip="$2"; shift 2 ;;
    --probe) probe=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$iface" ]] || { echo "Refusing to choose a default interface; pass --interface explicitly." >&2; exit 2; }
[[ "$iface" =~ ^[A-Za-z0-9_.:-]+$ ]] || { echo "Invalid interface name." >&2; exit 2; }
if [[ "$probe" -eq 1 && -z "$peer_ip" ]]; then
  echo "--probe requires an explicit --peer-ip; no subnet scan is performed." >&2
  exit 2
fi

IP="$(command -v ip)"
[[ -n "$IP" ]] || { echo "ip command not found." >&2; exit 1; }
if ! "$IP" link show dev "$iface" >/dev/null 2>&1; then
  echo "Interface not found: $iface" >&2
  exit 1
fi

echo "interface: $iface"
echo "--- link ---"
"$IP" -br link show dev "$iface"
echo "--- address ---"
"$IP" -br addr show dev "$iface"
echo "--- route (read-only) ---"
"$IP" route show dev "$iface" || true
echo "--- neighbors (read-only) ---"
"$IP" neigh show dev "$iface" || true

if command -v ethtool >/dev/null 2>&1; then
  echo "--- ethtool ---"
  ethtool "$iface" 2>&1 | /usr/bin/grep -E '^(Settings for|\s*Speed:|\s*Duplex:|\s*Link detected:)' || true
fi

carrier="$(/usr/bin/cat "/sys/class/net/${iface}/carrier" 2>/dev/null || echo unknown)"
echo "carrier: $carrier"
if [[ "$probe" -eq 1 ]]; then
  command -v ping >/dev/null 2>&1 || { echo "ping not found." >&2; exit 1; }
  echo "--- explicit peer probe: $peer_ip ---"
  ping -I "$iface" -c 1 -W 1 "$peer_ip"
fi

if [[ "$carrier" == "0" ]]; then
  echo "No physical carrier; do not configure or launch the Livox driver yet." >&2
  exit 2
fi
