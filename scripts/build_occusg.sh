#!/usr/bin/env bash
set -euo pipefail

readonly MIN_AVAILABLE_KIB=$((12 * 1024 * 1024))
readonly MIN_SWAP_FREE_KIB=$((2 * 1024 * 1024))

available_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
swap_free_kib="$(awk '/^SwapFree:/ {print $2}' /proc/meminfo)"

if (( available_kib < MIN_AVAILABLE_KIB )); then
  printf 'Refusing OccuSG build: only %.1f GiB memory is available (12 GiB required).\n' \
    "$(awk -v kib="$available_kib" 'BEGIN {printf "%.1f", kib / 1048576}')" >&2
  exit 2
fi

if (( swap_free_kib < MIN_SWAP_FREE_KIB )); then
  printf 'Refusing OccuSG build: only %.1f GiB swap is free (2 GiB required).\n' \
    "$(awk -v kib="$swap_free_kib" 'BEGIN {printf "%.1f", kib / 1048576}')" >&2
  exit 2
fi

printf 'OccuSG build preflight passed: %.1f GiB memory and %.1f GiB swap available.\n' \
  "$(awk -v kib="$available_kib" 'BEGIN {printf "%.1f", kib / 1048576}')" \
  "$(awk -v kib="$swap_free_kib" 'BEGIN {printf "%.1f", kib / 1048576}')"

exec docker compose build occusg
