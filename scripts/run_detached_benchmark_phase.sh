#!/usr/bin/env bash
set -uo pipefail

if (( $# != 2 )); then
  echo "usage: run_detached_benchmark_phase.sh RUNNER EXIT_FILE" >&2
  exit 2
fi

runner="$1"
exit_file="$2"
"$runner"
code=$?
printf '%s\n' "$code" >"$exit_file"
exit "$code"
