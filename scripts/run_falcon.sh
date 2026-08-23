#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
docker compose run --rm --name pre-map-vln-falcon falcon \
  roslaunch pre_map_bridge habitat_falcon.launch
