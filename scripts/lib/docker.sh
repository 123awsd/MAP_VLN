#!/usr/bin/env bash

# Resolve a Docker command for hosts using either the docker group or
# passwordless sudo.  Call pre_map_vln_resolve_docker before using docker_cmd.
pre_map_vln_resolve_docker() {
  if [[ -n "${PRE_MAP_VLN_DOCKER:-}" ]]; then
    read -r -a docker_cmd <<< "$PRE_MAP_VLN_DOCKER"
  elif docker info >/dev/null 2>&1; then
    docker_cmd=(docker)
  elif sudo -n docker info >/dev/null 2>&1; then
    docker_cmd=(sudo -n docker)
  else
    echo "Docker daemon unavailable; configure the docker group or PRE_MAP_VLN_DOCKER." >&2
    return 1
  fi
}
