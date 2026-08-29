#!/usr/bin/env bash

# Resolve a Docker command that works both on hosts using the docker group and
# on development machines whose Docker socket requires passwordless sudo.
pre_map_vln_resolve_docker() {
  if [[ -n "${PRE_MAP_VLN_DOCKER:-}" ]]; then
    read -r -a docker_cmd <<< "$PRE_MAP_VLN_DOCKER"
  elif docker info >/dev/null 2>&1; then
    docker_cmd=(docker)
  elif sudo -n docker info >/dev/null 2>&1; then
    docker_cmd=(sudo -n docker)
  else
    echo "Docker daemon is unavailable. Add the user to the docker group, or set PRE_MAP_VLN_DOCKER='sudo docker'." >&2
    return 1
  fi
}
