#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dds_home="${GO2_CYCLONEDDS_HOME:-$HOME/go2-offline/cyclonedds/install}"
venv="${GO2_POLICY_VENV:-$project_dir/.venv-policy}"

export CYCLONEDDS_HOME="$dds_home"
export CMAKE_PREFIX_PATH="$dds_home${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$dds_home/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec "$venv/bin/python" "$@"
