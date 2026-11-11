#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${UNITREE_ROS2_SETUP:-}" ]]; then
  # shellcheck disable=SC1090
  source "${UNITREE_ROS2_SETUP}"
fi

exec python3 -m uvicorn bridge.app:app \
  --host "${GO2_HOST:-0.0.0.0}" \
  --port "${GO2_PORT:-8765}"

