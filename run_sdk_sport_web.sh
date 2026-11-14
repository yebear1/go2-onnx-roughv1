#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GO2_BACKEND=sdk_sport
export GO2_NET_IFACE="${GO2_NET_IFACE:-eth0}"
export GO2_VIDEO_MODE="${GO2_VIDEO_MODE:-sdk}"
export GO2_WATCHDOG_MS="${GO2_WATCHDOG_MS:-350}"
export GO2_MAX_FORWARD="${GO2_MAX_FORWARD:-0.50}"
export GO2_MAX_LATERAL="${GO2_MAX_LATERAL:-0.30}"
export GO2_MAX_YAW="${GO2_MAX_YAW:-0.80}"

if [[ "${GO2_SDK_SPORT_ENABLE:-0}" != "1" || "${GO2_SDK_SPORT_CONFIRM:-}" != "GROUND_CLEAR_REMOTE_READY" ]]; then
  echo "SDK Sport sending is locked. Confirm a clear floor and physical remote before enabling." >&2
  exit 2
fi

exec "$project_dir/run_sdk.sh" -m uvicorn bridge.app:app \
  --host "${GO2_HOST:-0.0.0.0}" \
  --port "${GO2_PORT:-8765}"
