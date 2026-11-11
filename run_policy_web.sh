#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GO2_BACKEND=policy
export GO2_NET_IFACE="${GO2_NET_IFACE:-eth0}"
export GO2_DDS_LOW_STATE_TOPIC="${GO2_DDS_LOW_STATE_TOPIC:-rt/lowstate}"
export GO2_DDS_LOW_CMD_TOPIC="${GO2_DDS_LOW_CMD_TOPIC:-rt/lowcmd}"
export GO2_POLICY_MODEL="${GO2_POLICY_MODEL:-$project_dir/policy_runtime/models/go2_blind_rough_v1.onnx}"

# This launcher intentionally supplies no low-level enable/confirmation gates.
exec "$project_dir/run_sdk.sh" -m uvicorn bridge.app:app \
  --host "${GO2_HOST:-0.0.0.0}" \
  --port "${GO2_PORT:-8765}"
