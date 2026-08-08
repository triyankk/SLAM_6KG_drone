#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

export MAVLINK20=1
export PYTHONPATH="${ROOT_DIR}/src:${PYTHONPATH:-}"
exec "${ROOT_DIR}/.venv/bin/python" \
    -m optflow_slam.obstacle_avoidance_service "$@"
