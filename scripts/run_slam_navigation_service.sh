#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lio_env.sh"
export MAVLINK20=1
exec "${ROOT_DIR}/.venv/bin/python" \
    -m optflow_slam.slam_navigation_service "$@"
