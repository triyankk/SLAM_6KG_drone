#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lio_env.sh"
exec "${ROOT_DIR}/.venv/bin/python" \
    "${ROOT_DIR}/scripts/flight_supervisor_service.py" "$@"
