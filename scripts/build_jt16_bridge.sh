#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CMAKE="${ROOT_DIR}/.venv/bin/cmake"
SDK_DIR="${ROOT_DIR}/third_party/HesaiLidar_SDK_2.0"
SOURCE_DIR="${ROOT_DIR}/native/jt16_bridge"
BUILD_DIR="${ROOT_DIR}/build/jt16_bridge"

if [[ ! -x "${CMAKE}" ]]; then
    printf 'Project CMake is missing. Run ./optflow setup first.\n' >&2
    exit 2
fi

"${ROOT_DIR}/scripts/fetch_hesai_sdk.sh"
"${CMAKE}" \
    -S "${SOURCE_DIR}" \
    -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DHESAI_SDK_DIR="${SDK_DIR}"
"${CMAKE}" --build "${BUILD_DIR}" --parallel "$(nproc)"

printf 'JT16 bridge ready: %s\n' \
    "${BUILD_DIR}/optflow-jt16-bridge"
