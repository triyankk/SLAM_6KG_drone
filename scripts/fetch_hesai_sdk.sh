#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_DIR="${ROOT_DIR}/third_party/HesaiLidar_SDK_2.0"
SDK_URL="https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git"
SDK_TAG="v2.0.12"
SDK_COMMIT="534c707846a810e8211b93446f878dbf415f7000"

if [[ ! -d "${SDK_DIR}/.git" ]]; then
    git clone \
        --depth 1 \
        --branch "${SDK_TAG}" \
        --recurse-submodules \
        --shallow-submodules \
        "${SDK_URL}" \
        "${SDK_DIR}"
fi

actual_commit="$(git -C "${SDK_DIR}" rev-parse HEAD)"
if [[ "${actual_commit}" != "${SDK_COMMIT}" ]]; then
    printf 'Unexpected Hesai SDK revision: %s\n' "${actual_commit}" >&2
    printf 'Expected pinned revision: %s\n' "${SDK_COMMIT}" >&2
    exit 2
fi

printf 'Hesai SDK %s ready at %s\n' "${SDK_TAG}" "${SDK_DIR}"
