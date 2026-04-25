#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_DIR="${SCRIPT_DIR}/ch341_module"
KERNEL_BUILD_DIR="/lib/modules/$(uname -r)/build"

if [[ ! -d "${KERNEL_BUILD_DIR}" ]]; then
  echo "Missing kernel build directory: ${KERNEL_BUILD_DIR}" >&2
  exit 1
fi

make -C "${KERNEL_BUILD_DIR}" M="${MODULE_DIR}" modules
