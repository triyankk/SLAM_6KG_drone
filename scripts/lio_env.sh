#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf '%s\n' "Source this file; do not execute it." >&2
    exit 2
fi

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LIO_RUNTIME="${ROOT_DIR}/runtime/lio"
ROS_ROOT="${LIO_RUNTIME}/ros2"

if [[ ! -f "${ROS_ROOT}/setup.bash" ]]; then
    printf 'LIO runtime is missing. Run: %s/optflow build-lio\n' \
        "${ROOT_DIR}" >&2
    return 2
fi

# shellcheck disable=SC1091
lio_restore_nounset=false
if [[ "$-" == *u* ]]; then
    lio_restore_nounset=true
    set +u
fi
source "${ROS_ROOT}/setup.bash"
if [[ "${lio_restore_nounset}" == true ]]; then
    set -u
fi
unset lio_restore_nounset

PCL_PREFIX="${LIO_RUNTIME}/pcl"
PCL_MSGS_PREFIX="${LIO_RUNTIME}/pcl_ros/pcl_msgs"
PCL_CONVERSIONS_PREFIX="${LIO_RUNTIME}/pcl_ros/pcl_conversions"
FAST_LIO_PREFIX="${LIO_RUNTIME}/fastlio/fast_lio"
SYSROOT="${LIO_RUNTIME}/sysroot"

export AMENT_PREFIX_PATH="${FAST_LIO_PREFIX}:${PCL_CONVERSIONS_PREFIX}:${PCL_MSGS_PREFIX}:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${FAST_LIO_PREFIX}/lib:${PCL_MSGS_PREFIX}/lib:${PCL_PREFIX}/lib:${SYSROOT}/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${ROOT_DIR}/src:${FAST_LIO_PREFIX}/local/lib/python3.10/dist-packages:${PCL_MSGS_PREFIX}/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
