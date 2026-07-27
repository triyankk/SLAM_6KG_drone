#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
KERNEL_RELEASE="$(uname -r)"
MODULE_SOURCE="${ROOT_DIR}/hardware/kernel/ch341/ch341.ko"
MODULE_TARGET="/lib/modules/${KERNEL_RELEASE}/updates/optflow/ch341.ko"
RULE_SOURCE="${ROOT_DIR}/hardware/udev/99-optflow-slam-usb-serial.rules"
RULE_TARGET="/etc/udev/rules.d/99-optflow-slam-usb-serial.rules"

if [[ "${EUID}" -ne 0 ]]; then
    printf 'Root authentication is required. Run: sudo %s\n' "$0" >&2
    exit 2
fi

if [[ ! -f "${MODULE_SOURCE}" ]]; then
    printf 'Driver is not built. Run ./optflow build-ch341 first.\n' >&2
    exit 2
fi

module_release="$(modinfo -F vermagic "${MODULE_SOURCE}" | cut -d' ' -f1)"
if [[ "${module_release}" != "${KERNEL_RELEASE}" ]]; then
    printf '%s\n' \
        "Driver/kernel mismatch: ${module_release} != ${KERNEL_RELEASE}" \
        "Rebuild it with ./optflow build-ch341." >&2
    exit 2
fi

install -D -m 0644 "${MODULE_SOURCE}" "${MODULE_TARGET}"
install -D -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
depmod -a "${KERNEL_RELEASE}"
modprobe usbserial
modprobe ch341
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

printf '%s\n' \
    "CH341 driver installed for ${KERNEL_RELEASE}." \
    "IM10A device: /dev/imu_usb"
