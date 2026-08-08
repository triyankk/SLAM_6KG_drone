#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
KERNEL_RELEASE="$(uname -r)"
MODULE_SOURCE="${ROOT_DIR}/hardware/kernel/pl2303/pl2303.ko"
MODULE_TARGET="/lib/modules/${KERNEL_RELEASE}/updates/optflow/pl2303.ko"
RULE_SOURCE="${ROOT_DIR}/hardware/udev/99-optflow-slam-usb-serial.rules"
RULE_TARGET="/etc/udev/rules.d/99-optflow-slam-usb-serial.rules"

if [[ "${EUID}" -ne 0 ]]; then
    printf 'Root authentication is required. Run: sudo %s\n' "$0" >&2
    exit 2
fi

if [[ ! -f "${MODULE_SOURCE}" ]]; then
    printf 'Driver is not built. Run ./optflow build-pl2303 first.\n' >&2
    exit 2
fi

module_release="$(modinfo -F vermagic "${MODULE_SOURCE}" | cut -d' ' -f1)"
if [[ "${module_release}" != "${KERNEL_RELEASE}" ]]; then
    printf '%s\n' \
        "Driver/kernel mismatch: ${module_release} != ${KERNEL_RELEASE}" \
        "Rebuild it with ./optflow build-pl2303." >&2
    exit 2
fi

install -D -m 0644 "${MODULE_SOURCE}" "${MODULE_TARGET}"
install -D -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
depmod -a "${KERNEL_RELEASE}"
if [[ -d /sys/module/pl2303 ]]; then
    modprobe -r pl2303
fi
modprobe usbserial
modprobe pl2303
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

for _ in $(seq 1 20); do
    if [[ -e /dev/jt16_usb ]]; then
        printf '%s\n' \
            "PL2303 driver installed for ${KERNEL_RELEASE}." \
            "JT16 device: /dev/jt16_usb"
        exit 0
    fi
    sleep 0.25
done

printf 'Driver loaded, but /dev/jt16_usb did not appear.\n' >&2
exit 3
