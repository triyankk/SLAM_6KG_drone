#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SOURCE="${ROOT_DIR}/hardware/udev/99-optflow-realsense.rules"
RULE_TARGET="/etc/udev/rules.d/99-optflow-realsense.rules"

if [[ "${EUID}" -ne 0 ]]; then
    printf 'Root authentication is required. Run: sudo %s\n' "$0" >&2
    exit 2
fi

install -D -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
udevadm control --reload-rules
udevadm trigger \
    --action=change \
    --subsystem-match=usb \
    --attr-match=idVendor=8086 \
    --attr-match=idProduct=0ad3
udevadm settle

printf '%s\n' \
    "Installed the project RealSense D415 permission rule." \
    "Camera check: ./optflow sensor-check"
