#!/usr/bin/env bash
# Run:
#   sudo bash /home/atas/vscode/intellisense_slam/install/install_usb_serial_sensors_autostart.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd .. && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/hardware/configs"
SERVICE_TEMPLATE="${CONFIG_DIR}/templates/intellisense_usb_serial_sensors.service.template"
UDEV_TEMPLATE="${CONFIG_DIR}/99-intellisense-usb-serial.rules"
SERVICE_NAME="intellisense_usb_serial_sensors.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
UDEV_PATH="/etc/udev/rules.d/99-intellisense-usb-serial.rules"
TMP_SERVICE="$(mktemp)"
ENABLE_NOW=1

while [[ "$#" -gt 0 ]]; do
  arg="$1"
  case "${arg}" in
    --enable-now)
      ENABLE_NOW=1
      shift
      ;;
    --install-only)
      ENABLE_NOW=0
      shift
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      echo "Usage: sudo bash ${0} [--enable-now|--install-only]" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "${SERVICE_TEMPLATE}" ]]; then
  echo "Missing service template: ${SERVICE_TEMPLATE}" >&2
  exit 1
fi

if [[ ! -f "${UDEV_TEMPLATE}" ]]; then
  echo "Missing udev rules template: ${UDEV_TEMPLATE}" >&2
  exit 1
fi

chmod 0755 "${SCRIPT_DIR}/install/enable_usb_serial_sensors.sh"

sed -e "s|__WORKDIR__|${SCRIPT_DIR}|g" "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"
install -m 0644 "${TMP_SERVICE}" "${SERVICE_PATH}"
rm -f "${TMP_SERVICE}"

install -D -m 0644 "${UDEV_TEMPLATE}" "${UDEV_PATH}"

systemctl daemon-reload
udevadm control --reload

if [[ "${ENABLE_NOW}" -eq 1 ]]; then
  systemctl enable "${SERVICE_NAME}"
  systemctl restart "${SERVICE_NAME}"
  udevadm trigger --subsystem-match=tty >/dev/null 2>&1 || true
  echo "Installed, enabled, and started ${SERVICE_NAME}"
else
  systemctl disable "${SERVICE_NAME}" >/dev/null 2>&1 || true
  systemctl stop "${SERVICE_NAME}" >/dev/null 2>&1 || true
  echo "Installed ${SERVICE_NAME} and udev rules, but left it disabled/stopped."
fi

echo "Stable serial symlinks:"
echo "- JT16 lidar: /dev/jt16_usb"
echo "- IM10A IMU: /dev/imu_usb"
echo "Check service with: systemctl status ${SERVICE_NAME}"
