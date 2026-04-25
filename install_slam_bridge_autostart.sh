#!/usr/bin/env bash
# Run:
#   sudo bash /home/atas/vscode/intellisense_slam/install_slam_bridge_autostart.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENSOR_INSTALLER="${SCRIPT_DIR}/hardware/install_usb_serial_sensors_autostart.sh"
SERVICE_TEMPLATE="${SCRIPT_DIR}/systemd/intellisense_slam_bridge.service.template"
SERVICE_NAME="intellisense_slam_bridge.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_GROUP="$(id -gn "${RUN_USER}")"
ENABLE_NOW=1
INSTALL_SENSORS=1

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
    --no-sensor-install)
      INSTALL_SENSORS=0
      shift
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      echo "Usage: sudo bash ${0} [--enable-now|--install-only] [--no-sensor-install]" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "${SERVICE_TEMPLATE}" ]]; then
  echo "Missing service template: ${SERVICE_TEMPLATE}" >&2
  exit 1
fi

if [[ "${INSTALL_SENSORS}" -eq 1 ]]; then
  if [[ "${ENABLE_NOW}" -eq 1 ]]; then
    bash "${SENSOR_INSTALLER}" --enable-now
  else
    bash "${SENSOR_INSTALLER}" --install-only
  fi
fi

sed \
  -e "s|__USER__|${RUN_USER}|g" \
  -e "s|__GROUP__|${RUN_GROUP}|g" \
  -e "s|__WORKDIR__|${SCRIPT_DIR}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

install -m 0644 "${TMP_SERVICE}" "${SERVICE_PATH}"
rm -f "${TMP_SERVICE}"

systemctl daemon-reload

if [[ "${ENABLE_NOW}" -eq 1 ]]; then
  systemctl enable "${SERVICE_NAME}"
  systemctl restart "${SERVICE_NAME}"
  echo "Installed, enabled, and started ${SERVICE_NAME}"
else
  systemctl disable "${SERVICE_NAME}" >/dev/null 2>&1 || true
  systemctl stop "${SERVICE_NAME}" >/dev/null 2>&1 || true
  echo "Installed ${SERVICE_NAME} but left it disabled/stopped."
fi

echo "Bridge config: ${SCRIPT_DIR}/config/autostart.yaml"
echo "Check status with: systemctl status ${SERVICE_NAME}"
echo "Watch logs with: journalctl -u ${SERVICE_NAME} -f"
