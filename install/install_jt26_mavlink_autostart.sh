#!/usr/bin/env bash
# Run:
#   sudo bash /home/atas/vscode/intellisense_slam/install_jt26_mavlink_autostart.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_TEMPLATE="${SCRIPT_DIR}/systemd/jt26_to_mavlink.service.template"
SERVICE_NAME="jt26_to_mavlink.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"
RUN_USER="${SUDO_USER:-$(id -un)}"

if [[ ! -f "${SERVICE_TEMPLATE}" ]]; then
  echo "Missing service template: ${SERVICE_TEMPLATE}" >&2
  exit 1
fi

sed \
  -e "s|__USER__|${RUN_USER}|g" \
  -e "s|__WORKDIR__|${SCRIPT_DIR}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

install -m 0644 "${TMP_SERVICE}" "${SERVICE_PATH}"
rm -f "${TMP_SERVICE}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"
systemctl restart "${SERVICE_NAME}"

echo "Installed, enabled, and started ${SERVICE_NAME}"
echo "Check status with: systemctl status ${SERVICE_NAME}"
echo "Watch logs with: journalctl -u ${SERVICE_NAME} -f"
