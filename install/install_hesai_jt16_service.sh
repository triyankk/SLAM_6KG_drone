#!/usr/bin/env bash
# Run:
#   sudo bash /home/atas/vscode/intellisense_slam/install/install_hesai_jt16_service.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd .. && pwd)"
SERVICE_TEMPLATE="${SCRIPT_DIR}/hardware/configs/templates/hesai-jt16-obstacle.service.template"
SERVICE_NAME="hesai-jt16-obstacle.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_GROUP="$(id -gn "${RUN_USER}")"

if [[ ! -f "${SERVICE_TEMPLATE}" ]]; then
  echo "Missing service template: ${SERVICE_TEMPLATE}" >&2
  exit 1
fi

sed \
  -e "s|__USER__|${RUN_USER}|g" \
  -e "s|__GROUP__|${RUN_GROUP}|g" \
  -e "s|__WORKDIR__|${SCRIPT_DIR}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

install -m 0644 "${TMP_SERVICE}" "${SERVICE_PATH}"
rm -f "${TMP_SERVICE}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"
systemctl restart "${SERVICE_NAME}"

echo "Installed, enabled, and started ${SERVICE_NAME}"
echo "Watch logs with: journalctl -u ${SERVICE_NAME} -f"
