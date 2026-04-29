#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_SRC="${REPO_DIR}/systemd/slam-mavlink-monitor.service"
SERVICE_DST="/etc/systemd/system/slam-mavlink-monitor.service"
CURRENT_USER="${SUDO_USER:-$(id -un)}"
CURRENT_GROUP="$(id -gn "${CURRENT_USER}")"

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this without sudo. The script will ask for sudo only when installing the systemd service."
  exit 1
fi

cd "${REPO_DIR}"

python3 -m pip install --user -r requirements.txt

tmp_service="$(mktemp)"
sed \
  -e "s#__WORKDIR__#${REPO_DIR}#g" \
  -e "s#__USER__#${CURRENT_USER}#g" \
  -e "s#__GROUP__#${CURRENT_GROUP}#g" \
  "${SERVICE_SRC}" > "${tmp_service}"

sudo install -m 0644 "${tmp_service}" "${SERVICE_DST}"
rm -f "${tmp_service}"

sudo systemctl daemon-reload
sudo systemctl enable slam-mavlink-monitor.service
sudo systemctl restart slam-mavlink-monitor.service

echo "Installed and started slam-mavlink-monitor.service"
echo "Check status: sudo systemctl status slam-mavlink-monitor.service"
echo "View logs: journalctl -u slam-mavlink-monitor.service -f"
