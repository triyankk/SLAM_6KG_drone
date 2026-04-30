#!/usr/bin/env bash
set -euo pipefail

# User-space boot launcher for the legacy GPS2 optical-flow bridge.
# This exists because the root systemd unit can require sudo to modify on
# field hardware. The launcher is installed through the user's crontab and
# starts the same field command as run_field_legacy.sh after a short boot delay.

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

mkdir -p logs runtime

exec 9>runtime/legacy_flow_bridge.lock
if ! flock -n 9; then
  printf '%s legacy boot skipped: another legacy bridge is already running\n' "$(date -Is)" >> logs/legacy_flow_bridge_boot.log
  exit 0
fi

BOOT_DELAY_SEC="${LEGACY_BOOT_DELAY_SEC:-30}"
printf '%s legacy boot launcher waiting %ss before start\n' "$(date -Is)" "$BOOT_DELAY_SEC" >> logs/legacy_flow_bridge_boot.log
sleep "$BOOT_DELAY_SEC"

while true; do
  printf '%s legacy GPS2 bridge starting\n' "$(date -Is)" >> logs/legacy_flow_bridge_boot.log
  ./run_field_legacy.sh >> logs/legacy_flow_bridge_boot.log 2>&1
  rc=$?
  printf '%s legacy GPS2 bridge exited with rc=%s; restarting in 10s\n' "$(date -Is)" "$rc" >> logs/legacy_flow_bridge_boot.log
  sleep 10
done
