#!/usr/bin/env bash
set -uo pipefail

# User-space boot launcher for the legacy GPS2 optical-flow bridge.
# This exists because the root systemd unit can require sudo to modify on
# field hardware. The launcher is installed through the user's crontab and
# starts the same field command as run_field_legacy.sh after a short boot delay.

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SCRIPT_DIR"

mkdir -p logs runtime

exec 9>runtime/legacy_flow_bridge.lock
if ! flock -n 9; then
  printf '%s legacy boot skipped: another legacy bridge is already running\n' "$(date -Is)" >> logs/legacy_flow_bridge_boot.log
  exit 0
fi

BOOT_DELAY_SEC="${LEGACY_BOOT_DELAY_SEC:-45}"
BRIDGE_RESTART_DELAY_SEC="${LEGACY_BRIDGE_RESTART_DELAY_SEC:-20}"
PORT_RELEASE_WAIT_SEC="${LEGACY_PORT_RELEASE_WAIT_SEC:-60}"
CUBE_PORTS="${LEGACY_CUBE_PORTS:-/dev/ttyACM0 /dev/ttyACM1}"

wait_for_cube_ports() {
  local deadline
  deadline=$(( $(date +%s) + PORT_RELEASE_WAIT_SEC ))

  while true; do
    local busy
    busy=0
    for port in $CUBE_PORTS; do
      if [ ! -e "$port" ]; then
        continue
      fi
      if fuser "$port" >/dev/null 2>&1; then
        busy=1
        printf '%s waiting: %s is still owned by another process\n' "$(date -Is)" "$port" >> logs/legacy_flow_bridge_boot.log
        fuser -v "$port" >> logs/legacy_flow_bridge_boot.log 2>&1
      fi
    done

    if [ "$busy" -eq 0 ]; then
      return 0
    fi
    if [ "$(date +%s)" -ge "$deadline" ]; then
      printf '%s Cube USB ports still busy after %ss; bridge will try anyway\n' "$(date -Is)" "$PORT_RELEASE_WAIT_SEC" >> logs/legacy_flow_bridge_boot.log
      return 0
    fi
    sleep 2
  done
}

printf '%s legacy boot launcher waiting %ss before start\n' "$(date -Is)" "$BOOT_DELAY_SEC" >> logs/legacy_flow_bridge_boot.log
sleep "$BOOT_DELAY_SEC"

if [ "${LEGACY_START_LIDAR_AVOIDANCE:-1}" = "1" ]; then
  (
    cd "$REPO_ROOT"
    exec 8>legacy_flow_bridge/runtime/hesai_jt16_obstacle.lock
    if ! flock -n 8; then
      printf '%s JT16 obstacle node skipped: another instance is already running\n' "$(date -Is)" >> legacy_flow_bridge/logs/hesai_jt16_obstacle_boot.log
      exit 0
    fi
    LIDAR_START_DELAY_SEC="${LEGACY_LIDAR_START_DELAY_SEC:-5}"
    printf '%s JT16 obstacle node waiting %ss for bridge UDP\n' "$(date -Is)" "$LIDAR_START_DELAY_SEC" >> legacy_flow_bridge/logs/hesai_jt16_obstacle_boot.log
    sleep "$LIDAR_START_DELAY_SEC"
    while true; do
      printf '%s JT16 obstacle node starting\n' "$(date -Is)" >> legacy_flow_bridge/logs/hesai_jt16_obstacle_boot.log
      /usr/bin/python3 -u scripts/avoidance/hesai_jt16_obstacle_node.py \
        --config config/sensors.yaml \
        --mavport udpout:127.0.0.1:14555 \
        >> legacy_flow_bridge/logs/hesai_jt16_obstacle_boot.log 2>&1
      rc=$?
      printf '%s JT16 obstacle node exited rc=%s; restarting in 5s\n' "$(date -Is)" "$rc" >> legacy_flow_bridge/logs/hesai_jt16_obstacle_boot.log
      sleep 5
    done
  ) &
  printf '%s JT16 obstacle node supervisor launched\n' "$(date -Is)" >> logs/legacy_flow_bridge_boot.log
fi

while true; do
  wait_for_cube_ports
  printf '%s legacy GPS2 bridge starting\n' "$(date -Is)" >> logs/legacy_flow_bridge_boot.log
  ./run_field_legacy.sh >> logs/legacy_flow_bridge_boot.log 2>&1
  rc=$?
  printf '%s legacy GPS2 bridge exited with rc=%s; restarting in %ss\n' "$(date -Is)" "$rc" "$BRIDGE_RESTART_DELAY_SEC" >> logs/legacy_flow_bridge_boot.log
  sleep "$BRIDGE_RESTART_DELAY_SEC"
done
