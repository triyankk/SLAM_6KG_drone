#!/usr/bin/env bash
# Unified management script for the Intellisense SLAM & LiDAR stack.

SERVICES=(
  "intellisense_usb_serial_sensors.service"
  "hesai-jt16-obstacle.service"
  "intellisense_slam_bridge.service"
)

function usage() {
  echo "Usage: $0 {install|start|stop|restart|status|logs}"
  exit 1
}

if [[ $# -lt 1 ]]; then
  usage
fi

ACTION=$1

case "$ACTION" in
  install)
    echo "Starting master installation..."
    sudo bash "$(dirname "$0")/../install/install_all.sh"
    ;;
  start)
    echo "Starting Intellisense Flight Stack..."
    for svc in "${SERVICES[@]}"; do
      echo "--> Starting $svc"
      sudo systemctl start "$svc"
    done
    ;;
  stop)
    echo "Stopping Intellisense Flight Stack..."
    # Stop in reverse order
    for (( i=${#SERVICES[@]}-1; i>=0; i-- )); do
      svc="${SERVICES[$i]}"
      echo "--> Stopping $svc"
      sudo systemctl stop "$svc"
    done
    ;;
  restart)
    $0 stop
    $0 start
    ;;
  status)
    for svc in "${SERVICES[@]}"; do
      if systemctl is-active --quiet "$svc"; then
        status="ACTIVE"
      else
        status="INACTIVE"
      fi
      echo "[$status] $svc"
    done
    ;;
  logs)
    echo "Tailing all flight stack logs (Ctrl+C to exit)..."
    LOG_ARGS=""
    for svc in "${SERVICES[@]}"; do
      LOG_ARGS="$LOG_ARGS -u $svc"
    done
    journalctl $LOG_ARGS -f
    ;;
  *)
    usage
    ;;
esac
