#!/usr/bin/env bash
# Run:
#   sudo bash /home/atas/vscode/intellisense_slam/hardware/enable_usb_serial_sensors.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JT16_KO="${SCRIPT_DIR}/pl2303_module/pl2303.ko"
IMU_KO="${SCRIPT_DIR}/imu_module/ch341_module/ch341.ko"
JT16_SYMLINK="/dev/jt16_usb"
IMU_SYMLINK="/dev/imu_usb"

log() {
  echo "[slam-usb-serial-sensors] $*"
}

require_module() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    log "Missing module: ${path}"
    exit 1
  fi
}

module_loaded() {
  lsmod | awk '{print $1}' | grep -qx "$1"
}

load_custom_module() {
  local name="$1"
  local path="$2"

  if module_loaded "${name}"; then
    log "Module ${name} already loaded; skipping insmod."
    return 0
  fi

  log "Loading custom ${name} module from ${path}."
  if ! insmod "${path}" 2>/tmp/insmod.err; then
    local rc=$?
    log "insmod failed for ${name} (rc=${rc})."
    if [[ -s /tmp/insmod.err ]]; then
      log "insmod stderr: $(sed -n '1,3p' /tmp/insmod.err)"
    fi
    # If the module file already exists/registered, try to detect whether the
    # module is now present in lsmod; if so, continue. Otherwise proceed
    # without failing so the overall installer stays robust.
    if module_loaded "${name}"; then
      log "Module ${name} appears loaded after insmod attempt; continuing."
      rm -f /tmp/insmod.err || true
      return 0
    fi
    log "Continuing despite insmod failure for ${name}."
    rm -f /tmp/insmod.err || true
    return 0
  fi
}

stop_brltty_conflicts() {
  systemctl stop brltty.service >/dev/null 2>&1 || true
  pkill -9 brltty >/dev/null 2>&1 || true
  pkill -9 xbrlapi >/dev/null 2>&1 || true
}

find_usb_devices() {
  local vendor="$1"
  local product="$2"
  local dev
  for dev in /sys/bus/usb/devices/*; do
    [[ -f "${dev}/idVendor" ]] || continue
    [[ -f "${dev}/idProduct" ]] || continue
    if [[ "$(cat "${dev}/idVendor" 2>/dev/null || true)" == "${vendor}" && "$(cat "${dev}/idProduct" 2>/dev/null || true)" == "${product}" ]]; then
      echo "${dev}"
    fi
  done
}

reauthorize_devices() {
  local vendor="$1"
  local product="$2"
  local label="$3"
  local dev
  local found=0

  while IFS= read -r dev; do
    [[ -n "${dev}" ]] || continue
    found=1
    if [[ -w "${dev}/authorized" ]]; then
      log "Re-enumerating ${label} at ${dev}."
      echo 0 > "${dev}/authorized"
      sleep 1
      echo 1 > "${dev}/authorized"
    fi
  done < <(find_usb_devices "${vendor}" "${product}")

  if [[ "${found}" -eq 0 ]]; then
    log "${label} USB device not present right now."
  fi
}

wait_for_node() {
  local path="$1"
  local timeout_s="$2"
  local deadline_s=$((SECONDS + timeout_s))
  while (( SECONDS < deadline_s )); do
    if [[ -e "${path}" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

print_node_state() {
  log "Current serial nodes:"
  local entries=()
  [[ -e "${JT16_SYMLINK}" ]] && entries+=("${JT16_SYMLINK}")
  [[ -e "${IMU_SYMLINK}" ]] && entries+=("${IMU_SYMLINK}")
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "${dev}" ]] && entries+=("${dev}")
  done

  if [[ "${#entries[@]}" -eq 0 ]]; then
    log "No ttyUSB/ttyACM nodes found yet."
    return
  fi

  ls -l "${entries[@]}"
}

main() {
  require_module "${JT16_KO}"
  require_module "${IMU_KO}"

  stop_brltty_conflicts

  log "Loading USB serial core."
  modprobe usbserial

  load_custom_module "pl2303" "${JT16_KO}"
  load_custom_module "ch341" "${IMU_KO}"

  reauthorize_devices "067b" "23a3" "JT16 adapter"
  reauthorize_devices "1a86" "7523" "IM10A adapter"

  if command -v udevadm >/dev/null 2>&1; then
    udevadm settle >/dev/null 2>&1 || true
  fi

  wait_for_node "${JT16_SYMLINK}" 20 || log "JT16 stable symlink did not appear yet."
  wait_for_node "${IMU_SYMLINK}" 20 || log "IM10A stable symlink did not appear yet."
  print_node_state
}

main "$@"
