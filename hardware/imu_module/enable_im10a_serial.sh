#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CH341_KO="${REPO_ROOT}/imu_module/ch341_module/ch341.ko"

echo "Stopping processes that commonly steal CH341 serial devices..."
systemctl stop brltty.service >/dev/null 2>&1 || true
systemctl disable brltty.service >/dev/null 2>&1 || true
systemctl mask brltty.service >/dev/null 2>&1 || true
pkill -9 brltty >/dev/null 2>&1 || true
pkill -9 xbrlapi >/dev/null 2>&1 || true

echo "Loading USB serial support..."
modprobe usbserial

if lsmod | grep -q '^ch341 '; then
  rmmod ch341 || true
fi

echo "Loading custom CH341 module..."
insmod "${CH341_KO}"

IMU_SYSFS=""
for dev in /sys/bus/usb/devices/*; do
  [[ -f "${dev}/idVendor" ]] || continue
  vendor="$(cat "${dev}/idVendor" 2>/dev/null || true)"
  product="$(cat "${dev}/idProduct" 2>/dev/null || true)"
  if [[ "${vendor}" == "1a86" && "${product}" == "7523" ]]; then
    IMU_SYSFS="${dev}"
    break
  fi
done

if [[ -n "${IMU_SYSFS}" && -f "${IMU_SYSFS}/authorized" ]]; then
  echo "Re-enumerating IM10A at ${IMU_SYSFS}..."
  echo 0 > "${IMU_SYSFS}/authorized"
  sleep 1
  echo 1 > "${IMU_SYSFS}/authorized"
fi

sleep 1
echo
echo "Current serial nodes:"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No ttyUSB/ttyACM nodes found yet."
