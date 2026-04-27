#!/usr/bin/env bash
# Unified installation script for the entire Intellisense SLAM & LiDAR stack.
# Run: sudo bash install/install_all.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd .. && pwd)"
INSTALL_DIR="${SCRIPT_DIR}/install"

echo "===================================================="
echo " Starting Full Intellisense Stack Installation "
echo "===================================================="

# 1. Install USB Serial Sensors (Drivers & Udev)
echo ""
echo "[Step 1/4] Installing USB Serial Sensor drivers and rules..."
sudo bash "${INSTALL_DIR}/install_usb_serial_sensors_autostart.sh" --enable-now

# 2. Install Hesai JT16 LiDAR Obstacle Service
echo ""
echo "[Step 2/4] Installing LiDAR Obstacle Avoidance service..."
sudo bash "${INSTALL_DIR}/install_hesai_jt16_service.sh"

# 3. Install SLAM MAVLink Bridge
echo ""
echo "[Step 3/4] Installing SLAM MAVLink Bridge service..."
sudo bash "${INSTALL_DIR}/install_slam_bridge_autostart.sh" --no-sensor-install

# 4. Install VIO Flight Monitor
echo ""
echo "[Step 4/4] Installing VIO Flight Monitor service..."
sudo bash "${INSTALL_DIR}/install_vio_flight_service.sh" --no-sensor-install

echo ""
echo "===================================================="
echo " Installation Complete! "
echo "===================================================="
echo "All services have been installed, enabled, and started."
echo ""
echo "Unified Management Command:"
echo "  ./scripts/manage_flight_stack.sh status"
echo "  ./scripts/manage_flight_stack.sh logs"
echo "===================================================="
