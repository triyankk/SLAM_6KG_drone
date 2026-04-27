# Intellisense SLAM — Quick Reference

This repository contains helper scripts and a lightweight MAVLink bridge to provide ExternalNav (`ODOMETRY`) from a Jetson companion computer to a Cube flight controller for GPS-denied testing.

Purpose
- Provide safe, testable paths to validate visual/IMU pose sources on the Jetson
- Send ExternalNav `ODOMETRY` to the Cube only after explicit calibration and readiness checks
- Offer bench tools for VIO capture, IMU validation, and JT16/JT26 lidar testing
- **Obstacle Avoidance**: 360-degree LiDAR-based obstacle awareness and automatic resistance commands

## LiDAR Obstacle Avoidance

The system uses a Hesai JT16 Mini LiDAR to detect obstacles in 8 zones around the drone.

### How it works:
1.  **Filtering**: Only points between 0.15m and 7.0m are considered.
2.  **Zoning**: Points are divided into 8 sectors: Front, Front-Right, Right, Rear-Right, Rear, Rear-Left, Left, Front-Left.
3.  **Danger (<2m)**: If an object enters the 2m danger zone, the system calculates an avoidance vector and sends a short velocity pulse in the opposite direction.
4.  **Resistance**: If the pilot attempts to push the drone towards an obstacle inside the danger zone, the system resists that input.
5.  **Safety**: Movement is disabled by default (`enable_avoidance_motion: false`). It only activates when armed and in a supported mode (GUIDED, LOITER, POSHOLD).

### Useful Avoidance Commands:

**1. Check LiDAR health:**
```bash
python3 scripts/diagnostics/check_hesai_jt16.py
```

**2. Run visualization (Top-down view):**
```bash
python3 scripts/avoidance/visualize_lidar_avoidance.py
```
This shows the 8 zones, raw points, and the calculated avoidance vector (cyan arrow).

**3. Run in Dry-Run mode (Safe):**
```bash
python3 scripts/avoidance/hesai_jt16_obstacle_node.py --dry-run
```
Computes everything and sends GCS warnings, but sends NO movement commands.

**4. Enable real movement (Dangerous - Use with caution):**
```bash
python3 scripts/avoidance/hesai_jt16_obstacle_node.py --enable-motion
```

Requirements
- Jetson (Ubuntu) with camera and USB serial devices attached
- Python 3.10+ and the repo Python dependencies (see `pyproject.toml` or install via `pip` as needed)
- Cube flight controller connected via USB to the Jetson

Repository layout (important folders)
- `scripts/avoidance/` — LiDAR obstacle node and visualizer
- `scripts/calibration/` — SLAM, VIO, and sensor calibration tools
- `scripts/diagnostics/` — hardware health checks and MAVLink probes
- `scripts/runners/` — main flight bridges and local VIO runners
- `install/` — systemd and driver installation scripts
- `config/` — sensor and autostart configuration files
- `ardupilot_lua/` — safety and status scripts for the Cube

Quick start — full command walkthrough

1) Check hardware and dependencies
```bash
python3 scripts/diagnostics/check_slam_readiness.py
```

2) Verify Hesai JT16 Mini LiDAR
```bash
python3 scripts/diagnostics/check_hesai_jt16.py
```
If using Ethernet LiDAR, provide the IP:
```bash
python3 scripts/diagnostics/check_hesai_jt16.py --ip 192.168.1.201
```

3) Verify external IMU (IM10A)
```bash
python3 scripts/diagnostics/check_external_imu.py
```

4) Run the local VIO runner (validate visually before connecting to FC)
```bash
python3 scripts/runners/run_local_vio.py
```

5) Prepare the Cube for SLAM ExternalNav (one-time)
```bash
python3 scripts/calibration/configure_fc_for_slam.py --config config/default.yaml
```

6) Run the Hesai JT16 obstacle node
```bash
python3 scripts/avoidance/hesai_jt16_obstacle_node.py --config config/sensors.yaml --mavport /dev/ttyACM1
```

7) Run the SLAM odometry bridge (real VIO)
```bash
python3 scripts/runners/run_slam_odometry_bridge.py --ports /dev/ttyACM1 /dev/ttyACM0 --source vio
```

## SLAM / VIO Calibration Commands

Use these from the repo root:
```bash
cd /home/atas/vscode/intellisense_slam
```

1. Start local VIO preview:
```bash
python3 scripts/runners/run_local_vio.py
```

2. Run stationary calibration:
```bash
python3 scripts/calibration/run_stationary_calibration.py
```

3. Run stationary calibration with verbose logs:
```bash
python3 scripts/calibration/run_stationary_calibration.py --verbose
```

4. Check MAVLink heartbeat:
```bash
python3 scripts/diagnostics/check_mavlink.py
```

5. Check RealSense / camera health:
```bash
python3 scripts/diagnostics/check_realsense.py
```

6. Check IMU health:
```bash
python3 scripts/diagnostics/check_imu.py
```

7. Check rangefinder / lidar data:
```bash
python3 scripts/diagnostics/check_rangefinder.py
```

8. Check VIO drift while stationary:
```bash
python3 scripts/diagnostics/check_vio_drift.py
```

9. Restart flight-ready VIO service:
```bash
sudo systemctl restart vio-flight.service
```

10. Check service status:
```bash
sudo systemctl status vio-flight.service
```

11. View live logs:
```bash
journalctl -u vio-flight.service -f
```

12. Run Brake-mode SLAM calibration monitor:
```bash
python3 scripts/calibration/brake_slam_calibration.py
```

13. Run dry-run mode:
```bash
python3 scripts/calibration/brake_slam_calibration.py --dry-run
```

14. Run with movement disabled:
```bash
python3 scripts/calibration/brake_slam_calibration.py --disable-motion
```

15. Run full calibration only after safety checks pass:
```bash
python3 scripts/calibration/brake_slam_calibration.py --enable-motion
```

Install the headless boot service:
```bash
sudo bash install/install_vio_flight_service.sh
```

## Unified Management Script
```bash
# Install everything
./scripts/manage_flight_stack.sh install

# Start/Stop
./scripts/manage_flight_stack.sh start
./scripts/manage_flight_stack.sh stop

# Status & Logs
./scripts/manage_flight_stack.sh status
./scripts/manage_flight_stack.sh logs
```

## Hardware notes

IM10A IMU:
```bash
/home/atas/vscode/intellisense_slam/hardware/drivers/imu_module/ch341_module/ch341.ko
```

JT16 LiDAR:
```bash
/home/atas/vscode/intellisense_slam/hardware/drivers/pl2303_module/pl2303.ko
```

If the adapter is lost after a reboot, reload it with:
```bash
sudo bash install/enable_usb_serial_sensors.sh
```
Or reinstall the persistent service:
```bash
sudo bash install/install_usb_serial_sensors_autostart.sh
```
