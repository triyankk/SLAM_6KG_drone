# SLAM Bring-up Runbook — Quick Verification

This runbook lists minimal steps to verify Jetson SLAM -> Cube ExternalNav and JT26 obstacle publishing.

Prereqs
- Jetson services installed: `intellisense_usb_serial_sensors.service`, `intellisense_slam_bridge.service`, `jt26_to_mavlink.service`
- Camera, IMU, JT26 connected and symlinks present (`/dev/imu_usb`, `/dev/jt16_usb`) 
- Cube connected via USB (confirm `/dev/ttyACM*`)

1) Verify FC params and apply
- Run: `python3 scripts/configure_fc_for_slam.py --config config/autostart.yaml`
- If `Reboot recommended` is printed, reboot the Cube before continuing.

2) Start bridge and confirm ODOMETRY
- Enable/start the bridge: `sudo systemctl enable --now intellisense_slam_bridge.service`
- Watch bridge logs for EKF source_set switch and absence of VisOdom OOM:
  - `journalctl -u intellisense_slam_bridge.service -f`
- Expect logs like `EKF source set switched for SLAM: active=3` and repeated `SLAM bridge: ... status=...` without `VisOdom: out of memory`.

3) Sanity-check JT26 -> DISTANCE_SENSOR
- The bridge uses the Cube serial port; to let JT26 write to the Cube safely, stop the bridge temporarily:
  - `sudo systemctl stop intellisense_slam_bridge.service`
- Run JT26 translator (manual test):
  - `python3 scripts/jt26_to_mavlink.py --jtport auto --mavport /dev/ttyACM2 --rate 5 --safety-m 2.0`
  - Observe printed `Opening JT port...` and `Connecting to MAVLink on /dev/ttyACM2` and DISTANCE_SENSOR sends.
- If running as systemd service, update the service to point to the active Cube port (`/dev/ttyACM2` or `/dev/ttyACM3`) then `sudo systemctl enable --now jt26_to_mavlink.service`.

4) Restore bridge
- `sudo systemctl start intellisense_slam_bridge.service`

5) Final checks on GCS
- Confirm the Cube/ground station shows STATUSTEXT `SLAM/ExternalNav ACTIVE` and DISTANCE_SENSOR entries.
- Verify `posHold` using ExternalNav in a bench test (tethered) before any flight.

Notes & Troubleshooting
- If `jt26_to_mavlink.service` crashes with `BlockingIOError` update script to handle non-blocking reads (already patched in repo).
- If Cube shows `PreArm: VisOdom: out of memory` after the bridge is running, reboot the Cube to clear VisOdom allocation and re-run `configure_fc_for_slam.py`.
- The bridge intentionally avoids sending `VISION_*` messages — it sends ODOMETRY as ExternalNav to avoid FC VisOdom activation.

Files referenced
- Config: [config/autostart.yaml](config/autostart.yaml)
- Bridge: [scripts/run_slam_odometry_bridge.py](scripts/run_slam_odometry_bridge.py)
- FC config helper: [scripts/configure_fc_for_slam.py](scripts/configure_fc_for_slam.py)
- JT26 translator: [scripts/jt26_to_mavlink.py](scripts/jt26_to_mavlink.py)

Safety
- Do not attempt free flight until external nav is validated on a tethered bench.
