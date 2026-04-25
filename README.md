# Intellisense SLAM — Quick Reference

This repository contains helper scripts and a lightweight MAVLink bridge to provide ExternalNav (`ODOMETRY`) from a Jetson companion computer to a Cube flight controller for GPS-denied testing.

Purpose
- Provide safe, testable paths to validate visual/IMU pose sources on the Jetson
- Send ExternalNav `ODOMETRY` to the Cube only after explicit calibration and readiness checks
- Offer bench tools for VIO capture, IMU validation, and JT16/JT26 lidar testing

Requirements
- Jetson (Ubuntu) with camera and USB serial devices attached
- Python 3.10+ and the repo Python dependencies (see `pyproject.toml` or install via `pip` as needed)
- Cube flight controller connected via USB to the Jetson

Repository layout (important files)
- `scripts/check_slam_readiness.py` — hardware & dependency report
- `scripts/check_external_imu.py` — IM10A probe and decoder
- `scripts/run_local_vio.py` — run the local VIO runner (no FC/GCS output)
- `scripts/run_slam_odometry_bridge.py` — bridge: send `ODOMETRY` to Cube
- `scripts/configure_fc_for_slam.py` — apply FC ExternalNav params (writes into EKF source set 3)
- `scripts/bench_vio.py` — bench capture of VIO+IMU to CSV
- `install_slam_bridge_autostart.sh` — install systemd service for bridge
- `ardupilot_lua/jetson_nogps_status.lua` — FC-side Lua relay for STATUSTEXT messages
- `tools/` — diagnostic tools for IMU and JT16

Quick start — full command walkthrough

1) Check hardware and dependencies
```bash
python3 scripts/check_slam_readiness.py
```

2) Verify external IMU (IM10A)
```bash
python3 scripts/check_external_imu.py
```

3) Run the local VIO runner (validate visually before connecting to FC)
```bash
python3 scripts/run_local_vio.py
```
Options:
- disable cube height: `--cube-height off`
- record to CSV: `--csv-out /tmp/local_vio_session.csv`

4) Prepare the Cube for SLAM ExternalNav (one-time)
```bash
python3 scripts/configure_fc_for_slam.py --config config/default.yaml
```
This preserves GPS/flow setups and writes the SLAM parameters into EKF source set 3.

5) Run the SLAM odometry bridge (demo/demo sources or real VIO)
Demo hover source (no IMU binding):
```bash
python3 scripts/run_slam_odometry_bridge.py --ports /dev/ttyACM1 /dev/ttyACM0 --source hover --imu off
```
Run the bridge with the experimental in-repo `vio` source:
```bash
python3 scripts/run_slam_odometry_bridge.py --ports /dev/ttyACM1 /dev/ttyACM0 --source vio
```
Replay a CSV pose file to the bridge:
```bash
python3 scripts/run_slam_odometry_bridge.py --ports /dev/ttyACM1 /dev/ttyACM0 --source csv --csv-path /path/to/local_pose.csv
```
Bench capture (VIO+IMU) example:
```bash
python3 scripts/bench_vio.py --out /tmp/bench.csv --duration 30
```
Stream ODOMETRY during bench (use with caution):
```bash
python3 scripts/bench_vio.py --out /tmp/bench.csv --duration 30 --send --ports /dev/ttyACM1 /dev/ttyACM2
```

6) Install autostart services (optional)
```bash
sudo bash install_slam_bridge_autostart.sh
```
After installation, useful commands:
```bash
systemctl status intellisense_slam_bridge.service
journalctl -u intellisense_slam_bridge.service -f
```
If you need to work locally with the camera, stop the service first:
```bash
sudo systemctl stop intellisense_slam_bridge.service
```

Bundled diagnostics
- IMU stream check:
```bash
python3 tools/imu_stream_check.py
python3 tools/imu_serial_preview.py --port auto --baud auto
```
- JT16 tools:
```bash
python3 tools/jt16_validate_connection.py --no-probe
python3 tools/jt16_serial_probe.py --port auto
python3 tools/jt16_live_view.py --port auto
```

Notes and safety
- The in-repo `vio` provider is experimental and intended for bench testing only.
- Always validate the VIO output locally before enabling FC-facing ExternalNav.
- Use persistent device paths (e.g. `/dev/serial/by-id/...`) or udev rules to avoid renumbering issues with USB devices.

Support and next steps
- Replace the experimental `vio` source with a production VIO/VINS/ORB-SLAM backend for flight-grade use.
- Add time-synchronized IMU capture on the Jetson for robust visual-inertial fusion.

License / Contributing
- Fork and iterate; open a PR with improvements to the VIO backend or calibration tooling.

- valid packet types observed: `0x50 0x51 0x52 0x53 0x54 0x56 0x59`
- the IMU speaks a JY901-style binary packet stream, not plain text
- the SLAM repo now carries the local CH341 driver module in:

```bash
/home/atas/vscode/intellisense_slam/hardware/imu_module/ch341_module/ch341.ko
```

The biggest Jetson-specific trap was `brltty` stealing the CH341 serial device. If `/dev/ttyUSB0` appears briefly and disappears, check and remove `brltty` first.

## JT16 note

The JT16 path is now usable on this Jetson only because the Prolific `pl2303` module was patched for the adapter revision that reports `bcdDevice=0705`.

Patched module:

```bash
/home/atas/vscode/intellisense_slam/hardware/pl2303_module/pl2303.ko
```

If the adapter is lost after a reboot, reload it with:

```bash
sudo bash /home/atas/vscode/intellisense_slam/hardware/enable_usb_serial_sensors.sh
```

Or reinstall the persistent service from the SLAM repo:

```bash
sudo bash /home/atas/vscode/intellisense_slam/hardware/install_usb_serial_sensors_autostart.sh
```

## Next milestones

### Milestone 1. Verified external-nav bridge

Goal:

- prove the Cube accepts `ODOMETRY`
- prove Jetson-to-Cube framing and orientation are consistent

Status:

- started in this repo
- FC source set 3 is now reserved for SLAM ExternalNav so we can bench-test against the live Cube without overwriting the older flow path

### Milestone 2. Camera-side pose backend

Goal:

- replace the demo pose source with a real backend

Good candidate paths:

- external IMU + visual-inertial odometry
- depth-camera odometry with careful constraints
- lidar odometry or lidar-assisted mapping

A helper to collect paired VIO poses and IMU samples is provided in `scripts/collect_calibration_data.py`.
Run it to record datasets for offline hand-eye / extrinsic calibration before attempting flight validation.

### Milestone 3. Frame calibration

Goal:

- camera frame to body frame
- IMU frame to body frame
- lidar frame to body frame

### Milestone 4. Flight validation

Goal:

- bench test
- tethered hover
- GPS-available comparison flights
- then true GPS-denied trials

## Recommended next hardware step

If the target is a real SLAM/GPS-denied drone rather than only optical-flow hold, the best next addition is:

- an external IMU that the Jetson can read with stable timestamps

or

- replacing the D415 with a camera that already includes an IMU

That is the missing piece that changes this from “prototype integration repo” to “real SLAM candidate”.
