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
- `scripts/stationary_slam_calibrate.py` — ground-based stationary calibration
- `scripts/bench_vio.py` — bench capture of VIO+IMU to CSV
- `src/slam_core/` — reusable Python package used by the scripts
- `install_slam_bridge_autostart.sh` — install systemd service for bridge
- `ardupilot_lua/jetson_nogps_status.lua` — FC-side Lua relay for STATUSTEXT messages
- `tools/` — diagnostic tools for IMU and JT16
- `CALIBRATION_GUIDE.md` — detailed explanation of how calibration works

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

## SLAM / VIO Calibration Commands

Use these from the repo root:
```bash
cd /home/atas/vscode/intellisense_slam
```

1. Start local VIO preview:
```bash
python3 scripts/run_local_vio.py
```
Runs the local RealSense VIO preview without intentionally controlling the vehicle. Use this first to check if the trajectory is plausible.

2. Run stationary calibration:
```bash
python3 scripts/run_stationary_calibration.py
```
Runs the ground calibration wrapper. The drone should be disarmed, still, and sitting on the ground/table.

3. Run stationary calibration with verbose logs:
```bash
python3 scripts/run_stationary_calibration.py --verbose
```
Prints detailed timestamped sensor, MAVLink, VIO, IMU, and rangefinder checks.

Indoor GPS-denied variant:
```bash
python3 scripts/run_stationary_calibration.py --indoor --no-gps --verbose
```
Uses the same stationary checks without requiring GPS.

4. Check MAVLink heartbeat:
```bash
python3 scripts/check_mavlink.py
```
Connects to the Cube, auto-discovers Cube serial ports, waits for heartbeat, and prints mode/armed/range/EKF status.

5. Check RealSense / camera health:
```bash
python3 scripts/check_realsense.py
```
Lists RealSense devices and confirms depth/infrared frames are arriving with moving timestamps.

6. Check IMU health:
```bash
python3 scripts/check_imu.py
```
Checks the IM10A USB serial stream and prints orientation/gyro data if healthy.

7. Check rangefinder / lidar data:
```bash
python3 scripts/check_rangefinder.py
```
Reads Cube `DISTANCE_SENSOR` messages and reports rangefinder mean/noise.

8. Check VIO drift while stationary:
```bash
python3 scripts/check_vio_drift.py
```
Runs VIO while the drone/camera is still and fails if XY drift or pose noise exceeds conservative limits.

9. Restart flight-ready VIO service:
```bash
sudo systemctl restart vio-flight.service
```
Restarts the headless Brake-mode calibration monitor installed from this repo.

10. Check service status:
```bash
sudo systemctl status vio-flight.service
```
Shows whether the headless Jetson monitor is running.

11. View live logs:
```bash
journalctl -u vio-flight.service -f
```
Local file logs are also written here:
```bash
tail -f logs/slam_calibration.log
```

12. Run Brake-mode SLAM calibration monitor:
```bash
python3 scripts/brake_slam_calibration.py
```
Starts the bridge with Brake-mode calibration monitoring enabled from `config/autostart.yaml`.

13. Run dry-run mode:
```bash
python3 scripts/brake_slam_calibration.py --dry-run
```
Runs the monitor in a no-command field test mode. It may send GCS messages and beeps, but it does not send movement, RTL, fallback mode, EKF source switching, or odometry commands.

Current status snapshot:
```bash
python3 scripts/brake_slam_calibration.py --status
```
Prints the latest state written to `logs/slam_calibration_status.json`.

14. Run with movement disabled:
```bash
python3 scripts/brake_slam_calibration.py --disable-motion
```
Forces calibration movement commands off. This is the recommended first live monitor test.

15. Run full calibration only after safety checks pass:
```bash
python3 scripts/brake_slam_calibration.py --enable-motion
```
Allows bounded pitch/roll/yaw nudges after safety gates pass. No automatic takeoff is commanded. Keep this disabled until bench and dry-run tests pass.

Install the headless boot service:
```bash
sudo bash install_vio_flight_service.sh
```
This installs `vio-flight.service`, which runs `scripts/brake_slam_calibration.py --config config/autostart.yaml --disable-motion` at boot.

## Indoor GPS-denied test

1. Run stationary calibration:
```bash
python3 scripts/run_stationary_calibration.py --indoor --no-gps --verbose
```

2. Restart flight service:
```bash
sudo systemctl restart vio-flight.service
```

3. Check logs:
```bash
journalctl -u vio-flight.service -f
```

The indoor flow does not require GPS and does not move the drone.

## Outdoor headless field workflow

1. Power drone.
2. Wait for Jetson boot.
3. Open GCS.
4. Confirm STATUSTEXT says `VIO service ready.`
5. Switch to Brake mode.
6. Confirm: `Brake mode: SLAM calibration fused with Brake mode is active.`
7. Arm only after all checks pass.
8. Confirm: `Armed in Brake mode. SLAM calibration takeoff sequence active.`
9. Run dry-run first.
10. Enable movement only after bench and dry-run tests pass.

The boot service defaults to movement disabled. No automatic takeoff is commanded. If movement is later enabled, `calibration.kill_switch_confirmed` must also be set true after the physical kill switch/failsafe has been verified.

## Dry-run test

```bash
python3 scripts/brake_slam_calibration.py --dry-run
```

Dry-run monitors Brake mode, armed/disarmed state, sensors, and announcements without movement, RTL, fallback, EKF source switching, or odometry commands.

## Enable real movement

```bash
python3 scripts/brake_slam_calibration.py --enable-motion
```

Movement remains bounded and conservative, but this is **not** for first test day. Before enabling it, verify the kill switch/failsafe and set `calibration.kill_switch_confirmed: true` in `config/autostart.yaml`.

## Flight-Readiness Audit

Current status: **NOT READY for untethered GPS-denied flight yet.** The workflow has the core Brake-mode calibration state machine, but it still needs bench, dry-run, tethered, and GCS verification before trusting it in open flight.

1. Brake mode starts SLAM calibration: **Implemented.** The bridge reads MAVLink `HEARTBEAT`, detects `BRAKE`, and enters the calibration state machine.

2. Brake mode calibration announcement: **Implemented.** It sends MAVLink `STATUSTEXT`: `Brake mode: SLAM calibration fused with Brake mode is active.` It also sends `Calibration mode engaged.`

3. Lua requirement: **Not required for the primary announcements.** Jetson sends `STATUSTEXT` and `PLAY_TUNE` directly over MAVLink. Optional backup Lua is provided at `scripts/ardupilot/brake_slam_beeper.lua` for custom buzzer/status relay if Jetson-triggered tunes are unreliable.

4. Wait for arm: **Implemented.** It sends `Brake mode detected. Waiting for arm to start SLAM calibration.`

5. Armed + ground + Brake trigger: **Implemented.** It sends `Armed in Brake mode. SLAM calibration takeoff sequence active.` It does not command automatic takeoff.

6. Rangefinder 5m condition: **Implemented.** It uses Cube `DISTANCE_SENSOR` rangefinder height and announces `Reached 5 meters by rangefinder. Holding altitude for SLAM calibration.`

7. Pitch/roll/yaw/throttle checks: **Partially implemented.** Pitch, roll, and yaw calibration stages exist and are bounded/configurable. A passive throttle/altitude-hold response check runs using rangefinder height. Active throttle movement is intentionally **not implemented** for safety.

8. Ten-second active announcement: **Implemented.** It repeats `SLAM calibration active.` every 10 seconds while airborne calibration is active.

9. Success flow: **Implemented with safety gates.** Success requires healthy vehicle state, valid rangefinder, valid VIO, live MAVLink heartbeat, active SLAM EKF source set, stable calibration profile, available RTL mode, and accepted RTL mode change. It then plays the calibration-complete tune and sends `Calibration successful: SLAM PosHold calibration complete. Initiating RTL.`

10. Failure flow: **Implemented.** It sends `Calibration failed: not finished. Reason: <reason>`, stops calibration, and requests the configured fallback mode. In dry-run mode it does not request fallback mode changes.

11. Crucial remaining checks before flight:
- Bench test without props.
- Dry-run Brake mode test.
- Confirm `STATUSTEXT` appears in GCS.
- Confirm buzzer/beep works.
- Confirm rangefinder height is stable.
- Confirm VIO data is stable while stationary.
- Confirm EKF accepts external navigation and source set switches to the SLAM source.
- Confirm mode change to Brake is detected.
- Confirm calibration does not start when disarmed.
- Confirm calibration stops if pilot changes mode.
- Confirm calibration stops on RC failsafe.
- Confirm calibration stops if VIO tracking is lost.
- Confirm calibration stops if rangefinder becomes invalid.
- Confirm RTL triggers only after successful calibration.
- Confirm manual override works.
- Confirm kill switch works.
- First live test must be tethered or prop-guarded in an open area.

5b) Stationary SLAM calibration (while drone is on the ground)

This step is the safe ground check before any GPS-denied PosHold testing. Run it with the drone disarmed, sitting still, and pointed in the normal takeoff direction. It stops the active bridge service if it is using the camera, checks the required sensors, resets the VIO origin, measures stationary drift, saves `runtime/slam_calibration.json`, and restarts the bridge service.

**For a detailed explanation of what calibration does, see [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md).**

Recommended command:
```bash
python3 scripts/stationary_slam_calibrate.py --config config/autostart.yaml
```
Equivalent through the local VIO entrypoint:
```bash
python3 scripts/run_local_vio.py --stationary-calibration on --calibration-config config/autostart.yaml
```

This script will:
- Stop `intellisense_slam_bridge.service` if it is active
- Check MAVLink heartbeat, RealSense/VIO frames, IMU stability, and rangefinder height
- Detect no frames, frozen timestamps, noisy pose, bad rangefinder data, bad height scale, and stationary SLAM drift
- Reset the local VIO origin before collecting calibration samples
- Save calibration only if the profile is stable
- Restart the flight VIO service and print `System is flight ready.`

Options:
- `--duration N` — calibration duration in seconds (default: 25)
- `--source vio` — use the in-repo VIO (default)
- `--source hover` — use a stationary demo source instead
- `--imu on` — enable external IMU binding (default: on)
- `--output path/to/calibration.json` — save to a different path
- `--manage-service off` — do not stop/restart the bridge service
- `--restart-service never` — leave the service stopped after calibration

Output example:
```
2026-04-25 14:30:05 | stage=vio mode=BRAKE armed=no rangefinder=0.22m vio=ok+imu+rng/q78 imu=stable mavlink=ok
  Stationary drift detected: 3.2 cm over 15.0 seconds
2026-04-25 14:30:18 | stage=complete mode=BRAKE armed=no rangefinder=0.22m vio=ok+imu+rng/q81 imu=stable mavlink=ok
  Calibration passed. Saved profile to runtime/slam_calibration.json.
2026-04-25 14:30:19 | stage=complete mode=BRAKE armed=no rangefinder=0.22m vio=ok+imu+rng/q81 imu=stable mavlink=ok
  System is flight ready.
```

The SLAM bridge now loads the saved calibration profile configured by `calibration.profile_path`.

5c) Brake-mode SLAM calibration workflow

Brake mode is used as the supervised calibration mode because it is not Loiter, Stabilize, AltHold, RTL, Land, or PosHold. The Jetson does not command takeoff. It only starts the airborne calibration after the pilot arms in Brake and manually takes off.

Expected GCS messages:
- Enter Brake: `Brake mode: SLAM calibration fused with Brake mode is active.`
- Brake but disarmed: `Brake mode detected. Waiting for arm to start SLAM calibration.`
- Armed on ground in Brake: `Armed in Brake mode. SLAM calibration takeoff sequence active.`
- At rangefinder target height: `Reached 5 meters by rangefinder. Holding altitude for SLAM calibration.`
- During airborne calibration: `SLAM calibration active.`
- Success: `Calibration successful: SLAM PosHold calibration complete. Initiating RTL.`
- Failure: `Calibration failed: not finished. Reason: <reason>`

Safety gates:
- Calibration movement is blocked unless armed, still in Brake, MAVLink heartbeat is present, RC link is active, rangefinder is valid, VIO tracking is healthy, IMU is present, and FC health messages are clear.
- If the pilot changes mode, RC link drops, rangefinder fails, VIO tracking is lost, drift exceeds the configured limit, or a stage times out, calibration stops and the configured fallback mode is requested.
- Gentle pitch/roll/yaw calibration nudges are configurable but disabled by default with `calibration.movement_commands_enabled: false`.
- Rangefinder height is primary for this routine. Default target is `calibration.target_height_m: 5.0`.

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

7) Bundled diagnostics
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
- Stationary calibration requires the drone to be powered on, seated still, connected to the Flight Controller, with rangefinder, IMU, MAVLink, and VIO data available. GPS is not required for the basic stationary health check.

Support and next steps
- Replace the experimental `vio` source with a production VIO/VINS/ORB-SLAM backend for flight-grade use.
- Add time-synchronized IMU capture on the Jetson for robust visual-inertial fusion.

License / Contributing
- Fork and iterate; open a PR with improvements to the VIO backend or calibration tooling.

Hardware notes

IM10A IMU:

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
