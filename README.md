# Intellisense SLAM

This repo is the clean start for a real GPS-denied navigation stack.

## Current reality

We do **not** yet have every ingredient for a flight-grade SLAM drone.

What we already have:

- Jetson companion computer
- Cube flight controller
- working MAVLink stack on Jetson
- RealSense D415 camera access
- working JT16 lidar serial path on Jetson
- working IM10A external IMU serial path on Jetson
- a clean path to send `ODOMETRY` to the Cube
- a SLAM-owned FC setup path that prepares EKF source set 3 for ExternalNav

What is still missing for robust flight SLAM:

- a **time-synchronized IMU** available to the Jetson-side SLAM backend
- a real SLAM/VIO backend that outputs stable pose continuously
- calibration between camera, IMU, and vehicle frame
- controlled validation before airborne use

Important limitation:

- the RealSense **D415 does not include an IMU**
- that means this hardware is not enough by itself for strong visual-inertial odometry
- the Cube IMU is useful for flight control, but it is not a drop-in replacement for a properly synchronized SLAM IMU on the Jetson

So the honest answer is:

- enough to **start the SLAM repo and integration work**: yes
- enough for **reliable flight-ready SLAM today**: no

This repo starts the right way anyway.

## Current working rule

At this stage, the recommended development flow is:

- run VIO locally on the Jetson first
- verify trajectory, tracking state, and IMU binding without touching the Cube
- only after that, test FC-facing output paths deliberately and one at a time

Why:

- the Cube can run out of resources when pushed down the wrong visual-odometry path
- a shared telemetry path can interfere with the GCS link if the bridge is not isolated carefully

Current FC-side bridge rule:

- SLAM only becomes active in `POSHOLD`
- calibration runs only in `BRAKE`
- outside `POSHOLD`, the bridge releases SLAM and returns to the idle source set
- the ready chime only means "SLAM PosHold is actually ready"
- when enabled, Cube rangefinder height is used as the outgoing SLAM height reference
- the bridge will not declare `POSHOLD` ready until a saved calibration profile exists and the current pose stream is healthy

## Bridge events and tones

The bridge now announces its important states directly over MAVLink `STATUSTEXT` and `PLAY_TUNE`. The bundled Lua relay mirrors the same state codes on the FC side.

- `Jetson SLAM bridge initiated`
  Trigger: 60 seconds after MAVLink is detected.
  Tone: 3 short beeps.
- `Sensor quick check passed`
  Trigger: IMU, VIO pose, and rangefinder data are present and plausible enough to move toward readiness.
  Tone: 1 short beep.
- `SLAM ready for PosHold`
  Trigger: calibration profile exists, current pose stream is healthy, rangefinder is valid, and no recent FC warning is blocking SLAM.
  Tone: rising musical beep.
- `SLAM calibration active`
  Trigger: vehicle enters `BRAKE` and the calibration preconditions are satisfied.
  Tone: rising-low musical pattern.
- `Calibration complete, switching to RTL`
  Trigger: `BRAKE` calibration finishes and the saved profile passes stability checks.
  Tone: rising musical long beeps.
- `SLAM flight active`
  Trigger: vehicle is armed, in `POSHOLD`, using the SLAM source set, and the live pose stream remains healthy.
  Tone: 1 small beep every 6 seconds.
  GCS text: `SLAM flight active` every 10 seconds.

## Calibration flow

The current calibration mode is `BRAKE`.

Why `BRAKE`:

- it is not one of the banned modes
- it lets the FC hold position with GPS while the Jetson measures its own pose bias against the FC reference
- it is a much safer calibration reference than trying to calibrate while SLAM is already controlling `POSHOLD`

What gets calibrated:

- yaw offset between Jetson pose and FC attitude reference
- XY origin offset between the Jetson pose frame and the FC local-position frame
- rangefinder-backed height sanity as part of the plausibility checks

Where the profile is stored:

- `runtime/slam_calibration.json`

What happens in flight:

- switch the vehicle to `BRAKE`
- Jetson waits for healthy GPS, FC local position, FC attitude, rangefinder, and a stable VIO pose
- once the vehicle is settled, Jetson captures a stationary calibration window
- if the sample is stable enough, Jetson saves the calibration profile
- Jetson announces completion and commands `RTL`

## What this repo does today

- checks whether the current hardware is ready for SLAM work
- captures RealSense data and reports whether an IMU is present
- checks whether the external IM10A IMU is actually streaming healthy packets
- runs a local-only VIO session that does not touch Cube or GCS
- sends external navigation `ODOMETRY` to the Cube from a pose source
- can bind external IMU orientation and angular rates into that bridge
- auto-configures the Cube for a dedicated SLAM ExternalNav source set without overwriting the current GPS and flow source sets
- provides demo pose sources so the bridge path can be tested now
- creates a clean place to plug in a real SLAM backend next

## Repo layout

- `scripts/check_slam_readiness.py`
  Prints a practical readiness report for the current Jetson hardware.
- `scripts/check_external_imu.py`
  Verifies that the IM10A external IMU is alive and decodes usable orientation data.
- `scripts/run_local_vio.py`
  Runs the current VIO backend locally on Jetson, logs it, and optionally shows a preview without opening the FC or GCS links.
- `scripts/run_slam_odometry_bridge.py`
  Sends `ODOMETRY` to the Cube from a pose source, with optional external-IMU orientation binding, calibration-profile application, config loading, GCS announcements, and reconnect behavior for service use.
- `scripts/configure_fc_for_slam.py`
  Applies the FC-side ExternalNav parameters used by the SLAM bridge and preserves the existing GPS and optical-flow setup by writing into EKF source set 3.
- `scripts/jt26_to_mavlink.py`
  Reads JT16/JT26 packets and publishes MAVLink `DISTANCE_SENSOR` for obstacle bench tests.
- `install_slam_bridge_autostart.sh`
  Installs the SLAM bridge as a systemd autostart service.
- `install_jt26_mavlink_autostart.sh`
  Installs the JT26-to-MAVLink publisher service from this repo.
- `tools/`
  Self-contained IMU and JT16 diagnostic/viewer tools that no longer depend on `intellisense_cam`.
- `ardupilot_lua/jetson_nogps_status.lua`
  FC-side Lua relay for Jetson bridge state messages.
- `hardware/`
  Jetson-side USB-serial driver modules plus the sensor-restore service installer used by the SLAM stack.
- `src/intellisense_slam/readiness.py`
  Hardware and dependency checks.
- `src/intellisense_slam/external_imu.py`
  IM10A serial probing, packet parsing, and bridge-side IMU binding helpers.
- `src/intellisense_slam/realsense_capture.py`
  RealSense inspection and simple stream capture helpers.
- `src/intellisense_slam/pose_sources.py`
  Pose-source interfaces and demo providers.
- `src/intellisense_slam/mavlink_bridge.py`
  MAVLink connection and `ODOMETRY` send path.
- `config/default.yaml`
  Starter config for the odometry bridge.
- `config/autostart.yaml`
  Flight config for the systemd SLAM bridge service. It runs `vio`, calibrates in `BRAKE`, and only starts acting on the FC once `POSHOLD` is selected and the bridge is actually ready.

## Quick start

### 1. Check SLAM readiness

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/check_slam_readiness.py
```

This will tell you:

- whether Jetson sees the D415
- whether the D415 exposes an IMU
- whether the JT16 serial port exists
- whether the external IM10A IMU is alive enough for SLAM work
- whether Python dependencies are present
- whether the current setup is only good for prototype work or ready for the next step

### 2. Check the external IMU directly

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/check_external_imu.py
```

This should report:

- the bound IM10A serial port
- the selected baud
- whether valid IM10A packets are being decoded
- roll/pitch/yaw, gyro, acceleration, magnetometer, and pressure/altitude

### 3. Run local VIO first

This is the recommended next step right now.

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_local_vio.py
```

That path:

- opens RealSense + external IMU
- uses Cube rangefinder height by default when the Cube link is available
- runs the in-repo VIO backend locally
- does not forward or proxy any GCS telemetry
- shows tracking state, quality, features, inliers, and trajectory

If you want to keep the local runner camera-only and ignore Cube height:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_local_vio.py \
  --cube-height off
```

To record a session:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_local_vio.py \
  --csv-out /tmp/local_vio_session.csv
```

Near-field bench note:

- at around `0.2 m` above the ground, the D415 can fall into `low_depth_support`
- that usually makes the local preview stable, but it is not a strong XY-repeatability test yet
- judge the current backend at realistic hover heights, not only on a tabletop

### 4. Test the MAVLink external-nav path only after local VIO looks sane

Prepare the Cube once for the SLAM source set:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/configure_fc_for_slam.py \
  --config /home/atas/vscode/intellisense_slam/config/default.yaml
```

This keeps:

- source set 1 for GPS
- source set 2 for the current optical-flow / GPS-input path
- source set 3 for SLAM ExternalNav

Before using `POSHOLD` with SLAM for the first time, do one calibration flight in `BRAKE` so `runtime/slam_calibration.json` exists.

Then run the demo `ODOMETRY` bridge:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_slam_odometry_bridge.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --source hover
```

By default, that bridge now tries to bind the external IM10A IMU into the outgoing pose stream. If you want to disable that and use only the demo pose source:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_slam_odometry_bridge.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --source hover \
  --imu off
```

Other demo sources:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_slam_odometry_bridge.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --source circle
```

You can now try the experimental depth-aware VIO backend directly from the bridge using the `vio` source. This uses the RealSense depth+infrared streams and a simple PnP-based frame-to-frame estimator as a starting point:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_slam_odometry_bridge.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --source vio
```

Notes:
- This `vio` provider is an experimental, in-repo visual odometry starter. It is useful for bench testing and iterative improvement but is not yet a full VIO/SLAM system.
- For flight-grade SLAM you should replace this with a robust VIO (ORB-SLAM3, VINS-MONO, or similar) or add IMU-visual fusion and careful timestamps.
- The bridge intentionally refuses to mark `POSHOLD` as ready until calibration has been completed and the current pose stream still passes the live plausibility checks.

Bench test helper:

 - `scripts/bench_vio.py` records a short VIO+IMU CSV and can optionally stream ODOMETRY to the Cube during bench testing.

Example:

```bash
python3 scripts/bench_vio.py --out /tmp/bench.csv --duration 30
```

To stream ODOMETRY to the Cube during the bench run (use cautiously):

```bash
python3 scripts/bench_vio.py --out /tmp/bench.csv --duration 30 --send --ports /dev/ttyACM1 /dev/ttyACM2
```

Replay a CSV pose file:

```bash
python3 /home/atas/vscode/intellisense_slam/scripts/run_slam_odometry_bridge.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --source csv \
  --csv-path /path/to/local_pose.csv
```

Expected CSV columns for replay:

- `t_s`
- `x_m`
- `y_m`
- `z_m`
- optional `yaw_deg`
- optional `vx_m_s`
- optional `vy_m_s`
- optional `vz_m_s`

Important note before using the bridge with `source: vio`:

- the backend is still experimental
- the local-only runner is the safer validation path
- do not treat the FC-facing bridge as the first-line test tool for VIO iteration
- the bridge now waits for `POSHOLD` before it starts acting on SLAM
- outside `POSHOLD`, the VIO stack stays alive but the bridge sends `0` odometry packets to the FC
- when Cube rangefinder height is healthy, the outgoing SLAM `z` height is replaced with that Cube distance-sensor value
- the ready chime is intentionally delayed until the FC source set is active, the pose is healthy, and rangefinder height is valid
- there is no startup musical chime anymore; the sound is reserved for real GPS-less `POSHOLD` readiness

### 5. Install the boot services

Install the Jetson USB-serial sensor restore service and the SLAM bridge service:

```bash
sudo bash /home/atas/vscode/intellisense_slam/install_slam_bridge_autostart.sh
```

What that gives you:

- the USB-serial sensor restore service is now owned by `intellisense_slam/hardware`
- the stable serial symlinks remain:
  - `/dev/jt16_usb`
  - `/dev/imu_usb`
- the SLAM bridge service starts from:
  - `/home/atas/vscode/intellisense_slam/config/autostart.yaml`

Important safety default:

- `config/autostart.yaml` uses `source: vio`
- the service warms the camera + IMU stack at boot, but it still stays passive until `POSHOLD`
- the ready tone only means “GPS-less `POSHOLD` is actually ready now”
- if you want to work on the local VIO runner while the service is installed, stop it first so it releases the camera:

```bash
sudo systemctl stop intellisense_slam_bridge.service
```

Useful commands:

```bash
systemctl status intellisense_slam_bridge.service
journalctl -u intellisense_slam_bridge.service -f
```

The bridge now also auto-applies the FC-side SLAM setup on connect using the `fc_setup` section in the YAML config. By default it prepares EKF source set 3 for ExternalNav and only switches into that source once live `ODOMETRY` has been sent for a short warmup.

### 6. Bundled local tools

IMU:

```bash
python3 /home/atas/vscode/intellisense_slam/tools/imu_stream_check.py
python3 /home/atas/vscode/intellisense_slam/tools/imu_serial_preview.py --port auto --baud auto
```

JT16:

```bash
python3 /home/atas/vscode/intellisense_slam/tools/jt16_validate_connection.py --no-probe
python3 /home/atas/vscode/intellisense_slam/tools/jt16_serial_probe.py --port auto
python3 /home/atas/vscode/intellisense_slam/tools/jt16_live_view.py --port auto
```

ArduPilot Lua relay to copy onto the FC SD card:

```bash
/home/atas/vscode/intellisense_slam/ardupilot_lua/jetson_nogps_status.lua
```

## External IMU note

The current RealSense D415 does not provide an onboard motion sensor, so the IM10A is now the Jetson-side inertial source for SLAM integration work.

Current known-good behavior on this Jetson:

- USB device `1a86:7523`
- default useful baud: `9600`
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
