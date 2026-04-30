# Intellisense SLAM Field Manual

Jetson-side SLAM/VIO, sensor monitoring, GPS-assisted learning, Brake-mode
calibration, and MAVLink GPS2 bridging for a Cube Orange+ ArduPilot drone.

This repository is a field-test project. It is not a certified autopilot, not a
drop-in safety system, and not a guarantee of GPS-denied flight. Treat every
SLAM/GPS2/PosHold feature as experimental until logs, GCS messages, fallback
behavior, and manual override are validated on the actual aircraft.

## What This Project Does

The Jetson runs a companion service that:

- connects to the Cube over MAVLink serial
- starts after boot through `intellisense_slam_bridge.service`
- reads VIO/SLAM pose, external IMU, rangefinder, GPS, EKF, RC, battery, and mode telemetry
- forwards useful telemetry and `STATUSTEXT` messages to QGC/MK15
- observes normal GPS LOITER flight to score SLAM quality
- runs the real SLAM calibration workflow when the pilot selects BRAKE
- feeds SLAM-derived position to ArduPilot through GPS2 MAVLink `GPS_INPUT`
- keeps MAVLink `ODOMETRY` suppressed in GPS2 mode to avoid VisOdom rejection
- logs field status to JSON and text logs for later review

The intended high-level flight sequence is:

1. Boot Jetson and Cube.
2. Let the bridge start automatically.
3. Wait for normal GPS readiness outdoors.
4. Take off and fly normally in LOITER.
5. Let LOITER observation collect stable reference data.
6. Switch to BRAKE to run the real calibration workflow.
7. Test SLAM/GPS2 PosHold only after the GCS says quality/calibration is ready.
8. Keep pilot override and LOITER fallback available at all times.

## What This Project Does Not Do

This project does not:

- arm the drone
- take off automatically
- climb from the ground automatically
- replace pilot judgment
- make GPS-denied PosHold automatically safe
- command movement in LOITER observation mode
- disable ArduPilot safety checks by itself
- make obstacle avoidance flight-ready without validation

The current config keeps active calibration movement commands off:

```yaml
calibration:
  movement_commands_enabled: false
```

## Current Flight Architecture

The active field-test method is **GPS2 via MAVLink `GPS_INPUT`**.

Important current behavior:

- ArduPilot VisOdom is disabled with `VISO_TYPE=0`.
- `ODOMETRY` is suppressed while GPS2 mode is active.
- `GPS2_TYPE=14` tells ArduPilot that GPS2 is MAVLink GPS.
- `GPS_AUTO_SWITCH=1` allows ArduPilot to use the best GPS source.
- Before SLAM is ready, GPS2 standby can mirror real GPS1 when GPS1 has a real 3D fix.
- When SLAM is ready, POSHOLD can receive the SLAM-corrected GPS2 stream.
- LOITER remains normal GPS/EKF flight.
- BRAKE remains the real calibration trigger.

Why GPS2 instead of VisOdom right now:

- ArduPilot's preferred non-GPS integration path is `ODOMETRY` / ExternalNav.
- This aircraft was reporting VisOdom unhealthy and memory/parser issues.
- The practical field-test path is therefore GPS2 `GPS_INPUT`.
- The code keeps this choice explicit so it can be reversed later if ExternalNav becomes reliable.

Official references:

- GPS_INPUT/MAVLink GPS type 14: https://ardupilot.org/mavproxy/docs/modules/GPSInput.html
- ArduPilot non-GPS position estimation: https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html

## Legacy Flow Bridge

The older flight logic that previously held GPS-less PosHold has been copied
into `legacy_flow_bridge/` as a parallel path. It keeps the old RealSense
optical-flow plus GPS2 `GPS_INPUT` method, then adds the new external IMU,
LOITER observation, Brake calibration bookkeeping, and non-zero GPS_INPUT
week/time fields.

Do not run it together with `intellisense_slam_bridge.service`.

Current field-machine boot setup:

- `config/autostart.yaml` has been moved to `config/autostart.yaml.disabled_legacy_boot`.
- That makes the still-enabled `intellisense_slam_bridge.service` skip itself on boot.
- The user's crontab starts `legacy_flow_bridge/legacy_boot_cron.sh` at reboot.
- The legacy launcher waits 30 seconds, then runs `legacy_flow_bridge/run_field_legacy.sh`.

```bash
sudo systemctl stop intellisense_slam_bridge.service
cd legacy_flow_bridge
./run_field_legacy.sh
```

Bench-only health stream:

```bash
cd legacy_flow_bridge
./run_field_legacy.sh --flow-health-test
```

## Safety Rules

Use these as hard rules while developing or field testing:

- Do not run two bridge processes at the same time.
- Do not run the passive MAVLink monitor during flight.
- Do not let Mission Planner or another process grab the same Cube serial port on the Jetson.
- Do not restart the bridge while the aircraft is airborne.
- Do not test SLAM PosHold until LOITER/GPS flight is stable.
- Do not trust SLAM when `vio_tracking=pnp_reject`, `vio=bad`, or score is critical.
- Do not test No-GPS behavior over people, vehicles, animals, structures, or tight spaces.
- Always keep a pilot mode switch ready for STABILIZE/LOITER/LAND.
- After changing `GPS2_TYPE`, `VISO_TYPE`, or EKF parameters, reboot the Cube.

## Repository Map

Short directory map:

```text
.
|-- README.md                         Public field manual
|-- AI_AGENT_CONTEXT.md                Dense handoff file for AI agents and new maintainers
|-- CALIBRATION_GUIDE.md               Extra calibration notes
|-- LIDAR_VERIFICATION.md              LiDAR-specific validation notes
|-- RUNBOOK_SLAM_BRINGUP.md            Older bring-up runbook
|-- ardupilot_lua/                     Cube-side Lua beeper/status scripts
|-- config/                            Field and default YAML configs
|-- hardware/                          USB rules and hardware driver notes
|-- install/                           Installer scripts for systemd and USB serial support
|-- scripts/                           User-facing runners, diagnostics, calibration tools
|-- src/slam_core/                     Shared Python library used by runners
|-- systemd/                           Service unit files
|-- tests/                             Unit and smoke tests
|-- tools/                             Hardware probing and live-view helpers
```

More detail:

- `scripts/calibration/brake_slam_calibration.py` is the systemd wrapper. It prevents duplicate bridge processes, resolves paths, prints status, and launches the child bridge.
- `scripts/runners/run_slam_odometry_bridge.py` is the main flight-facing loop.
- `src/slam_core/fc_config.py` contains MAVLink parameter, telemetry, GPS_INPUT, GCS message, and beep helpers.
- `src/slam_core/slam_observer.py` contains the LOITER soft-calibration observer and SLAM quality score.
- `src/slam_core/bridge_config.py` maps YAML config into dataclasses.
- `src/slam_core/qgc_bridge.py` forwards Cube MAVLink telemetry to UDP QGC/MK15 endpoints.
- `src/slam_core/vio_backend.py` and `src/slam_core/pose_sources.py` provide VIO pose samples.
- `src/slam_core/lidar.py` parses LiDAR/proximity data.
- `tests/` verifies message packing, config parsing, observer behavior, LiDAR filtering, QGC forwarding, and IMU handling.

## Active Service

The field service is:

```bash
intellisense_slam_bridge.service
```

Check it:

```bash
sudo systemctl status intellisense_slam_bridge.service
journalctl -u intellisense_slam_bridge.service -f
```

Restart it only on the ground:

```bash
sudo systemctl restart intellisense_slam_bridge.service
```

Install or refresh autostart:

```bash
cd /home/atas/vscode/intellisense_slam
sudo bash install/install_slam_bridge_autostart.sh
```

The service runs:

```bash
python3 scripts/calibration/brake_slam_calibration.py --config config/autostart.yaml
```

The wrapper launches:

```bash
python3 scripts/runners/run_slam_odometry_bridge.py --config <resolved-temp-config>
```

## Do Not Run In Field

The passive MAVLink monitor is for bench diagnosis only. Do not run it during
flight because it can fight with the SLAM bridge for the same Cube serial stream.

Stop and disable it:

```bash
sudo systemctl stop slam-mavlink-monitor.service
sudo systemctl disable slam-mavlink-monitor.service
```

Confirm:

```bash
systemctl is-active slam-mavlink-monitor.service
systemctl is-enabled slam-mavlink-monitor.service
```

Expected:

```text
inactive
disabled
```

## MAVLink Port Ownership

Only one Jetson process should own the Cube serial port.

Check:

```bash
fuser -v /dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00
```

Expected owner:

```text
python3 ... run_slam_odometry_bridge.py
```

Not OK:

- `MissionPlanner.exe`
- `src/main.py`
- another `run_slam_odometry_bridge.py`
- a second MAVLink reader on the same Cube serial port

QGC/MK15 should connect through telemetry radio or the UDP bridge, not by taking
the Jetson's serial connection away from the service.

## Current Autostart Config

Main field config:

```bash
config/autostart.yaml
```

Important field defaults:

```yaml
source: vio
fc_setup:
  viso_type: 0
  gps2_type: 14
  gps_auto_switch: 1
  select_source_set_on_stream: false
gps_input:
  enabled: true
  gps_id: 1
  update_rate_hz: 8.0
calibration:
  mode: BRAKE
  movement_commands_enabled: false
slam_observer:
  enable_loiter_observation: true
  enable_live_soft_correction: true
  enable_auto_fallback_to_loiter: true
```

Meaning:

- `source: vio` means the bridge uses the VIO backend as the pose source.
- `viso_type: 0` keeps ArduPilot Visual Odometry disabled.
- `gps2_type: 14` configures GPS2 as MAVLink GPS input.
- `gps_id: 1` sends MAVLink GPS_INPUT as the second GPS.
- `update_rate_hz: 8.0` keeps GPS2 updates faster than ArduPilot's GPS health timing window; slow GPS2 updates show up as `GPS 2: not healthy` even when lat/lon/satellites are present.
- `select_source_set_on_stream: false` avoids live EKF source-set switching for the current GPS2 path.
- `movement_commands_enabled: false` means calibration does not command takeoff or motion.
- `enable_live_soft_correction: true` lets LOITER observation learn bounded yaw/scale/XY corrections for the GPS2 SLAM feed.
- `enable_auto_fallback_to_loiter: true` lets the bridge request LOITER only when SLAM quality is critical and real GPS is healthy.

## Status Command

Use this to see the latest bridge state:

```bash
python3 scripts/calibration/brake_slam_calibration.py --status
```

Raw JSON status:

```bash
cat logs/slam_calibration_status.json
cat logs/slam_loiter_observer_status.json
```

Healthy ground-monitoring example:

```text
state=IDLE
stage=idle
mode=STABILIZE or LOITER
armed=False
vio=ok
imu=stable
mavlink=ok
rc_link=ok
ekf_external_nav=gps2_bridge
odom_sent=0
```

Not ready examples:

```text
gps1=1/0
gps2=1/0
vio=bad
vio_tracking=pnp_reject
imu=missing
mavlink=timeout
observer=2.5/10
recommendation=critical
```

## Logs

Main status and calibration logs:

```bash
logs/slam_calibration_status.json
logs/slam_calibration.log
```

LOITER observation logs:

```bash
logs/slam_loiter_observer_status.json
logs/slam_loiter_observer.log
```

Follow service logs:

```bash
journalctl -u intellisense_slam_bridge.service -f
```

Follow local logs:

```bash
tail -f logs/slam_calibration.log
tail -f logs/slam_loiter_observer.log
```

## GCS Messages

The bridge sends `STATUSTEXT` messages prefixed with `SLAM:`.

Typical startup messages:

```text
SLAM: JETSON EVENT: Jetson booted; SLAM bridge script started; MAVLink connected on /dev/...
SLAM: FC SLAM setup already matched source 3
SLAM: VIO feed method: GPS2 GPS_INPUT. VisOdom/ExternalNav is disabled.
SLAM: MAVLink ODOMETRY is suppressed in GPS2 mode to avoid VisOdom health errors.
SLAM: GPS2 stream is gated by Brake calibration or LOITER observer quality.
SLAM: SLAM observer ready. LOITER soft calibration available.
SLAM: BEEP: startup check alive after 30s; monitoring only until FIELD GATE OK
SLAM: FIELD GATE WAIT: GPS1 not ready fix=1 sats=0; GPS2 standby not confirmed fix=1 sats=0
SLAM: JETSON EVENT: script running; disarmed mode=STABILIZE; waiting for FIELD GATE OK.
SLAM: FIELD GATE OK: GPS LOITER and BRAKE calibration inputs ready. Wait for NO-GPS POSHOLD GATE before SLAM PosHold.
```

`FIELD GATE OK` is the message to wait for before treating the outdoor field
setup as ready for normal GPS LOITER and Brake calibration. It does **not** mean
GPS-less PosHold is ready. For GPS-less PosHold, wait for:

```text
SLAM: NO-GPS POSHOLD GATE OK: Brake calibration profile ready score=8.0/10. POSHOLD can use SLAM/VIO GPS2 feed cautiously.
```

If the bridge cannot reach the field gate, it sends `FIELD GATE WAIT` every 20
seconds or whenever the blocking reason changes.

The bridge also sends a slow `JETSON EVENT` heartbeat about every 30 seconds,
plus immediate messages when arm/disarm state changes. These messages mean the
Jetson script is alive even if the aircraft is still waiting for GPS, EKF,
rangefinder, IMU, or arm state.

Typical LOITER messages:

```text
LOITER active: SLAM observation mode started.
SLAM observing LOITER data for soft calibration.
SLAM quality weak: 4.2/10. Use LOITER longer or run calibration.
SLAM quality ready for No-GPS PosHold: 7.3/10
```

Typical GPS2 messages:

```text
SLAM: GPS2 standby mirrors real GPS until SLAM is ready.
SLAM: GPS2 origin locked from healthy GPS/EKF reference.
SLAM: VIO mirrored to GPS2 GPS_INPUT.
SLAM: No-GPS POSHOLD active: SLAM/VIO GPS2 feed is flying without real GPS.
```

Typical safety messages:

```text
SLAM quality critical: fallback to LOITER recommended.
SLAM quality critical: switching to LOITER.
```

## Beeps

The Cube Lua beeper listens to bridge state params:

```bash
ardupilot_lua/brake_slam_beeper.lua
```

Install it on the Cube SD card:

```text
APM/scripts/brake_slam_beeper.lua
```

Then reboot the Cube.

Current intent:

- 30 seconds after Jetson/MAVLink start: three short beeps plus `SLAM: BEEP: startup check alive after 30s; monitoring only until FIELD GATE OK`
- sensor quick check passed: one short beep plus `SLAM: BEEP: sensor quick check passed; VIO/IMU basic health only, not full readiness`
- field readiness: GCS text only, `SLAM: FIELD GATE OK...`
- No-GPS PosHold gate: rising tone plus `SLAM: NO-GPS POSHOLD GATE OK...`
- Brake detected while disarmed/on ground: GCS warning plus the explicit ground/takeoff reason
- active calibration: calibration tone plus `SLAM: BEEP: BRAKE calibration active; hold altitude and keep pilot override ready`
- success: rising long tone plus `SLAM: BEEP: calibration successful; SLAM PosHold calibration profile saved`
- failure: descending warning tone plus `SLAM: BEEP: SLAM calibration failed; leave SLAM PosHold disabled`
- real SLAM/GPS2 PosHold active: short heartbeat beep plus a GCS message every 10 seconds

The source of truth is Jetson `STATUSTEXT`. Every Jetson-commanded beep is
prefixed with `SLAM: BEEP:` so the operator sees the reason for the sound. The
Lua helper on the Cube is only a backup status relay and should not duplicate
normal Jetson startup/ready/calibration tunes. If the aircraft still plays the
old musical tune immediately after entering Brake while disarmed, the Cube is
still running an old Lua script from its SD card.

## LOITER Soft Calibration

LOITER remains normal GPS/EKF flight. The Jetson observer does not change
ArduPilot LOITER control, does not override GPS, does not command movement, and
does not interfere with pilot input.

While the vehicle is in LOITER, the observer compares:

- GPS position and velocity
- EKF local position and velocity
- yaw, pitch, and roll
- throttle
- IMU sample
- rangefinder or LiDAR height when available
- barometer altitude
- EKF status
- VIO pose and velocity
- VIO drift
- SLAM tracking state
- MAVLink heartbeat health

The score is `0.0` to `10.0`:

- `9.0-10.0`: excellent
- `7.0-8.9`: good
- `5.0-6.9`: usable but keep observing
- `3.0-4.9`: weak, run LOITER longer or run real calibration
- below `3.0`: unsafe for No-GPS PosHold

Live soft correction:

- learned only from LOITER data
- bounded for yaw, scale, and XY offset
- applied only to the gated GPS2 SLAM feed
- not applied to normal LOITER control
- not applied if SLAM quality is below the configured threshold

The startup beep/status delay is 30 seconds. During real SLAM/GPS2 POSHOLD,
the bridge sends the No-GPS POSHOLD status message every 10 seconds while the
SLAM-derived GPS2 stream is actually being sent.

## Brake Mode Calibration

BRAKE is the real calibration trigger.

Disarmed on ground:

```text
Brake mode detected. Waiting for arm to start SLAM calibration.
```

Expected state:

```text
state=WAITING_FOR_ARM
stage=waiting_arm
```

Armed on ground:

```text
Armed in Brake mode. SLAM calibration takeoff sequence active.
No automatic takeoff: pilot must take off and enter Brake near 5m.
```

Airborne near target height:

```text
Reached 5 meters by rangefinder. Holding altitude for SLAM calibration.
SLAM calibration active.
```

Success:

```text
Calibration successful: SLAM PosHold calibration complete. Initiating RTL.
```

Failure:

```text
Calibration failed: not finished. Reason: <reason>
```

The calibration compares SLAM/VIO against GPS/EKF reference data. It needs a
healthy GPS/EKF reference for the current workflow.

## Field Test Workflow

Use this sequence for the current GPS-assisted validation path:

1. Power Cube and Jetson.
2. Confirm the bridge service is active.
3. Confirm passive monitor is disabled.
4. Confirm only one process owns the Cube serial port.
5. Open QGC/MK15.
6. Wait outside for GPS1 3D fix and good satellites.
7. Confirm QGC does not show blocking prearm errors.
8. Arm in a normal pilot-controlled mode.
9. Take off normally.
10. Switch to LOITER and verify normal GPS flight.
11. Let LOITER observation run.
12. Watch GCS messages and `logs/slam_loiter_observer.log`.
13. Switch to BRAKE to run real calibration.
14. Wait for calibration success or explicit failure reason.
15. Return to LOITER if anything looks wrong.
16. Only test POSHOLD after SLAM quality/calibration is ready.

Recommended first tests:

- tethered or prop-guarded
- wide open field
- low altitude
- short mode changes
- one change at a time
- logs reviewed after each hop

## Bench Checks Before Field

Run from repo root:

```bash
cd /home/atas/vscode/intellisense_slam
```

Smoke checks:

```bash
python3 scripts/diagnostics/run_smoke_checks.py
```

Hardware checks:

```bash
python3 scripts/diagnostics/check_mavlink.py
python3 scripts/diagnostics/check_realsense.py
python3 scripts/diagnostics/check_imu.py
python3 scripts/diagnostics/check_rangefinder.py
python3 scripts/diagnostics/check_hesai_jt16.py
```

VIO checks:

```bash
python3 scripts/diagnostics/check_vio_drift.py
python3 scripts/runners/bench_vio.py
```

Brake dry-run:

```bash
python3 scripts/calibration/brake_slam_calibration.py --dry-run
```

LOITER observer config check:

```bash
python3 scripts/slam_loiter_observer.py --dry-run
python3 scripts/slam_loiter_observer.py --status
```

Stationary calibration:

```bash
python3 scripts/calibration/run_stationary_calibration.py --indoor --no-gps --verbose
```

## Development Workflow

Install dependencies:

```bash
python3 -m pip install -r requirements.txt
```

Run tests:

```bash
python3 scripts/diagnostics/run_smoke_checks.py
```

Compile check:

```bash
python3 -m compileall src scripts tests
```

Inspect changed files:

```bash
git status --short
git diff --stat
```

When changing flight behavior:

- add or update tests when possible
- keep defaults conservative
- add GCS messages for operator-visible state changes
- write status to logs
- avoid silent fallback behavior
- document whether a Cube reboot is required
- test with props off before field use

## QGC / MK15 Telemetry

The bridge can forward MAVLink to UDP for QGC:

```yaml
qgc:
  enabled: true
  forward_host: 127.0.0.1
  forward_port: 14550
  bind_host: 0.0.0.0
  bind_port: 14555
```

Behavior:

- downlink from the Cube is forwarded to QGC
- uplink from QGC is relayed back to the Cube
- localhost output also adds LAN broadcast for same-network MK15/QGC visibility

If MK15 is not on the same subnet, set `qgc.forward_host` to the MK15 IP and
restart the bridge on the ground.

## GPS2 Bad Fix Notes

`GPS2_TYPE=14` means GPS2 expects MAVLink `GPS_INPUT`.

Before GPS1 has outdoor 3D fix:

- GPS2 may show `fix=1`
- GPS2 may have zero lat/lon
- this is expected indoors

After GPS1 has outdoor 3D fix:

- GPS2 standby should mirror real GPS
- GCS should see the standby mirror message
- GPS2 should stop being permanently bad
- `GPS_INPUT` must include non-zero GPS week and week-milliseconds
- DataFlash `GPS` for instance `I:1` should show non-zero `GWk` and `GMS`
- DataFlash `GPA` for instance `I:1` should keep `Delta` below 200 ms; if it is around 250-300 ms, ArduPilot can report `GPS 2: not healthy` even when GPS2 position and satellites look valid.

When POSHOLD/SLAM is gated ready:

- the bridge locks a GPS2 origin from the healthy GPS/EKF reference
- GPS2 can switch from standby mirror to SLAM-derived GPS_INPUT
- GCS should show `VIO mirrored to GPS2 GPS_INPUT`

If Cube still complains after changing GPS params:

```bash
sudo systemctl stop intellisense_slam_bridge.service
# reboot or power-cycle the Cube
sudo systemctl start intellisense_slam_bridge.service
```

## Common Troubleshooting

Service not running:

```bash
sudo systemctl restart intellisense_slam_bridge.service
sudo systemctl status intellisense_slam_bridge.service
```

MAVLink timeout:

- check Cube USB cable
- check `/dev/serial/by-id`
- make sure no other app owns the port
- restart the service on the ground

QGC crashing during parameter download:

- stop duplicate MAVLink readers
- keep QGC off the Jetson serial port
- let QGC use telemetry radio or UDP bridge
- avoid running Mission Planner on the same Cube port

`VisOdom unhealthy`:

- current GPS2 profile suppresses `ODOMETRY`
- confirm `odom_sent=0` in status
- confirm `VISO_TYPE=0`
- reboot Cube if VISO/GPS params changed

`GPS 1: Bad fix`:

- move outdoors
- wait for 3D fix and enough satellites
- do not expect LOITER calibration indoors without GPS reference

`GPS2 bad fix`:

- indoors this is expected until GPS1 has a healthy reference
- outside, GPS2 standby should mirror real GPS
- confirm bridge is running
- confirm `GPS2_TYPE=14`
- check DataFlash `GPS I:1`: if `Status` and satellites look good but `GWk=0`
  and `GMS=0`, the GPS_INPUT timestamp is invalid; restart with the patched bridge

`vio_tracking=pnp_reject`:

- VIO cannot currently trust the camera pose
- improve lighting and features
- check camera exposure
- check rangefinder height
- do not test SLAM PosHold while this persists

`observer score critical`:

- keep flying normal LOITER or run calibration
- do not switch into SLAM PosHold
- review `logs/slam_loiter_observer.log`

## LiDAR / Obstacle Tools

LiDAR support is present but not the current flight-readiness gate.

Check Hesai JT16:

```bash
python3 scripts/diagnostics/check_hesai_jt16.py
```

Visualize LiDAR:

```bash
python3 scripts/avoidance/visualize_lidar_avoidance.py
```

Dry-run obstacle node:

```bash
python3 scripts/avoidance/hesai_jt16_obstacle_node.py --dry-run
```

Do not enable real obstacle movement until SLAM flight itself is stable.

## Recovery Commands

Restart bridge:

```bash
sudo systemctl restart intellisense_slam_bridge.service
```

Stop old or extra services:

```bash
sudo systemctl stop slam-mavlink-monitor.service
sudo systemctl disable slam-mavlink-monitor.service
sudo systemctl stop vio-flight.service 2>/dev/null || true
sudo systemctl disable vio-flight.service 2>/dev/null || true
```

Refresh USB serial support:

```bash
sudo bash install/enable_usb_serial_sensors.sh
sudo bash install/install_usb_serial_sensors_autostart.sh
```

Reinstall all project services:

```bash
sudo bash install/install_all.sh
```

## Public Project Status

Current status:

- ground monitoring is usable
- service autostart is usable
- GPS2 bridge path is implemented
- LOITER observation is implemented
- live soft correction is implemented with bounds
- auto fallback to LOITER is implemented and gated by healthy GPS
- real Brake calibration workflow remains separate

Still requiring real flight validation:

- GPS2 standby mirror with outdoor GPS lock
- LOITER observation score behavior in real flight
- Brake calibration completion in real flight
- GCS message visibility on MK15/QGC
- SLAM drift under aircraft vibration
- GPS2 SLAM feed behavior during cautious POSHOLD tests
- auto fallback to LOITER
- pilot manual override
- obstacle avoidance integration

Do not describe the project as fully ready for No-GPS flight until those items
are validated with logs.

## Glossary

- **Cube**: Cube Orange+ flight controller running ArduPilot.
- **Jetson**: companion computer running this repo.
- **GCS**: ground control station, usually QGC/MK15.
- **LOITER**: normal ArduPilot GPS/EKF hold mode used as a safe observation reference.
- **BRAKE**: pilot-selected mode used here as the real SLAM calibration trigger.
- **POSHOLD**: target mode for cautious SLAM/GPS2 position-hold testing.
- **VIO**: visual inertial odometry from camera/IMU.
- **SLAM quality score**: 0 to 10 observer score estimating whether SLAM is usable.
- **GPS_INPUT**: MAVLink message that lets a companion computer provide GPS-like data.
- **GPS2 bridge**: current method for feeding SLAM pose to ArduPilot as MAVLink GPS2.
- **VisOdom / ExternalNav**: ArduPilot visual odometry path using MAVLink `ODOMETRY`; currently disabled here.
- **Soft correction**: bounded yaw/scale/XY correction learned during LOITER and applied only to GPS2 SLAM pose after quality is good.
- **Fallback**: warning or automatic switch back to LOITER if SLAM quality becomes critical and real GPS is healthy.
