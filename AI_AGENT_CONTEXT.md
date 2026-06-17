# AI Agent Context Dictionary


Read this file before changing the repository. It is written for another AI
agent, a new developer, or a field engineer who needs the whole project shape
without reading every file first.

## One Sentence

This repo runs a Jetson companion service that observes and calibrates SLAM/VIO
against normal GPS-assisted ArduPilot flight, then cautiously feeds SLAM-derived
position to a Cube Orange+ through MAVLink GPS2 `GPS_INPUT`.

## Current Operating Truth

- Active boot path on this Jetson: user crontab launches
  `legacy_flow_bridge/legacy_boot_cron.sh`.
- Active field bridge process: `legacy_flow_bridge/realsense_optical_flow_to_cube.py`
  through `legacy_flow_bridge/run_field_legacy.sh`.
- Active field bridge config is encoded in `legacy_flow_bridge/run_field_legacy.sh`
  command-line flags plus `runtime/legacy_flow_calibration.json`.
- The newer `intellisense_slam_bridge.service` is intentionally blocked on this
  field machine because `config/autostart.yaml` was moved to
  `config/autostart.yaml.disabled_legacy_boot`.
- Newer-service wrapper, inactive during legacy boot:
  `scripts/calibration/brake_slam_calibration.py`.
- Newer-service bridge loop, inactive during legacy boot:
  `scripts/runners/run_slam_odometry_bridge.py`.
- Current pose feed method: GPS2 `GPS_INPUT`
- Current Visual Odometry method: disabled in ArduPilot with `VISO_TYPE=0`
- MAVLink `ODOMETRY`: intentionally suppressed in GPS2 mode
- LOITER behavior: normal GPS/EKF flight, observation only
- BRAKE behavior: real calibration trigger
- POSHOLD behavior: target for cautious SLAM/GPS2 testing after gates are ready
- Movement commands: off by default
- Obstacle avoidance: Hesai JT16 LiDAR node runs beside the legacy bridge and
  uses `udpout:127.0.0.1:14555`, never direct Cube USB.
- Obstacle mode: `config/sensors.yaml` uses `avoidance.mode: "rc_toggle"`.
  RC7 low/neutral means detect-only (`PRX1_TYPE=2`, `AVOID_ENABLE=0`).
  RC7 high means ArduPilot native avoidance engaged (`AVOID_ENABLE=7`).
- Obstacle confirmation audio: RC7 high and RC7 low both play short mode
  confirmation tunes; obstacle-distance warning beeps remain armed-only.
- Reliable OA telemetry text on QGC/MK15 may require the updated Cube Lua script
  on the SD card as `APM/scripts/brake_slam_beeper.lua`.
- Auto takeoff: not implemented
- Auto fallback: enabled only for critical SLAM score plus healthy real GPS
- Live soft correction: enabled, bounded, and applied only to the GPS2 SLAM pose

## Non-Negotiable Safety Invariants

Do not break these:

- Never auto-arm.
- Never auto-takeoff.
- Never command movement in LOITER observation mode.
- Never run duplicate Cube serial readers in field mode.
- Never restart flight services while the aircraft is airborne.
- Never point the LiDAR obstacle node at `/dev/ttyACM0` while the legacy bridge
  is running; use `udpout:127.0.0.1:14555`.
- Never feed SLAM to GPS2 without a valid origin or standby GPS reference.
- Never send GPS2 `GPS_INPUT` with zero GPS week/time during field testing.
- Never claim No-GPS flight is fully ready without real flight validation logs.
- Keep real Brake calibration separate from LOITER soft observation.
- Keep pilot override and LOITER fallback visible in GCS messages.
- Keep RC7 low/neutral as the safe obstacle detect-only state before takeoff.

## Why GPS2 Is Used

ArduPilot's preferred non-GPS path is MAVLink `ODOMETRY` / ExternalNav. This
aircraft previously reported VisOdom unhealthy and memory/parser problems. The
current field-test path therefore disables VisOdom and mirrors SLAM into GPS2
using MAVLink `GPS_INPUT`.

## Parallel Legacy Flow Bridge

`legacy_flow_bridge/` is a separate path copied from the older
`intellisense_cam` repo that previously achieved GPS-less PosHold. It should not
be run at the same time as `intellisense_slam_bridge.service`.

Important files:

- `legacy_flow_bridge/realsense_optical_flow_to_cube.py`: old RealSense
  optical-flow GPS2 bridge with new external IMU, LOITER observer, and Brake
  calibration hooks.
- `legacy_flow_bridge/legacy_slam_support.py`: new support layer for scoring,
  bounded velocity-scale learning, GPS_INPUT week/time, and calibration profile
  writing.
- `legacy_flow_bridge/run_field_legacy.sh`: field command wrapper.

Default legacy method:

- RealSense optical flow plus down-facing range
- GPS2 MAVLink `GPS_INPUT` (`gps_id=1`)
- PosHold selects no-GPS source set 2 after healthy flow sends
- LOITER observes only and learns a bounded velocity scale
- BRAKE writes `runtime/legacy_flow_calibration.json`

That means:

- `fc_setup.viso_type: 0`
- `fc_setup.gps2_type: 14`
- `gps_input.enabled: true`
- `gps_input.gps_id: 1`
- `run_slam_odometry_bridge.py` should keep `odom_sent=0` in GPS2 mode
- `GPS_INPUT` senders must populate `time_week` and `time_week_ms`; a DataFlash
  `GPS I:1` entry with `GWk=0` and `GMS=0` can still trigger GPS2 unhealthy.
- `GPS_INPUT` must also arrive consistently faster than ArduPilot's GPS health
  timing window. Keep `gps_input.update_rate_hz` at 8 Hz or higher unless logs
  prove `GPA I:1 Delta` stays below 200 ms.

## End-to-End Runtime Flow

Current field-machine boot override:

- The newer `intellisense_slam_bridge.service` unit is still enabled at the
  systemd level, but it is intentionally blocked because
  `config/autostart.yaml` has been moved to
  `config/autostart.yaml.disabled_legacy_boot`.
- The user's crontab contains
  `@reboot /home/atas/vscode/intellisense_slam/legacy_flow_bridge/legacy_boot_cron.sh`.
- That launcher waits 45 seconds, locks
  `legacy_flow_bridge/runtime/legacy_flow_bridge.lock`, then runs
  `legacy_flow_bridge/run_field_legacy.sh`.
- The same launcher also starts `scripts/avoidance/hesai_jt16_obstacle_node.py`
  after an additional short delay, with `--mavport udpout:127.0.0.1:14555`.
- This means the next Jetson reboot should start the legacy GPS2 optical-flow
  bridge, not the newer SLAM/VIO bridge.

Default newer-bridge boot flow, not active while the legacy boot override is in
place:

1. systemd starts `intellisense_slam_bridge.service`.
2. systemd runs `scripts/calibration/brake_slam_calibration.py --config config/autostart.yaml`.
3. The wrapper resolves paths and refuses duplicate bridge instances.
4. The wrapper spawns `scripts/runners/run_slam_odometry_bridge.py`.
5. The bridge connects to the Cube over MAVLink serial.
6. The bridge applies ArduPilot params from `fc_setup`.
7. The bridge opens VIO, IMU, optional LiDAR, and optional QGC UDP bridge.
8. The bridge drains telemetry and writes status logs.
9. The bridge sends GCS messages and companion heartbeat.

Normal flight flow:

1. Pilot flies normal GPS LOITER.
2. `SlamLoiterObserver` watches GPS/EKF and VIO together.
3. Observer calculates drift, yaw error, scale, health, and quality score.
4. Observer logs to `logs/slam_loiter_observer.log`.
5. If GPS1 is healthy and SLAM is not ready, GPS2 standby mirrors real GPS.
6. If BRAKE is selected, real calibration workflow starts.
7. If POSHOLD is selected and gates are ready, GPS2 can receive SLAM pose.

Obstacle avoidance flow:

1. `legacy_boot_cron.sh` starts the JT16 node after the legacy bridge opens
   UDP port `14555`.
2. The JT16 node reads `config/sensors.yaml`.
3. Default mode is `avoidance.mode: "rc_toggle"`.
4. LiDAR always detects and publishes MAVLink `OBSTACLE_DISTANCE` so ArduPilot
   proximity stays connected.
5. RC7 low/neutral writes `AVOID_ENABLE=0`; this is detect-only.
6. RC7 high writes `AVOID_ENABLE=7`; this engages ArduPilot native avoidance.
7. The node sends GCS text and plays a short tune on both engage and disengage.
8. The node writes `SCR_USER4` for Cube Lua relay:
   `10` native active, `12` monitor/detect-only, `20` warning, `30` keepout,
   `40` critical, `90` stale.
9. Direct Jetson velocity pulses remain disabled unless explicitly switched to
   `direct_velocity` / `--enable-motion`; do not use that with props until
   no-prop testing is complete.

## Flight Modes

### STABILIZE

Manual attitude mode. The bridge monitors only. SLAM does not command movement.

### LOITER

Normal GPS/EKF ArduPilot hold. The observer is active, but does not control the
drone. Use this mode to collect safe reference data.

### BRAKE

Real calibration trigger. The code checks armed state, GPS/EKF reference,
rangefinder, RC link, battery, attitude, EKF status, and VIO health before
collecting calibration samples.

### POSHOLD

Target mode for cautious SLAM/GPS2 position-hold experiments. GPS2 SLAM feed is
gated by calibration or LOITER observer quality.

### RTL / LAND / ALTHOLD

Bridge should monitor and avoid autonomous navigation commands. These modes are
not where new SLAM motion logic should be introduced.

## Important Config Dictionary

### `config/autostart.yaml`

Primary field config. Edit this for actual service behavior.

Keys:

- `ports`: Cube serial candidates.
- `baud`: MAVLink serial baud.
- `source`: pose source, usually `vio`.
- `boot_delay_seconds`: optional Jetson sensor settle delay, default 30 seconds.
- `fc_setup`: ArduPilot parameter plan.
- `gps_input`: GPS2 MAVLink feed settings.
- `slam_observer`: LOITER observer and fallback settings.
- `calibration`: Brake-mode calibration workflow.
- `obstacle`: LiDAR/proximity publishing.
- `qgc`: UDP forwarding for QGC/MK15.

### `config/default.yaml`

Safer example/default config. Do not assume field service is using it.

### `config/sensors.yaml`

Sensor-specific settings.

## Directory Tree With Roles

```text
.
|-- README.md
|   Public field manual and first human-facing document.
|
|-- AI_AGENT_CONTEXT.md
|   This file. Dense handoff map for AI agents and new maintainers.
|
|-- CALIBRATION_GUIDE.md
|   Existing long-form calibration notes. Use as supporting context.
|
|-- LIDAR_VERIFICATION.md
|   LiDAR verification notes.
|
|-- RUNBOOK_SLAM_BRINGUP.md
|   Older bring-up sequence. Useful history, but README is more current.
|
|-- ardupilot_lua/
|   Cube-side Lua scripts.
|   `brake_slam_beeper.lua` maps SCR_USER bridge states to audible beeps.
|
|-- config/
|   YAML configuration.
|   `autostart.yaml` is active field config.
|   `default.yaml` is safer template/default.
|   `sensors.yaml` stores sensor details.
|
|-- hardware/
|   Hardware support files.
|   `configs/99-intellisense-usb-serial.rules` pins USB serial behavior.
|   `drivers/` contains local USB/IMU driver support.
|
|-- install/
|   System install scripts.
|   `install_slam_bridge_autostart.sh` installs the active bridge service.
|   `install_usb_serial_sensors_autostart.sh` makes USB serial support persist.
|   `install_all.sh` chains installation tasks.
|
|-- logs/
|   Runtime output. Usually not committed.
|   `slam_calibration_status.json` is the current health snapshot.
|   `slam_calibration.log` is the human-readable bridge log.
|   `slam_loiter_observer_status.json` is the observer snapshot.
|   `slam_loiter_observer.log` is JSONL observation history.
|
|-- scripts/
|   User-facing executables.
|
|-- scripts/avoidance/
|   LiDAR obstacle/proximity tools.
|   `hesai_jt16_obstacle_node.py` reads JT16, publishes `OBSTACLE_DISTANCE`,
|   handles RC7 detect-only/avoidance toggle, and writes `SCR_USER4`.
|   `visualize_lidar_avoidance.py` shows LiDAR sectors.
|
|-- scripts/calibration/
|   Calibration and service wrapper tools.
|   `brake_slam_calibration.py` is systemd wrapper and status CLI.
|   `configure_fc_for_slam.py` applies FC params manually.
|   `run_stationary_calibration.py` and `stationary_slam_calibrate.py` are bench tools.
|   `collect_calibration_data.py` collects data for analysis.
|   `estimate_extrinsics.py` estimates sensor extrinsics.
|
|-- scripts/diagnostics/
|   Bench and preflight checks.
|   `run_smoke_checks.py` runs compile/import/unit checks.
|   `check_mavlink.py` tests Cube MAVLink.
|   `check_realsense.py` tests camera.
|   `check_imu.py` and `check_external_imu.py` test IMU input.
|   `check_rangefinder.py` tests rangefinder MAVLink data.
|   `check_vio_drift.py` checks stationary VIO drift.
|   `feed_fake_gps.py` is diagnostic only, not flight behavior.
|
|-- scripts/runners/
|   Long-running or direct runnable nodes.
|   `run_slam_odometry_bridge.py` is the main bridge.
|   `run_local_vio.py` runs VIO locally.
|   `bench_vio.py` benchmarks VIO without flight side effects.
|   `jt26_to_mavlink.py` is LiDAR-to-MAVLink support.
|
|-- src/
|   Python source root.
|
|-- src/main.py
|   Passive monitor entrypoint. Bench use only, not field bridge service.
|
|-- src/mavlink_reader.py
|   MAVLink reading helpers for passive tools.
|
|-- src/slam_core/
|   Shared library.
|   `bridge_config.py` parses YAML into dataclasses.
|   `calibration.py` defines calibration profiles and math helpers.
|   `external_imu.py` reads and applies external IMU orientation.
|   `fc_config.py` owns ArduPilot MAVLink params/messages/telemetry.
|   `lidar.py` parses and filters LiDAR data.
|   `obstacle_avoidance.py` contains 360-degree keepout math and zone reduction.
|   `mavlink_bridge.py` connects to Cube and sends ODOMETRY when enabled.
|   `pose_sources.py` chooses standby/csv/vio pose sources.
|   `qgc_bridge.py` forwards MAVLink UDP for QGC/MK15.
|   `readiness.py` contains simple readiness helpers.
|   `realsense_capture.py` wraps RealSense capture.
|   `slam_observer.py` scores LOITER observation and fallback behavior.
|   `types.py` defines PoseSample and ImuSample.
|   `vio_backend.py` implements the VIO pipeline.
|
|-- systemd/
|   Service unit templates.
|   `slam-mavlink-monitor.service` is bench monitor, not field bridge.
|
|-- tests/
|   Unit tests for message packing, config, LiDAR, IMU, QGC, and observer logic.
|
|-- tools/
|   Hardware probing and live-view helpers.
```

## Key Files To Read Before Editing Flight Behavior

Read these in order:

1. `README.md`
2. `AI_AGENT_CONTEXT.md`
3. `legacy_flow_bridge/legacy_boot_cron.sh`
4. `legacy_flow_bridge/run_field_legacy.sh`
5. `legacy_flow_bridge/realsense_optical_flow_to_cube.py`
6. `legacy_flow_bridge/legacy_slam_support.py`
7. `config/sensors.yaml` and `scripts/avoidance/hesai_jt16_obstacle_node.py`
    before changing obstacle behavior.
8. `config/autostart.yaml.disabled_legacy_boot` for newer-service history.
9. `scripts/calibration/brake_slam_calibration.py`
10. `scripts/runners/run_slam_odometry_bridge.py`
11. `src/slam_core/fc_config.py`
12. `src/slam_core/slam_observer.py`
13. `tests/test_fc_config.py`
14. `tests/test_slam_observer.py`

## Status Fields Dictionary

`logs/slam_calibration_status.json` fields:

- `armed`: ArduPilot armed flag from heartbeat.
- `mode`: current flight mode as decoded from heartbeat.
- `landed_state`: MAVLink landed state when available.
- `on_ground`: bridge interpretation of landed/rangefinder state.
- `rangefinder_height_m`: current rangefinder distance.
- `gps1_fix_type`: real GPS fix type.
- `gps1_satellites`: real GPS satellite count.
- `gps2_fix_type`: MAVLink GPS2 fix type as reported by Cube.
- `gps2_satellites`: MAVLink GPS2 satellite count.
- `vio_health`: `ok` or `bad` from bridge pose gate.
- `vio_quality`: VIO quality metric.
- `vio_tracking`: VIO tracking string, for example `ok_hold` or `pnp_reject`.
- `imu_stability`: external IMU state.
- `mavlink_status`: heartbeat health.
- `rc_link`: RC link status.
- `ekf_external_nav_status`: current SLAM feed interpretation, often `gps2_bridge`.
- `current_action`: short human status.
- `failure_reason`: latest blocking reason.
- `slam_observer`: nested LOITER observer summary.

## SLAM Observer Dictionary

`slam_observer.score`:

- `9.0-10.0`: excellent
- `7.0-8.9`: good
- `5.0-6.9`: usable but needs more observation
- `3.0-4.9`: weak
- `<3.0`: critical

`slam_observer.recommendation`:

- `inactive`: not currently observing or no useful score.
- `critical`: do not test SLAM PosHold.
- `weak`: keep LOITER or run calibration.
- `observe_longer`: quality improving but not ready.
- `ready_for_no_gps_poshold`: score above configured threshold.

`slam_observer.correction`:

- `valid`: whether bounded soft correction has been learned.
- `yaw_offset_deg`: learned yaw correction.
- `scale_xy`: learned planar scale correction.
- `x_offset_m`, `y_offset_m`: learned XY offsets.
- `samples`: correction samples accepted.

## Main Control Gates

The important gates are in `scripts/runners/run_slam_odometry_bridge.py`:

- `pose_safe_for_fc(...)`: checks pose tracking, quality, finite values, quaternion, speed, and rangefinder agreement.
- `bridge_ready_for_poshold(...)`: decides whether SLAM/GPS2 can be used.
- `gps_input_stream_requested(...)`: decides whether to send SLAM pose as GPS2.
- `observer_ready_for_gps2_poshold(...)`: checks observer recommendation and score.
- `calibration_block_reason(...)`: explains why Brake calibration is blocked.

When debugging field behavior, read those functions before changing anything.

## GCS Message Contract

Operator-visible transitions should send GCS `STATUSTEXT`.

Messages should be:

- short enough for MAVLink STATUSTEXT splitting
- explicit about what mode/action is active
- explicit when something is only monitoring
- explicit when GPS2 or fallback behavior is active
- non-spammy

Examples:

- `SLAM observer ready. LOITER soft calibration available.`
- `JETSON EVENT: Jetson booted; SLAM bridge script started; MAVLink connected on /dev/...`
- `JETSON EVENT: script running; disarmed mode=STABILIZE; waiting for FIELD GATE OK.`
- `JETSON EVENT: script running; BRAKE selected but vehicle disarmed; waiting for ARM.`
- `BEEP: startup check alive after 30s; monitoring only until FIELD GATE OK`
- `FIELD GATE WAIT: GPS1 not ready fix=1 sats=0; GPS2 standby not confirmed fix=1 sats=0`
- `FIELD GATE OK: GPS LOITER and BRAKE calibration inputs ready. Wait for NO-GPS POSHOLD GATE before SLAM PosHold.`
- `NO-GPS POSHOLD GATE OK: Brake calibration profile ready score=8.0/10. POSHOLD can use SLAM/VIO GPS2 feed cautiously.`
- `LOITER active: SLAM observation mode started.`
- `SLAM quality ready for No-GPS PosHold: X/10`
- `GPS2 origin locked from healthy GPS/EKF reference.`
- `VIO mirrored to GPS2 GPS_INPUT.`
- `No-GPS POSHOLD active: SLAM/VIO GPS2 feed is flying without real GPS.`
- `SLAM quality critical: switching to LOITER.`
- `LiDAR OA RC7 toggle ready: low detect, high avoid.`
- `OA DETECT ONLY by RC7: avoidance disabled.`
- `OA ENGAGED by RC7: FC native avoidance active.`
- `OA DETECT ONLY: RC7=1045, obstacle detection active.`
- `OA ENGAGED: RC7=1945, FC avoidance active.`

Every Jetson-commanded audible tune should have a matching `BEEP:` message
first. The Cube Lua helper should relay state and beep only for FC-local
mode/arm/error events, not duplicate Jetson startup/ready/calibration tunes.
Exception: the LiDAR RC7 obstacle toggle plays short mode-confirmation tunes
and sends `OA ENGAGED...` / `OA DETECT ONLY...` text instead of `BEEP:` prefix.
If the operator hears OA toggle beeps but does not see GCS text over QGC/MK15,
upload the updated `ardupilot_lua/brake_slam_beeper.lua` to the Cube SD card as
`APM/scripts/brake_slam_beeper.lua` and reboot the Cube.

## Testing Dictionary

Fast full smoke:

```bash
python3 scripts/diagnostics/run_smoke_checks.py
```

Specific useful tests:

```bash
python3 -m pytest tests/test_fc_config.py
python3 -m pytest tests/test_slam_observer.py
python3 -m pytest tests/test_qgc_bridge.py
python3 -m pytest tests/test_lidar.py tests/test_obstacle_avoidance.py
```

If pytest is unavailable, use the smoke script. It imports modules and runs the
repo's lightweight test runner.

## Common Edit Patterns

When adding config:

1. Add a dataclass field in `src/slam_core/bridge_config.py`.
2. Parse it in `SlamBridgeConfig.from_mapping`.
3. Add YAML keys to `config/autostart.yaml` and `config/default.yaml`.
4. Add docs to `README.md`.
5. Add a test.

When changing MAVLink messages:

1. Edit `src/slam_core/fc_config.py`.
2. Add or update a fake-master test in `tests/test_fc_config.py`.
3. Run smoke checks.
4. Document the operator-visible effect.

When changing LOITER/POSHOLD behavior:

1. Edit `src/slam_core/slam_observer.py` or the gate functions in the bridge.
2. Preserve LOITER as no-motion observation.
3. Preserve real Brake calibration as separate.
4. Add a GCS message for new operator-visible states.
5. Add tests in `tests/test_slam_observer.py`.

When changing service behavior:

1. Edit `scripts/calibration/brake_slam_calibration.py` or install scripts.
2. Keep duplicate-process refusal.
3. Keep clean child shutdown.
4. Test with `systemctl status` and logs.

When changing obstacle behavior:

1. Edit `config/sensors.yaml`, `scripts/avoidance/hesai_jt16_obstacle_node.py`,
   and, if operator messages changed, `ardupilot_lua/brake_slam_beeper.lua`.
2. Preserve RC7 low/neutral as detect-only.
3. Keep `PRX1_TYPE=2` in detect-only so PRX stays connected, but set
   `AVOID_ENABLE=0`.
4. Use RC7 high to set `AVOID_ENABLE=7`.
5. Do not enable `direct_velocity` by default.
6. Run Python syntax checks and the obstacle/lidar tests.
7. Restart only the LiDAR node when safe; do not restart the main bridge in air.

## Current Field Validation Checklist

Do not call No-GPS flight ready until all pass:

- service starts automatically after Jetson boot
- Cube params are applied and Cube was rebooted if needed
- QGC receives SLAM GCS messages
- QGC/MK15 receives OA switch messages or updated Cube Lua relay is installed
- GPS1 gets stable outdoor 3D fix
- GPS2 standby mirrors real GPS outside
- GPS2 DataFlash `GPS I:1` has non-zero `GWk` and `GMS`
- GPS2 DataFlash `GPA I:1 Delta` stays under 200 ms during standby and SLAM feed
- LOITER observation logs stable data
- SLAM score rises and falls sensibly
- Brake calibration completes
- VIO drift is low after calibration
- POSHOLD GPS2 feed starts only when gated
- fallback to LOITER works with healthy GPS
- pilot manual override is verified
- logs are reviewed after field tests

Obstacle validation checklist:

- JT16 node starts through `legacy_boot_cron.sh` after the bridge.
- Only the legacy bridge owns `/dev/ttyACM0`; LiDAR node uses UDP `14555`.
- RC7 low/neutral shows detect-only, `PRX1_TYPE=2`, `AVOID_ENABLE=0`.
- RC7 high shows engaged, `PRX1_TYPE=2`, `AVOID_ENABLE=7`.
- RC7 engage and disengage both play distinct confirmation tunes.
- GCS text appears for RC7 switch states; if not, update Cube Lua on SD card.
- Armed obstacle beeps only occur when an obstacle is inside configured ranges.
- Slow open-field LOITER/ALTHOLD test confirms ArduPilot resists motion toward
  obstacles before any tight-space trust.

## Known Risk Areas

- GPS2 bad fix before outdoor GPS lock.
- VIO `pnp_reject` in poor visual texture or bad lighting.
- QGC/MK15 routing if not on the same network as Jetson UDP broadcast.
- Companion `STATUSTEXT` may be audible through `PLAY_TUNE` but invisible in
  QGC telemetry; Cube Lua relay is the reliable text path.
- ArduPilot parameter changes requiring Cube reboot.
- Duplicate MAVLink readers causing QGC crashes or parameter-download failures.
- Over-trusting observer score before real flight validation.
- Enabling obstacle avoidance movement before SLAM position hold is stable.
- Bench arming near the drone frame can trigger real obstacle warning beeps
  because the top-mounted LiDAR sees close returns.

## If You Are An AI Agent

Before making changes:

1. Read `git status --short`.
2. Do not revert user changes.
3. Inspect the relevant config and tests.
4. Keep edits scoped.
5. Prefer adding tests when changing behavior.
6. Run `python3 scripts/diagnostics/run_smoke_checks.py`.
7. Do not restart flight services while `armed=true` unless the user confirms the vehicle is safe and on ground.
8. Say clearly if live service was not restarted.

Default safe answer for flight readiness:

```text
Configured for field validation, not proven fully ready for No-GPS flight.
```

## Glossary

- **ArduPilot**: autopilot firmware running on the Cube.
- **Cube**: Cube Orange+ flight controller.
- **Jetson**: companion computer running this repo.
- **GCS**: ground control station, such as QGC or MK15.
- **MAVLink**: message protocol between Jetson, Cube, and GCS.
- **GPS_INPUT**: MAVLink message used to provide GPS-like data from the companion.
- **GPS2**: second GPS instance in ArduPilot, currently used for SLAM pose.
- **VISO / VisOdom**: ArduPilot visual odometry interface. Disabled in current profile.
- **ODOMETRY**: preferred ExternalNav MAVLink message. Suppressed in current GPS2 profile.
- **VIO**: visual-inertial odometry.
- **SLAM**: simultaneous localization and mapping, used broadly here for local pose estimation.
- **LOITER**: GPS-assisted ArduPilot position hold.
- **BRAKE**: mode used as the explicit calibration trigger.
- **POSHOLD**: target mode for cautious GPS2 SLAM position hold.
- **Soft correction**: bounded yaw/scale/XY correction learned from LOITER.
- **Fallback**: warning or automatic switch to LOITER when SLAM quality is critical and GPS is healthy.
- **OA detect-only**: LiDAR proximity is connected, but `AVOID_ENABLE=0`.
- **OA engaged**: RC7 high; ArduPilot native avoidance active with
  `AVOID_ENABLE=7`.
- **SCR_USER4**: Cube Lua obstacle status relay param. Current values:
  `10` native active, `12` detect-only/monitor, `20` warning, `30` keepout,
  `40` critical, `90` stale.
