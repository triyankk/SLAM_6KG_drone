# optFlow_slam

GPS-denied perception, obstacle awareness, mapping, and guarded local-return
development for a heavy Cube Orange+ quadrotor with an NVIDIA Jetson companion.

The repository contains the complete project-local software boundary: Cube and
sensor configuration, Jetson services, hardware drivers, visualizers, flight
recording, calibration tools, shadow LIO/VIO, local-return logic, tests, and
operator documentation. It does not depend on code or data from an older
workspace.

## Read This First

This is flight-critical experimental software. The currently installed boot
profile is **obstacle avoidance only**. It publishes JT16 proximity to the Cube
but does not run FAST-LIO, VIO, SLAM navigation, ExternalNav, or companion
movement commands.

The following are not approved today:

- Removing GPS for autonomous flight.
- Sending SLAM pose into the Cube EKF.
- Automatic GPS-denied return.
- Autonomous obstacle flanking or A* flight.
- Arming, takeoff, landing, mode selection, attitude control, or motor mixing
  from the Jetson.

Keep GPS connected during field commissioning. Keep RC7 low until the
props-off proximity direction test passes. Never use `STABILIZE` as a fallback
on this aircraft, and never use `GUIDED_NOGPS` for this architecture.

## Project State

State last consolidated on **2026-08-08**.

| Capability | State | Evidence or boundary |
| --- | --- | --- |
| Cube to Jetson UART | Working | TELEM2 `/dev/ttyTHS1`, MAVLink2, 460800 baud, bidirectional parameter audit passed |
| H-Flow optical flow and range | Working | DroneCAN on CAN2, downward, offsets applied |
| Cube-native optical-flow flight | Flight tested | Manual Loiter/FlowHold remained controllable; GPS stays fitted during commissioning |
| JT16 serial point cloud | Working | Official Hesai SDK bridge, `/dev/jt16_usb`, 3,000,000 baud |
| JT16 body transform | Verified | Four-cardinal target sequence and correction table passed |
| D415 RGB-depth | Working for diagnostics | Intrinsics verified; outdoor control extrinsics/tracking are not approved |
| IM10A stream | Working | `/dev/imu_usb`, 230400 baud, TIME + ACC + GYRO near 200 Hz |
| Obstacle proximity to Cube | Active | JT16-only OA service, eight paced MAVLink faces, `PRX1_TYPE=2` |
| RC7 native avoidance | Configured | `RC7_OPTION=40`, 1.50 m CG margin; requires field commissioning |
| Passive flight evidence | Implemented | Arm-triggered Cube, IMU, camera, lidar, point-cloud, and analysis recording |
| FAST-LIO2 | Shadow only | Pinned JT16 backend and project-local ROS 2 runtime |
| RGB-D odometry | Shadow only | Metric D415 odometry with IM10A rotation prior |
| Cube ODOMETRY transport | Bench proven | Disarmed-only transport proof; Cube EKF fusion remains disabled |
| Breadcrumb local return | Implemented but locked | Shadow proposals and replay only; live control approval is absent |
| ExternalNav | Disabled | No EKF source set selects ExternalNav |
| GPS-denied RTL | Not approved | Standard RTL is not the intended return mechanism |
| A* obstacle navigation | Design stage | Global planner selected; collision-aware local planner remains TBD |

The latest live OA-only validation held Cube link and real proximity health for
`150/150` samples over 75 seconds with zero reconnects and no fresh
`PRX1: No Data`. The full Python suite passed `212` tests after that change.
This evidence validates the bench transport, not flight behavior.

## Aircraft and Sensors

The aircraft is a roughly 6 kg-class quadrotor with a low-hanging 6S 22 Ah
battery. Opposite motor centers measure 0.85 m diagonally and the aircraft uses
18 inch propellers. The physical rotating radius is approximately 0.6536 m;
software uses a conservative 0.75 m protected radius.

| Component | Connection | Mount relative to CG in body FRD | Current use |
| --- | --- | --- | --- |
| Cube Orange+ | Airframe | `(+0.08, 0.00, -0.08) m`, Yaw270 | EKF3, stabilization, optical flow, altitude, motors |
| Holybro H-Flow | Cube CAN2 | `(0.00, 0.00, +0.10) m`, downward | Cube optical-flow velocity and downward range |
| Hesai JT16 | Jetson USB-RS485 | `(0.00, 0.00, -0.10) m`, calibrated yaw 180 deg | 360-degree proximity and shadow LIO |
| Intel RealSense D415 | Jetson USB | `(+0.19, 0.00, +0.10) m`, forward | RGB-depth, mapping diagnostics, shadow VIO |
| Hiwonder IM10A | Jetson USB serial | `(+0.08, 0.00, -0.09) m` | Shadow LIO/VIO inertial stream |
| Four individual ESCs | Cube motor outputs | One per motor, 40 A each | Replaced the former shared 55 A 4-in-1 ESC |
| 6S 22 Ah battery | Power module | Suspended below CG | Main power; voltage/current monitored by Cube |

FRD means X forward, Y right, and Z down. A negative Z mount value is above
the CG. The browser uses launch-local FLU for display, while Cube local flight
data uses NED/FRD contracts documented in
[`docs/INTERFACES.md`](docs/INTERFACES.md).

### Power Notes

- One motor measured about 11 A at 75 percent throttle; 15 A per motor is used
  as a conservative bench planning value.
- Individual 40 A ESCs provide thermal and current margin and isolate failures
  better than the former 4-in-1 unit. They do not by themselves prove the
  propulsion system is reliable.
- The first flights with individual ESCs showed hot motors/ESCs but no observed
  thrust loss. Continue recording temperature, voltage sag, current, vibration,
  motor saturation, and thrust balance.
- Battery parameters use `BATT_MONITOR=4`, `BATT_CAPACITY=22000`, 23.1 V arm
  threshold, 22.2 V low threshold, and 21.6 V critical threshold.
- A 6S pack is approximately 25.2 V when full. GCS percentage is only useful
  after voltage/current scaling and capacity estimation are validated; use
  measured voltage and current as the primary evidence during bring-up.

The electrical acceptance sequence is in
[`docs/POWER_AND_ESC.md`](docs/POWER_AND_ESC.md).

## Cube and RC Contract

The checked-in [`mav.parm`](mav.parm) is a reference snapshot of the current
Cube parameters. It is not a universal parameter file and must not be loaded
blindly onto another aircraft.

Important current values:

| Area | Parameters |
| --- | --- |
| Firmware | ArduCopter 4.6.3, EKF3 enabled |
| Orientation | `AHRS_ORIENTATION=6` (`Yaw270`) |
| Pilot attitude limit | `ANGLE_MAX=3000` (30 deg) |
| TELEM1/QGC | `SERIAL1_PROTOCOL=2`, `SERIAL1_BAUD=57` |
| TELEM2/Jetson | `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=460` |
| Optical flow | `FLOW_TYPE=6`, `FLOW_POS_Z=0.10` |
| Downward range | `RNGFND1_TYPE=24`, `RNGFND1_POS_Z=0.10` |
| EKF source set 1 | POSXY none, VELXY optical flow, POSZ barometer, compass yaw |
| Guided behavior | `GUID_OPTIONS=64`, `GUID_TIMEOUT=0.5` s |
| Proximity | `PRX1_TYPE=2`, MAVLink backend |
| Avoidance | `AVOID_ENABLE=7`, `AVOID_MARGIN=1.50`, `AVOID_DIST_MAX=1.50` |
| Avoidance dynamics | `AVOID_BACKUP_SPD=0.50`, `AVOID_ACCEL_MAX=1.00` |
| Pilot speed limits | horizontal 2.0 m/s, descent 1.0 m/s, ascent 1.5 m/s |
| Terrain | `TERRAIN_ENABLE=0` |

RC allocation:

| Channel | Role |
| --- | --- |
| RC5 | Cube flight-mode switch |
| RC7 | Cube proximity avoidance, `RC7_OPTION=40` |
| RC8 | FlowHold, `RC8_OPTION=71` |
| RC9 | Reserved companion SLAM-return request; must remain unassigned in Cube |
| RC10 | Pilot-commanded Land, `RC10_OPTION=18` |
| RC11 | Motor emergency stop, `RC11_OPTION=31` |

The project never assigns or changes a disarm switch without prompting the
operator first. RC9 low cancels the future companion return immediately. RC10
remains a Cube-owned pilot action, not a Jetson landing command.

### UART Wiring

Cube TELEM2 connects to the Jetson 40-pin header using signal and ground only:

- Jetson physical pin 6: ground.
- Jetson physical pin 8: TX to Cube RX.
- Jetson physical pin 10: RX from Cube TX.
- Both sides use 3.3 V UART logic. Do not connect a TELEM power pin to the
  Jetson header.

USB MAVLink is not required by the active project runtime. QGC normally uses
the independent TELEM1 radio link, so the Jetson service must not request a
large TELEM2 stream that can interfere with flight telemetry.

## Runtime Architecture

The Cube owns every fast flight-control loop:

```text
H-Flow + range + Cube IMUs/baro/mag
                 |
                 v
          Cube EKF3 and controllers
                 |
                 v
              ESCs/motors
```

The Jetson owns perception and future high-level local navigation:

```text
JT16 ----> proximity sectors ----------> Cube native avoidance
  |
  +------> FAST-LIO2 ----+
                         +--> local map --> planner --> guarded local targets
IM10A -------------------+
D415 RGB-depth --> VIO --+
```

The Jetson never mixes motors and does not send raw attitude commands. Future
navigation may send bounded local velocity targets only after all approval
gates pass and the pilot deliberately selects regular `GUIDED`.

## Active OA-Only Service

`optflow-flight-logger.service` is a legacy unit name. Its current executable
is [`scripts/run_obstacle_avoidance_service.sh`](scripts/run_obstacle_avoidance_service.sh),
which starts only:

- The official JT16 decoder.
- JT16-to-body-FRD obstacle extraction.
- A direct Cube MAVLink UART link.
- Paced proximity packets.
- RC7 state monitoring and obstacle warning tones.

It explicitly does not start the D415, IM10A, ROS 2, FAST-LIO, RGB-D odometry,
SLAM, trajectory recording, ExternalNav, or companion movement output.

### Proximity Data Path

1. JT16 frames arrive near 4 to 5 Hz through `/dev/jt16_usb` at 3,000,000
   baud using the pinned Hesai SDK and unit correction file.
2. Points are transformed from JT16 axes to body FRD and translated to the CG.
3. Only the horizontal slice from `-0.40 m` to `+0.40 m` around the CG is used.
4. Valid points from 0.30 m through 8.00 m are grouped into 72 five-degree
   sectors.
5. A sector requires at least eight points and uses the 10th percentile.
6. A three-frame conservative temporal minimum suppresses brief dropouts.
7. The 72 sectors are reduced to eight 45-degree MAVLink `DISTANCE_SENSOR`
   faces using the nearest known value in each face.
8. A fresh face with no return is encoded as `max_distance + 1`, meaning no
   obstacle inside the configured sensor range.
9. All eight short packets are spaced by 12 ms instead of being sent as one
   UART burst.
10. A scan older than 0.45 s is not transmitted. ArduPilot then marks MAVLink
    proximity as no-data after its own 0.5 s timeout.

This fixes the earlier behavior where a healthy empty scan emitted no packet
and intermittently produced `PRX1: No Data`. It also avoids the long
`OBSTACLE_DISTANCE` frame loss seen at 921600 baud and the saturation seen when
the combined SLAM runtime requested high-rate Cube telemetry.

### Clearance and Audio

Distances are referenced to the aircraft CG after applying sensor extrinsics.
The hard horizontal margin is 1.50 m.

- `1.50 m < distance <= 2.00 m`: one warning beep per second, no action from
  the Jetson.
- `1.25 m <= distance <= 1.50 m`: three beeps per second.
- Below 1.25 m: the warning rate rises toward 10 Hz near the protected
  airframe envelope.
- Audio requires a fresh scan, RC7 enabled, and an armed aircraft.
- The service sends one rising initialization tune after Cube connection.
- It does not duplicate ArduPilot arming or mode-change tones.

RC7 controls ArduPilot's native avoidance response, not whether proximity data
is published. The current layer can stop or lean away in supported Cube modes;
it is not yet an autonomous obstacle-flanking planner.

### Service Commands

Install once and start on every Jetson boot:

```bash
./optflow install-flight-service
```

Operate and inspect it:

```bash
systemctl --user restart optflow-flight-logger.service
systemctl --user status optflow-flight-logger.service --no-pager --full
systemctl --user is-enabled optflow-flight-logger.service
./optflow obstacle-status
```

The status command reads `runtime/obstacle_avoidance_status.json` and never
opens a second UART. Do not run MAVProxy, the full visualizer, a calibration
tool, or another hardware-owning service while this unit is active.

## SLAM, VIO, and Mapping

The estimator stack exists but remains shadow-only.

### Lidar-Inertial Odometry

- Backend: Hesai JT-series FAST-LIO2 revision
  `bb2842d34990761eebbd4cc3188e94c7c662a673`.
- ROS 2 topic for lidar: `/optflow/jt16/points`.
- ROS 2 topic for IMU: `/optflow/im10a/imu`.
- Odometry topic: `/Odometry`.
- Expected IMU rate: 200 Hz.
- Typical JT16 odometry rate: approximately 5 Hz.
- Measured lidar-to-IMU time offset: `+0.010 s`.
- Map output is disabled by default during validation.
- Validation approval is false and no approval digest is configured.

The project-local build pins ROS 2 Humble, PCL, Hesai FAST-LIO2, and ikd-Tree
in [`third_party/lio.lock.json`](third_party/lio.lock.json). It builds under
`runtime/lio/` without installing a second system ROS workspace.

### Visual Odometry

The D415 path performs metric RGB-D odometry with timestamp-matched IM10A gyro
rotation priors. Dense RGB-D tracking has measured near 0.87 Hz on this Jetson,
so it is an independent consistency signal with a 2.0 s freshness gate, not the
primary fast motion source.

D415 intrinsics and depth scale are verified. Its final body rotation and
outdoor control reliability are not verified, so D415 depth is excluded from
Cube proximity and flight control.

### What the Current Estimator Is Not

- It is not approved loop-closed, globally consistent SLAM.
- Relocalization after tracking loss is not proven.
- A persistent map across flights is not approved.
- Pose covariance is not approved for Cube EKF fusion.
- It cannot yet prove GPS-denied RTL or obstacle-aware autonomous navigation.

The shadow and proof workflow is documented in
[`docs/LIO_SHADOW.md`](docs/LIO_SHADOW.md).

## Guarded Local Return

`SLAM RETURN` is not standard ArduPilot RTL. The intended design records the
launch pose and outbound local breadcrumbs in one SLAM map, then follows those
breadcrumbs in reverse using bounded velocity-only
`SET_POSITION_TARGET_LOCAL_NED` messages in regular `GUIDED`.

Current return configuration:

- Stage: `locked`.
- Live control: disabled.
- Request: RC9 low after arm, then deliberate low-to-high transition.
- Required mode: regular `GUIDED`.
- Operating altitude: 1.0 m to 8.0 m.
- Commissioning speed: 0.30 m/s.
- Absolute configuration ceiling: 0.75 m/s.
- Horizontal acceleration: 0.50 m/s2.
- Breadcrumb spacing: 0.15 m.
- Arrival radius: 0.20 m.
- Unexpected 1.50 m clearance breach: zero-velocity handoff and cancellation.

Live output additionally requires fresh Cube heartbeat, RC, local position,
optical flow, range, battery, LIO, RGB-D, obstacle data, valid EKF origin,
approved calibrations, approved LIO report, and a digest-bound approval file.
No code should fabricate those approvals.

ArduCopter 4.6 does not provide a persistent local EKF origin across every
GPS-free cold boot. A future field run must initialize a valid origin while GPS
is present or have the operator explicitly establish it in the GCS. The
software must never invent a geographic origin.

See [`docs/SLAM_RETURN.md`](docs/SLAM_RETURN.md) for the exact gates and
operator sequence.

## Installation

Target platform:

- NVIDIA Jetson, Ubuntu 22.04, aarch64.
- CPython 3.10.
- Node/npm for the Three.js frontend.
- Matching kernel headers for the project USB serial drivers.
- Cube Orange+ running the documented ArduCopter configuration.

Create the offline project-local Python environment:

```bash
./optflow setup
```

Build and test the browser application:

```bash
./optflow build
./optflow test
```

Install host support only when required:

```bash
./optflow build-ch341
sudo ./optflow install-ch341

./optflow build-pl2303
sudo ./optflow install-pl2303

sudo ./optflow install-realsense-rules
```

Build the native JT16 bridge and optional project-local LIO runtime:

```bash
./optflow build-jt16
./optflow build-lio
```

The Python wheels needed for offline Jetson setup are mirrored under
`vendor/python/`. Generated build products remain ignored.

## Command Reference

Run `./optflow help` for the authoritative launcher list.

| Command | Purpose | Hardware ownership or safety note |
| --- | --- | --- |
| `setup` | Create the offline local Python environment | No flight hardware |
| `build` | Build the visualizer frontend | No flight hardware |
| `test` | Run Python tests and frontend build | No flight hardware |
| `preflight` | Read-only Cube/sensor readiness probe | Stop active UART owner first |
| `sensor-check` | Check IM10A, D415, and JT16 | Does not open Cube UART |
| `obstacle-check` | Bench-check body-frame sectors | No Cube output |
| `obstacle-service` | Run OA-only service in foreground | Owns Cube UART and JT16 |
| `obstacle-status` | Read cached OA health | Safe beside service |
| `jt16-calibrate` | Guided cardinal calibration | Props off; owns JT16 |
| `jt16-plane` | Validate wall/floor planes and rings | Props off; owns JT16 |
| `camera` | Serve D415 RGB | Owns D415 |
| `flight-log` | Record a manual flight session | Hardware-owning foreground tool |
| `flight-service` | Arm-triggered passive logger | Owns Cube, IMU, D415, JT16 |
| `flight-status` | Read passive logger state | Cache only |
| `analyze` | Analyze a recording and Cube BIN | Offline |
| `slam-timing` | Measure rates, jitter, drops, clocks | Offline recording analysis |
| `im10a` | Audit or reversibly configure IM10A | Stop other IMU owner |
| `imu-noise` | Record stationary noise profile | Aircraft stationary, props off |
| `imu-align` | Compare IM10A with shadow sessions | Offline |
| `build-lio` | Build pinned local ROS 2/FAST-LIO | No flight hardware |
| `lio-shadow` | Record JT16 + IM10A LIO | Owns Cube reference, IMU, JT16 |
| `cube-odom-shadow` | Disarmed MAVLink ODOMETRY transport proof | Stops if armed; no EKF fusion |
| `slam-poc` | Visible D415 VIO + FAST-LIO proof | Props off unless using approved flight-shadow mode |
| `slam-flight-shadow` | GPS-Loiter shadow evidence flight | No movement output |
| `rtl-shadow` | Replay local-return proposals | Offline, no Cube output |
| `slam-return` | Run guarded return runtime | Currently monitor-only and locked |
| `slam-return-setup` | Audit Cube return contract | Writes only explicitly requested safe parameters while disarmed |
| `slam-return-status` | Read return gates and transport state | Cache only |
| `prearm-status` | Read recent pre-arm telemetry | Live UART mode requires service stopped |
| `lio-validate` | Re-score LIO session | Offline |
| `cube-mount` | Inspect/apply Cube orientation | Disarmed-only parameter tool |
| `cube-avoidance` | Inspect/apply native OA parameters | Disarmed-only parameter tool |
| `lidar-status` | Inspect JT16 USB/driver state | Read-only |
| `visualizer` | Full Cube/IMU/point-cloud browser | Hardware owner unless monitor/demo mode |

## Visual Interfaces

| Interface | Default URL | Contents |
| --- | --- | --- |
| Drone motion visualizer | `http://<jetson>:8765` | Cube attitude/IMU, H-Flow, range, IM10A, 3D scan |
| LIO visual assist | `http://<jetson>:8766/lio-assist` | Guided carry sequence, live LIO diagnostics |
| SLAM proof dashboard | `http://<jetson>:8767/slam-poc` | LIO/VIO trajectories and proof instructions |
| D415 RGB stream | `http://<jetson>:8770` | Forward camera stream |

Start the full visualizer only after stopping the boot OA service:

```bash
systemctl --user stop optflow-flight-logger.service
./optflow visualizer --host 0.0.0.0 --no-browser
```

The **3D SCAN** view renders measured airframe geometry, D415 and JT16 point
cloud layers, Cube motion, and trajectory layers. JT16 is off by default in the
browser to reduce load; D415 and trajectory are independently toggleable. A
visual trajectory is diagnostic evidence, not proof of drift-free SLAM.

For headless operation, install and verify the project-owned VNC setup:

```bash
sudo ./optflow install-headless-vnc
./optflow check-headless-vnc
```

See [`docs/HEADLESS_ACCESS.md`](docs/HEADLESS_ACCESS.md).

## Calibration and Bench Work

All manual sensor work requires props removed unless a specific field procedure
explicitly says otherwise.

### Cube and H-Flow

```bash
systemctl --user stop optflow-flight-logger.service
./optflow preflight --profile fc_bench
./optflow cube-mount
```

### JT16 and D415 Geometry

```bash
./optflow obstacle-check
./optflow obstacle-check --target-distance 1.50 --target-angle 0
./optflow jt16-calibrate
./optflow jt16-plane
```

The calibration definitions, measured-target sequence, and verification gates
are in [`docs/SENSOR_CALIBRATION.md`](docs/SENSOR_CALIBRATION.md).

### IM10A

```bash
./optflow im10a
./optflow imu-noise --duration 1800
./optflow imu-align data/recordings/lio/<session-a> \
  data/recordings/lio/<session-b>
```

The currently validated live IM10A profile is 230400 baud near 200 Hz with
sensor time, acceleration, and gyro enabled. Configuration changes must remain
reversible and include baud recovery, checksum, drop, rate, and jitter checks.

## Evidence and Data

Generated data stays under `data/`:

```text
data/
  calibrations/   measured intrinsics, extrinsics, biases, noise
  logs/           application/readiness logs
  maps/           versioned maps and metadata
  recordings/     flight, SLAM, LIO, camera, lidar, and Cube evidence
  flight_logs/    local imported Cube logs; ignored by Git
```

Runtime state stays under `runtime/` and is not approval evidence. Examples
include service status JSON, locks, temporary point-cloud frames, and the local
ROS 2/LIO build.

The passive arm-triggered logger records:

- Cube MAVLink telemetry and raw events.
- IM10A samples.
- D415 color/depth and RealSense bag data.
- JT16 raw serial evidence and decoded point clouds.
- Sensor timing records.
- Shadow ideal-motion and local-return proposals.
- Merged PLY environment cloud.
- Optional matching Cube DataFlash log.
- Generated manifest, statistics, and analysis report.

After a flight:

```bash
./optflow analyze data/recordings/flights/<session> \
  --cube-log /path/to/latest.BIN
```

Flight logs, tlogs, ROS build products, recordings, maps, and runtime state are
ignored by Git. Calibration metadata and source-controlled reference files must
be small, reproducible, and documented.

## Staged Field Progression

Do not skip directly to GPS removal.

1. Mechanical, CG, battery restraint, wiring, propeller, ESC, and motor
   acceptance.
2. Props-off Cube, H-Flow, range, RC, compass, battery, UART, and proximity
   checks.
3. GPS-connected manual AltHold/Loiter flight with avoidance off.
4. GPS-connected low-speed Loiter obstacle-avoidance test with RC7.
5. GPS-connected low-speed FlowHold avoidance test with RC7.
6. Passive GPS-Loiter SLAM shadow flight with zero Jetson movement output.
7. Repeatable LIO/VIO trajectory validation and return-to-start evidence.
8. Regular-Guided bounded local target commissioning with GPS still available.
9. Obstacle-aware local planning and controlled cancellation tests.
10. Only after repeated evidence, test GPS denial while retaining a tested
    pilot recovery mode and connected GPS hardware.

The exact ordered gates are in [`docs/BRINGUP.md`](docs/BRINGUP.md).

## Troubleshooting

### `PRX1: No Data`

```bash
./optflow obstacle-status
systemctl --user status optflow-flight-logger.service --no-pager --full
```

Require a fresh JT16 frame, continuous packet count, Cube link, and
`PRX_HEALTH=True`. Do not disable `PRX1` or arming checks to hide the error.
Stop the service before any direct UART diagnostic.

### Cube UART or telemetry loss

- Confirm Cube is disarmed before changing link settings.
- Confirm `/dev/ttyTHS1` at 460800 and `SERIAL2_PROTOCOL=2`.
- Check `runtime/locks/cube_mavlink.lock` and stop competing processes.
- Do not request raw IMU and every telemetry stream on TELEM2.
- Leave QGC on the independent TELEM1 radio where possible.
- The experimental localhost MAVLink router remains disabled in the active
  OA-only profile.

### RC state missing

`RC7_PWM=None` means no valid `RC_CHANNELS` value has reached the service. It
does not mean avoidance is enabled. Resolve the receiver/pre-arm condition and
verify the configured 1300/1700 thresholds before flight.

### Compass inconsistency

Treat `PreArm: Compasses inconsistent` as a real Cube issue. Recheck mounting,
orientation, current-carrying wires, calibration, and magnetic environment. Do
not weaken arming checks to proceed.

### Sensor or visualizer will not start

One process owns each hardware resource. Stop the boot service before using the
full visualizer, calibration tools, passive logger, or SLAM proof. The
trajectory-monitor and status commands are the read-only exceptions described
in their docs.

### Motors or ESCs are hot

Land, disarm, and measure temperatures. Review current, voltage sag, motor
balance, propeller condition, vibration, commanded thrust, and DataFlash motor
outputs. Individual 40 A ESCs improve margin but do not make heat acceptable.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `config/system.yaml` | Single source of hardware geometry, limits, stages, and safety gates |
| `config/lio/` | Resolved FAST-LIO2 template inputs |
| `optflow` | Project-relative command launcher |
| `src/optflow_slam/` | Python implementation |
| `scripts/` | Thin executable wrappers and build tools |
| `native/jt16_bridge/` | C++ official-Hesai-SDK serial bridge |
| `visualizer/` | Three.js/Vite browser interfaces |
| `hardware/` | Kernel drivers, udev rules, systemd units, VNC setup |
| `ros_ws/` | Project-local ROS 2 source boundary |
| `third_party/` | Pinned external revisions and local patches |
| `vendor/python/` | Offline aarch64 Python wheel mirror |
| `tests/` | Non-flight unit and visual contract tests |
| `docs/` | Architecture, interfaces, calibration, bring-up, and field procedures |
| `data/` | Persistent local evidence and calibrations |
| `runtime/` | Transient state, locks, status, and local builds |
| `mav.parm` | Reference snapshot of current Cube parameters |
| `AGENTS.md` | Complete engineering context and agent safety contract |

The boundary test forbids imports from sibling workspaces. External datasets
must be copied into `data/recordings/` with provenance rather than read in
place.

## Development and Validation

Python dependencies are pinned in `requirements.lock` and mirrored under
`vendor/python/`. Frontend dependencies are pinned in
`visualizer/package-lock.json`.

Run the complete project checks:

```bash
./optflow test
git diff --check
```

Hardware tests are separate from unit tests. A passing unit suite never grants
permission to arm, change Cube parameters, activate ExternalNav, or remove GPS.

## Documentation Index

- [`AGENTS.md`](AGENTS.md): exhaustive current context for coding agents and
  maintainers.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): estimator, mapper, planner,
  Cube, and failure boundaries.
- [`docs/INTERFACES.md`](docs/INTERFACES.md): frames, timing, proximity,
  odometry, ExternalNav, and command contracts.
- [`docs/BRINGUP.md`](docs/BRINGUP.md): ordered bench-to-GPS-denied progression.
- [`docs/OBSTACLE_AVOIDANCE.md`](docs/OBSTACLE_AVOIDANCE.md): current active
  JT16 proximity path and field sequence.
- [`docs/SENSOR_CALIBRATION.md`](docs/SENSOR_CALIBRATION.md): D415/JT16
  calibration and CG-referenced transforms.
- [`docs/LIO_SHADOW.md`](docs/LIO_SHADOW.md): IM10A/JT16 time alignment,
  shadow runs, and validation limits.
- [`docs/SLAM_RETURN.md`](docs/SLAM_RETURN.md): locked local-return gates and
  operator workflow.
- [`docs/FLIGHT_LOGGER.md`](docs/FLIGHT_LOGGER.md): evidence layout and flight
  analysis.
- [`docs/POWER_AND_ESC.md`](docs/POWER_AND_ESC.md): propulsion acceptance.
- [`docs/HEADLESS_ACCESS.md`](docs/HEADLESS_ACCESS.md): headless X/VNC setup.
- [`hardware/README.md`](hardware/README.md): host-level installation details.

## Non-Negotiable Rules

- Never select or automate `STABILIZE` as a fallback.
- Never use `GUIDED_NOGPS` for this project.
- Never treat FlowHold as a globally drift-free position source.
- Never use standard RTL as the GPS-denied local-return implementation.
- Never arm, take off, land, change mode, or mix motors from Jetson code during
  bring-up.
- Never send ExternalNav until a real approved report and digest-bound approval
  gate exist.
- Never run two owners of the Cube UART, IM10A, D415, or JT16.
- Never hide `PRX`, RC, compass, battery, or EKF pre-arm failures by disabling
  checks.
- Never invent an EKF origin or assume GPS-free cold-boot origin persistence.
- Never mark assumed, hand-moved, bench-only, or visualizer-only data as flight
  validation.
- Never change the RC disarm or emergency-stop assignment without prompting
  the operator.
- Always keep a tested pilot takeover path and land in the currently stable
  altitude-controlled mode. Do not fall through to `STABILIZE`.
