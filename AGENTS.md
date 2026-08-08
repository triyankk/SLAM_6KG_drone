# optFlow_slam Agent Context

This file applies to the entire repository. It is the durable engineering and
safety context for coding agents and maintainers. Read it before changing code,
Cube parameters, services, hardware ownership, flight procedures, or claims of
readiness.

## Source of Truth and Precedence

When facts disagree, use this order:

1. Live, disarmed hardware readback collected in the current session.
2. `config/system.yaml` for intended project configuration and safety gates.
3. `mav.parm` for the latest checked-in Cube parameter snapshot.
4. Generated session manifests and reports under `data/recordings/`.
5. The focused documents under `docs/`.
6. This file and `README.md`.
7. Historical comments, old logs, assumptions, or conversation memory.

Do not turn an assumption into a verified flag. If hardware state is cheap and
safe to read while disarmed, read it. If a value may be stale and cannot be
verified, label it as historical.

## Project Objective

Build a staged GPS-denied autonomy stack for a heavy Cube Orange+ quadrotor:

- Reliable manual optical-flow flight without depending on GPS position.
- CG-referenced 360-degree obstacle awareness.
- Timestamped lidar-inertial and visual-inertial local motion estimation.
- Local 3D mapping and fixed-altitude 2.5D planning first.
- A guarded return to the stored launch pose in the same local map.
- Eventual GPS-denied operation with pilot takeover and controlled failure.

The Cube owns stabilization, EKF3, altitude, optical-flow velocity fusion,
flight modes, motor mixing, and motor outputs. The Jetson owns sensor decoding,
SLAM/VIO, mapping, planning, evidence recording, and only future bounded local
targets. Jetson code must never become a replacement flight controller.

## Current Snapshot

Snapshot consolidated on 2026-08-08:

- Project stage: `hardware_bringup`.
- Active boot profile: JT16 obstacle avoidance only.
- Active systemd unit: `optflow-flight-logger.service`.
- The unit name is historical; its executable is
  `scripts/run_obstacle_avoidance_service.sh`.
- Cube UART: direct `/dev/ttyTHS1`, MAVLink2, 460800 baud.
- Experimental local MAVLink router: disabled and not part of active runtime.
- JT16 proximity: active and live through the MAVLink proximity backend.
- D415 contribution to flight avoidance: disabled.
- FAST-LIO, RGB-D odometry, SLAM navigation, and local return: inactive in the
  boot profile.
- Cube ExternalNav fusion: disabled.
- Jetson companion movement output: disabled.
- Return stage: `locked`; live control false.
- LIO validation approval: false.
- Full Python suite after the OA correction: 212 tests passed.
- Live OA evidence: 150/150 ready/link/proximity-health samples across 75 s,
  zero reconnects, no fresh `PRX1: No Data`.
- Last volatile bench pre-arm state: Cube disarmed in AltHold; RC receiver not
  found and compasses inconsistent. Re-read this before relying on it.

This proves a bench data path, not obstacle-avoidance flight performance and
not GPS-denied readiness.

## Non-Negotiable Safety Contract

- Never select, recommend, or automate `STABILIZE` as a fallback. On this heavy
  aircraft an inadequate throttle position can cause immediate altitude loss.
- Never use `GUIDED_NOGPS` for this architecture.
- Never arm, take off, land, select a mode, send raw attitude, mix motors, or
  command motor outputs from Jetson code during bring-up.
- Never use standard ArduPilot RTL as the GPS-denied local-return strategy.
- Never remove GPS because a visualizer trajectory looks plausible.
- Never send ExternalNav into Cube EKF3 without approved estimator evidence,
  physical calibration gates, and an explicit digest-bound approval artifact.
- Never fabricate an approval file or mark assumed data as verified.
- Never hide PRX, RC, compass, battery, vibration, EKF, or other pre-arm faults
  by disabling checks.
- Never invent an EKF origin. ArduCopter 4.6 does not guarantee a persistent
  GPS-free origin after a cold boot.
- Never let two processes own the Cube UART, IM10A, D415, or JT16.
- Never treat optical flow as globally drift-free position.
- Never claim obstacle flanking or path planning from native proximity stop or
  lean-away behavior.
- Never change the RC disarm or emergency-stop assignment without prompting the
  operator once and observing the intended transmitter channel.
- Never use hand motion, assumed distances, or a bench carry test as flight
  validation.
- GPS remains physically connected during commissioning unless an approved
  phase explicitly requires denial.
- Props are removed for bench configuration, calibration, direct UART work,
  and estimator carry tests.
- Field changes are made one variable at a time and reviewed from both Cube and
  Jetson evidence before progression.

## Airframe and Propulsion

- Aircraft class: approximately 6 kg heavy quadrotor.
- Opposite motor-center diagonal: 0.85 m.
- Propellers: 18 inch diameter.
- Physical rotating radius: `0.425 + 0.2286 = 0.6536 m`.
- Configured protected radius: 0.75 m.
- Hard obstacle clearance: 1.50 m measured horizontally from the CG.
- Battery: 6S, 22,000 mAh, mounted below the CG.
- Full-charge pack voltage: approximately 25.2 V.
- Former propulsion: one shared 55 A 4-in-1 ESC.
- Current propulsion: four individual 40 A ESCs, one per motor.
- Bench current evidence: about 11 A for one motor at 75 percent throttle;
  15 A per motor is the conservative planning case.
- Individual ESC flights showed hot motors/ESCs but no observed thrust loss.
  Heat remains a fault signal requiring measurement and review.

The low battery creates a pendulum-like mechanical response when attitude
commands are aggressive. Do not mask mechanical CG or thrust imbalance with
arbitrary controller limits. Verify CG, battery restraint, motor alignment,
propellers, thrust balance, and vibration before tuning attitude response.

Relevant Cube limits from the checked-in snapshot:

- `ANGLE_MAX=3000` or 30 degrees.
- `WPNAV_SPEED=200` cm/s.
- `WPNAV_SPEED_DN=100` cm/s.
- `WPNAV_SPEED_UP=150` cm/s.
- `PILOT_SPEED_DN=100` cm/s.
- `PILOT_SPEED_UP=150` cm/s.
- `LAND_SPEED=50` cm/s.
- `MOT_THST_HOVER` last snapshot: approximately 0.2443.
- `MOT_SPIN_ARM=0.15`, `MOT_SPIN_MIN=0.18`, `MOT_SPIN_MAX=0.95`.

Power monitoring:

- `BATT_MONITOR=4`.
- `BATT_CAPACITY=22000` mAh.
- `BATT_ARM_VOLT=23.1` V.
- `BATT_LOW_VOLT=22.2` V.
- `BATT_CRT_VOLT=21.6` V.
- Voltage pin 14, current pin 15 in the current Cube setup.
- Current and voltage scaling are aircraft-specific and must be checked against
  a wattmeter. GCS percentage is not primary evidence until scaling and
  capacity estimation are validated.

See `docs/POWER_AND_ESC.md` before propulsion changes or field progression.

## Coordinate Frames

Unless explicitly stated otherwise, all rigid-body positions and rotations use
body FRD:

- X forward.
- Y right.
- Z down.

Consequences:

- A negative mount Z is above the CG.
- A positive mount Z is below the CG.
- Horizontal bearing zero is forward.
- Positive horizontal bearing rotates toward aircraft right.

Other frames:

- Cube local flight position: NED.
- Cube body velocity/attitude contracts: FRD where specified by MAVLink.
- Browser world and launch-local trajectory: FLU for Three.js display.
- FAST-LIO backend frame follows its resolved ROS configuration and is
  explicitly transformed before comparison or Cube transport.
- Copter local velocity targets use local NED after applying launch-frame yaw.

Never change a frame conversion without updating `docs/INTERFACES.md`, adding a
test, and checking all signs with a known physical movement.

## Mechanical Sensor Geometry

All positions below are relative to the aircraft CG in body FRD.

| Device | Position | Orientation and notes |
| --- | --- | --- |
| Cube Orange+ | `(+0.08, 0.00, -0.08) m` | Mounted Yaw270; `AHRS_ORIENTATION=6` |
| Holybro H-Flow | `(0.00, 0.00, +0.10) m` | Downward on CAN2; connectors-rear yaw assumption |
| H-Flow range | `(0.00, 0.00, +0.10) m` | Downward range paired with flow |
| Hesai JT16 | `(0.00, 0.00, -0.10) m` | Centered, 0.10 m above CG, body rotation yaw 180 deg |
| Intel D415 | `(+0.19, 0.00, +0.10) m` | Forward, 0.19 m ahead and 0.10 m below CG |
| IM10A | `(+0.08, 0.00, -0.09) m` | Directly above Cube IMU, approximately 0.01 m higher |

Calibration flags:

- Camera intrinsics: verified.
- Camera-to-body extrinsics: not verified for control.
- IMU-to-body extrinsics: verified from physical position and dynamic axis
  correlation.
- JT16-to-body extrinsics: verified from cardinal targets.
- IMU noise profile: not verified.
- End-to-end sensor time synchronization: not verified.
- JT16 unit correction: verified.
- JT16 live baud: verified.
- Airframe geometry: verified from measured dimensions.

## Cube Hardware and UART

Flight controller: Cube Orange+ running ArduCopter 4.6.3 with EKF3 enabled.

Jetson TELEM2 wiring uses only 3.3 V signals and ground:

- Jetson physical pin 6: GND.
- Jetson physical pin 8: TX to Cube RX.
- Jetson physical pin 10: RX from Cube TX.
- Do not connect a TELEM power pin to Jetson GPIO power.

Current serial contract:

- TELEM1/QGC: `SERIAL1_PROTOCOL=2`, `SERIAL1_BAUD=57` (57,600 baud).
- TELEM2/Jetson: `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=460` (460,800 baud).
- Project endpoint: `/dev/ttyTHS1`.
- MAVLink2 is required for extended ODOMETRY and proximity fields.
- USB MAVLink is not required by the active runtime.

UART history and decision:

- 921600 passed some short messages but lost long 179-byte proximity frames.
- 115200 saturated under broad high-rate telemetry requests.
- 460800 passed a 15/15 bidirectional disarmed parameter audit.
- A small standalone sender held proximity health when the combined runtime
  did not.
- The active path therefore uses direct UART, minimal incoming requests, short
  eight-face messages, latest-only data, and paced writes.
- `mavlink_router.py` and its systemd unit are experimental/inactive. Do not
  silently re-enable them.

Use the process lock at `runtime/locks/cube_mavlink.lock`. A second project
reader must fail rather than compete. Stop the active service before direct
MAVProxy or a live-UART diagnostic.

## Cube EKF and Flight Configuration

Core values:

- `AHRS_EKF_TYPE=3`.
- `EK3_ENABLE=1`.
- `AHRS_ORIENTATION=6`.
- `ARMING_CHECK=1`.
- `GUID_OPTIONS=64` so Guided XY position/velocity stabilization remains
  enabled.
- `GUID_TIMEOUT=0.5` s, verified 2026-08-01.
- `TERRAIN_ENABLE=0`.

EKF source sets from the current parameter snapshot:

- Source set 1: POSXY none, POSZ barometer, VELXY optical flow, VELZ none,
  compass yaw.
- Source set 2: POSXY none, POSZ barometer, VELXY optical flow, VELZ none,
  compass yaw.
- Source set 3: GPS POSXY, barometer POSZ, GPS VELXY, VELZ none, compass yaw.
- No source set selects ExternalNav.
- Automatic EKF source switching from this project is disabled.

H-Flow and range:

- `FLOW_TYPE=6`.
- `FLOW_POS_Z=+0.10` m.
- `FLOW_ORIENT_YAW=0` in the current snapshot.
- `RNGFND1_TYPE=24`.
- `RNGFND1_ORIENT=25` downward.
- `RNGFND1_MIN_CM=8`, `RNGFND1_MAX_CM=3000`.
- `RNGFND1_POS_Z=+0.10` m.

Flight-mode snapshot:

- `FLTMODE_CH=5`.
- Slots 1 and 2: mode id 5.
- Slots 3 and 4: mode id 16.
- Slots 5 and 6: mode id 6.

Do not infer a safe field switch layout from numeric ids alone. Confirm the GCS
mode label, transmitter switch, and pilot intent before flight. Standard RTL
must not be used as a GPS-denied return.

## RC Channel Contract

- RC5: Cube flight-mode selection.
- RC7: proximity avoidance, `RC7_OPTION=40`.
- RC8: FlowHold, `RC8_OPTION=71`.
- RC9: companion SLAM-return request, deliberately unassigned in Cube with
  `RC9_OPTION=0`.
- RC10: Cube Land, `RC10_OPTION=18`.
- RC11: motor emergency stop, `RC11_OPTION=31`.
- RC7 project thresholds: engage at or above 1700 us, disengage at or below
  1300 us.
- RC9 project thresholds: same low/high values.

The return controller requires RC9 to be observed low after arm and then moved
high deliberately. RC9 low cancels immediately. A flight-mode change blocks
output. The Jetson does not select Land; RC10 remains pilot/Cube-owned.

## Holybro H-Flow Contract

- Transport: DroneCAN on Cube CAN2.
- Mount: downward, 0.10 m below CG.
- Optical flow supplies local horizontal velocity, not absolute position.
- Downward range scales optical flow and supports altitude-dependent validity.
- Flow can drift or fail with poor texture, low light, repeating surfaces,
  excess height, motion blur, vibration, bad calibration, or range failure.
- FlowHold may maintain useful local XY without GPS, but it is not globally
  anchored and may drift like AltHold over time.
- Position-hold naming such as "GPS assist" must not be interpreted as
  GPS-denied global position.

Flight progression must verify flow quality, range freshness, pilot authority,
and mode behavior with GPS still available before any GPS denial.

## JT16 Contract

- Model: Hesai JT16.
- Transport: serial RS485 through PL2303 adapter.
- Stable symlink: `/dev/jt16_usb`.
- USB VID/PID: `0x067B:0x23A3`.
- Unit serial identifier: `DCCEb114J19`.
- Active baud: 3,000,000.
- Legacy baud retained for recovery: 3,125,000.
- Official SDK revision:
  `534c707846a810e8211b93446f878dbf415f7000`.
- Native bridge: `build/jt16_bridge/optflow-jt16-bridge`.
- Correction file: `data/calibrations/jt16/JT16_sample_angle-2.csv`.
- Correction and cardinal orientation are verified.
- Native SDK XYZ uses +Y at zero azimuth, +X right, +Z up.
- Conversion to lidar-forward FRD is `(Y, X, -Z)` before mount rotation and
  CG translation.
- Final configured body rotation is roll 0, pitch 0, yaw 180 degrees.

The C++ bridge emits a binary header plus packed point records. It can preserve
raw serial packets for evidence, but the OA-only service intentionally does not
record raw data continuously.

## D415 Contract

- Model: Intel RealSense D415.
- Stable selected serial: `327322062285`.
- Configured stream: 640 x 480 at 30 FPS.
- RGB HTTP port: 8770.
- Mount: forward, +0.19 m X, +0.10 m Z-down.
- Intrinsics and depth scale passed measured-wall checks at 1.50 m and 2.50 m.
- Final camera-to-body rotation is not control-verified.
- Outdoor depth was intermittent in the 2026-08-03 flight evidence.
- D415 is excluded from active Cube proximity.
- D415 remains available for RGB streaming, point clouds, mapping diagnostics,
  and shadow RGB-D odometry.

Invalid depth must remain unknown. Do not turn zero/invalid outdoor depth into
clear space for flight control.

## IM10A Contract

- Model: Hiwonder IM10A.
- Stable symlink: `/dev/imu_usb`.
- USB VID/PID: `0x1A86:0x7523`.
- Current validated profile: 230400 baud, approximately 200 Hz.
- Enabled records: sensor TIME, acceleration, and angular velocity.
- Body sign map: X `+1`, Y `-1`, Z `-1`.
- Axis map was dynamically verified against Cube gyro motion.
- Position: +0.08 m X, 0 Y, -0.09 m Z.
- Sensor time is a relative device clock, not UTC.
- A 2026-07-31 audit measured approximately 199.84 Hz with no observed
  checksum, payload, ordering, or drop errors.

Historical note: the sensor originally ran the factory 9600-baud, 10 Hz WIT
stream. Do not revert or change baud/rate casually. Any configuration change
must use the reversible configurator with automatic baud recovery and must
re-test rate, jitter, checksum, ordering, and drops.

Noise covariance remains unapproved until the stationary noise procedure
passes. Do not tune estimator covariance from a short hand-held sample.

## Active Obstacle-Avoidance-Only Runtime

Systemd unit:

```text
optflow-flight-logger.service
  -> scripts/run_obstacle_avoidance_service.sh
  -> python -m optflow_slam.obstacle_avoidance_service
  -> native JT16 bridge
```

Active resources:

- Cube `/dev/ttyTHS1`.
- JT16 `/dev/jt16_usb`.

Explicitly inactive resources/features:

- D415.
- IM10A.
- ROS 2 and FAST-LIO.
- RGB-D odometry.
- SLAM map and trajectory.
- Navigation and return controller.
- ExternalNav/ODOMETRY output.
- Companion movement commands.

Status file: `runtime/obstacle_avoidance_status.json`.

The service requests only RC channels at 5 Hz and `SYS_STATUS` at 2 Hz, plus
normal heartbeats. It does not request all streams or high-rate raw IMU.

### OA Point Filtering

From `config/system.yaml`:

- Stage `active`.
- MAVLink output true.
- JT16 input true.
- D415 input false.
- Sector increment 5 degrees, 72 internal sectors.
- Horizontal distance 0.30 m to 8.00 m.
- Body vertical slice -0.40 m to +0.40 m around CG.
- Protected self-filter radius: max of 0.30 m and 0.75 m, therefore 0.75 m.
- Minimum points per sector: 8.
- Distance statistic: 10th percentile.
- Temporal window: 3 scans.
- Temporal selection: conservative minimum.
- Source stale timeout: 0.45 s.
- Target transport cycle: 10 Hz.

### MAVLink Proximity Transport

- Internal 72-sector scan is reduced to eight 45-degree faces.
- Each face uses the minimum known five-degree sector.
- A fresh healthy face with no in-range return sends 801 cm, which is
  `max_distance + 1` for the configured 800 cm sensor range.
- All eight `DISTANCE_SENSOR` packets are sent per cycle.
- Face id and orientation both use values 0 through 7.
- Face zero is forward; values increase clockwise by 45 degrees.
- Packet gap is 12 ms to prevent a serial burst.
- Latest-only scan scheduling prevents stale queue buildup.
- A stale JT16 scan stops transport instead of replaying old clear/obstacle
  data.
- ArduPilot's MAVLink proximity timeout is 0.5 s.
- Health monitoring uses `MAV_SYS_STATUS_SENSOR_PROXIMITY`, not the laser
  rangefinder bit.

Earlier bugs and their causes:

- Full `OBSTACLE_DISTANCE` frames were lost at some link rates.
- The combined runtime requested enough telemetry to create queue/scheduling
  pressure and intermittent PRX expiry.
- A tuple passed to `pymavlink.recv_match(type=...)` was treated as one invalid
  type and discarded post-connect heartbeats; it now uses a list.
- Fresh JT16 scans with zero accepted sectors originally sent zero packets,
  causing `PRX1: No Data`; they now send healthy no-return faces.
- Initial status watched `MAV_SYS_STATUS_SENSOR_LASER_POSITION`; it now watches
  the real proximity bit.

Never "fix" PRX by setting `PRX1_TYPE=0`, disabling avoidance, or weakening
arming checks. Fix data freshness and verify direction/distance.

### Native Cube Avoidance

Current desired/read-back parameters:

- `RC7_OPTION=40`.
- `PRX1_TYPE=2`.
- `AVOID_ENABLE=7`.
- `AVOID_MARGIN=1.50` m.
- `AVOID_DIST_MAX=1.50` m.
- `AVOID_BEHAVE=0`.
- `AVOID_BACKUP_SPD=0.50` m/s.
- `AVOID_ACCEL_MAX=1.00` m/s2.

The parameter tool first writes `AVOID_ENABLE=0`, applies other values, then
restores the configured enable mask and verifies readback. It is disarmed-only.
Changing proximity type may require Cube reboot.

Native avoidance is a short-range reactive layer. It does not run A*, Dijkstra,
SLAM relocalization, or obstacle flanking. The 1.50 m boundary is a target, not
a physical guarantee; braking distance, mode behavior, latency, mass, and
approach speed still matter.

### OA Audio

- Startup: one rising tune after the first successful Cube heartbeat.
- No companion arming tune.
- No companion mode-change tune.
- 1.50 to 2.00 m: 1 beep/s, warning only.
- 1.25 to 1.50 m: 3 beeps/s.
- Below 1.25 m: linear rise toward 10 beeps/s at 0.75 m.
- Beeps require fresh data, RC7 high, and an armed aircraft because
  `only_when_armed=true`.
- RC7 low or unknown, disarmed, stale, or clear states are silent.

## Runtime Ownership and Services

Exactly one process owns each hardware endpoint.

| Runtime/tool | Cube UART | IM10A | D415 | JT16 | Control output |
| --- | --- | --- | --- | --- | --- |
| OA-only boot service | Owns | No | No | Owns | Proximity only |
| Full hardware visualizer | Owns | Owns | Optional | Optional | Diagnostic/tunes; no movement |
| Passive flight service | Owns | Owns | Armed capture | Armed capture | None |
| LIO shadow | Reference owner | Owns | Optional by mode | Owns | None unless explicit disarmed ODOMETRY shadow |
| SLAM proof | Reference owner | Owns | Owns | Owns | Ready tune only |
| SLAM navigation runtime | Owns | Owns | Owns | Owns | Locked/monitor-only today |
| Camera stream | No | No | Owns | No | None |
| `obstacle-status` | Cache only | No | No | No | None |
| Trajectory monitor | Cache/stream only | No | No | No | None |

Use `RuntimeResourceLock` for Cube and JT16 ownership. Existing direct readers
must honor the lock. Do not weaken the lock because a tool is "read-only";
serial reads still consume bytes from the owner.

Boot persistence uses a per-user systemd service and user lingering. The
installed unit is a symlink to the repository-owned service definition.

Useful operations:

```bash
systemctl --user restart optflow-flight-logger.service
systemctl --user status optflow-flight-logger.service --no-pager --full
systemctl --user is-enabled optflow-flight-logger.service
./optflow obstacle-status
```

## FAST-LIO2 Shadow Runtime

The LIO stack is project-local and does not depend on `/opt/ros` as a mutable
system workspace.

Pinned components are in `third_party/lio.lock.json`:

- ROS 2 Humble binary archive `release-humble-20260220` with checked SHA256.
- PCL revision `e8ed4be802f7d0b1acff2f8b01d7c5f381190e05`.
- `pcl_msgs` revision `8a925a7c4626df52dba7ccc5bda5900d63678880`.
- `pcl_conversions` revision
  `67a5c2ba4c4de3ca21c5cd495812a01ced3fb69a`.
- Hesai FAST-LIO2 revision
  `bb2842d34990761eebbd4cc3188e94c7c662a673`.
- ikd-Tree revision `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
- Local user-runtime patch:
  `third_party/patches/fast_lio_hesai-user-runtime.patch`.

Build root: `runtime/lio/`.

ROS contract:

- JT16 topic `/optflow/jt16/points`.
- IM10A topic `/optflow/im10a/imu`.
- Odometry topic `/Odometry`.
- Diagnostics topic `/optflow/lio/diagnostics`.
- Map output false by default.
- Pose output to Cube false.
- Disarmed ODOMETRY shadow transport true as an explicit bench-only path.

### Time Synchronization

- Common reference: Jetson monotonic time.
- IM10A sensor time is mapped affinely into host monotonic time.
- JT16 point/frame time is mapped separately.
- Window sample count: 2000.
- Maximum IMU fit span: 20 s.
- Maximum lidar fit span: 60 s.
- Minimum IMU samples: 200.
- Minimum lidar samples: 20.
- Minimum fitted span: 2 s.
- Current lidar-to-IMU offset: +0.010 s.
- Maximum allowed drift: 5000 ppm during current bring-up.
- Maximum IMU residual p95: 5 ms.
- Maximum lidar residual p95: 15 ms.

Five dynamic sessions supported the +10 ms offset. A matched yaw run improved
Cube/LIO attitude p95 from approximately 2.84 deg at zero offset to 2.27 deg.
Global sensor-time verification remains false because D415 alignment is not
closed.

### LIO Validation Gates

Current approval is false. Required report limits:

- Minimum duration: 60 s.
- Minimum odometry rate: 4 Hz.
- Stationary window: 5 s.
- Maximum stationary drift: 0.15 m.
- Maximum return-to-start error: 0.35 m.
- Maximum position jump: 0.50 m.
- Maximum speed: 3.0 m/s.
- Maximum attitude jump: 10 deg.
- Maximum clock resets: zero.
- Minimum Cube reference samples: 100.
- Minimum Cube reference path: 2.0 m.
- Maximum Cube horizontal RMSE: 0.50 m.
- Maximum Cube vertical RMSE: 0.40 m.
- Maximum Cube attitude p95: 10 deg.
- Cube/reference path ratio: 0.70 through 1.30.

Cube local position is an independent comparison signal, not ground truth and
never an input to FAST-LIO.

Do not set `validation.approved=true` manually. Approval requires a generated
passing report and matching digest.

## RGB-D Odometry and SLAM Proof

`rgbd_odometry.py` performs metric D415 RGB-D odometry with a time-matched
IM10A gyro rotation prior. It runs beside FAST-LIO in proof and shadow modes.

Known characteristics:

- D415 input is approximately 30 FPS.
- Dense RGB-D odometry measured around 0.87 Hz on the flight Jetson.
- Navigation visual freshness timeout is therefore 2.0 s.
- Visual/LIO disagreement limit is 0.35 m.
- RGB-D is an independent consistency source, not the fast primary estimator.

`slam-poc` is the shortest visible proof:

- It can guide a props-off carry sequence.
- It flashes the browser twice before each new movement instruction.
- It records LIO and RGB-D trajectories and maps.
- It captures a shadow local-return proposal.
- It sends no pose, mode, arm, navigation, or movement command.
- Its only normal Cube output is a one-shot ready tune.
- Default browser: `http://<jetson>:8767/slam-poc`.

A good visual trail is not sufficient. Reports must score timing, drift,
return-to-start, jumps, source agreement, and Cube reference behavior.

Current estimator remains local odometry/mapping rather than proven
loop-closed/relocalizing SLAM. Do not claim persistent global mapping.

## Cube ODOMETRY Shadow Boundary

`cube-odom-shadow` is the only explicit estimator-to-Cube bench transport path
currently enabled in config.

It must:

- Run disarmed with props removed.
- Audit `AHRS_EKF_TYPE`, `EK3_ENABLE`, and all source axes in all three EKF
  source sets.
- Refuse to send if any Cube source selects ExternalNav.
- Stop immediately if Cube arms or heartbeat becomes stale.
- Send MAVLink2 ODOMETRY with local-FRD/body-FRD contracts.
- Write no Cube parameter.
- Cause no EKF fusion.

Historical bench proof on 2026-07-31 sent 112 packets near 4.999 Hz with zero
parameter writes while ExternalNav was absent. Re-verify before relying on it.

## Mapping and Planning Contract

Version 1 architecture is fixed-altitude 2.5D:

1. Build a local occupancy/cost map from JT16 geometry and validated D415
   depth.
2. Use A* for global route selection.
3. Use a proven collision-aware local planner to validate and adjust immediate
   motion.
4. Send conservative local velocity targets through regular Guided only.
5. Retain launch pose and breadcrumbs for local return.

Current config:

- Architecture `fixed_altitude_2_5d_first`.
- Global planner `astar`.
- Local planner `proven_collision_aware_library_tbd`.
- Autonomous control false.
- ExternalNav output false.
- Target rate 10 Hz.
- Initial horizontal speed 0.5 m/s for general navigation design.
- Initial vertical speed 0.3 m/s.
- Initial yaw rate 20 deg/s.
- Local pose stale timeout 0.45 s.
- Command stale timeout 0.50 s.

Do not hand-roll the collision-aware local planner when a proven ROS-compatible
library can meet the contract. A* does not consume raw camera frames directly;
it consumes a validated map/costmap.

The current OA-only runtime is not this planner. It provides reactive Cube
proximity only.

## Guarded Local Return Contract

`SLAM RETURN` is a local-map breadcrumb return, not ArduPilot RTL.

Intended behavior:

1. Capture launch pose after estimator initialization.
2. Record outbound breadcrumbs in the same map revision.
3. On deliberate RC9 request in regular Guided, traverse collision-checked
   breadcrumbs in reverse.
4. Send bounded velocity-only `SET_POSITION_TARGET_LOCAL_NED` targets.
5. Stop and release authority on any stale or failed gate.

Current config:

- Stage `locked`.
- Live control false.
- Approval file `runtime/approvals/slam_return_live.json`.
- Status file `runtime/slam_navigation_status.json`.
- Required mode `GUIDED`.
- RC request channel 9.
- Pilot Land channel 10.
- Engage 1700 us, disengage 1300 us.
- EKF source set 1.
- Altitude 1.0 m through 8.0 m.
- Minimum flow quality 50.
- Telemetry stale timeout 0.35 s.
- Battery stale timeout 2.0 s.
- Minimum voltage 22.20 V.
- Command rate 10 Hz.
- Commissioning horizontal speed 0.30 m/s.
- Absolute loader-enforced speed ceiling 0.75 m/s.
- Horizontal acceleration 0.50 m/s2.
- Arrival radius 0.20 m.
- Breadcrumb spacing 0.15 m.
- Waypoint radius 0.12 m.
- Visual stale timeout 2.0 s.
- Visual disagreement limit 0.35 m.

Output gates include:

- Fresh Cube heartbeat.
- Disarmed/armed lifecycle correctly observed.
- Required regular Guided mode.
- RC9 low observed after arm and deliberate high request.
- Fresh RC, local position, optical flow, range, battery, LIO, RGB-D, obstacle,
  and command state.
- Valid EKF origin.
- Exact Cube parameters.
- Approved physical calibrations.
- Approved LIO report with matching revision/digest.
- Digest-bound live approval file.
- Collision-free reverse breadcrumb path.
- No 1.50 m clearance breach.

Any failure latches live return off for the arm cycle and commands a
zero-velocity handoff before authority release where a command path exists.
This version stops for unexpected obstacles; it does not flank or replan.

The project must never automatically switch to Stabilize. Pilot takeover uses
the already-tested altitude-controlled mode or manual landing strategy.

## GPS-Denied Origin and Return Reality

- Optical flow plus range can support local drift-limited hold without GPS.
- It does not provide a geographic home.
- Standard RTL requires valid global home/position and is not reliable as the
  project GPS-denied return.
- Local return needs launch pose and current pose in the same live map.
- A cold boot without GPS may lack a usable EKF local origin.
- The operator may explicitly set a valid origin in the GCS, or the system may
  initialize while GPS is available before denial.
- Software must never silently create fake latitude/longitude.
- If localization is lost, correct responses are hold, pilot takeover, or
  controlled landing, not blind dead reckoning.

## Flight Evidence Logger

The passive logger is a manual tool while the OA service is installed. Stop the
OA service before starting it.

Arm-triggered behavior:

- Five seconds of pre-arm telemetry/event buffer.
- Full recording while armed.
- Ten seconds of tail after disarm.
- Re-arming during tail keeps the same session.
- Refuses repeated partial sessions in one armed period.
- Stops before free space falls below 5 GB.
- Full D415 bag has measured around 1.3 GB/min in bench evidence.

Session evidence can include:

- MAVLink telemetry and raw event NDJSON.
- Sensor timing NDJSON.
- IM10A records.
- D415 RealSense bag, RGB/depth, sampled point clouds.
- JT16 raw serial capture, decoded points, and stats.
- Merged environment PLY.
- Shadow ideal-motion and return proposals.
- Optional Cube DataFlash BIN and decoded summary.
- Manifest and generated analysis reports.

DataFlash interpretation must inspect vibration, clipping, attitude, desired
versus achieved rates, motor outputs, thrust loss flags, voltage/current,
failsafes, mode transitions, EKF, optical flow, range, and crash endpoint. Do
not infer a single root cause from one symptom.

## Visualizer Contract

Main visualizer default: port 8765.

- Primary view always shows the drone using Cube attitude/IMU.
- Small corner view shows external IM10A motion.
- IM10A display uses measured X/-Y/-Z body mapping and startup reference
  alignment.
- Browser transport targets 60 Hz; source rates remain visible.
- 3D SCAN shows measured drone geometry, rolling D415/JT16 clouds, Cube motion,
  trajectory layers, and protected radius.
- 3D SCAN is diagnostic; a route line is not proof of SLAM.
- In the spatial UI, trajectory is on by default, JT16 is off by default, and
  D415/JT16 are independently toggleable.
- LIO, RGB-D, Cube, breadcrumbs, and return target are independently
  toggleable.
- Rejected raw LIO points may remain visible for diagnosis but must never feed
  accepted navigation state.
- A red SLAM fault freezes the guarded navigation path.

Other ports:

- LIO assist: 8766, `/lio-assist`.
- SLAM proof: 8767, `/slam-poc`.
- D415 RGB: 8770.

Use `--host 0.0.0.0 --no-browser` for LAN access. Another device on the same
network opens `http://<jetson-ip>:<port>`.

Headless VNC is project-owned and should start at boot after installation.
Verify headless operation without a physical HDMI display before enclosing the
Jetson. See `docs/HEADLESS_ACCESS.md`.

## Audio Contract

Useful project tones are retained:

- OA service rising initialization tone.
- Obstacle-distance warning tones.
- LIO/SLAM proof ready tone.
- User-intervention prompt tones in guided calibration/proof sequences.

Do not add companion arming or generic mode-change tones because ArduPilot
already owns those audible events. Avoid overlapping or indistinguishable tune
sequences.

## Data and Artifact Policy

Project boundary:

- `.venv/`: project Python runtime.
- `vendor/python/`: offline aarch64 wheel mirror.
- `visualizer/node_modules/` and `visualizer/dist/`: generated frontend state.
- `data/calibrations/`: measured calibration artifacts.
- `data/logs/`: application logs.
- `data/maps/`: versioned local maps and metadata.
- `data/recordings/`: sessions and reports.
- `data/flight_logs/`: imported raw Cube evidence; local and Git-ignored.
- `runtime/`: transient status, locks, point frames, and local LIO build.
- `ros_ws/`: only ROS workspace source boundary.
- `third_party/`: pinned external sources/revisions and patches.
- `hardware/`: host configuration source of truth.

Do not import code, configuration, maps, logs, binaries, or Python environments
from sibling workspaces. `tests/test_project_boundary.py` enforces this.

Git must not contain generated flight logs, tlogs, raw recordings, maps,
runtime status, local ROS builds, frontend builds, node modules, or fetched
third-party source trees. Small reproducible calibration/configuration files,
the pinned wheel mirror, native source, tests, and parameter reference snapshot
belong in source control.

`mav.parm` is a reference snapshot, not an automatic install profile. Never
apply it wholesale to another aircraft.

## Command Safety Classification

Cache/read-only beside active service:

```bash
./optflow obstacle-status
./optflow flight-status
./optflow slam-return-status
```

Offline and no flight hardware:

```bash
./optflow build
./optflow test
./optflow analyze <session> --cube-log <BIN>
./optflow slam-timing <session>
./optflow rtl-shadow <session>
./optflow lio-validate <session>
```

Hardware-owning, stop active service first:

```bash
./optflow preflight --profile fc_bench
./optflow sensor-check
./optflow obstacle-check
./optflow visualizer --host 0.0.0.0 --no-browser
./optflow flight-service
./optflow lio-shadow
./optflow slam-poc
```

Disarmed parameter tools:

```bash
./optflow cube-mount
./optflow cube-avoidance
./optflow slam-return-setup
./optflow im10a
```

An `--apply` flag changes the safety category. Confirm Cube disarmed, props
removed, current readback, intended scope, and backup behavior before use.

Host-changing tools may require sudo:

```bash
sudo ./optflow install-ch341
sudo ./optflow install-pl2303
sudo ./optflow install-realsense-rules
sudo ./optflow install-headless-vnc
```

Do not ask for or handle a password unless the current operation actually
needs sudo. Let the terminal prompt the user.

## Staged Bring-Up Order

1. Mechanical frame, CG, battery restraint, wiring, propellers, motors, ESCs,
   and power-module acceptance.
2. Props-off Cube orientation, accelerometer, compass, RC, emergency stop,
   battery, UART, H-Flow, range, and PRX checks.
3. GPS-connected manual flight with OA off in the already-tested
   altitude-controlled mode.
4. GPS-connected low-speed Loiter OA test, RC7 deliberate.
5. GPS-connected low-speed FlowHold OA test.
6. Passive GPS-Loiter LIO/VIO/OA shadow flight with zero movement output.
7. Repeat trajectory, timing, return-to-start, and Cube comparison evidence.
8. Keep GPS connected while commissioning regular Guided bounded targets.
9. Prove cancellation, stale-data handling, pilot takeover, and collision
   response.
10. Prove map/path planning and obstacle handling.
11. Only then test intentional GPS denial while retaining connected recovery
    hardware and pilot authority.

For every field run:

- One-shot plan before leaving Jetson access.
- Services autostart and write QGC status where required.
- Keep a clear area and a second person when practical.
- Approach test obstacles at no more than the procedure's limit.
- Abort on wrong sector, unexpected lean, oscillation, stale proximity,
  estimator fault, loss of telemetry, loss of pilot authority, motor
  saturation, abnormal sound, or excessive heat.
- Do not switch to Stabilize during abort.
- Review Cube and Jetson evidence before the next attempt.

## Known Gaps and Blockers

- Last volatile pre-arm errors included RC not found and compasses
  inconsistent.
- D415 body extrinsics are not control-verified.
- Outdoor D415 depth reliability is not approved.
- IM10A stationary noise profile is not approved.
- End-to-end D415/JT16/IM10A clock synchronization is not approved.
- LIO validation approval is false.
- FAST-LIO remains local odometry without proven loop closure or
  relocalization.
- Persistent maps and cross-flight localization are not approved.
- ExternalNav-to-Cube fusion is disabled.
- Live SLAM return is locked.
- Collision-aware local planner is TBD.
- Native OA does not flank obstacles or optimize paths.
- GPS-free cold-boot EKF origin handling is unresolved operationally.
- Hot motors/ESCs require continued thermal and current evidence.
- GPS-denied RTL has no flight proof.

Do not collapse these into one score or wave them away. Each has a separate
test and failure consequence.

## Next Engineering Sequence

The quickest defensible route from the current state is:

1. Resolve RC and compass pre-arm faults with OA service still bench-only.
2. Complete props-off JT16 four-direction and empty-space PRX checks on the
   final assembled aircraft.
3. Perform low-speed GPS-connected Loiter then FlowHold OA commissioning.
4. Complete IM10A stationary noise and D415 timing/extrinsic evidence.
5. Fly passive GPS-Loiter shadow sessions and validate LIO/VIO trajectories.
6. Add a proven collision-aware local planner and test it in replay/simulation.
7. Commission bounded regular-Guided targets with GPS connected and return
   output still tightly gated.
8. Prove local breadcrumb return and cancellation repeatedly.
9. Test deliberate GPS denial only after all previous reports pass.

## Source Map

Configuration and shared infrastructure:

- `config/system.yaml`: all intended geometry, stages, limits, and gates.
- `src/optflow_slam/config.py`: strict loader and cross-field safety checks.
- `src/optflow_slam/paths.py`: canonical project-local paths.
- `src/optflow_slam/models.py`: shared data models.
- `src/optflow_slam/runtime_lock.py`: exclusive hardware ownership.
- `src/optflow_slam/mavlink_compat.py`: pinned pymavlink guards.
- `optflow`: project launcher.

Cube and proximity:

- `cube_mount.py`: Cube orientation audit/apply.
- `cube_avoidance.py`: native OA parameter audit/apply.
- `mavlink_proximity.py`: 8-face proximity encoding.
- `obstacles.py`: point filtering, sectors, fusion, clearance, alerts.
- `obstacle_check.py`: no-Cube bench sector check.
- `obstacle_avoidance_service.py`: active minimal runtime.
- `mavlink_router.py`: inactive experimental raw UART/UDP relay.
- `slam_return_setup.py`: Cube return contract audit.

Sensors and calibration:

- `im10a.py`: WIT/IM10A decoder.
- `im10a_config.py`: reversible serial profile audit/apply/recovery.
- `imu_noise_calibration.py`: stationary noise capture/report.
- `imu_alignment.py`: dynamic Cube/IMU timing and scale comparison.
- `jt16_calibration.py`: guided four-cardinal calibration.
- `jt16_plane_calibration.py`: wall/floor/ring validation.
- `clock_sync.py`: robust affine device-to-host time mapping.
- `camera_server.py`: D415 RGB service.
- `sensor_check.py`: Cube-independent sensor check.
- `native/jt16_bridge/`: C++ official SDK bridge.

Flight evidence:

- `flight_logger.py`: synchronized session recording.
- `flight_service.py`: arm-triggered orchestration and status.
- `flight_analysis.py`: reports and DataFlash integration.
- `flight_guide.py`: QGC-guided shadow-flight prompts.
- `flight_supervisor.py`: one guided shadow run per boot plus logger resume.
- `pointcloud.py`: PLY helpers.
- `slam_timing.py`: rate/jitter/drop/clock analysis.

Estimation and SLAM:

- `lio_bridge.py`: synchronized JT16/IM10A ROS 2 publishing.
- `lio_shadow.py`: shared LIO/RGB-D/Cube shadow orchestration.
- `lio_validation.py`: trajectory pass/fail report.
- `lio_visual_assist.py`: browser-guided carry sequence.
- `rgbd_odometry.py`: D415 metric visual odometry.
- `slam_poc.py`: proof launcher.
- `slam_poc_visual.py`: proof dashboard state/report.
- `cube_odometry.py`: disarmed transport-only ODOMETRY sender.

Return and navigation:

- `rtl_shadow.py`: breadcrumbs, bounded proposals, offline replay.
- `slam_navigation.py`: gates, controller, transport, status.
- `slam_navigation_service.py`: full hardware-owning monitor/live runtime.

Visualization:

- `visualizer_server.py`: Cube/IMU/point-cloud HTTP/SSE backend.
- `spatial_stream.py`: bounded atomic live point-cloud frames.
- `visualizer/src/main.js`: main drone-motion UI.
- `visualizer/src/drone.js`: reusable measured drone scene.
- `visualizer/src/spatial.js`: 3D clouds and trajectories.
- `visualizer/src/lio-assist.js`: LIO guidance UI.
- `visualizer/src/slam-poc.js`: proof UI.

Host support:

- `hardware/kernel/ch341/`: IM10A USB serial kernel support.
- `hardware/kernel/pl2303/`: JT16 USB-RS485 kernel support.
- `hardware/udev/`: stable sensor symlinks and D415 permissions.
- `hardware/systemd/`: OA service, experimental router, VNC unit.
- `hardware/headless/`: EDID and Xorg configuration.
- `scripts/bootstrap_lio_runtime.sh`: project-local ROS/PCL/FAST-LIO build.
- `third_party/lio.lock.json`: pinned revisions and hashes.

## Documentation Map

- `README.md`: operator/developer overview and command reference.
- `docs/ARCHITECTURE.md`: responsibility and failure boundaries.
- `docs/INTERFACES.md`: frames, clocks, proximity, odometry, ExternalNav, and
  command messages.
- `docs/BRINGUP.md`: staged bench/field progression.
- `docs/OBSTACLE_AVOIDANCE.md`: active JT16/Cube path and field sequence.
- `docs/SENSOR_CALIBRATION.md`: D415/JT16 measured-target procedure.
- `docs/LIO_SHADOW.md`: IMU profile, timing, shadow sequence, validation.
- `docs/SLAM_RETURN.md`: output gates, RC9 behavior, current lock.
- `docs/FLIGHT_LOGGER.md`: recording contract and analysis.
- `docs/POWER_AND_ESC.md`: propulsion acceptance.
- `docs/HEADLESS_ACCESS.md`: VNC/headless setup.
- `hardware/README.md`: host changes and install commands.
- `runtime/README.md`: transient-state policy.
- `data/README.md`: persistent evidence policy.

## Testing Contract

Run before committing:

```bash
./optflow test
git diff --check
```

Current Python suite count at this snapshot: 212 tests.

Frontend visual contracts use Playwright and include:

- Main responsive/visual check.
- Live spatial 3D check.
- LIO assist check.
- SLAM proof guide check.

For 3D changes, verify desktop and mobile screenshots, nonblank canvas pixels,
camera framing, trajectory visibility, layer toggles, no overlap, and live
movement. A successful frontend build alone is insufficient.

Test scope must increase with safety impact:

- Pure parser/math change: focused unit tests.
- Frame/sign/timing change: unit tests plus recorded replay.
- MAVLink change: packet-level test plus disarmed live health observation.
- Parameter tool: backup, disarmed guard, write/readback test.
- Service ownership: lock and systemd restart tests.
- Flight behavior: staged field evidence, never unit tests alone.

## Engineering Rules for Agents

- Read surrounding code and use existing dataclasses, parsers, status files,
  locks, and command wrappers.
- Keep changes within this repository and preserve project-local/offline setup.
- Use structured MAVLink, YAML, JSON, ROS, and point-cloud APIs rather than
  ad-hoc strings where possible.
- Use latest-only queues for safety-sensitive live sensor data.
- Stale or missing data is unknown and must fail closed. A protocol-defined
  fresh no-return value is distinct from stale input.
- Keep shadow output and active control physically and logically separate.
- Every active output needs an explicit config enable and runtime gate.
- Preserve the rule that no approved evidence means no live output.
- Do not silently broaden telemetry stream requests.
- Do not add beeps for events ArduPilot already signals.
- Keep comments short and explain only non-obvious safety or frame logic.
- Update README, AGENTS, focused docs, tests, and config together when a
  contract changes.
- Do not commit generated logs, maps, recordings, runtime state, fetched SDK
  trees, local builds, or secrets.
- Do not revert unrelated user changes in a dirty worktree.
- Before a hardware diagnostic, verify Cube disarmed from a filtered
  ArduPilot autopilot heartbeat, not the first arbitrary component heartbeat.
- After any live service change, observe it long enough to cross relevant
  watchdog/pre-arm intervals and inspect actual `STATUSTEXT`.

## Definition of Done by Change Type

OA service change:

- Focused transport/service tests pass.
- Full suite passes.
- Service is active and boot-enabled.
- Cube remains disarmed during validation.
- JT16 freshness stays inside 0.45 s.
- Real proximity health bit remains true.
- Packet counter advances.
- No fresh `PRX1: No Data` across multiple pre-arm cycles.
- No competing hardware process is active.

Estimator change:

- Timing, frame, and covariance contracts are explicit.
- Shadow report is generated from real data.
- No Cube pose or movement output occurs unless explicitly testing the
  disarmed transport boundary.
- Validation remains false unless every threshold passes.

Navigation change:

- Replay and shadow behavior pass first.
- Speed/acceleration/freshness limits are enforced in config and runtime.
- RC cancellation, mode loss, stale command, stale pose, obstacle breach, and
  estimator disagreement fail closed.
- No live approval is created automatically.

Documentation/release change:

- Current active runtime is described accurately.
- Locked/shadow/active states are not conflated.
- Commands identify hardware ownership.
- Generated artifacts are ignored.
- `./optflow test` and `git diff --check` pass.
