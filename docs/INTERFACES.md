# Interface Contracts

## Coordinate Frames

The SLAM side uses ROS-style frames:

- World: ENU, X east, Y north, Z up.
- Body: FLU, X forward, Y left, Z up.

ArduPilot local navigation uses:

- World: NED, X north, Y east, Z down.
- Body: FRD, X forward, Y right, Z down.

The MAVLink bridge owns the complete ENU/FLU to NED/FRD transform. Application
code must not swap individual axes ad hoc.

Position conversion:

```text
north = enu_y
east  = enu_x
down  = -enu_z
```

Body vector conversion:

```text
forward = flu_x
right   = -flu_y
down    = -flu_z
```

Quaternions must be transformed with a tested rotation library and unit tests.
Do not derive a quaternion conversion by changing signs until it looks right.

The pinned FAST-LIO path is explicit: its sensor bridge converts JT16 and
IM10A measurements into aircraft body FRD before estimation, so the resulting
`camera_init` frame retains the initial FRD heading. `cube-odom-shadow` removes
the initial position and yaw, publishes pose in `MAV_FRAME_LOCAL_FRD`, and
publishes derived velocity plus IMU rates in `MAV_FRAME_BODY_FRD`. It does not
pretend that this backend output is generic ROS ENU/FLU.

## Time

All Jetson sensor samples must enter the estimator with timestamps tied to a
single clock.

Required measurements:

- Camera frame timestamp and arrival timestamp.
- IMU sample timestamp and measured output rate.
- Lidar packet, scan, and per-point timing fields.
- Cube message arrival timestamp.

Before SLAM selection, record at least five minutes and calculate:

- Mean sensor rate and jitter.
- Dropped-frame/packet percentage.
- Camera to IMU offset and variation.
- Lidar to IMU offset and variation.
- Worst scheduling pause on the Jetson under full compute load.

Software receive time is not an acceptable substitute for hardware timestamps
when the estimator requires motion deskew.

## Sensor Extrinsics

Use one body origin, preferably the measured aircraft CG.

For every sensor record:

- Translation in meters from body origin.
- Rotation from sensor frame to body frame.
- Measurement method and uncertainty.
- Calibration date and mounting revision.

Any physical remount invalidates that sensor's extrinsic verification flag.

Known current value:

- H-Flow focal point and range datum are approximately +0.10 m body Z, meaning
  10 cm below the CG in ArduPilot FRD coordinates.
- D415 center is approximately `(0.19, 0.00, 0.10) m` from CG in body FRD.
- JT16 center is `(0.00, 0.00, -0.10) m` from CG in body FRD.
- IM10A center is `(0.08, 0.00, -0.09) m` from CG in body FRD: directly over
  the Cube IMU and 1 cm above it.
- External IM10A dynamic gyro correlation measured the body-axis signs as
  `X/-Y/-Z` on 2026-07-29. This verifies the discrete axis rotation only, not
  the complete IMU extrinsic calibration.

Still required:

- H-Flow X/Y offsets if not directly below the CG.
- H-Flow connector direction for exact yaw.
- Camera, IMU, and lidar roll/pitch/yaw.

## Visualizer Spatial Contract

The browser receives two independent streams:

- `/api/stream`: latest Cube, H-Flow, range, and IM10A state at 60 Hz.
- `/api/spatial`: bounded D415 and JT16 frames only while the 3D view is open.

Spatial points are body-FRD centimetres encoded as little-endian signed
16-bit values plus per-point RGB. The browser associates each frame with the
nearest buffered Cube local pose, converts FRD into its Z-up Three.js scene,
and retains at most 220,000 points in a configurable rolling window.

This rolling display is not a SLAM map. Cube local position is rebased when the
browser starts, and the visualizer does not provide loop closure,
relocalization, covariance, estimator quality, or an external-navigation pose.
No spatial browser endpoint sends movement or avoidance commands.

## Sensor Timing Contract

Each estimator recording preserves both arrival and source timing in
`sensor_timing.ndjson`:

- Jetson acquisition uses `CLOCK_MONOTONIC`; UTC/realtime is metadata only.
- D415 depth and color retain device timestamp, timestamp domain, and frame
  number before alignment or point-cloud processing.
- JT16 retains the SDK callback monotonic time, frame index, and the minimum,
  maximum, and span of per-point timestamps.
- The confirmed factory IM10A stream has Jetson decode-arrival time only. The
  LIO profile adds the on-sensor `0x50` time frame and emits only time, raw
  acceleration, and raw angular velocity at 200 Hz.
- Cube timing rows retain `time_boot_ms` or `time_usec` when the MAVLink message
  supplies one.

Clock epochs are never assumed equal. Analysis compares each clock relative to
its first sample and keeps `sensor_time_sync_verified` false until a dynamic
alignment test passes.

The LIO bridge independently fits IM10A-sensor-to-monotonic and
JT16-point-to-monotonic affine models. JT16 scan headers use the mapped first
point timestamp; IMU headers use each mapped sensor timestamp. Missing or
unready sensor time blocks publication rather than falling back to receive
time.

## Proximity Contract

Obstacle scans use body FRD with 72 five-degree sectors. Zero degrees is
forward and positive angles point right.

- Publish `OBSTACLE_DISTANCE` at 10 Hz.
- Use `65535` for unknown sectors; never invent free space from missing depth.
- Drop a source after 0.45 seconds.
- Fuse the nearest valid distance per sector.
- Apply sensor rotation and translation before calculating horizontal range, so
  every published distance is referenced to the aircraft CG.
- Remove returns inside the verified `0.75 m` airframe envelope.
- Treat a known distance at or inside `1.50 m` as a hard clearance breach.
- Treat missing or stale sectors as unknown, not clear.
- Keep only the newest pending MAVLink scan.
- Do not enable output until source extrinsics and airframe geometry pass.
- Keep Cube `AVOID_ENABLE=0` during GCS and DataFlash validation.

The D415 and JT16 paths share this contract. See `OBSTACLE_AVOIDANCE.md`.
The JT16 SDK frame is converted explicitly as `forward=Y`, `right=X`,
`down=-Z` before applying the measured lidar-to-body rotation.

## SLAM Pose Contract

A pose is usable for navigation only when all conditions hold:

- Position, attitude, and velocity are finite.
- Timestamp age is under 0.20 s.
- Estimator reports initialized and tracking.
- Covariance is finite and below measured limits.
- No unreported reset or map-frame jump occurred.
- IMU and primary geometry source are fresh.

The estimator must expose:

```text
timestamp
map pose
odom pose
linear velocity
angular velocity
pose covariance
velocity covariance
quality 1..100
reset counter
tracking state and reason
```

The planner consumes `map` pose. The flight bridge consumes the smooth `odom`
pose unless a tested map correction strategy prevents jumps.

## ExternalNav Contract

MAVLink ODOMETRY is the preferred Cube input.

- Target rate: 20 to 30 Hz.
- Hard minimum: 4 Hz.
- Quality below the configured threshold is invalid, not merely low confidence.
- Increment `reset_counter` for pose, velocity, attitude, or map resets.
- Fill covariance from the estimator.
- Use a dedicated onboard component ID.

ExternalNav starts disabled. The Cube's source 3 mapping will be changed only
after shadow logs prove frame direction, scale, timing, reset handling, and
covariance.

## Navigation Command Contract

Version 1 uses local velocity targets:

- Regular GUIDED mode, not GUIDED_NOGPS.
- 10 Hz target rate.
- Commissioning horizontal limit: 0.30 m/s.
- Absolute configuration ceiling: 0.75 m/s.
- Vertical and yaw fields are ignored for local return; Cube holds altitude
  and heading.
- Command expires after 0.50 s inside the Jetson supervisor.
- ArduPilot `GUID_TIMEOUT=0.5` s is independently audited.

Every target passes:

1. Pilot authority check.
2. Flight mode check.
3. SLAM freshness and quality check.
4. Obstacle clearance and stopping-distance check.
5. Speed, acceleration, jerk, and yaw-rate limits.
6. Geofence/local-map bounds check.

The bridge must confirm the observed Cube state rather than assuming MAVLink
delivery. MAVLink packets are not guaranteed to arrive.

## Local Return Contract

Local return is a Jetson mission state, not ArduPilot RTL.

Required state:

- SLAM initialized before launch pose capture.
- Launch pose stored in the active map revision.
- Current pose localized in that same map.
- Path to launch pose collision-free.
- Cube local position controller healthy.
- H-Flow/range or tested ExternalNav available.

Return is cancelled if any required state becomes stale. Cancellation produces
a zero-velocity request and authority release; it never produces a blind direct
vector toward the remembered home coordinates.

## MAVLink Link Ownership

Exactly one process opens the configured Cube MAVLink endpoint. The active
OA-only boot service owns `/dev/ttyTHS1` directly; bench tools require that
service to be stopped first.

A later combined runtime may restore a MAVLink router and expose separate local
endpoints to:

- State observer.
- ODOMETRY bridge.
- Command bridge.
- Logger.
- QGC route.

Applications must not independently open the configured Cube endpoint.
