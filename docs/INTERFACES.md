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
- External IM10A dynamic gyro correlation measured the body-axis signs as
  `X/-Y/-Z` on 2026-07-29. This verifies the discrete axis rotation only, not
  the complete IMU extrinsic calibration.

Still required:

- H-Flow X/Y offsets if not directly below the CG.
- H-Flow connector direction for exact yaw.
- Camera, IMU, and lidar XYZ plus roll/pitch/yaw.

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
- Initial horizontal limit: 0.5 m/s.
- Initial vertical limit: 0.3 m/s.
- Initial yaw-rate limit: 20 deg/s.
- Command expires after 0.50 s inside the Jetson supervisor.
- ArduPilot `GUID_TIMEOUT` is independently verified before flight.

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

## UART Ownership

Exactly one process opens `/dev/ttyTHS1`.

During bench work, `scripts/preflight.py` may own it directly. During the future
runtime, a MAVLink router owns the UART and exposes local UDP endpoints to:

- State observer.
- ODOMETRY bridge.
- Command bridge.
- Logger.
- QGC route.

Applications must not independently open the UART.
