# Architecture

## Objective

Fly without GPS using two independent local estimation layers:

1. The Cube uses H-Flow, downward range, and its internal sensors for
   stabilization and short-term local XY hold.
2. The Jetson uses the Hesai JT16, external IM10A, and forward depth camera for
   GPS-independent SLAM, obstacle mapping, path planning, and local return.

The system is deliberately asymmetric. Jetson navigation may fail without
taking away the Cube's basic ability to hold the aircraft.

## Flight Controller Layer

The Cube owns:

- Rate, attitude, altitude, and position controllers.
- Motor mixing and actuator output.
- Internal IMUs, barometer, compass, H-Flow, and H-Flow rangefinder.
- Pilot mode selection and RC failsafes.
- The final acceptance or rejection of MAVLink commands.

H-Flow provides angular ground motion, not global position. The rangefinder
provides scale and height-above-ground information. The EKF integrates these
signals into a local estimate. Position quality remains conditional on surface
texture, light, height, motion blur, vibration, and calibration.

Current Cube source set 1 uses no horizontal position source and optical flow
for horizontal velocity, with barometer vertical position and compass yaw.
ExternalNav remains disabled. GPS may stay connected to establish a valid EKF
origin during commissioning, but it is not the local return trajectory source.

## Jetson Estimation Layer

The first intended estimator is lidar-inertial:

- Hesai JT16 supplies 3D geometry.
- IM10A supplies angular rate and acceleration.
- Depth camera supplies forward RGB-D geometry and may later supply visual loop
  closure or visual odometry.

Do not fuse all sensor outputs simply because they exist. One estimator must
own the state, timestamps, covariance, reset counter, and frame tree.

The selected shadow backend is Hesai's JT16-specific ROS 2 FAST-LIO2 revision
`bb2842d34990761eebbd4cc3188e94c7c662a673`. The bridge publishes the exact
`x/y/z/intensity/ring/timestamp` PointCloud2 fields registered by that backend
and a raw SI-unit IMU stream. The official Hesai SDK remains the single JT16
decoder.

ROS 2 Humble, minimal PCL, and the estimator are pinned under `runtime/lio`.
Two robust affine models map IM10A sensor time and JT16 point time onto Jetson
monotonic time. Neither stream is published until both models pass residual,
drift, span, and reset gates.

FAST-LIO2 remains shadow-only. It records `/Odometry`, clock diagnostics, and a
read-only Cube reference. The project contains no LIO-to-Cube pose sender at
this stage.

## Mapping Layer

Version 1 mapping products:

- `map`: loop-corrected local mission frame.
- `odom`: smooth local estimator frame.
- `base_link`: aircraft body in FLU convention inside the SLAM stack.
- A 2.5D occupancy/cost grid for fixed-altitude planning.
- A short-horizon obstacle representation for immediate stopping.

The 3D lidar is the main geometry source. The forward depth camera fills
near-field detail and gives a fast collision stop in its field of view. The
project's hard navigation constraint is at least `1.50 m` horizontal distance
between the aircraft CG and occupied space. The sensor mounting translations
are applied before this distance is calculated. Braking begins outside that
boundary, and unknown space remains blocked.

## Planning Layer

Implemented version 1 return:

- Fixed operating altitude controlled by the Cube.
- Reverse traversal of the known outbound breadcrumb path.
- Candidate movement is blocked if occupied space breaches `1.50 m` from CG.
- Horizontal speed starts at `0.30 m/s` and has an absolute `0.75 m/s` code
  ceiling; the Jetson sends no vertical or yaw target.
- Stop immediately when the path becomes unsafe. Obstacle flanking and A*
  replanning remain later work.

Dijkstra explores the whole reachable graph. A* uses a heuristic toward the
goal and is the better default for a known goal. Both require a valid costmap;
neither consumes a raw depth frame directly.

Later:

- 3D voxel map.
- Collision-checked 3D trajectory planner.
- Dynamic obstacle tracking.
- Higher limits only after logged stopping-distance measurements.

## MAVLink Boundary

One process owns the configured Cube MAVLink endpoint. Other Jetson processes use local routed
endpoints once the production runtime is installed.

Initial command path:

1. Jetson reads Cube state.
2. Planner emits a local path.
3. Safety supervisor validates SLAM health, command age, obstacle clearance,
   speed, acceleration, and pilot authority.
4. Bridge sends local NED velocity targets to regular GUIDED mode at 10 Hz.
5. Cube closes all flight-control loops.

Not allowed in version 1:

- Direct motor output.
- Raw attitude target control.
- GUIDED_NOGPS.
- Automatic arming.
- Automatic EKF source switching.
- Normal RTL as a substitute for local SLAM return.

## Optional ExternalNav

ExternalNav is a later reliability upgrade, not the first control method.

Sequence:

1. Produce fused SLAM odometry with covariance and health.
2. Record it without transmitting to the Cube.
3. Transmit MAVLink ODOMETRY in shadow mode while Cube ignores it.
4. Compare SLAM pose, H-Flow estimate, attitude, and known ground truth.
5. Configure EKF source 3 for ExternalNav.
6. Test explicit source switching with GPS still available.
7. Only then consider using ExternalNav for long-term position correction.

Step 3 now has a disarmed transport proof. `cube-odom-shadow` audits every
EKF3 source set before sending, refuses any ExternalNav-enabled profile, and
keeps active pose and navigation enables false. The 2026-07-31 run delivered
112 packets at 4.999 Hz. It is not an EKF-fusion or flight approval.

The ODOMETRY stream must include:

- Pose and velocity in a documented frame.
- Covariance based on estimator output, not fixed optimistic numbers.
- Quality from 1 to 100, or failure when invalid.
- A reset counter incremented whenever the estimator jumps or resets.
- At least 4 Hz; target 20 to 30 Hz.

Raw IMU samples are never sent as position corrections.

## GPS-Denied Local Return

The guarded implementation currently performs these steps:

1. Store the launch pose in `map`.
2. Store the traversed path and map updates.
3. On an RC9 low-to-high request in regular Guided, reverse the recorded
   breadcrumbs toward the launch pose.
4. Follow the path using bounded horizontal velocity targets.
5. Stop above the launch region.
6. Ask the pilot to take control and land; Jetson never descends or disarms.

If localization is lost:

- Do not integrate IMU blindly toward home.
- Stop Jetson motion commands.
- Let the Cube hold using optical flow if that estimate remains healthy.
- Notify QGC and the pilot.
- Pilot takes over or commands a controlled landing.

## Failure Contract

| Failure | Jetson response | Cube/pilot response |
| --- | --- | --- |
| Depth camera stale | Remove camera obstacles; reduce capability | Continue only if lidar coverage is valid |
| Lidar stale | Stop autonomous translation | Optical-flow hold or pilot takeover |
| IMU stale | Mark SLAM invalid immediately | Optical-flow hold or pilot takeover |
| SLAM covariance/quality bad | Stop targets; do not send valid odometry | Hold, land, or manual takeover |
| MAVLink link lost | No more Jetson commands | Guided timeout must stop motion; pilot selects tested hold mode |
| Jetson reboot | Autonomous control remains disabled after boot | Pilot retains authority |
| H-Flow unhealthy | Jetson must not assume Cube can hold | Pilot lands using a tested non-STABILIZE path |
| Map/relocalization lost during return | Cancel return and stop | Hold, pilot takeover, or land |

## References

- ArduPilot non-GPS estimation:
  https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
- ArduPilot Guided mode:
  https://ardupilot.org/copter/docs/ac2_guidedmode.html
- Hesai ROS 2 driver:
  https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0
- FAST-LIO:
  https://github.com/hku-mars/FAST_LIO
- LIO-SAM:
  https://github.com/TixiaoShan/LIO-SAM
