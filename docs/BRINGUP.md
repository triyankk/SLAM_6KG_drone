# Bring-Up Guide

Do not skip phases. A later phase starts only when the previous exit criteria
are recorded and repeatable.

## Phase 0: Mechanical and Power Acceptance

Props removed unless a test explicitly requires thrust.

1. Confirm CG with the 22 Ah battery installed in flight position.
2. Confirm H-Flow lens points down.
3. Report H-Flow connector direction and X/Y offset from CG.
4. Secure every Jetson header wire and CAN cable with strain relief.
5. Confirm each 40 A ESC has airflow, correct wire gauge, and secure power
   joints.
6. Confirm motor order, direction, propeller orientation, and output protocol.
7. Calibrate Cube accelerometers on the final assembled airframe.
8. Verify voltage and current monitoring against a trusted wattmeter.
9. Configure the requested RC disarm switch only after the operator toggles the
   intended transmitter channel during an interactive assignment.

Measured Cube mounting revision:

- Cube center is `+0.08 m` forward and `-0.08 m` down (8 cm above) from CG.
- Cube arrow points left, which is ArduPilot `Yaw270`.
- Run `./optflow cube-mount` to inspect and
  `./optflow cube-mount --apply` to back up, write, and verify the parameters.
- Re-run accelerometer calibration after applying the mounting revision.

Exit:

- No loose wiring or unknown motor direction.
- Current/voltage error is within the chosen tolerance.
- ESC and battery acceptance in `POWER_AND_ESC.md` passes.
- RC mode takeover is proven.

## Phase 1: Cube and H-Flow Bench

Props removed, Cube disarmed.

```bash
./optflow preflight --profile fc_bench
```

1. Place the vehicle over a textured, well-lit floor.
2. Translate it forward/back and left/right without large rotation.
3. Verify flow quality remains healthy and X/Y rates respond.
4. Roll and pitch the frame about the H-Flow focal point.
5. Record a Cube DataFlash log with `OF`, `RFND`, `IMU`, and EKF data.
6. Check H-Flow body rates against Cube gyro rates.
7. Verify range against measured floor distance at several heights.

Exit:

- No sign reversal.
- Scale and orientation calibration pass.
- Range error is measured and acceptable.
- H-Flow remains healthy for at least ten continuous minutes.

## Phase 2: Flight Controller Only

Jetson navigation disabled. GPS remains connected for recovery and comparison.
Never use STABILIZE.

1. Validate basic hover and attitude response in a tested altitude-controlled
   mode with conservative lean and speed limits.
2. Validate GPS Loiter and review vibration, motor output, battery sag, and ESC
   temperature.
3. Keep H-Flow logging in shadow while GPS is the active source.
4. Configure an RC switch for explicit EKF source selection.
5. At low altitude in a large clear area, switch from GPS source 1 to H-Flow
   source 2.
6. Hold for a few seconds, return to GPS source 1, and land.
7. Extend duration only after each log passes.

Abort:

- Unexpected lean or acceleration.
- Flow quality collapse.
- Range dropout.
- EKF innovation growth.
- Motor saturation, thrust-loss warning, excess vibration, voltage sag, or hot
  ESC.

Exit:

- Repeatable low-altitude optical-flow hold.
- Manual source return works.
- No unstable correction loop.
- At least three clean flights before physically removing GPS.

## Phase 3: Individual Jetson Sensors

Connect one new sensor at a time.

Depth camera:

1. Record model and serial number.
2. Start RGB capture with `./optflow camera --no-browser`.
3. Verify `/healthz` stays healthy and RGB rate is stable for ten minutes.
4. Add and verify the depth stream for ten minutes.
5. Measure valid depth range and blind zone.
6. Calibrate intrinsics.
7. Measure body extrinsics.

External IM10A:

1. Verify stable device identity and baud.
2. Decode raw gyro and acceleration.
3. Keep the confirmed factory setting at 9600 baud and 10 Hz until the
   reversible configurator and automatic baud recovery are tested.
4. Target 230400 baud and 200 Hz with only sensor time, raw acceleration, and
   raw angular velocity enabled.
5. Measure actual sample rate, jitter, checksums, and dropped packets.
6. Calibrate bias, scale, axis direction, and noise.
7. Measure body extrinsics.

Hesai JT16:

1. Verify whether it is using Ethernet UDP or its JT16 serial path.
2. Verify the configured IP/port or serial baud from the device.
3. Use the official Hesai driver to decode point clouds.
4. Confirm ring and per-point timestamp fields.
5. Measure packet loss under full Jetson load.
6. Measure body extrinsics.

Run:

```bash
./optflow preflight --profile slam_bench
```

Exit:

- Every sensor streams for 30 minutes without stale data.
- Rates and timestamps are documented.
- All extrinsics have a measured value.

## Phase 4: Recorded SLAM Evaluation

No flight commands.

1. Install the selected ROS 2 runtime only after sensor drivers are proven.
2. Record synchronized camera, IMU, and lidar data while moving the sensor rig
   by hand.
3. Evaluate lidar-inertial estimators using the same dataset.
4. Compare drift, CPU/GPU load, relocalization, loop closure, and reset behavior.
5. Select one primary estimator.
6. Add depth data to obstacle mapping; add visual estimation only if it improves
   measured performance.

Exit:

- Repeated trajectory returns close to its physical start.
- Estimator survives rapid yaw, low texture, and lighting changes.
- CPU/GPU temperature and timing remain within limits.
- Quality and covariance correlate with real failures.

## Phase 5: Live Shadow Mode

Jetson observes; Cube ignores Jetson odometry and receives no motion commands.

1. Run live SLAM on the assembled aircraft.
2. Log SLAM pose beside Cube H-Flow/GPS/EKF data.
3. Start MAVLink ODOMETRY transmission with Cube ExternalNav still disabled.
4. Verify NED/ENU conversion, attitude, scale, timestamp age, covariance, and
   reset counter.
5. Repeat ground movement and GPS-supported flights.

Exit:

- No axis or yaw inversion.
- No unreported map jumps.
- ODOMETRY remains fresh at the target rate.
- SLAM drift and failure detection meet measured limits.

## Phase 6: Bounded Jetson Navigation

GPS remains connected and pilot takeover is immediate.

1. Prove the command bridge in SITL.
2. Prove Guided timeout behavior with props removed.
3. Send zero-velocity targets on the bench.
4. Perform a stationary hold command in flight.
5. Command a 0.5 m move at no more than 0.5 m/s.
6. Add one waypoint at fixed altitude.
7. Add A* path following around static obstacles.

ExternalNav remains optional. The first navigation tests can use the Cube's
H-Flow local estimate while Jetson sends bounded targets.

Exit:

- Removing Jetson commands causes a controlled stop.
- Pilot takeover always wins.
- Obstacle stop distance exceeds measured braking distance.
- No mode transition to STABILIZE or GUIDED_NOGPS exists.

## Phase 7: Local Return

Do not call normal RTL.

1. Capture launch pose only after SLAM is healthy.
2. Fly a short mapped route.
3. Request local return while GPS remains available for monitoring.
4. Plan back through known free space.
5. Stop above the launch pose.
6. Initially let the pilot land.
7. Add autonomous descent only after downward landing safety is proven.

Exit:

- Return succeeds repeatedly from multiple directions.
- Map loss cancels return and stops translation.
- Jetson reboot cannot automatically resume movement.

## Phase 8: True GPS-Denied Operation

Only after all earlier phases pass:

1. Set and verify EKF origin before takeoff.
2. Confirm H-Flow source and local position are healthy.
3. Confirm SLAM and local home are healthy.
4. Disable or disconnect GPS.
5. Repeat the smallest validated mission first.
6. Review every log before increasing distance, speed, altitude, or complexity.
