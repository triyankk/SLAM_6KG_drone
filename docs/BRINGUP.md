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
- IM10A center is directly over the Cube IMU and 1 cm higher, at
  `(0.08, 0.00, -0.09) m` from CG in body FRD.
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
2. Install the D415 rule with `sudo ./optflow install-realsense-rules`.
3. Reconnect the camera after installing the rule.
4. Run `./optflow sensor-check` and require sustained synchronized RGB-depth.
5. Start RGB capture with `./optflow camera --no-browser`.
6. Verify `/healthz` stays healthy and RGB rate is stable for ten minutes.
7. Verify the synchronized depth stream for ten minutes.
8. Measure valid depth range and blind zone.
9. Calibrate intrinsics.
10. Measure body extrinsics.

External IM10A:

1. Verify stable device identity and baud.
2. Decode raw gyro and acceleration.
3. The rollback-protected configurator has applied 230400 baud and 200 Hz with
   only relative sensor time, raw acceleration, and raw angular velocity.
4. The passing 2026-07-31 audit measured 199.84 Hz, 5.00 ms sensor intervals,
   and zero checksum, payload, ordering, or drop errors.
5. Repeat the read-only audit after IMU power cycles and before LIO sessions.
6. Calibrate bias, scale, axis direction, and noise.
7. Measure body extrinsics.

Hesai JT16:

1. Supply the JT16 with 12-16 V and connect its half-duplex RS485 adapter.
2. Build and install the adapter driver with `./optflow build-pl2303`, then
   `sudo ./optflow install-pl2303`.
3. Build the pinned official decoder with `./optflow build-jt16`.
4. Run `./optflow lidar-status`, then `./optflow sensor-check`.
5. Require decoded XYZ frames, not USB enumeration or header bytes alone.
6. Confirm whether the stream is 3,000,000 baud or legacy 3,125,000 baud.
7. Replace or validate the SDK's generic JT16 angle correction against this
   unit's factory correction.
8. Confirm ring, timestamp, frame rate, and packet-loss behavior under full
   Jetson load.
9. Measure lidar-to-body FRD translation and rotation.

Run:

```bash
./optflow preflight --profile slam_bench
```

Then make a passive timing baseline with the full visualizer stopped and its
lightweight telemetry path running:

```bash
./optflow visualizer --host 0.0.0.0 --no-browser --no-spatial
./optflow flight-log --name slam-timing-baseline --duration 30 \
  --no-realsense-bag
./optflow slam-timing data/recordings/flights/<session>
```

This 30-second run is only a plumbing check. Before estimator selection, repeat
for 30 minutes under representative Jetson load and with hand-carried motion.

Exit:

- Every sensor streams for 30 minutes without stale data.
- Rates and timestamps are documented.
- All extrinsics have a measured value.

## Phase 3A: Obstacle Proximity Commissioning

Props removed and RC7 low. The JT16-only Cube proximity path is active, but
flight response stays disabled through the RC switch during bench checks.

1. Stop the boot service, then verify the current profile with
   `./optflow cube-avoidance`; require `PRX1_TYPE=2`, `AVOID_ENABLE=7`, and
   `RC7_OPTION=40`, then restart the service.
2. Enter measured D415 and JT16 transforms in `config/system.yaml`.
3. Confirm `hard_cg_clearance_m: 1.50`; this distance is measured from the CG,
   after applying sensor extrinsics.
4. Follow `SENSOR_CALIBRATION.md`. Move a flat target around each sensor and
   verify body-FRD direction, CG distance, blind zones, sector coverage, and
   stale-data timeout.
5. Cover or disconnect each sensor in turn. Its sectors must disappear in less
   than 0.5 seconds; old distances must never replay.
6. Run the logger through stop/start and verify Cube IMU, flow, and range remain
   fresh.
7. Verify RC7 crosses the configured low/high thresholds, then run
   `./optflow cube-avoidance --apply` to back up and assign RC option 40.

Exit:

- D415 frontal and JT16 360-degree sectors match measured obstacles.
- Both mount transforms, JT16 baud/correction, and airframe geometry are marked
  verified.
- GCS proximity display is correct with `AVOID_ENABLE=0`.
- The avoidance enable switch has been tested with props removed.

The active Loiter/FlowHold sequence is in `OBSTACLE_AVOIDANCE.md`.

## Phase 4: Recorded SLAM Evaluation

No flight commands.

1. Build the pinned runtime with `./optflow build-lio`.
2. Apply the rollback-protected IM10A TIME + ACC + GYRO 200 Hz profile only
   while the Cube is disarmed.
3. Clear every blocker in `analysis/slam_timing.json`; do not infer
   synchronization from host arrival time alone.
4. Follow `LIO_SHADOW.md` and record the measured props-off carry loop with
   `./optflow lio-shadow --duration 90`.
5. Compare drift, clock resets, static stability, loop closure, and the
   required read-only Cube local-position agreement in
   `analysis/lio_validation.json`.
6. Repeat in both directions and approve only the exact digest of a passing
   report.
7. Add depth data to obstacle mapping; add visual estimation only if it improves
   measured performance.

Exit:

- Repeated trajectory returns close to its physical start.
- Estimator survives rapid yaw, low texture, and lighting changes.
- CPU/GPU temperature and timing remain within limits.
- Quality and covariance correlate with real failures.

## Phase 5: Live Shadow Mode

Jetson observes; Cube ignores Jetson odometry and receives no motion commands.

1. Run live SLAM on the assembled aircraft with pose output absent.
2. Log SLAM pose beside Cube H-Flow/GPS/EKF data.
3. Stop the boot OA-only service and run `./optflow slam-flight-shadow` as
   an intentional foreground evidence flight. Wait for its ready tune plus
   `SLAM TEST READY: ARM IN LOITER` in QGC, then follow the low GPS-Loiter
   out-and-back prompts inside the initial 1 m horizontal envelope.
4. Require `analysis/slam_flight_shadow.json` to pass its airborne LIO, VIO,
   H-Flow, range, D415/JT16 sector, and local-return proposal gates.
5. Repeat low, tethered ground-effect and GPS-supported shadow flights.
6. Require two passing trajectory reports with no clock reset, axis inversion,
   scale error, or unexplained jump.
7. Keep Cube ExternalNav fusion and active LIO pose output disabled until a
   report is explicitly approved by path and SHA-256 digest. The dedicated
   disarmed `cube-odom-shadow` transport proof may run only while its live
   source audit proves that Cube cannot fuse ExternalNav.

Exit:

- No axis or yaw inversion.
- No unreported map jumps.
- Shadow ROS odometry remains fresh at the target rate.
- SLAM drift and failure detection meet measured limits.

The reviewed Cube ODOMETRY transport bridge is now implemented and passed one
stationary disarmed run with ExternalNav disabled. In-air output, EKF source
selection, and real obstacle-avoidance commissioning remain blocked until this
LIO gate and every sensor geometry gate pass.

## Phase 6: Bounded Jetson Navigation

GPS remains connected and pilot takeover is immediate.

1. Prove the command bridge in SITL.
2. Prove Guided timeout behavior with props removed.
3. Send zero-velocity targets on the bench.
4. Perform a stationary hold command in flight.
5. Command a 0.5 m move at no more than 0.30 m/s.
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
