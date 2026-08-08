# JT16 + IM10A LIO Shadow Validation

## Safety Boundary

The first LIO runtime is observation-only:

- Hesai FAST-LIO2 revision
  `bb2842d34990761eebbd4cc3188e94c7c662a673` is pinned.
- The sensor bridge publishes ROS 2 IMU and point-cloud topics only.
- The normal `lio-shadow` Cube UART reader receives only. The proof-first
  `slam-poc` runner additionally sends one ready tune while disarmed.
- The separate `cube-odom-shadow` command can send MAVLink2 `ODOMETRY` only
  after a read-only audit proves all three EKF3 source sets exclude
  ExternalNav. It stops immediately on arm, heartbeat loss, stale LIO,
  unhealthy clocks, a pose jump, or an attitude jump.
- No shadow command sends a mode, arm, navigation target, or movement command.
- `pose_output_to_cube_enabled: false` and
  `external_nav_to_cube_enabled: false` remain required.
- `odometry_shadow_to_cube_enabled: true` permits only the explicit bench
  command; it does not start output at boot and does not enable EKF fusion.
- Active obstacle MAVLink output also requires an approved, digest-matched LIO
  trajectory report. It remains disabled during this sequence.

## Time Contract

IM10A output must be exactly TIME + raw acceleration + raw angular velocity at
200 Hz and 230400 baud. Its verified unit emits a zero-calendar, monotonic
time-of-day counter, so the bridge treats it only as a relative clock. JT16
retains every SDK per-point timestamp.

The bridge fits two independent robust affine clock models:

```text
Jetson CLOCK_MONOTONIC = slope * sensor_time + offset
```

The IM10A model uses its on-sensor time frame and serial arrival. The JT16 model
uses the final point time and native SDK callback time. Clock epochs are never
assumed equal. Frames stay buffered until both models satisfy sample-count,
span, drift, and residual gates. Receive time is never substituted for missing
sensor time.

The affine fits are also bounded by elapsed sensor time: 20 seconds for IM10A
and 60 seconds for JT16. The sample-count cap alone represented about 10
seconds at 200 Hz but roughly 400 seconds at the lidar's 5 Hz rate; the latter
mixed changing USB callback latency into one stale model and could stop
publication despite healthy sensors. The time-bounded windows retain the same
drift and residual gates while tracking gradual transport-clock change.

The boot recorder uses separate ROS callback groups for odometry/diagnostics,
IMU logging, and point-cloud processing. A multi-threaded executor prevents
the high-rate IMU and heavier lidar visualization work from delaying the pose
freshness path. FAST-LIO map publication follows `map_output_enabled`; the
normal boot configuration leaves it off while raw JT16/D415 spatial frames and
trajectory evidence continue to be recorded.

`clock_sync.time_offset_lidar_to_imu_s` feeds the pinned backend's fixed-offset
correction; FAST-LIO subtracts that value from IMU stamps. It remains zero
until slow multi-axis motion provides enough angular-rate excitation to
measure both its sign and magnitude. Affine residuals alone do not prove that
fixed transport latency is correct, so `sensor_time_sync_verified` remains a
separate active-mode gate.

## One-Time Runtime Build

```bash
./optflow build-lio
```

This creates an ignored, project-local `runtime/lio/` containing the official
ROS 2 Humble arm64 archive, minimal PCL, ROS PCL conversions, and the pinned
Hesai FAST-LIO2 binary. It requires no root install and uses no sibling ROS
workspace.

## IM10A Profile

First stop every process that owns `/dev/imu_usb`, including the automatic
flight logger. Keep the Cube connected and disarmed.

Read-only recovery scan:

```bash
./optflow im10a --duration 3
```

Apply the profile:

```bash
./optflow im10a --apply-lio-profile --duration 10
```

The command:

1. Finds the current baud and records the observed profile.
2. Writes a recovery file under `data/calibrations/im10a/`.
3. Verifies the Cube is disarmed.
4. Changes volatile output to TIME + ACC + GYRO, 200 Hz, 230400 baud.
5. Measures rate, sensor-time monotonicity, checksum errors, jitter, and drops.
6. Saves only after the new stream passes.
7. Automatically scans all supported bauds and restores the previous profile
   if any step fails.

The profile passed its final audit on 2026-07-31 and the three IM10A fields in
`config/system.yaml` now match the live unit. Re-run the read-only scan after a
power cycle before collecting an acceptance trajectory.

## IM10A Stationary Noise Profile

Place the disarmed, props-off aircraft on a rigid surface and do not touch it
until the Cube emits the completion beep:

```bash
./optflow imu-noise --duration 1800
```

The command temporarily stops and later restores the automatic flight logger,
confirms the live 200 Hz TIME+ACC+GYRO profile, records body-FRD acceleration
and angular velocity, rejects physical movement, and saves raw NPZ samples plus
a digest-checked report under `data/calibrations/im10a/noise/`. It calculates
Allan deviation, white-noise density, and observed bias instability. Candidate
FAST-LIO measurement covariance is report-only; the command never writes Cube
or estimator parameters, and it does not infer bias random walk from a single
30-minute capture.

The 2026-07-31 capture passed with 360,624 samples at 200 Hz, zero estimated
drops, and no checksum, payload, or ordering errors. Its report digest is
`d6133aa3e14b335acdbf4a1aec93d7f4314c68b8bf1e05e53ceee61acdb3f990`.
Accelerometer white-noise density was 0.000418-0.000478 m/s2/sqrt(Hz). The
stationary gyro stayed at one quantized output code on every axis, so its
analog noise and bias random walk remain unobservable and estimator covariance
must not be accepted from this capture alone.

## Dynamic IMU Alignment

Create a digest-checked timing and gyro-scale report from at least two motion
sessions without touching the aircraft:

```bash
./optflow imu-align data/recordings/lio/<session-a> \
  data/recordings/lio/<session-b>
```

Six 2026-07-31 sessions produced a +0.010 s lidar-to-IMU shadow candidate.
The independent Cube rate comparison gave +0.01675 s and agreed on sign. The
latest report is under
`data/calibrations/im10a/alignment/20260731T122028Z/`, with SHA-256
`1b7d63f141ff5c80927b924c6180bd92043e9d8a9127d8a0a131ad71c8694309`.
The command never applies its candidate.

A matched guided yaw run then compared the old zero-offset session
`20260731T101712Z_lio-shadow` with the +0.010 s session
`20260731T120956Z_lio-shadow`. Cube/LIO attitude p95 improved from 2.84 degrees
to 2.27 degrees, the final LIO return error was 0.012 m, and every timing,
sensor-health, continuity, and attitude gate passed. The +0.010 s value is
therefore retained for shadow LIO. The global `sensor_time_sync_verified` gate
remains false until D415 timestamps are dynamically aligned to the LIO clock.

The +0.010 s translation run `20260731T121719Z_lio-shadow` also passed every
tape-grounded check: forward scale 0.995, right scale 1.120, cross-axis error
0.015 m, vertical error 0.013 m, and center-return error 0.033 m. Its Cube/LIO
attitude p95 was 1.28 degrees. The generic Cube path-scale check remained bad
because disarmed Cube local position drifted; the tape marks are the declared
translation reference. This run also provided accepted x/y/z dynamic gyro
correlations, completing the IM10A body-axis and position extrinsic evidence.

## Test Sequence

### Cube ODOMETRY Transport Proof

Stop the boot OA-only service, remove props, keep Cube disarmed, and run:

```bash
./optflow cube-odom-shadow --duration 30
```

The command sends no packet until it reads `AHRS_EKF_TYPE=3`,
`EK3_ENABLE=1`, and all 15 `EK3_SRCn_*` values and confirms none equals
ExternalNav (`6`). It rebases FAST-LIO `camera_init -> body` into
`MAV_FRAME_LOCAL_FRD -> MAV_FRAME_BODY_FRD`, derives body velocity from fresh
poses, carries conservative estimator-backed pose errors, and records every
packet under `data/recordings/lio/<session>/`.

The first passing bench session is
`20260731T144647Z_cube-odom-shadow`: 112 packets at 4.999 Hz, maximum packet
gap 0.269 s, healthy sensor clocks, no arm event, no parameter write, and no
Cube source configured for ExternalNav. This proves transport only. The Cube
did not fuse the stream, and it does not approve in-flight ExternalNav.

### Fast Proof First

Calibration approval is not required to prove that the live architecture can
track and map. With props removed and Cube disarmed, run:

```bash
./optflow slam-poc --no-browser
```

Open `http://<jetson-ip>:8767/slam-poc`. Wait for the one-shot ready beep,
then follow the live still, horizontal move, hold, return, and final-hold cues.
The screen flashes twice before every new instruction. Stop and save from the
dashboard after the sequence completes. The proof passes only when D415 RGB-D
tracking, timestamp-matched IM10A gyro priors, FAST-LIO output, both maps, real
motion, and gross trajectory agreement are present. It automatically yields
the sensors from the flight logger and restores that service on exit.

The same run records `analysis/rtl_shadow_commands.ndjson` and
`analysis/rtl_shadow_live.json`. These contain launch-relative breadcrumb
targets and bounded local velocity proposals, but no MAVLink output. Use
`./optflow rtl-shadow <session>` to regenerate the shadow evidence offline.

The output is deliberately marked as a proof of local VIO/LIO mapping. It has
no loop closure, relocalization, planner, or Cube pose output and cannot be
approved for flight-control use.

The boot OA-only service and LIO runner cannot share JT16 or the Cube UART.
Stop that service for every manual shadow session and restart it afterward.

### First Airborne Shadow

The first airborne shadow is now an intentional foreground evidence run. Stop
the boot OA-only service, then start it with:

```bash
./optflow slam-flight-shadow
```

The only outbound MAVLink messages are a temporary stream-rate request and the
one-shot ready tune. Pose, proximity, velocity, mode, arm, and parameter output
are absent. GPS remains connected and the pilot uses tested Loiter.

1. Start disarmed in a clear area with textured ground and RC7 low.
2. Wait for the ready tune and `SLAM TEST READY: ARM IN LOITER` in QGC.
3. Arm in Loiter and take off manually. QGC asks for a continuous 10-second
   hold between 1 m and 3 m after fresh RC input is confirmed.
4. Follow the QGC prompts: move forward 0.5 m slowly, hold for 5 seconds,
   return to the marked launch XY, and hold for 10 seconds.
5. Land manually in Loiter when QGC reports `SLAM TEST COMPLETE`, then disarm.
   Never select STABILIZE.

The run stops after a three-second post-disarm tail. It records D415 and JT16
CG-referenced sectors but sends neither to Cube, so there is no automatic
obstacle response on this flight. Keep at least 5 m of clear space, then
restart `optflow-flight-logger.service` after the report is finalized.

The decisive report is
`data/recordings/slam_flights/<session>/analysis/slam_flight_shadow.json`.
It requires an arm/disarm cycle, at least 20 seconds airborne, at least 4 Hz LIO,
bounded LIO jumps, at least 70 percent RGB-D tracking, both obstacle sources,
fresh primary JT16 sectors, airborne H-Flow and downward range, a passing
out-and-back local-return shadow, and zero flight-control output.

1. Props removed, Cube disarmed, aircraft untouched for at least 15 seconds.
2. Roll, pitch, and yaw one axis at a time, keeping motion under 30 deg/s.
3. Move it 0.5 m forward and back twice, then 0.5 m sideways and back.
4. Return it to the exact marked start pose.
5. Leave it untouched for at least 15 seconds.
6. Record for at least 60 seconds total:

```bash
./optflow lio-shadow --duration 90
```

The command creates `data/recordings/lio/<session>/` and automatically writes
`analysis/lio_validation.json` plus its SHA-256 digest.

For the measured translation-only scale check, mark the start, 0.50 m forward,
and 0.50 m right positions on the floor, then run:

```bash
./optflow lio-shadow --visual-assist --visual-guide translation \
  --visual-host 0.0.0.0 --no-browser
```

The final three seconds of every phase are median-filtered into
`guide_result.json`. Its hash is pinned in the session manifest. The guided
scale check uses those physical tape marks; disarmed Cube local position stays
in the report as a diagnostic and is not treated as the tape-test ground truth.

Re-run analysis without touching hardware:

```bash
./optflow lio-validate data/recordings/lio/<session>
```

## Pass Criteria

A report passes only when all configured checks pass:

- No pose was sent to Cube.
- At least 60 seconds and at least 4 Hz LIO odometry.
- Finite values, monotonic stamps, position jumps at most 0.50 m, speed at
  most 3.0 m/s, and attitude jumps at most 10 degrees.
- Start/end static drift no more than 0.15 m.
- Return-to-start error no more than 0.35 m.
- Both sensor clock models ready with zero resets.
- Healthy sensor bridges with no IM10A checksum errors, non-monotonic JT16
  frames, or malformed JT16 frame timing after synchronization.
- Direct timestamp traces, not only bridge counters, show IM10A at least
  90 percent of its required 200 Hz and JT16 at least 4 Hz with the expected
  point layout.
- A guided translation run captures every tape-marked phase, measures each
  0.50 m axis at 0.70-1.30 scale, stays within 0.15 m cross-axis and vertical
  error, and returns within 0.15 m of both center marks.
- At least 100 paired Cube `LOCAL_POSITION_NED` samples over 2.0 m of motion;
  after yaw alignment, horizontal RMSE must be at most 0.50 m, vertical RMSE
  at most 0.40 m, and the path-length ratio must remain from 0.70 to 1.30.
- Cube attitude and LIO attitude have at least 100 paired samples and at most
  10 degrees p95 disagreement after initial world-frame alignment.
- Measured IMU/lidar extrinsics, IMU position, IMU noise profile, and
  unit-specific JT16 correction are marked verified.

Cube local position is an independent acceptance reference, not estimator
input and not absolute ground truth. A stationary-only run cannot pass this
gate.

Repeat the hand-carried loop in both directions before a tethered hover. Then
run two low, slow shadow flights with GPS connected and review both reports.
Only a passing report whose exact digest is explicitly approved in
`config/system.yaml` can satisfy the later Cube-pose or active-avoidance gates.

## Current Blockers

These acceptance gates remain deliberately false:

- IM10A gyro bias/noise covariance; stationary accelerometer noise and digital
  quantization are measured.
- JT16 full body extrinsic.
- D415-to-LIO time-sync verification; the IM10A/JT16 +0.010 s offset passed its
  shadow yaw A/B check.

Therefore a trajectory collected before those measurements is diagnostic and
must fail approval.
