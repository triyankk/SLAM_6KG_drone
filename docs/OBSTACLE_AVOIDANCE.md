# Obstacle Avoidance Bring-Up

This layer is separate from SLAM navigation. It supplies short-range obstacle
distances to ArduPilot while the Cube still owns attitude, altitude, and
position control.

## Current State

- D415 RGB-depth is live for mapping diagnostics but excluded from avoidance.
- The official Hesai SDK bridge and PL2303 adapter are live at 3,000,000 baud.
- Cube `PRX1_TYPE=2`, `AVOID_ENABLE=7`, and `RC7_OPTION=40` are applied and
  verified after reboot.
- Project `mavlink_output_enabled: true` with JT16 as the only control source.
- JT16 translation, cardinal rotation, unit correction, baud, and airframe
  geometry are verified. D415 body extrinsics remain unverified.
- LIO validation remains unapproved, but it is independent of Cube proximity
  output and does not gate manual RC7 avoidance.

The boot service is currently obstacle-avoidance-only. It owns the single Cube
link, sends latest-only JT16 proximity, and contains the RC7/native-avoidance
and audible warning path. D415, IM10A, FAST-LIO, SLAM, trajectory, and
companion movement output are not started. RC7 must remain low until a
props-off direction check is complete in the current assembly.

The hard boundary is `1.50 m` horizontal distance from the aircraft CG. Sensor
points are rotated and translated into body FRD at the CG before distance is
calculated. A known sector at or inside `1.50 m` is a clearance breach. A fresh
full JT16 frame with no horizontal return in a face is encoded as no obstacle
through the configured `8 m` range; a stale or missing JT16 frame stops output
and remains unknown. Returns inside the verified `0.75 m` airframe envelope are
removed as self-returns before this check.

## Data Path

```text
D415 aligned depth -> mapping and visual diagnostics only

JT16 serial -> official Hesai decoder -> body FRD sectors
                                            |
                                            +-> eight 45-degree DISTANCE_SENSOR
                                                faces, paced over each cycle
```

Face zero is forward and face numbers increase clockwise in 45-degree steps.
Each face uses the nearest known 5-degree sector, so the reduction is
conservative. The source expires after 0.45 seconds. The Cube's MAVLink
proximity backend expires after 0.5 seconds, so a dead source cannot keep an old
obstacle alive. The UART queue holds only the newest scan and spaces packets by
12 ms instead of transmitting a burst. A healthy face with no return uses
`max_distance + 1`, which ArduPilot treats as no in-range obstacle while still
refreshing the proximity watchdog.

The JT16 SDK frame uses `+Y` at zero azimuth, `+X` right, and `+Z` up. The
bridge converts it to FRD as `(Y, X, -Z)` before applying mount extrinsics.

The D415 covers only its forward field of view. The JT16 is the intended
360-degree source. Neither one is a complete vertical collision-avoidance
system.

## RC Toggle and Audio

RC7 is the project obstacle-avoidance switch. Its Cube auxiliary option is
`40`:

- `RC7 >= 1700`: avoidance and armed obstacle audio enabled.
- `RC7 <= 1300`: avoidance and obstacle audio disabled.
- An unknown RC value fails disabled.
- RC7 does not stop the proximity stream; ArduPilot owns the actual avoidance
  gate.

Audio uses the nearest fresh CG-referenced sector:

- `1.50 m < distance <= 2.00 m`: one short beep per second, warning only.
- `1.25 m <= distance <= 1.50 m`: three short beeps per second.
- `distance < 1.25 m`: rate rises linearly from `3 Hz` to `10 Hz` at the
  verified `0.75 m` airframe envelope.
- Disarmed, stale, unknown, or RC7-low states are silent.

The boot service emits one rising tune after its first successful Cube
heartbeat. It does not add companion arming or flight-mode tunes.

Manage and inspect the active runtime with:

```bash
systemctl --user restart optflow-flight-logger.service
./optflow obstacle-status
journalctl --user -u optflow-flight-logger.service -n 100 --no-pager
```

The 1.50 m value is a target boundary, not a physical guarantee. In Loiter,
ArduPilot can begin braking early enough to stop at `AVOID_MARGIN=1.50`. In
FlowHold, `AVOID_DIST_MAX=1.50` means the non-GPS lean-away response begins at
the boundary, so approach speed must remain low and some boundary overshoot is
possible.

Inspect the prepared Cube profile while disarmed:

```bash
./optflow cube-avoidance
```

Back up, apply, and verify it:

```bash
./optflow cube-avoidance --apply
```

The command always writes `AVOID_ENABLE=0` first. It only restores active
`PRX1_TYPE=2` and `AVOID_ENABLE=7` when `stage: active`,
`mavlink_output_enabled: true`, and every enabled sensor's geometry/correction
gate loads successfully. Changing `PRX1_TYPE` requires an autopilot reboot.

## Required Measurements

The active JT16 path requires these values on the final assembled aircraft:

1. JT16 center from CG and its complete axis rotation.
2. Airframe radius including propeller clearance.
3. JT16 live baud and unit-specific angle correction.

Before D415 can join avoidance, also verify its center and complete body
rotation plus outdoor minimum range and invalid-depth behavior.

Enter the values in `config/system.yaml`, repeat the target-walk test, then mark
only the measurements that actually passed as verified.

Recorded 2026-07-30:

- JT16 center: `(0.00, 0.00, -0.10) m` in body FRD.
- D415 center: approximately `(0.19, 0.00, 0.10) m` in body FRD.
- Opposite motor centers: `0.85 m`.
- Propeller diameter: `18 in` (`0.4572 m`).
- Physical outer radius: `0.425 + 0.2286 = 0.6536 m`.
- Configured protected radius: `0.75 m`, leaving `0.0964 m` clearance.

The JT16 cardinal rotation passed. The camera rotation must still pass the
props-off target-direction test before its complete extrinsic is marked
verified.

The measured-target commands and vendor calibration sequence are in
`SENSOR_CALIBRATION.md`.

## Props-Off Gate

1. Remove propellers, keep RC7 low, and run `./optflow sensor-check`.
2. Start the boot service and verify fresh JT16 proximity in the GCS.
3. Move a measured flat target through front, right, rear, and left; verify both
   sector and CG-referenced distance.
4. In a clear test area, require no persistent false sector inside 1.50 m.
5. Disconnect JT16 and verify proximity becomes stale instead of replaying the
   last scan.
6. Reconnect JT16 and verify flow, range, Cube IMU, and proximity all recover.
7. Toggle RC7 while disarmed and verify PWM crosses `1300`/`1700`.
8. Run `./optflow cube-avoidance` and require exact readback of `RC7_OPTION=40`,
   `PRX1_TYPE=2`, and `AVOID_ENABLE=7` before reinstalling propellers.

## Active Field Sequence

GPS stays connected. Use a large, clear area, low altitude, textured ground,
one lightweight target, and a second person at the aircraft. Never select
STABILIZE.

1. Start with avoidance switched off and verify normal Loiter control.
2. Hover stationary, confirm proximity is fresh, then switch avoidance on.
3. Approach the target no faster than 0.3 m/s.
4. Verify the aircraft stops with margin and still accepts retreat and yaw.
5. Switch avoidance off, confirm normal control, and land.
6. Review the Cube and Jetson logs before repeating from left, right, and rear.
7. Establish a stable FlowHold hover with avoidance off.
8. Repeat the same stationary-target sequence in FlowHold.
9. Perform at least three clean tests in each mode before increasing speed.

Abort immediately for a wrong sector, stale proximity, unexpected lean,
oscillation, loss of pilot authority, estimator warning, or motor saturation.
Switch avoidance off first. Stay in the currently stable altitude-controlled
mode or use tested GPS Loiter; land without selecting STABILIZE.

Only after the synchronized LIO shadow sequence and measured braking tests
should `AVOID_BEHAVE`, margin, backup speed,
and acceleration be chosen for this heavy airframe. GPS removal and autonomous
SLAM navigation remain later phases.

ArduPilot references:

- [Simple object avoidance modes and parameters](https://ardupilot.org/copter/docs/common-simple-object-avoidance.html)
- [MAVLink depth-camera proximity setup](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html)
- [ArduCopter 4.6.3 FlowHold avoidance call](https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/ArduCopter/mode_flowhold.cpp)
