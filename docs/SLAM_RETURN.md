# Guarded SLAM Return

## What It Is

`SLAM RETURN` is a local breadcrumb return, not ArduPilot's global RTL mode.
FAST-LIO2 records the outbound path in a launch-fixed frame. On an explicit
RC9 request, the planner walks those breadcrumbs in reverse and sends bounded
horizontal velocity targets to Copter in regular Guided mode. The Cube keeps
control of attitude, height, H-Flow fusion, and motors.

The first-flight limit is 0.30 m/s and the hard software ceiling is 0.75 m/s.
Vertical velocity and yaw targets are always zero/ignored. Arrival only holds a
zero horizontal target and asks the pilot to take control and land; Jetson does
not land or disarm the aircraft.

## Output Gates

Every gate is checked continuously:

- Cube armed, heartbeat fresh, and mode exactly `GUIDED`.
- RC9 fresh, observed low after arm, then deliberately switched high.
- FAST-LIO pose and synchronized sensor diagnostics fresh.
- RGB-D tracking no older than 2.0 s and consistent with LIO displacement.
- H-Flow quality at least 50 and downward range between 1 m and 8 m.
- Cube local position and EKF origin available.
- Battery telemetry fresh and voltage at least 22.2 V.
- D415/JT16 fused obstacle scan fresh with no object inside 1.50 m from CG.
- Cube parameter audit complete, including `GUID_TIMEOUT <= 1.0 s`.
- Configuration and digest-bound flight approval permit live output.

RC9 low, any failed gate, mode change, stale command, estimator disagreement,
or clearance breach latches an abort for that arm cycle. If a live command had
already been sent, the bridge streams zero velocity for one second; Copter's
0.5 second Guided timeout is the independent backstop.

## Current Lock

The implementation is complete but live movement is intentionally locked in
`config/system.yaml`:

```yaml
navigation:
  autonomous_control_enabled: false
  slam_return:
    stage: locked
    live_control_enabled: false
```

The locked runtime still runs the estimator, records breadcrumbs, evaluates
every gate, writes the trajectory viewer feed, and records the exact command it
would have sent. It cannot transmit a velocity target.

Live output cannot be enabled until the config's LIO validation and physical
calibration approvals are backed by actual flight evidence. The final config
must then be hashed into `runtime/approvals/slam_return_live.json` together with
the approved flight report digest. Do not create this marker from assumed data.

## Boot And Status

The user service starts the locked runtime automatically:

```bash
systemctl --user restart optflow-flight-logger.service
systemctl --user status optflow-flight-logger.service --no-pager --full
./optflow slam-return-status
./optflow prearm-status
```

`prearm-status` reports recent Cube `PreArm:`/`Arm:` `STATUSTEXT` captured by
the service and never opens a competing MAVLink connection. It also prints the
configured endpoint, which is `/dev/ttyTHS1` at 460800 baud.

To inspect the receive-only TELEM2 path directly during commissioning, stop the
service so there is still only one Cube reader, sample for a complete pre-arm
report interval, and restart it:

```bash
systemctl --user stop optflow-flight-logger.service
./optflow prearm-status --live-uart --duration 35
systemctl --user restart optflow-flight-logger.service
```

Heartbeat alone is not a clean result. The command fails closed when no
`STATUSTEXT` arrives and includes all warning/error-severity messages so faults
such as `PRX1: No Data` are not hidden by a missing `PreArm:` prefix.

Run the same runtime in the foreground only when the service is stopped:

```bash
./optflow slam-return
```

Audit Cube parameters without writing them:

```bash
./optflow slam-return-setup
```

Supported setup writes are disarmed-only. Apply the configured `RC9` return and
`RC10` LAND mapping with read-back verification:

```bash
./optflow slam-return-setup --apply-channel-map
```

The Guided timeout has its own explicit write:

```bash
./optflow slam-return-setup --apply-guided-timeout
```

For the 57,600-baud TELEM1/QGC radio, apply the conservative stream profile
that prevents the flight-log saturation seen on 2026-08-03:

```bash
./optflow slam-return-setup --apply-telem1-profile
```

QGC may request different volatile stream rates after connecting. Keep its
telemetry-rate settings at or below this profile and verify the next DataFlash
log does not accumulate MAVLink channel-1 buffer-full events.

## Next Flight: 5 m Shadow Rehearsal

The boot service now presents a one-shot QGC flight card for the next evidence
flight. This run validates the airborne trajectory and exact reverse velocity
proposal; it does **not** move or land the aircraft.

1. Keep GPS physically connected, RC9 low, and use a flat, textured test area
   with at least 10 m of clear space. Use the strain-relieved TELEM2 UART and
   leave Cube USB disconnected so there is only one companion MAVLink link.
   GPS remains available for origin and pilot recovery while the selected EKF
   source audit checks optical-flow velocity.
2. Reboot Cube and Jetson on level ground. Wait for the QGC message
   `SLAM TEST: GPS ON, RC9 LOW, USE FLOWHOLD`.
3. Take off manually in FlowHold to about 1.5 m. Hold level for the requested
   10 seconds.
4. Fly straight forward slowly to 5 m. Do not exceed 6 m or 0.75 m lateral
   error. Hold level for three seconds.
5. Only after `SLAM TEST: HOLD, THEN RC9 HIGH` and its ready beep, switch RC9
   high and leave it high.
6. This locked rehearsal computes the reverse path but cannot command it. Fly
   back manually in FlowHold when QGC says `SHADOW ONLY: FLY BACK IN FLOWHOLD`.
7. At `AT START: HOLD, LAND MANUALLY`, hold briefly, land using a tested
   altitude-controlled mode, and disarm. Never select Stabilize.

The run passes only when `./optflow slam-return-status` reports
`flight_test.profile_pass: true`, the odometry guard has no fault, the visual
and LIO tracks agree, and `transport.commands_sent` remains zero. An early RC9
trigger marks the rehearsal invalid but still sends no movement command.

FlowHold and Guided are separate Copter modes. A later live return will require
the pilot to select regular Guided before RC9; optical flow and range must then
be valid EKF navigation sources. Automatic takeoff, mode selection, landing,
and disarming remain outside Jetson authority.

## Trajectory Viewer

While the navigation service owns all hardware, start the read-only monitor:

```bash
./optflow visualizer \
  --trajectory-monitor --host 0.0.0.0 --no-browser
```

Open `http://<jetson-ip>:8765`. It starts in **3D SCAN** with trajectory and
D415 enabled and JT16 disabled. D415 and JT16 can be toggled independently; use
the Route icon to show or hide LIO, RGB-D, Cube, reverse breadcrumbs, and the
current target.

The cyan `LIO RAW MONITOR` trajectory is deliberately display-only and remains
continuous after a pose is rejected. A red `SLAM FAULT` badge means the guarded
trajectory has frozen at the last accepted pose. Raw monitor poses never feed
the return controller or Cube.

For the full D415/JT16 point-cloud view, stop the service first and run:

```bash
./optflow visualizer --host 0.0.0.0 --no-browser
```

## Field Boundary

ArduCopter 4.6.3 needs a valid EKF origin for regular Guided. GPS may provide
that origin before entering a GPS-denied area; otherwise the operator must set
the origin in QGC. The project does not silently install a fake location.

No first active test should combine return, obstacle replanning, automatic
landing, and a GPS-free cold boot. Complete three or four clean 5 m shadow
rehearsals first, then prove a tethered/low-altitude Guided zero-velocity
handoff, then authorize a short live straight return with GPS still connected
as recovery. Physical GPS removal comes only after those live returns and a
repeatable no-GPS EKF-origin procedure pass. Obstacle flanking remains a later
stage; the current active behavior is hard stop only.

TELEM2 passed a complete 15-parameter request/response audit on 2026-08-08.
Before any live Guided movement test, separately prove a benign command/ACK
round trip over `/dev/ttyTHS1`; telemetry and parameter reads alone do not
approve movement output.
