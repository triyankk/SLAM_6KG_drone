# Passive Flight Logger

The flight logger records evidence for estimator development. It does not arm,
change mode, send movement targets, or publish ExternalNav.

## Before A Flight

The passive arm-triggered logger is now a manual evidence tool. The installed
`optflow-flight-logger.service` runs the OA-only runtime and already owns the
Cube MAVLink endpoint and JT16. Stop it before starting this logger:

```bash
systemctl --user stop optflow-flight-logger.service
./optflow flight-service
./optflow flight-status
```

The SLAM shadow plays a ready tune only after LIO and RGB-D have initialized,
then QGC displays `SLAM TEST READY: ARM IN LOITER`. This confirms the proof
pipeline reached its readiness threshold; it does not approve active obstacle
avoidance or autonomous control.

Before arming, the expected logger status is `STATE=waiting_for_arm` and
`ACTIVE_CONTROL=false`.

Do not run the full visualizer, standalone RGB stream, boot OA-only service,
or another logger in parallel. The logger owns the configured Cube MAVLink endpoint and
`/dev/imu_usb` continuously, then claims the RealSense and JT16 serial device
while recording.

## Automatic Trigger

An armed Cube heartbeat starts a session. The logger first writes its five
seconds of buffered telemetry and raw sensor events, then starts the full-rate
RealSense bag, sampled point clouds, and raw JT16 packet capture. A disarm starts
a ten-second tail. Re-arming during that tail keeps the same session open;
otherwise the service finalizes the bag, PLY map, manifest, and analysis report.

The recorder stops before free space falls below 5 GB and will not repeatedly
start partial sessions while the same armed period continues. It becomes
eligible again only after detecting a disarm. The full 640x480 at 30 Hz
color/depth bag is intentionally high fidelity. The current bench capture used
about 1.3 GB per minute; the generated report records the measured rate for
every session.

Follow service activity locally with:

```bash
journalctl --user -u optflow-flight-logger.service -f
```

## Session Layout

Each run is stored below `data/recordings/flights/`:

```text
YYYYMMDDTHHMMSSZ_field-01/
  manifest.json
  telemetry.ndjson
  sensor_events.ndjson
  sensor_timing.ndjson
  shadow_predictions.ndjson
  events.ndjson
  analysis/
    report.json
    report.md
    slam_timing.json
    timeline.csv
  cube/
  lidar/
    jt16_serial.bin
    jt16_bridge.log
  realsense/
    flight.bag
    intrinsics.json
  pointcloud/
    flight_environment.ply
    frames/
```

`telemetry.ndjson` preserves Cube, H-Flow, range, IM10A, local/global position,
GPS, power, vibration, EKF, RC, actuator, target, and timing snapshots using a
single Jetson clock. The RealSense bag preserves the camera streams at their
configured hardware rate. The pinned official Hesai SDK decodes JT16 XYZ
frames with per-point timestamp, ring, intensity, and confidence while the
original serial packets are preserved in `jt16_serial.bin`.
`sensor_events.ndjson` preserves every decoded MAVLink and IM10A event rather
than decimating them to the visualization rate; source sequence gaps are
recorded explicitly.
`sensor_timing.ndjson` is the estimator timing contract. It records Jetson
monotonic/realtime arrival clocks, D415 depth/color device timestamps and frame
numbers, JT16 SDK callback time, frame index, and per-frame point timestamp
ranges. IM10A and selected Cube events retain arrival time plus any available
device time. The analyzer compares elapsed clocks without assuming that their
epochs are equal.

The merged PLY is currently a provisional telemetry-registered D415 cloud. It
uses Cube local position when fresh and H-Flow dead reckoning otherwise. It is
not a loop-closed SLAM map, and the camera transform is an explicit
forward-mount assumption until measured extrinsics and timing pass calibration.

## Shadow Prediction

The shadow model assumes:

- local pose is exact, as it would be under the requested perfect-SLAM thought
  experiment;
- a fresh Cube local target is used when available, otherwise the desired XY
  position is the session origin;
- a conservative position/velocity return model is limited to 10 degrees.

It records predicted roll and pitch beside measured Cube attitude. A comparison
is marked applicable while armed in `POSHOLD`, `LOITER`, `FLOWHOLD`, or `BRAKE`,
and in `GUIDED` only when a fresh local target is present. Even then, a residual
is not automatically a control fault because pilot intent, wind, estimator
delay, and the Cube controller remain unmodeled. Predictions never enter a
MAVLink command path.

## After A Flight

For the first flight after boot, inspect
`data/recordings/slam_flights/<session>/analysis/slam_flight_shadow.json`.
For later passive flights, wait for `STATE=waiting_for_arm`, then inspect
`LAST_SESSION` and `LAST_REPORT`:

```bash
./optflow flight-status
```

Attach the corresponding Cube DataFlash log and regenerate the report:

```bash
./optflow analyze data/recordings/flights/<session> \
  --cube-log /path/to/latest.BIN
```

Re-run only the timestamp gate with:

```bash
./optflow slam-timing data/recordings/flights/<session>
```

`analysis/slam_timing.json` reports observed rate, period distribution, jitter,
estimated drops, frame-number gaps, relative clock drift, and explicit
lidar-inertial replay blockers. A short disarmed capture proves acquisition
plumbing only; it does not verify dynamic synchronization, extrinsics, IMU
noise, or estimator accuracy.

The `.BIN` file is copied into the session. Selected attitude, EKF, power,
vibration, rate, mode, motor, event, and error messages are extracted. Logger
attitude is aligned to DataFlash `ATT` using Cube boot time.

The first review should use:

- `analysis/report.md` for coverage and summary observations;
- `analysis/slam_timing.json` for clock, rate, jitter, and drop evidence;
- `analysis/timeline.csv` for plots and event windows;
- `telemetry.ndjson`, `sensor_events.ndjson`, and
  `shadow_predictions.ndjson` for exact values;
- the DataFlash `.BIN` for high-rate flight-controller evidence;
- `realsense/flight.bag` and `lidar/jt16_serial.bin` for estimator replay;
- `pointcloud/flight_environment.ply` for a quick environment preview.

## Current Ownership

While manually active, the logger owns the configured Cube MAVLink endpoint and `/dev/imu_usb`; it
does not depend on the visualizer or an HTTP stream. It owns the D415 and
`/dev/jt16_usb` between arm and post-disarm finalization. The full visualizer
owns all four devices continuously, so stop whichever hardware-owning process
is active before switching:

```bash
systemctl --user stop optflow-flight-logger.service
./optflow visualizer --host 0.0.0.0
systemctl --user start optflow-flight-logger.service
```

Once a ROS 2 Hesai driver is active, raw lidar capture must move to that
driver's packet or `PointCloud2` topic rather than sharing the serial device.

The older `./optflow flight-log` command remains available for deliberately
manual bench captures. It needs the service stopped and the visualizer running;
it is not the field workflow.
