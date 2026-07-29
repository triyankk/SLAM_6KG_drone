# Passive Flight Logger

The flight logger records evidence for estimator development. It does not arm,
change mode, send movement targets, or publish ExternalNav.

## Before A Flight

The installed `optflow-flight-logger.service` starts with the Jetson and owns
the Cube UART and IM10A. Check it before leaving for the field:

```bash
./optflow flight-status
```

The expected idle result is `STATE=waiting_for_arm`, `LINK=true`, and
`ARMED=false`. No session folder or camera recording is created while disarmed.

Do not run the visualizer, standalone RGB stream, or manual logger in parallel
with this service. The service owns `/dev/ttyTHS1` and `/dev/imu_usb`
continuously, then claims the RealSense and JT16 UDP port only while recording.

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
  shadow_predictions.ndjson
  events.ndjson
  analysis/
    report.json
    report.md
    timeline.csv
  cube/
  lidar/
    jt16_packets.pcap
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
configured hardware rate. JT16 UDP payloads are wrapped in a valid raw-IP PCAP.
`sensor_events.ndjson` preserves every decoded MAVLink and IM10A event rather
than decimating them to the visualization rate; source sequence gaps are
recorded explicitly.

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

Wait for `STATE=waiting_for_arm`, then inspect `LAST_SESSION` and `LAST_REPORT`:

```bash
./optflow flight-status
```

Attach the corresponding Cube DataFlash log and regenerate the report:

```bash
./optflow analyze data/recordings/flights/<session> \
  --cube-log /path/to/latest.BIN
```

The `.BIN` file is copied into the session. Selected attitude, EKF, power,
vibration, rate, mode, motor, event, and error messages are extracted. Logger
attitude is aligned to DataFlash `ATT` using Cube boot time.

The first review should use:

- `analysis/report.md` for coverage and summary observations;
- `analysis/timeline.csv` for plots and event windows;
- `telemetry.ndjson`, `sensor_events.ndjson`, and
  `shadow_predictions.ndjson` for exact values;
- the DataFlash `.BIN` for high-rate flight-controller evidence;
- `realsense/flight.bag` and `lidar/jt16_packets.pcap` for estimator replay;
- `pointcloud/flight_environment.ply` for a quick environment preview.

## Current Ownership

The automatic logger owns `/dev/ttyTHS1` and `/dev/imu_usb`; it does not depend
on the visualizer or an HTTP stream. It owns the D415 and UDP port 2368 only
between arm and post-disarm finalization. For bench visualization, first stop
the service, run the visualizer, then restart the service afterward:

```bash
systemctl --user stop optflow-flight-logger.service
./optflow visualizer --host 0.0.0.0
systemctl --user start optflow-flight-logger.service
```

Once a ROS 2 Hesai driver is active, raw lidar capture must move to the driver's
packet or `PointCloud2` topic rather than sharing the UDP socket.

The older `./optflow flight-log` command remains available for deliberately
manual bench captures. It needs the service stopped and the visualizer running;
it is not the field workflow.
