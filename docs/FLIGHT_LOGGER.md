# Passive Flight Logger

The flight logger records evidence for estimator development. It does not arm,
change mode, send movement targets, or publish ExternalNav.

## Before A Flight

Start the visualizer so it remains the only owner of the Cube UART and IM10A:

```bash
./optflow visualizer --host 0.0.0.0 --no-browser
```

Do not run `./optflow camera` during a flight recording. The logger owns the
RealSense while it writes the full-rate bag and sampled point clouds.

Start a named recording:

```bash
./optflow flight-log --name field-01
```

The logger starts before arming and records until `Ctrl+C`. Stop it only after
landing and disarming so shutdown can finalize the RealSense bag, PLY map,
manifest, and analysis report. It stops itself before free disk space falls
below 5 GB.

The full 640x480 at 30 Hz color/depth bag is intentionally high fidelity. The
current bench capture used about 1.3 GB per minute; the generated report records
the measured rate for every session. Use `--no-realsense-bag` only when replay
data is deliberately unnecessary.

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

The visualizer owns `/dev/ttyTHS1` and `/dev/imu_usb`. The logger subscribes to
its HTTP event stream, so it never opens either serial device. The logger owns
the D415 and UDP port 2368 while recording. Once a ROS 2 Hesai driver is active,
raw lidar capture must move to the driver's packet or `PointCloud2` topic rather
than sharing the UDP socket.
