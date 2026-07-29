# optFlow_slam

Clean GPS-independent flight and SLAM project for the Cube Orange+ and Jetson.

## Current Status

The project is at `hardware_bringup`.

- Cube to Jetson MAVLink over TELEM2 UART is working at 921600 baud.
- Holybro H-Flow flow and downward range data are reaching the Cube over CAN2.
- H-Flow and rangefinder Z offsets are set to +0.10 m below the CG.
- Intel RealSense D415 RGB capture is connected and available locally.
- Hiwonder IM10A is connected as `/dev/imu_usb`; its factory 9600-baud,
  10 Hz WIT stream has passed checksum and sanity checks.
- D415 depth bring-up and Hesai JT16 are still pending.
- Jetson autonomous command output is disabled.
- Cube ExternalNav input is disabled.
- No service in this project arms the vehicle or commands movement.
- Source, dependencies, generated data, ROS workspace, and runtime state are
  contained within this project directory.

## System Flow

```text
H-Flow + range -> Cube EKF -> attitude/position controller -> ESCs -> motors
                       ^
                       | bounded local velocity/position targets
                       |
depth camera -> local obstacles ----+
external IMU -> SLAM estimator -----+-> map -> planner -> safety supervisor
Hesai JT16 -> 3D geometry ----------+                    |
                                                         +-> local return path

fused SLAM pose -> shadow log first -> optional MAVLink ODOMETRY later
```

The Cube owns every fast flight-control loop. The Jetson never directly mixes
motor outputs and will not send raw attitude commands in the first navigation
version.

Optical flow measures motion relative to textured ground. With downward range,
the Cube can integrate that velocity into a local XY estimate and hold position
at low altitude. It is not an absolute global position source and can still
drift with poor texture, bad lighting, excessive height, vibration, or bad
calibration.

The external IMU is an input to the Jetson SLAM estimator. An IMU by itself
cannot correct position because integrating its bias causes rapid drift. Only a
healthy fused lidar-inertial or visual-inertial SLAM pose may later be sent to
the Cube as ExternalNav.

## First Commands

Create the project-local Python environment, build the frontend, and run the
tests without reading another workspace or Python environment:

```bash
./optflow setup
```

Run the Cube and H-Flow bench gate with props removed:

```bash
./optflow preflight --profile fc_bench
```

Show all currently missing SLAM hardware:

```bash
./optflow preflight --profile slam_bench
```

Run the live drone-motion visualizer:

```bash
./optflow visualizer --host 0.0.0.0
```

Open `http://127.0.0.1:8765` on the Jetson or use the Jetson's LAN address from
another device. The main aircraft view uses Cube attitude, Cube IMU, H-Flow, and
range data. H-Flow's angular-speed-compensated X/Y flow is displayed in `m/s`;
the raw angular rates remain visible as a diagnostic. The corner view animates
the external IM10A using the future `sensor_msgs/Imu` contract. Until ROS 2 is
installed, that stream is decoded directly from `/dev/imu_usb`. Its display is
mapped through the measured `X/-Y/-Z` body-axis signs and reference-aligned to
Cube at startup. Full sensor-to-body extrinsics remain explicitly unverified.
The browser transport runs at 60 Hz, but the current factory IM10A stream remains
10 Hz; increasing the real sensor rate requires the reversible baud/rate
configuration and recovery workflow.

The visualizer owns both `/dev/ttyTHS1` and `/dev/imu_usb` while it runs. Stop
the automatic flight logger before a bench visualization, then restart it
afterward. Use `--demo` to exercise both views without opening either hardware
link.

Run the forward RealSense RGB stream:

```bash
./optflow camera --no-browser
```

Open `http://127.0.0.1:8770` on the Jetson or port `8770` at the Jetson's LAN
address from another device. This process owns only the RealSense camera; it
does not open the Cube UART or issue flight commands.

The field logger starts automatically with the Jetson and waits for the Cube to
arm:

```bash
./optflow flight-status
```

It buffers five seconds of telemetry before arm, records throughout the armed
period, and finalizes ten seconds after disarm. It saves synchronized telemetry
and shadow predictions, a full RealSense bag, raw JT16 PCAP, sampled PLY frames,
a merged 3D environment cloud, and an analysis report under
`data/recordings/flights/`. The visualizer and RGB stream stay off during this
workflow. Its perfect-SLAM local-target reference is offline comparison only
and cannot send commands.

After landing, attach the matching Cube DataFlash log:

```bash
./optflow analyze data/recordings/flights/<session> \
  --cube-log /path/to/latest.BIN
```

See [FLIGHT_LOGGER.md](docs/FLIGHT_LOGGER.md) for the folder contract,
interpretation limits, and field workflow.

Run tests:

```bash
./optflow test
```

The `navigation` profile is expected to remain blocked until every sensor,
calibration, the RC disarm switch, and command output have passed their staged
tests:

```bash
./optflow preflight --profile navigation
```

## Project Boundary

The directory is a standalone project boundary:

- `.venv/` is the local Python runtime.
- `vendor/python/` contains the offline Jetson aarch64 wheel bundle.
- `visualizer/node_modules/` and `visualizer/dist/` contain local frontend
  dependencies and runtime assets.
- `data/` owns calibrations, logs, maps, and recordings.
- `runtime/` owns transient process state.
- `ros_ws/` is the only ROS workspace used by the project.
- `third_party/` contains pinned estimator and sensor-driver source.
- `hardware/` is the source of truth for host-level hardware rules.

Operating-system devices such as `/dev/ttyTHS1`, kernel drivers, CUDA, and ROS
remain host dependencies. Project code must never import source, configuration,
maps, logs, or binaries from a sibling workspace folder. A boundary test
enforces this rule.

## Responsibility Split

| Layer | Owns | Must not own |
| --- | --- | --- |
| Cube | IMU/baro/mag fusion, H-Flow velocity, range scaling, stabilization, motor control | SLAM map or path search |
| Jetson estimator | Timestamped IMU/lidar/camera fusion, local pose, covariance, map | Raw motor or attitude control |
| Jetson planner | A* path on a validated costmap, local collision checking, local return path | Treating one depth ray as a map |
| Safety supervisor | Pose/command freshness, speed limits, obstacle stop, authority release | Arming or automatic EKF switching during bring-up |
| Pilot | Arm/disarm, flight mode, manual takeover, final go/no-go | Assuming software can recover from every estimator failure |

## Navigation Strategy

Version 1 uses fixed-altitude 2.5D navigation:

1. Build a local occupancy/cost map from the 3D lidar and forward depth camera.
2. Use A* for the global route on that map.
3. Use a proven collision-aware local planner to reject unsafe path segments.
4. Send conservative local velocity targets to regular GUIDED mode.
5. Record the launch pose in the SLAM map and plan back to that pose for local
   return.

The forward depth camera supplies near-field obstacles. A* or Dijkstra runs on
the resulting map, not directly on camera frames. Full 3D planning comes only
after fixed-altitude navigation is repeatable.

Standard ArduPilot RTL is not the GPS-denied return mechanism. Without a valid
global home and global position, the Jetson must perform a local-map return to
the stored launch pose. If SLAM loses localization, the correct response is
hold, pilot takeover, or controlled landing, not blind dead reckoning.

## Project Map

- `config/system.yaml`: single source of hardware, limits, and enable gates.
- `optflow`: project-relative launcher for setup, tests, services, preflight,
  and bench tools.
- `data/`: all project-generated calibrations, logs, recordings, and maps.
- `hardware/`: project-owned host hardware configuration.
- `hardware/kernel/ch341/`: pinned IM10A USB serial driver for this Jetson
  kernel.
- `ros_ws/`: project-local ROS 2 workspace.
- `third_party/`: pinned external estimator and driver source.
- `vendor/`: offline runtime dependencies.
- `docs/ARCHITECTURE.md`: estimator, planner, Cube, and failure boundaries.
- `docs/BRINGUP.md`: ordered bench and field sequence.
- `docs/INTERFACES.md`: frames, timing, odometry, and command contracts.
- `docs/FLIGHT_LOGGER.md`: passive flight recording and analysis workflow.
- `docs/POWER_AND_ESC.md`: acceptance tests for the individual ESC conversion.
- `scripts/preflight.py`: read-only live hardware gate.
- `scripts/rgb_stream.py`: project-local RealSense RGB web stream.
- `scripts/flight_logger_service.py`: boot-time arm-triggered flight recorder.
- `scripts/visualizer.py`: live Cube attitude, H-Flow, range, trace, and CSV UI.
- `visualizer/`: Three.js client and responsive visual checks.
- `src/optflow_slam/`: new code only; no old bridge imports.
- `tests/`: non-flight unit tests.

## Non-Negotiable Rules

- Never select STABILIZE as an automated fallback.
- Do not use GUIDED_NOGPS for this architecture.
- Do not send Cube ExternalNav until SLAM has passed offline and shadow tests.
- Do not allow two processes to own `/dev/ttyTHS1`.
- Do not use normal RTL as the GPS-denied return strategy.
- Do not arm from this project during bring-up.
- Prompt the operator before assigning the requested RC disarm channel.

See [BRINGUP.md](docs/BRINGUP.md) before connecting the remaining sensors.
