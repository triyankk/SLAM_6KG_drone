# Legacy Flow Bridge

This folder is a parallel path built from the older `intellisense_cam` code that
previously held GPS-less PosHold. The current `src/slam_core` bridge is left in
place. This legacy bridge is for returning to the older working method while
keeping the new external IMU, LOITER observation, and Brake calibration support.

## Method

- RealSense optical flow is converted to `OPTICAL_FLOW_RAD`.
- Downward range comes from the Cube `DISTANCE_SENSOR` by default.
- Flow plus range is integrated into local north/east velocity.
- The integrated flow pose is sent as MAVLink `GPS_INPUT` with `gps_id=1`, so it
  appears to ArduPilot as GPS2.
- PosHold switches to the old no-GPS source set when flow/range health is good.
- Non-PosHold modes remain on the GPS startup source set.
- `ODOMETRY` / VisOdom is not used in the default legacy path.

## New Additions

- External IM10A is opened by default with `--imu-port auto`.
- External IMU roll/pitch are used for attitude plausibility and flow navigation.
- Yaw defaults to the Cube/FC yaw with `--external-imu-yaw-mode fc`; use
  `external` only after IMU yaw alignment is verified.
- LOITER observation scores flow/GPS/IMU consistency from `0.0` to `10.0`.
- LOITER learns a bounded velocity scale for the GPS2 flow feed.
- BRAKE mode records the current learned legacy calibration profile.
- `GPS_INPUT` now includes non-zero GPS week/week-ms fields.

## Field Safety

Do not run this at the same time as `intellisense_slam_bridge.service`.

## Current Jetson Boot Setup

This field machine now starts the legacy bridge from the user crontab:

```bash
@reboot /home/atas/vscode/intellisense_slam/legacy_flow_bridge/legacy_boot_cron.sh
```

The launcher waits 30 seconds, takes `runtime/legacy_flow_bridge.lock`, and
runs `./run_field_legacy.sh`.

The current SLAM bridge is blocked on boot because
`/home/atas/vscode/intellisense_slam/config/autostart.yaml` has been moved to
`config/autostart.yaml.disabled_legacy_boot`. Restore that filename only when
you intentionally want the newer bridge to boot again.

Boot log:

```bash
tail -f logs/legacy_flow_bridge_boot.log
```

Before starting legacy:

```bash
sudo systemctl stop intellisense_slam_bridge.service
```

Run manually:

```bash
cd /home/atas/vscode/intellisense_slam/legacy_flow_bridge
python3 realsense_optical_flow_to_cube.py --ports /dev/ttyACM1 /dev/ttyACM0
```

Useful bench mode:

```bash
python3 realsense_optical_flow_to_cube.py --flow-health-test --ports /dev/ttyACM1 /dev/ttyACM0
```

Logs:

```bash
tail -f logs/legacy_flow_loiter_observer.log
cat runtime/legacy_flow_calibration.json
```

## What To Expect

In LOITER:

- Drone remains normal GPS LOITER.
- Bridge only observes and learns.
- GCS messages start with `LEGACY SLAM:`.

In BRAKE:

- Legacy calibration bookkeeping starts.
- No automatic takeoff is commanded.
- No deliberate movement is commanded by the new observer/calibration layer.

In POSHOLD:

- If flow/range/GPS2 health is good, the legacy bridge selects the no-GPS source
  set and announces `GPS-Less flight active`.
- If flow health becomes bad, the old failsafe path switches back to GPS/LOITER.

This is still field-test code. Validate logs before trusting GPS-less PosHold.
