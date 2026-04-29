# Intellisense SLAM Field Manual

This repo runs Jetson-side SLAM/VIO support for a Cube Orange+ GPS-denied drone.

The current safe field process is:
- keep only `intellisense_slam_bridge.service` running
- do not run the passive MAVLink monitor during flight
- do not let Mission Planner or another local process share the same Cube serial port
- use Brake mode only as the SLAM calibration trigger
- treat GPS-denied PosHold as not flight-ready until ExternalNav is accepted and calibration passes

## Current Flight Status

As of the latest bench checks:
- SLAM bridge service runs on boot.
- Passive monitor is disabled and should stay disabled for field use.
- VIO is alive when status shows `vio=ok`.
- IMU is alive when status shows `imu=stable`.
- RC is alive when status shows `rc_link=ok`.
- Brake mode detection works.
- Disarmed Brake mode should show `WAITING_FOR_ARM`, not `SLAM calibration active`.
- GPS-denied PosHold is not ready until status shows ExternalNav accepted/ready.

Known current blocker:
- If QGC shows `PreArm: GPS 1: Bad fix`, the Cube is still expecting GPS for that arming/navigation path.
- Brake calibration needs a valid GPS/EKF reference because it compares SLAM/VIO against GPS/EKF local position.
- If `ekf_external_nav=pending_or_rejected`, the Cube is not confirmed to be using SLAM as navigation.

## Active Service

The only field autostart service should be:

```bash
intellisense_slam_bridge.service
```

Check it:

```bash
sudo systemctl status intellisense_slam_bridge.service
journalctl -u intellisense_slam_bridge.service -f
```

Restart it:

```bash
sudo systemctl restart intellisense_slam_bridge.service
```

Install or refresh autostart:

```bash
cd /home/atas/vscode/intellisense_slam
sudo bash install/install_slam_bridge_autostart.sh
```

The service runs:

```bash
python3 scripts/calibration/brake_slam_calibration.py --config config/autostart.yaml
```

## Do Not Run In Field

The passive monitor is useful for bench diagnosis only. Do not run it during flight because it can fight with the SLAM bridge for the same Cube serial stream.

Stop and disable it:

```bash
sudo systemctl stop slam-mavlink-monitor.service
sudo systemctl disable slam-mavlink-monitor.service
```

Confirm:

```bash
systemctl is-active slam-mavlink-monitor.service
systemctl is-enabled slam-mavlink-monitor.service
```

Expected:

```text
inactive
disabled
```

## MAVLink Port Ownership

Only one Jetson process should own the Cube serial port.

Check:

```bash
fuser -v /dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00
```

Expected owner:

```text
python3 ... run_slam_odometry_bridge.py
```

Not OK:
- `MissionPlanner.exe`
- `src/main.py`
- another `run_slam_odometry_bridge.py`
- any second MAVLink reader on the same port

If Mission Planner is running on the Jetson, close it before field testing.

## Status Command

Use this to see the latest bridge state:

```bash
python3 scripts/calibration/brake_slam_calibration.py --status
```

Healthy ground-monitoring example:

```text
state=IDLE
stage=idle
mode=LOITER or STABILIZE
armed=False
vio=ok
imu=stable
mavlink=ok
rc_link=ok
```

Not flight-ready examples:

```text
ekf_external_nav=pending_or_rejected
gps1=1/0
rc_link=missing
mavlink=timeout
vio=bad
imu=missing
```

## Current Autostart Config

Main file:

```bash
config/autostart.yaml
```

Important current defaults:
- `source: vio`
- `calibration.mode: BRAKE`
- `gps_input.enabled: false`
- `gps2_type: 0`
- `gps_auto_switch: 0`
- `select_source_set_on_stream: false`
- `movement_commands_enabled: false`
- `auto_rtl_after_complete: true`
- `fallback_mode: LOITER`

Meaning:
- The bridge monitors SLAM/VIO.
- GPS2 spoofing is off.
- Active motion is off by default.
- The bridge does not automatically take off.
- The bridge does not climb from the ground.
- Calibration can request RTL only after success.

## Brake Mode Behavior

Brake mode is the calibration trigger.

Disarmed on ground:

```text
Brake mode detected. Waiting for arm to start SLAM calibration.
```

Expected state:

```text
state=WAITING_FOR_ARM
stage=waiting_arm
```

This should not play the musical calibration-active beep.

Armed on ground:

```text
Armed in Brake mode. SLAM calibration takeoff sequence active.
No automatic takeoff: pilot must take off and enter Brake near 5m.
```

Airborne near target height:

```text
Reached 5 meters by rangefinder. Holding altitude for SLAM calibration.
SLAM calibration active.
```

Only this active calibration stage should play the musical calibration-active tune.

Success:

```text
Calibration successful: SLAM PosHold calibration complete. Initiating RTL.
```

Failure:

```text
Calibration failed: not finished. Reason: <reason>
```

## Why GPS Bad Fix Matters

The current Brake calibration logic compares SLAM/VIO against the Cube's GPS/EKF local position reference.

If GPS is unavailable:
- Brake detection still works.
- VIO/IMU monitoring still works.
- Calibration will not complete.
- GPS-denied PosHold should not be trusted yet.

If QGC shows:

```text
PreArm: GPS 1: Bad fix
```

then the Cube is still blocking or warning on GPS for the current arming/navigation configuration. Fix that before any GPS-denied field flight.

## Lua Beeper

Lua file:

```bash
ardupilot_lua/brake_slam_beeper.lua
```

Install it on the Cube SD card:

```text
APM/scripts/brake_slam_beeper.lua
```

Then reboot the Cube.

Current beep rules:
- Jetson boot: three short beeps
- sensor quick check passed: one short beep
- SLAM ready for PosHold: rising tune
- Brake detected but disarmed: GCS notice only
- active calibration: calibration tune and 10 second reminder beep
- success: rising long tune
- failure: descending warning tune
- SLAM flight active: single small beep

Every beep should have a matching GCS notice.

If you still hear the old musical tune immediately after entering Brake while disarmed, the Cube is still running the old Lua script. Copy the updated Lua file and reboot the Cube.

## Bench Checks Before Field

Run from repo root:

```bash
cd /home/atas/vscode/intellisense_slam
```

Smoke checks:

```bash
python3 scripts/diagnostics/run_smoke_checks.py
python3 scripts/diagnostics/check_mavlink.py
python3 scripts/diagnostics/check_realsense.py
python3 scripts/diagnostics/check_imu.py
python3 scripts/diagnostics/check_vio_drift.py
```

Stationary calibration:

```bash
python3 scripts/calibration/run_stationary_calibration.py --indoor --no-gps --verbose
```

Brake dry-run:

```bash
python3 scripts/calibration/brake_slam_calibration.py --dry-run
```

Service logs:

```bash
journalctl -u intellisense_slam_bridge.service -f
tail -f logs/slam_calibration.log
```

## Field Ground Test

This is allowed before flight:

1. Power drone and Jetson.
2. Confirm only `intellisense_slam_bridge.service` is running.
3. Confirm the passive monitor is disabled.
4. Confirm only one process owns the Cube port.
5. Open QGC.
6. Switch to Brake while disarmed.
7. Confirm QGC says waiting for arm.
8. Confirm no musical calibration-active tune plays.
9. Switch back to Stabilize/Loiter.
10. Confirm status returns to `IDLE`.

Commands:

```bash
systemctl is-active slam-mavlink-monitor.service
systemctl is-enabled slam-mavlink-monitor.service
fuser -v /dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00
python3 scripts/calibration/brake_slam_calibration.py --status
```

## Flight Readiness Checklist

Do not take off in GPS-denied PosHold until all are true:

- Cube serial port has only one owner.
- MAVLink shows no heartbeat timeouts for at least 2 minutes.
- VIO status is `ok`.
- IMU status is `stable`.
- RC link is `ok`.
- QGC messages match the current mode.
- Brake/disarmed shows waiting-for-arm only.
- Brake calibration can run without state bouncing.
- ExternalNav is accepted by the EKF.
- `ekf_external_nav` is not `pending_or_rejected`.
- GPS-denied arming checks are intentionally configured and understood.
- Fallback to `LOITER` is tested.
- RTL is tested separately.
- First live test is tethered or prop-guarded in a wide open area.

Current recommendation:

```text
Ground testing: OK
Untethered GPS-denied PosHold flight: NO-GO until ExternalNav is accepted and calibration passes.
```

## Optional LiDAR / Obstacle Tools

LiDAR is not part of the current SLAM field-readiness gate.

Check Hesai JT16:

```bash
python3 scripts/diagnostics/check_hesai_jt16.py
```

Visualize LiDAR:

```bash
python3 scripts/avoidance/visualize_lidar_avoidance.py
```

Dry-run obstacle node:

```bash
python3 scripts/avoidance/hesai_jt16_obstacle_node.py --dry-run
```

Do not enable real obstacle motion until SLAM flight itself is stable.

## Recovery Notes

Restart bridge:

```bash
sudo systemctl restart intellisense_slam_bridge.service
```

Stop old/extra services:

```bash
sudo systemctl stop slam-mavlink-monitor.service
sudo systemctl disable slam-mavlink-monitor.service
sudo systemctl stop vio-flight.service 2>/dev/null || true
sudo systemctl disable vio-flight.service 2>/dev/null || true
```

If USB serial adapters disappear after reboot:

```bash
sudo bash install/enable_usb_serial_sensors.sh
sudo bash install/install_usb_serial_sensors_autostart.sh
```

If Cube keeps reporting `GPS2 bad fix`:

```text
Confirm GPS2_TYPE=0 in ArduPilot, then reboot the Cube.
Only enable GPS2 when a real MAVLink GPS_INPUT stream is intentionally configured.
```
