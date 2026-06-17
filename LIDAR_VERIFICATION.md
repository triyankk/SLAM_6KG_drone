# 3D LiDAR Verification & Testing Plan

This document provides a structured approach to verify the Hesai JT16 Mini LiDAR integration and the obstacle avoidance layer.

The current flight path is ArduPilot-native avoidance first:

- `config/sensors.yaml` selects `avoidance.mode: "rc_toggle"`
- RC channel 7 low means detect-only: the Jetson publishes `OBSTACLE_DISTANCE`, `PRX1_TYPE=2`, and `AVOID_ENABLE=0`
- RC channel 7 high means avoidance engaged: the Jetson sets `AVOID_ENABLE=7`
- ArduPilot keeps the configured margin while it remains in charge of flight control
- set `avoidance.mode: "monitor_only"` to disable FC PRX/avoidance but keep scan logs and GCS status
- direct Jetson velocity pulses are disabled unless `--enable-motion` is explicitly used
- the configured keepout margin is 1.5 m, with critical response at 0.5 m
- fresh avoidance publishes `OA DETECT ONLY: RC7=...` or `OA ENGAGED: RC7=...` to GCS periodically
- obstacle text is mirrored directly to Jetson UDP `14550`/`14551`
- obstacle audio is armed-only; disarmed obstacle events are GCS text only
- armed obstacles inside 1.5 m trigger warning beeps; armed critical obstacles near 0.5 m trigger rapid critical beeps

USB rule for field testing:

- Keep Cube <-> Jetson on USB.
- Let the legacy bridge own `/dev/ttyACM0`.
- LiDAR avoidance must use `udpout:127.0.0.1:14555`, which is local Jetson routing into the USB bridge.
- Do not connect the LiDAR node directly to `/dev/ttyACM0` while the bridge is running.
- For direct Cube USB/telemetry GCS visibility, install the updated `ardupilot_lua/brake_slam_beeper.lua` on the Cube SD card as `APM/scripts/brake_slam_beeper.lua`; it relays obstacle status from `SCR_USER4`.

## 1. Hardware Connection Check
Confirm the LiDAR is powered and the data link is active.
```bash
# Verify raw UDP/Serial packets
python3 scripts/diagnostics/check_hesai_jt16.py
```

## 2. Visualization & Zoning (Ground Test)
Run the visualizer to confirm the 8 zones and detection boundaries.
```bash
python3 scripts/avoidance/visualize_lidar_avoidance.py
```
**Test Criteria:**
- [ ] Objects at 7m+ are ignored (grey/none).
- [ ] Objects within 7m appear as Green points.
- [ ] Objects within 2.5m appear as Yellow points.
- [ ] Objects within 1.5m appear as Red points.
- [ ] The Cyan arrow correctly points **away** from the obstacle.

## 3. GCS Reporting (Disarmed Test)
Power the drone and connect your GCS (Mission Planner/QGC).
**Test Criteria:**
- [ ] Start the node with `python3 scripts/avoidance/hesai_jt16_obstacle_node.py --dry-run`.
- [ ] **GCS STATUSTEXT**: Confirm "OA DETECT ONLY: RC7=..." appears with channel 7 low.
- [ ] Flick channel 7 high.
- [ ] **GCS STATUSTEXT**: Confirm "OA ENGAGED: RC7=..." appears.
- [ ] **Audio**: Confirm one short rising confirmation beep plays when RC7 engages avoidance.
- [ ] Flick channel 7 low again.
- [ ] **GCS STATUSTEXT**: Confirm "OA DETECT ONLY: RC7=..." appears again.
- [ ] Move an object within 1.5m of the LiDAR while disarmed.
- [ ] **GCS STATUSTEXT**: Confirm "LiDAR keepout: [Zone] [Dist]m push vx=... vy=..." appears.
- [ ] **Audio**: Confirm there are no obstacle beeps while disarmed.
- [ ] **Audio**: Confirm warning beeps only after arming with an obstacle inside 1.5m.
- [ ] **Audio**: Confirm rapid beeps trigger near 0.5m only when armed.

## 4. Safety Gates (Bench Test - No Props)
Verify that commands are *not* sent when it's unsafe.
**Test Criteria:**
- [ ] Confirm native params read back as `PRX1_TYPE=2`, `AVOID_ENABLE=7`, `AVOID_MARGIN=1.5`.
- [ ] With channel 7 low, confirm `PRX1_TYPE=2` and `AVOID_ENABLE=0`.
- [ ] With channel 7 high, confirm `PRX1_TYPE=2` and `AVOID_ENABLE=7`.
- [ ] Confirm `PRX2_TYPE=0`, `PRX3_TYPE=0`, and `PRX4_TYPE=0`.
- [ ] Confirm `PreArm: PRX1: Not Connected` does not repeat after the LiDAR node is publishing.
- [ ] Arm the drone (Props Off!).
- [ ] Switch to a non-supported mode (e.g., STABILIZE).
- [ ] Move obstacle within 1m.
- [ ] **Verification**: Confirm GCS warnings appear, but NO velocity commands are logged in the console.

## 5. Avoidance Command (Airborne Simulation)
This step is only for direct companion motion experiments. Native ArduPilot
avoidance can be validated separately through Mission Planner/QGC proximity
inspection and open-space flight testing.

*Note: ArduPilot needs to report "In Air" for direct velocity pulses to trigger.*
**Test Criteria:**
- [ ] Start the node with `--enable-motion` only after no-prop bench testing.
- [ ] Arm in GUIDED.
- [ ] Simulate "In Air" state (or verify `landed_state` in MAVLink inspector).
- [ ] Move obstacle within 1.5m.
- [ ] **Verification**: Console should print "Sending LiDAR avoidance velocity pulse vx=... vy=...".

## 6. Service Integration
Confirm all services start together.
```bash
./scripts/manage_flight_stack.sh start
./scripts/manage_flight_stack.sh status
```

---
**DANGER**: Never enable motion commands (`enable_avoidance_motion: true`) for the first time with propellers attached. Always perform a "Dry Run" field test first.
