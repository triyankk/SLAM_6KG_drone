# 3D LiDAR Verification & Testing Plan

This document provides a structured approach to verify the Hesai JT16 Mini LiDAR integration and the obstacle avoidance layer.

## 1. Hardware Connection Check
Confirm the LiDAR is powered and the data link is active.
```bash
# Verify raw UDP/Serial packets
python3 scripts/check_hesai_jt16.py
```

## 2. Visualization & Zoning (Ground Test)
Run the visualizer to confirm the 8 zones and detection boundaries.
```bash
python3 scripts/visualize_lidar_avoidance.py
```
**Test Criteria:**
- [ ] Objects at 7m+ are ignored (grey/none).
- [ ] Objects within 7m appear as Green points.
- [ ] Objects within 3m appear as Yellow points.
- [ ] Objects within 2m appear as Red points.
- [ ] The Cyan arrow correctly points **away** from the obstacle.

## 3. GCS Reporting (Disarmed Test)
Power the drone and connect your GCS (Mission Planner/QGC).
**Test Criteria:**
- [ ] Move an object within 2m of the LiDAR while disarmed.
- [ ] **GCS STATUSTEXT**: Confirm "LiDAR obstacle danger: [Zone] [Dist]m" appears.
- [ ] **Audio**: Confirm a single loud beep triggers at 2m.
- [ ] **Audio**: Confirm rapid continuous beeps trigger at 1.2m.

## 4. Safety Gates (Bench Test - No Props)
Verify that commands are *not* sent when it's unsafe.
**Test Criteria:**
- [ ] Arm the drone (Props Off!).
- [ ] Switch to a non-supported mode (e.g., STABILIZE).
- [ ] Move obstacle within 1m.
- [ ] **Verification**: Confirm GCS warnings appear, but NO velocity commands are logged in the console.

## 5. Avoidance Command (Airborne Simulation)
*Note: ArduPilot needs to report "In Air" for this to trigger.*
**Test Criteria:**
- [ ] Arm in LOITER/GUIDED.
- [ ] Simulate "In Air" state (or verify `landed_state` in MAVLink inspector).
- [ ] Move obstacle within 2m.
- [ ] **Verification**: Console should print "Sending velocity pulse: [vx, vy]".

## 6. Service Integration
Confirm all services start together.
```bash
./scripts/manage_flight_stack.sh start
./scripts/manage_flight_stack.sh status
```

---
**DANGER**: Never enable motion commands (`enable_avoidance_motion: true`) for the first time with propellers attached. Always perform a "Dry Run" field test first.
