# SLAM/VIO Calibration Guide

This document explains what calibration does, why it matters, and how to use the two calibration modes available in this SLAM stack.

## Table of Contents
1. [What is Calibration?](#what-is-calibration)
2. [The Alignment Problem](#the-alignment-problem)
3. [Calibration Offsets](#calibration-offsets)
4. [Stationary Calibration](#stationary-calibration)
5. [BRAKE Mode Calibration](#brake-mode-calibration)
6. [Comparing the Two Approaches](#comparing-the-two-approaches)
7. [How Offsets Are Applied](#how-offsets-are-applied)
8. [Reading Calibration Results](#reading-calibration-results)

---

## What is Calibration?

Calibration measures the **misalignment between two coordinate frames**:
- **Frame 1:** Flight Controller (Cube) reference frame
- **Frame 2:** SLAM/VIO reference frame

These frames start **completely independent** and unaligned. Calibration finds the transformation (offsets and rotation) needed to convert SLAM poses back into the FC's coordinate frame.

### Why Does Misalignment Exist?

1. **Different sensors** — FC uses its IMU; SLAM uses a camera/depth sensor
2. **Different processing** — FC fuses GPS/compass/IMU; SLAM fuses vision features
3. **Different mount points** — camera is mounted differently than the FC's IMU
4. **Different time bases** — FC runs at one rate; SLAM camera at another
5. **Different origins** — Each system bootstraps its own "origin" independently

### Why Does It Matter?

When the SLAM bridge sends `ODOMETRY` messages to the Cube's EKF, the EKF expects pose coordinates in **the FC's frame**. If the pose is in the wrong frame:
- Position confusion → wrong navigation estimates
- Heading confusion → wrong yaw control
- Mixed reference confusion → instability and poor control

**Good calibration** = EKF confidence that SLAM position matches flight reality.

---

## The Alignment Problem

### Concrete Example

Imagine your drone sitting on the ground:

**Flight Controller perspective:**
- Home position: (0, 0, 0) — the spot the FC boots up at
- Forward direction: North (yaw = 0°)
- All positions and headings measured relative to this reference

**SLAM perspective:**
- Home position: (0, 0, 0) — where the camera tracker starts at
- Forward direction: wherever the camera was pointing (yaw = 0°)
- All positions and headings measured relative to the camera origin

**The Reality:**
- The camera **is NOT at the same spot as the FC's sensors**
- The camera **is NOT pointing the same way as the FC's IMU**
- The camera's coordinate axes **may be rotated relative to the FC**

### Sources of Misalignment

| Misalignment | Example | Effect |
|---|---|---|
| **Position offset (X)** | Camera is 5cm forward of FC | Raw SLAM says (0,0) but FC reference is really (0.05, 0) |
| **Position offset (Y)** | Camera is 3cm to the right | Raw SLAM says (0,0) but FC reference is really (0, 0.03) |
| **Heading offset (yaw)** | Camera is rotated 2° left | Raw SLAM says facing 0° but FC says 2° |
| **Mounting rotation** | Camera is tilted/rolled | Vertical alignment differs |
| **Time offset** | Sensor clocks are not synchronized | Position measurements represent different times |

---

## Calibration Offsets

Calibration measures and stores **three primary offsets**:

### 1. **Yaw Offset** (degrees)

The heading difference between SLAM and FC.

```
yaw_offset_deg = FC_heading - SLAM_heading
```

**Example:**
- FC says: "I'm facing North (0°)"
- SLAM says: "I'm facing Northwest (+2° counterclockwise)"
- Yaw offset: 0° - (-2°) = **+2°**

**When stored and applied later:**
- Raw SLAM heading changes: 0° → +2° (now matches FC's "North")

**Impact:** Corrects rotational misalignment in the pose quaternion and attitude.

---

### 2. **X Position Offset** (meters)

The forward/backward misalignment.

```
x_offset_m = FC_local_x - SLAM_rotated_x
```

**Example:**
- FC local position: X = 1.0 m (1 meter forward)
- SLAM raw position: X = 0.92 m (appears 8cm behind)
- After accounting for heading rotation: appears 10cm behind
- X offset: 1.0 - 0.92 = **+0.08 m**

**When stored and applied later:**
- Raw SLAM position (0, 0) → Corrected position: (+0.08, 0)

**Impact:** Shifts all SLAM positions forward by this amount.

---

### 3. **Y Position Offset** (meters)

The left/right misalignment.

```
y_offset_m = FC_local_y - SLAM_rotated_y
```

**Example:**
- FC local position: Y = 0.5 m (50cm to the right)
- SLAM raw position: Y = 0.48 m (appears 2cm to the left)
- Y offset: 0.5 - 0.48 = **+0.02 m**

**When stored and applied later:**
- Raw SLAM position (0, 0) → Corrected position: (+0.08, +0.02)

**Impact:** Shifts all SLAM positions rightward by this amount.

---

### 4. **Noise Metrics** (standard deviation)

Calibration also measures **how stable** each offset is:

```
yaw_std_deg = standard_deviation(all sampled yaw offsets)
x_std_m = standard_deviation(all sampled x offsets)
y_std_m = standard_deviation(all sampled y offsets)
```

**Interpretation:**
- `yaw_std_deg = 1.5°` → Yaw offset varies ±1.5°, very stable ✓
- `x_std_m = 0.15 m` → Position offset varies ±15cm, acceptable ✓
- `x_std_m = 0.5 m` → Position offset varies ±50cm, noisy ✗

**Why it matters:**
- Low noise = offset is reliable
- High noise = offset may not help; underlying sensor is unstable

---

## Stationary Calibration

### When to Use It

**Best for:**
- Ground testing (drone on bench, no flight)
- Validating SLAM/VIO is working at all
- Getting a quick baseline offset before flight
- Testing in a lab with good GPS or local position

**Not suitable for:**
- Correcting mounting errors mid-flight
- Real-time calibration during operation

### How It Works

The `stationary_slam_calibrate.py` script:

1. **Stops conflicting services**
   - Stops `intellisense_slam_bridge.service` if it is active
   - This frees the RealSense/VIO pipeline for calibration

2. **Checks required sensors**
   - MAVLink heartbeat and FC attitude telemetry
   - RealSense/VIO frames and timestamps
   - IMU stream and stability
   - Rangefinder height and noise

3. **Resets local VIO origin**
   - Uses the same VIO backend as `run_local_vio.py`
   - Applies the available reset logic before measuring drift

4. **Collects stationary samples**
   - Samples VIO at ~15 Hz for 25 seconds by default
   - Uses rangefinder height when available
   - Measures stationary drift, pose noise, timestamp freshness, and height scale

5. **Builds offset measurements**
   - For each stable SLAM pose, compare to FC yaw and local XY reference when available
   - Calculate yaw offset, X offset, and Y offset

6. **Pass/Fail Check**
   ```
   PASS if:
     - MAVLink heartbeat is present
     - IMU is stable
     - rangefinder is valid and not noisy
     - VIO timestamps are not frozen
     - VIO quality is high enough
     - stationary drift <= configured limit
     - pose noise <= configured limit
   
   FAIL otherwise (reason reported)
   ```

7. **Save and restart**
   - If PASS: write `runtime/slam_calibration.json`
   - Includes offsets, noise levels, sample count, timestamp
   - Restarts the flight VIO service after calibration

### Example Output

```
2026-04-25 14:30:05 | stage=vio mode=BRAKE armed=no rangefinder=0.22m vio=ok+imu+rng/q78 imu=stable mavlink=ok
  Stationary drift detected: 3.2 cm over 15.0 seconds
2026-04-25 14:30:18 | stage=complete mode=BRAKE armed=no rangefinder=0.22m vio=ok+imu+rng/q81 imu=stable mavlink=ok
  Calibration passed. Saved profile to runtime/slam_calibration.json.
2026-04-25 14:30:19 | stage=complete mode=BRAKE armed=no rangefinder=0.22m vio=ok+imu+rng/q81 imu=stable mavlink=ok
  System is flight ready.
```

**Interpretation:**
- ✓ Yaw noise (0.89°) is very small → heading is stable
- ✓ Position noise (8-8.3cm) is good → position is stable
- ✓ Drift (8.9cm) is small, as expected while stationary
- ✓ Rate (14.5 Hz) is healthy
- → **Safe to use this calibration**

### Data Quality Checks

**Stationary calibration is robust to:**
- Camera FOV changing (autofocus, exposure)
- Small lighting variations
- Minor vibration or wind

**Stationary calibration is NOT robust to:**
- Missing depth data (if VIO needs depth)
- Severe motion blur (if drone moved during collection)
- A rangefinder that is reporting false height while the drone is still

---

## BRAKE Mode Calibration

### When to Use It

**Best for:**
- Supervised GPS-denied calibration using Brake mode as the safety envelope
- Confirming VIO, IMU, rangefinder, MAVLink, RC link, and FC health before PosHold
- Using rangefinder height as the primary altitude reference during calibration
- Running a gentle, bounded pitch/roll/yaw calibration sequence after pilot takeoff

**Use case scenario:**
1. Switch to BRAKE mode
2. If disarmed, the bridge announces that it is waiting for arm
3. Arm in BRAKE while on the ground
4. The bridge runs prechecks and warns that the calibration takeoff sequence is active
5. Pilot manually takes off; the Jetson does **not** command takeoff
6. At about 5m AGL by rangefinder, the bridge announces hold-for-calibration
7. The bridge runs gentle pitch, roll, and yaw calibration stages
8. If stable, it saves calibration and requests RTL
9. If unsafe, it stops calibration and requests the configured fallback mode

### How It Works

The SLAM bridge's built-in calibration (_not_ the stationary script):

1. **Detect BRAKE Mode Entry**
   - Announces: "Brake mode: SLAM calibration fused with Brake mode is active."
   - Resets calibration state

2. **Check Preconditions**
   - Vehicle must be **armed**
   - FC attitude available
   - Rangefinder healthy
   - MAVLink heartbeat present
   - RC link active
   - EKF/FC status text not reporting blocking failures
   - SLAM/VIO pose quality good (≥55/100 by default)

3. **Ground armed in BRAKE**
   - Announces: "Armed in Brake mode. SLAM calibration takeoff sequence active."
   - Repeats a short warning beep while waiting on the ground
   - Does not command takeoff

4. **Pilot takeoff and rangefinder target**
   - Waits for the pilot to fly to the configured rangefinder height
   - Default target is 5.0m AGL with 0.35m tolerance
   - Announces: "Reached 5 meters by rangefinder. Holding altitude for SLAM calibration."

5. **Axis stages**
   - Announces pitch, roll, and yaw stage start/complete
   - Announces: "SLAM calibration active." every 10 seconds
   - Monitors rangefinder, RC link, VIO tracking, MAVLink, mode, drift, and timeouts
   - Optional tiny pitch/roll/yaw nudges exist but are disabled by default

6. **Build Profile**
   - After all stages, check noise/stability
   - If stable (yaw_std ≤ 10°, xy_std ≤ 0.75m):
     - Save `runtime/slam_calibration.json`
     - Announce: "Calibration successful: SLAM PosHold calibration complete. Initiating RTL."
     - Request RTL if RTL is available and final health checks pass
   - If unsafe:
     - Announce: "Calibration failed: not finished. Reason: <reason>"
     - Stop calibration and request the configured fallback mode

### Key Differences from Stationary

| Aspect | Stationary | BRAKE Mode |
|---|---|---|
| **Drone armed?** | No (ground bench) | Yes (armed in BRAKE) |
| **GPS used?** | Not required for basic health checks | Not required by the calibration gate |
| **Flight modes** | No mode changes required | Must be in BRAKE mode |
| **Duration** | 25 seconds by default | Stage-based; timeout protected |
| **FC reference** | Attitude, rangefinder, optional local position | Attitude, rangefinder, optional local position |
| **Triggering** | Manual script run | Automatic on BRAKE mode entry |
| **Failure handling** | Immediate FAIL report | Fail-closed with fallback mode |
| **GCS feedback** | One-time pass/fail | Continuous status updates |
| **Safety** | No arming/mode changes | No auto-takeoff; stops on mode/RC/range/VIO faults |

### Example GCS Sequence

```
[Pilot switches to BRAKE mode]
GCS: "Brake mode: SLAM calibration fused with Brake mode is active."

GCS: "Brake mode detected. Waiting for arm to start SLAM calibration."
[Pilot arms]

GCS: "Armed in Brake mode. SLAM calibration takeoff sequence active."
[Pilot manually takes off and climbs]

GCS: "Reached 5 meters by rangefinder. Holding altitude for SLAM calibration."
GCS: "Pitch axis calibration started."
GCS: "SLAM calibration active."
GCS: "Pitch axis calibration complete."
GCS: "Roll axis calibration started."
GCS: "Roll axis calibration complete."
GCS: "Yaw axis calibration started."
GCS: "Yaw axis calibration complete."
GCS: "Calibration successful: SLAM PosHold calibration complete. Initiating RTL."
```

Failure example:

```
GCS: "Calibration failed: not finished. Reason: SLAM/VIO lost tracking"
```

---

## Comparing the Two Approaches

### Stationary Calibration

**Pros:**
- No flight required
- No arming required
- Quick validation (25 seconds by default)
- Great for lab/bench testing
- Full control over duration and parameters
- Can retry easily

**Cons:**
- Doesn't reflect armed/real-flight dynamics
- Need to manually run the script
- Requires RealSense, IMU, rangefinder, and MAVLink to be available at the same time

**Best for:**
- Validating SLAM health before first flight
- Testing new VIO backends
- Ground-based system validation

---

### BRAKE Mode Calibration

**Pros:**
- Happens during flight operations (when armed)
- Uses real flight controller attitude/rangefinder references
- Automatic on mode entry
- Fail-closed if mode, RC, rangefinder, MAVLink, VIO, or drift becomes unsafe
- Captures armed system behavior

**Cons:**
- Requires drone to be armed
- Requires pilot takeoff to the configured rangefinder height
- Takes flight time and clear airspace to calibrate
- Results only saved if stable

**Best for:**
- Real pre-flight calibration
- Validating alignment before switching to POSHOLD
- Automatic readiness checks

---

## How Offsets Are Applied

When calibration offsets are **loaded and applied** to a raw SLAM pose:

### Step 1: Rotate Position by Yaw Offset

The position XY must be rotated because the coordinate frame is rotated:

```python
rotated_x = raw_x * cos(yaw_offset) - raw_y * sin(yaw_offset)
rotated_y = raw_x * sin(yaw_offset) + raw_y * cos(yaw_offset)
```

**Example:**
- Raw SLAM position: (1.0, 0.0) {1 meter forward}
- Yaw offset: +2°
- After rotation: (0.9994, 0.0349) {slightly forward and right}

### Step 2: Add Position Offsets

After rotation, add the measured offsets:

```python
final_x = rotated_x + x_offset
final_y = rotated_y + y_offset
```

**Example (continuing):**
- Position after rotation: (0.9994, 0.0349)
- Add offsets: x_offset=+0.05, y_offset=+0.02
- Final: (1.0494, 0.0549)

### Step 3: Apply Yaw Offset to Attitude

The quaternion is updated by composing with a rotation around the vertical axis:

```python
rotation_quat = quat_from_yaw(yaw_offset)
final_quat = rotation_quat * original_quat
```

**Result:** Heading is rotated by the offset, attitude is corrected.

### Step 4: Apply to Velocity

Velocity is also rotated (velocities live in the rotated frame):

```python
rotated_vx = raw_vx * cos(yaw_offset) - raw_vy * sin(yaw_offset)
rotated_vy = raw_vx * sin(yaw_offset) + raw_vy * cos(yaw_offset)
```

### Summary

One raw SLAM pose:
```
Input:  x=1.0 m, y=0.0 m, yaw=0.0°, vx=0.5 m/s, vy=0.0 m/s
Calibration: yaw_offset=+2°, x_offset=+0.05 m, y_offset=+0.02 m
Output: x=1.049 m, y=0.055 m, yaw=+2.0°, vx=0.499 m/s, vy=0.017 m/s
```

Every pose sent to the FC gets this transformation applied.

---

## Reading Calibration Results

### When Calibration PASSES

```json
{
  "valid": true,
  "calibration_mode": "STATIONARY",
  "sample_count": 87,
  "yaw_offset_deg": 1.23,
  "x_offset_m": 0.047,
  "y_offset_m": -0.012,
  "yaw_std_deg": 0.89,
  "x_std_m": 0.083,
  "y_std_m": 0.076,
  "range_mean_m": 1.38,
  "saved_at_epoch_s": 1719338820.0
}
```

**What to check:**

| Metric | Good | Acceptable | Poor |
|---|---|---|---|
| `yaw_std_deg` | < 2° | 2°–10° | > 15° |
| `x_std_m` | < 0.10 m | 0.10–0.50 m | > 1.0 m |
| `y_std_m` | < 0.10 m | 0.10–0.50 m | > 1.0 m |
| `sample_count` | > 100 | 50–100 | < 50 |

**Interpretation:**
- Std dev (noise) should be **much smaller** than the offset itself
- If `yaw_offset=+1.2°` and `yaw_std=0.9°`, the std is 75% of the offset → questionable
- If `yaw_offset=+1.2°` and `yaw_std=0.1°`, the std is 8% of the offset → solid

---

### When Calibration FAILS

**Stationary mode:**
```
FAIL: Calibration failed: not finished - position Y noise too high: 1.2m (limit: 1.0m)
```

**Likely causes (in order of probability):**
1. **SLAM tracking unstable** — camera is losing features, lighting poor
2. **GPS signal weak** — if used for FC reference, noisy position
3. **IMU drift** — drone being bumped, vibration during collection
4. **Sensor lag** — FC and SLAM sampling at very different times
5. **Wind** — if drone tethered or outside, wind can move it

**What to do:**
- Try again in a different location (better lighting, less wind)
- Check that SLAM is tracking (run `run_local_vio.py` first to verify)
- Increase collection time → more averaging → lower noise
- Check if rangefinder is seeing the ground (debug depth sensor)

**BRAKE mode:**
```
GCS: "Calibration failed: not finished - vehicle still moving at 0.45 m/s"
GCS: "Calibration failed: not finished - vehicle not level enough roll=+12.5 pitch=-8.2"
GCS: "Calibration failed: not finished - SLAM pose is not stable enough yet"
```

**Common reasons:**
- Rangefinder becomes invalid
- RC link or MAVLink heartbeat drops
- Pilot changes out of BRAKE mode
- Vehicle drifts beyond the configured safe limit
- SLAM quality is poor (features hard to track)
- EKF rejects external navigation data

---

## Best Practices

### Before First Flight

1. **Run stationary calibration**
   ```bash
   python3 scripts/stationary_slam_calibrate.py --config config/autostart.yaml
   ```
   - Check the output for pass/fail
   - If pass: note the offsets and noise levels
   - If fail: diagnose (lighting, vibration, etc.) and retry

2. **Inspect `runtime/slam_calibration.json`**
   ```bash
   cat runtime/slam_calibration.json | jq .
   ```
   - Look at noise levels
   - Are offsets reasonable? (usually < 0.5 m position, < 5° yaw)

3. **Understand the results**
   - If noise is high: SLAM may be tracking poorly → debug VIO further
   - If offsets are large: mounting is unusual → document for future reference

### Before Flight Tests (BRAKE Mode)

1. **Arm in BRAKE mode**
   - Enter Brake mode first
   - The Jetson does not command takeoff
   - Arm only after the ground setup is safe

2. **Take off manually**
   - Wait for the ground armed warning
   - Fly gently to the configured rangefinder height, 5m by default
   - Hold Brake mode unless you need to abort

3. **Check the announcement**
   - If "Calibration successful: SLAM PosHold calibration complete. Initiating RTL." appears, the bridge saved the profile and requested RTL
   - If "Calibration failed: not finished. Reason: ..." appears, fix the named issue before switching to POSHOLD

### In Flight (POSHOLD with SLAM)

- Bridge will NOT recalibrate
- Uses the offsets from the start of this flight (or no calibration if none saved)
- If you want to recalibrate: land, disarm, arm again, wait for BRAKE calibration

---

## Summary

**Calibration solves a fundamental problem:** SLAM and FC coordinate frames start misaligned.

- **Stationary calibration:** Quick ground test, validates SLAM is working, no flight needed
- **BRAKE mode calibration:** Real preflight check, happens while armed with GPS, automatic readiness gating

**What gets corrected:**
- Heading offset (yaw rotation)
- Position offset (XY shift)
- Both are measured, stored, and can be applied to all future poses

**Quality indicators:**
- Std dev (noise) should be small relative to the offset
- Drift should be tiny (stationary) or acceptable (armed)
- Update rate should be healthy (≥5 Hz)

Use both complementary approaches in your workflow, and always inspect the results before trusting them in flight.
