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

1. **Connect to Flight Controller**
   - Establishes MAVLink link
   - Reads FC telemetry (position, attitude, rangefinder, GPS)
   - Does NOT arm or change flight modes

2. **Collect SLAM Poses**
   - Opens the VIO pose source (same backend as `run_local_vio.py`)
   - Samples the pose at ~15 Hz for 30 seconds (default)
   - Each sample represents one moment in time

3. **Build Offset Measurements**
   - For each SLAM pose, compare to FC's **current** reference
   - Calculate yaw offset, X offset, Y offset
   - Store all measurements in arrays

4. **Analyze Stability**
   - Average all offset measurements → **mean offset** (final calibration value)
   - Calculate standard deviation → **noise level** (confidence indicator)
   - Measure peak-to-peak drift → **drift** (should be small while stationary)

5. **Pass/Fail Check**
   ```
   PASS if:
     - yaw_std <= 15°
     - x_std <= 1.0 m
     - y_std <= 1.0 m
     - pose_drift <= 0.5 m (stationary, so should be tiny)
     - slam_rate >= 5 Hz (enough samples)
   
   FAIL otherwise (reason reported)
   ```

6. **Save Result**
   - If PASS: write `runtime/slam_calibration.json`
   - Includes offsets, noise levels, sample count, timestamp

### Example Output

```
PASS: Calibration data is stable and usable

Calibration results:
  Samples: 87
  Yaw offset: +1.23 deg +/- 0.89 deg      ← mean ± std
  X offset: +0.047 m +/- 0.083 m
  Y offset: -0.012 m +/- 0.076 m
  SLAM drift: 0.089 m                     ← position change while stationary
  SLAM rate: 14.5 Hz
  Rangefinder height: 1.38 m

Calibration saved to: runtime/slam_calibration.json
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
- GPS outage (if relying on GPS for FC reference)
- Missing depth data (if VIO needs depth)
- Severe motion blur (if drone moved during collection)

---

## BRAKE Mode Calibration

### When to Use It

**Best for:**
- Pre-flight calibration while FC holds position with GPS
- Calibrating with the drone **armed but stationary**
- Using the Cube's **attitude and local position** as reference (very trustworthy)
- Detecting if SLAM drifts once armed

**Use case scenario:**
1. Arm the drone in BRAKE mode (FC holds position)
2. Wait for GPS lock and attitude to stabilize
3. Trigger calibration → captures 12 seconds of stationary data
4. If stable → saves calibration
5. If not → announcements tell you what's wrong

### How It Works

The SLAM bridge's built-in calibration (_not_ the stationary script):

1. **Detect BRAKE Mode Entry**
   - Announces: "Brake mode: calibration initiated"
   - Resets calibration state

2. **Check Preconditions**
   - Vehicle must be **armed**
   - GPS must be healthy (fix type ≥3, ≥8 satellites)
   - FC local position available
   - FC attitude available
   - Rangefinder healthy
   - SLAM pose quality good (≥55/100)

3. **While Preconditions Met**
   - Accumulate offset samples at each pose update
   - Announces: "Calibration active" (once, then every 10 seconds)
   - Collects for 12 seconds (configurable)

4. **If Preconditions Not Met**
   - Announces specific reason: "GPS reference not healthy", "vehicle not level", etc.
   - Waits for conditions to improve
   - Does NOT fail; keeps trying

5. **Build Profile**
   - After 12 seconds, check noise/stability
   - If stable (yaw_std ≤ 10°, xy_std ≤ 0.75m):
     - Save `runtime/slam_calibration.json`
     - Announce: "Calibration completed for SLAM PosHold, initiating RTL"
     - Auto-switch to RTL
   - If noisy:
     - Announce: "Calibration sample was noisy; holding BRAKE and retrying"
     - Restart collection

### Key Differences from Stationary

| Aspect | Stationary | BRAKE Mode |
|---|---|---|
| **Drone armed?** | No (ground bench) | Yes (armed in BRAKE) |
| **GPS used?** | Optional; can ignore GPS | Required; uses GPS for positioning |
| **Flight modes** | No mode changes required | Must be in BRAKE mode |
| **Duration** | 30 seconds (configurable) | 12 seconds (configurable) |
| **FC reference** | Local position + attitude | Local position + attitude (same) |
| **Triggering** | Manual script run | Automatic on BRAKE mode entry |
| **Failure handling** | Immediate FAIL report | Waits, announces preconditions |
| **GCS feedback** | One-time pass/fail | Continuous status updates |
| **Safety** | No arming/mode changes | Uses existing BRAKE hold |

### Example GCS Sequence

```
[Pilot switches to BRAKE mode]
FC: "Brake mode: calibration initiated"

[Vehicle armed and settling]
FC: "Calibration waiting: drone must be armed"
[Pilot arms]

[GPS acquiring lock]
FC: "Calibration waiting: GPS reference not healthy enough"

[After ~10 seconds, GPS lock achieved]
FC: "Calibration active"

[After another 10 seconds]
FC: "Calibration active"

[After 12 total seconds of collection]
FC: "Calibration completed for SLAM PosHold, initiating RTL"
FC: "RTL command accepted after calibration"
```

---

## Comparing the Two Approaches

### Stationary Calibration

**Pros:**
- No flight required
- No arming required
- Quick validation (30 seconds)
- Great for lab/bench testing
- Full control over duration and parameters
- Can retry easily

**Cons:**
- Requires some GPS or FC local position
- Doesn't reflect armed/real-flight dynamics
- Need to manually run the script
- Results not automatically used

**Best for:**
- Validating SLAM health before first flight
- Testing new VIO backends
- Ground-based system validation

---

### BRAKE Mode Calibration

**Pros:**
- Happens during flight operations (when armed)
- Uses real flight controller reference
- Automatic on mode entry
- Can retry during same flight
- Captures armed system behavior

**Cons:**
- Requires GPS lock first
- Requires drone to be armed
- Takes flight time to calibrate
- Can fail if GPS is weak
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
- Vehicle is swaying or hasn't settled (wind, soft landing)
- Pilot is testing hover stability (need it perfectly level)
- SLAM quality is poor (features hard to track)
- Rangefinder is malfunctioning

---

## Best Practices

### Before First Flight

1. **Run stationary calibration**
   ```bash
   python3 scripts/stationary_slam_calibrate.py --duration 30
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
   - Ensure GPS lock first
   - Vehicle should be level and stationary

2. **Wait for calibration**
   - Bridge will automatically attempt calibration
   - GCS shows "Calibration active" repeatedly
   - Wait 12+ seconds for completion

3. **Check the announcement**
   - If "Calibration completed" → ready for POSHOLD
   - If "Calibration failed: ..." → fix the issue (wind, level, etc.) before switching to POSHOLD

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
