# JT16 and D415 Calibration

This procedure separates vendor calibration from aircraft mounting calibration.
Props stay removed, the Cube stays disarmed, and obstacle output stays in shadow
mode throughout.

## Definitions

- Sensor intrinsics convert raw measurements into sensor-frame geometry.
- Mount extrinsics rotate and translate that geometry into aircraft body FRD.
- All project obstacle distances are horizontal distances from the aircraft CG
  after mount extrinsics are applied.
- The hard planning boundary is `1.50 m` from the CG. It is not measured from a
  sensor housing or from the nearest propeller.

Use a rigid, matte, flat target at least `1 m x 1 m`, a laser distance meter,
and a level aircraft stand. Mark the aircraft CG on the floor with a plumb line.
Measure horizontally from that CG mark to the target plane.

Only one process can own the D415 or JT16 stream. Stop the visualizer and flight
logger before running a calibration check.

Run the complete JT16 cardinal sequence in one guided terminal:

```bash
./optflow jt16-calibrate
```

The wizard requires the Cube to remain disarmed and obstacle output to remain
in shadow. It waits 10 seconds before each measurement. After a direction
passes, it sends one short `PLAY_TUNE` beep directly over MAVLink, shows the
next direction, and starts the next 10-second positioning countdown. A failed
direction stops the sequence without a completion beep. The command temporarily
stops the automatic flight logger and restores it on every exit path. Use
`--start left` when the wall is already on the aircraft's left.

## D415 Intrinsics and Depth Scale

1. Record the current factory calibration before changing anything:

   ```bash
   mkdir -p data/calibrations/d415
   rs-enumerate-devices -c \
     > data/calibrations/d415/factory-calibration.txt
   ```

2. Open `rs-depth-quality`. Warm the camera for several minutes, point it
   normally at a large flat textured wall, enter the laser-measured wall
   distance, and record depth accuracy and fill rate at `640x480`.
3. If accuracy is already acceptable, do not rewrite the camera calibration.
4. If it is not acceptable, use RealSense Viewer self-calibration. Run On-Chip
   calibration first and Tare second using an accurately measured target.
   Compare the reported health and depth-quality result before applying the new
   table.
5. Repeat the depth-quality test at approximately `1.5 m`, `2.5 m`, and, when
   practical, `3.0 m`.
6. Save another `rs-enumerate-devices -c` snapshot after any accepted change.

The runtime reads intrinsics and depth scale from the active RealSense profile.
Do not replace those values with generic D415 numbers.

The 2026-07-30 checks found `1.547 m` at a measured `1.50 m` and `2.533 m` at a
measured `2.50 m`. Both central-ROI errors are within `0.05 m`, fill was 100
percent, and white-wall on-chip health was acceptable. The upward magenta arrow
in the second target image was visually confirmed. Reversible On-Chip, Tare, and
runtime-preset candidates did not demonstrate an improvement, so the original
factory table was retained and `camera_intrinsics_verified` was set true. See
`../data/calibrations/d415/2026-07-30-calibration-report.md`.

Official references:

- [RealSense D400 self-calibration](https://dev.realsenseai.com/docs/intel-realsense-self-calibration-for-d400-series-depth-cameras/)
- [RealSense calibration tools](https://dev.realsenseai.com/docs/calibration/)

## JT16 Channel Correction

The SDK example correction file is not proof of calibration. Every JT16 channel
has unit-specific horizontal and vertical offsets.

1. Export the angle correction file from this lidar using PandarView 2, the
   supported API, or a file supplied by Hesai for this unit.
2. Store it under `data/calibrations/jt16/` and record its SHA-256 hash.
3. Point `sensors.lidar.correction_file` in `config/system.yaml` at that file.
4. Leave `correction_verified: false` until the point cloud and measured-target
   checks below pass. The bridge loads the configured file at startup.
5. In a clear room, inspect a large flat wall. All 16 channel traces should
   coincide with one plane without doubled, bowed, or vertically separated
   edges.

Hesai states that the accurate per-channel azimuth and elevation values are in
the individual lidar's angle correction file. The file can be exported through
PandarView 2 or obtained from Hesai:

- [JT16 user manual](https://www.hesaitech.com/wp-content/uploads/2026/01/JT16_User_Manual_J03-en-260120.pdf)
- [JT16 downloads and utilities](https://www.hesaitech.com/product_downloads/jt16/)

Validate the configured file against this physical unit with a large forward
wall and a visible, level floor. Level the aircraft, place the JT16 center
`2.50 m` from the wall, remove nearby clutter, and run:

```bash
./optflow jt16-plane --distance 2.50 --duration 10
```

The command requires a disarmed Cube, temporarily stops and restores the
automatic flight logger, and captures native SDK XYZ plus ring IDs. It robustly
fits the wall and floor, checks all 16 ring traces, measures plane residuals and
body-normal error, then stores compressed raw points and a digest-checked report
under `data/calibrations/jt16/planes/`. It sends one completion beep when the
aircraft may be touched. The report proposes the correction and extrinsic flags
but never changes the CSV, configuration, Cube, or obstacle-output state.

## Mount Extrinsics to the CG

Current measured translations in body FRD are:

- D415: `(forward=0.19, right=0.00, down=0.10) m`
- JT16: `(forward=0.00, right=0.00, down=-0.10) m`
- IM10A: `(forward=0.08, right=0.00, down=-0.09) m`

For the current forward-wall check, the wall is `1.50 m` from the D415 face.
Because the D415 is `0.19 m` forward of the CG, use `1.69 m` as the measured
horizontal CG-to-wall target and isolate the camera:

```bash
./optflow obstacle-check \
  --no-lidar \
  --duration 8 \
  --target-distance 1.69 \
  --target-angle 0 \
  --target-tolerance 0.08
```

The D415 raw center depth should be approximately `1.50 m`; the transformed
project distance should be `1.69 m`. To test the hard boundary itself, put the
wall `1.50 m` from the CG, which is approximately `1.31 m` from the D415 face.

Check the JT16 independently in all horizontal directions:

```bash
./optflow obstacle-check --no-depth --duration 8 \
  --target-distance 1.69 --target-angle 0
./optflow obstacle-check --no-depth --duration 8 \
  --target-distance 1.69 --target-angle 90
./optflow obstacle-check --no-depth --duration 8 \
  --target-distance 1.69 --target-angle 180
./optflow obstacle-check --no-depth --duration 8 \
  --target-distance 1.69 --target-angle -90
```

Body FRD angles are `0` forward, `+90` right, `180` rear, and `-90` left.
Repeat at `3.00 m`. For the D415, also move the target within its field of view
to verify that right is positive and that pitch does not move a vertical wall
systematically nearer or farther.

Adjust only the measured translation and rotation entries in
`config/system.yaml`, then repeat the complete sequence. An initial check passes
within `0.08 m`; tighten this to `0.05 m` before active avoidance. A wrong
direction, missing target, doubled plane, or inconsistent error at different
distances is a failure even if one sample passes.

The 2026-07-30 JT16 forward-wall check used Hesai's published
`JT16_sample_angle-2.csv` (`SHA-256
8dd90c9ad1e4ab22ce7c95777b311b0e6bf11e7c30818552ca19b7037de3dbb0`).
The target was reported as `2.50 m` from the centered JT16 with approximately
`0.06 m` tape-measure uncertainty. The wall initially appeared at body bearing
`180 deg`; setting the provisional lidar mount yaw to `180 deg` moved it to
the correct forward sector. The corrected center sector measured `2.46 m` and
the conservative neighboring-sector value was `2.44 m`, so the forward check
was accepted within the stated ground-truth uncertainty. This does not verify
the website sample as a unit-specific correction file. After rotating the
aircraft `90 deg` counterclockwise, the wall appeared in the expected body-right
`+90 deg` sector at `2.44 m`; this independently passed the same uncertainty
window without another transform change. A second `90 deg` counterclockwise
rotation placed the wall in the expected rear `180 deg` sector at `2.44 m`,
again with all 16 rings and no extraction errors. The final left check placed
the wall in the expected `-90 deg` sector; its center measured `2.47 m` and
the conservative neighboring-sector value was `2.45 m`. The four cardinal
checks therefore verify the provisional `180 deg` mount yaw.

The guided 2026-07-30 hard-boundary run then repeated all four directions at
`1.50 m`, with a 10-second positioning countdown and one Cube completion beep
per passed direction. It measured left `1.50 m`, forward `1.47 m`, right
`1.49 m`, and rear `1.48 m`; all passed the `0.08 m` gate. The raw run is
`data/calibrations/jt16/cardinal_runs/20260730T150333Z.json`. This verifies the
horizontal hard-boundary response but does not replace the per-channel plane or
self-return checks required before active avoidance.

The 2026-07-31 native point capture preserved 485,392 points over 51 frames.
Reanalysis with the full JT16 vertical field observed all 16 rings on the
forward wall. The fitted distance was `2.4988 m` against the `2.500 m` target,
plane p95 residual was `0.0257 m`, and the largest absolute per-ring median
residual was `0.0162 m`. The digest-checked report is
`data/calibrations/jt16/planes/20260731T123553Z-wall-floor/reanalysis.json`
(`SHA-256 ddb30e0a15830e5af79cd7c73e7e25bc3ee88a5ffc2f97425851623e22a99e23`).
This validates the configured angle table on this physical unit, so
`correction_verified` is true. The completed four-cardinal run at
`data/calibrations/jt16/cardinal_runs/20260730T150333Z.json` verifies the
horizontal lidar-to-body transform used by the active proximity path.

## Verification Gates

Set these flags to true only after saving the results:

- `camera_intrinsics_verified`
- `camera_to_body_extrinsics_verified`
- `lidar_to_body_extrinsics_verified`
- `sensors.lidar.correction_verified`

Only enabled obstacle sources must pass their full gates. JT16 is active;
D415 remains excluded from obstacle output until its camera-to-body extrinsic
is verified. Calibration by itself does not enable navigation control.
