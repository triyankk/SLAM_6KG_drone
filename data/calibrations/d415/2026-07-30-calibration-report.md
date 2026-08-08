# D415 Wall Calibration Check - 2026-07-30

## Setup

- Camera serial: `327322062285`
- Reported camera-to-wall distance: `1.500 m`
- Configured camera position from aircraft CG: `+0.190 m` forward
- Expected transformed CG-to-wall distance: `1.690 m`
- Camera remained fixed and the aircraft remained disarmed.
- No calibration table was written to camera flash.

## Baseline

- Active depth scale: `0.0010000000474974513 m/unit`
- Central 20 percent ROI median: `1.547 m`
- Central ROI temporal range: `1.542 m` to `1.596 m`
- Central ROI valid-pixel fraction: `100 percent`
- Difference from reported camera-to-wall distance: `+0.047 m` (`+3.1 percent`)

The conservative obstacle-sector check reported `0.88 m` because the floor enters
the configured CG-height obstacle band in the lower part of the image. The
central wall ROI confirms that this is foreground geometry, not a depth-scale
failure.

## On-Chip Health Candidate

- Existing-table health result: `-0.210434`
- Project acceptance interpretation: acceptable because absolute health is below
  `0.25`
- Candidate was saved for comparison but was not applied or written.
- Original table SHA-256:
  `11bb46d8dd75f87865372fc7637a71bed43b9513e7cdad6140a5e7ba6a161697`
- Candidate table SHA-256:
  `745a7559694eb2898c42d1db6779f92e2c26ff42c2e634674d4cae610cc8bb68`

## Reversible 1500 mm Tare Evaluation

- Original-table median in High Accuracy mode: `1.548 m`
- Candidate-table median in RAM: `1.550 m`
- Restored-original median: `1.551 m`
- The Tare candidate did not improve the measured error.
- The original table was restored in RAM before the camera was closed.
- The candidate was not written to flash.

## 2500 mm Confirmation

- The upward magenta arrow on the wall target was visually confirmed.
- Reported left-imager-to-wall distance: `2.500 m`
- Expected transformed CG-to-wall distance: `2.690 m`
- Runtime-profile central 20 percent ROI median: `2.533 m`
- Absolute error: `+0.033 m` (`+1.32 percent`)
- Central ROI valid-pixel fraction: `100 percent`
- Target-plane fit RMSE: `0.0073 m`
- Target-plane angle from the optical axis: `3.26 degrees`
- White-wall on-chip health result: `-0.225631`

Projector-off textured mode rejected the mostly plain target with a low fill
factor. White-wall mode with the projector enabled completed successfully. The
On-Chip candidate was saved but not applied.

At the runtime `640x480` profile, the existing custom preset measured `2.533 m`;
the High Accuracy preset measured `2.539 m` with effectively the same fill rate.
The original advanced-settings JSON was restored with an exact SHA-256 match.

The `2500 mm` Tare candidate reported health `0.004459`, but the SDK did not load
the candidate through the volatile table API: the active table hash remained the
original hash. The candidate was not written to flash because the existing table
already passed and no runtime improvement was demonstrated.

## Status

The unchanged factory table passed at both measured distances within the
project's `0.05 m` tolerance. `camera_intrinsics_verified` is now true.
`camera_to_body_extrinsics_verified` remains false until the complete
CG-referenced direction and translation sequence passes.
