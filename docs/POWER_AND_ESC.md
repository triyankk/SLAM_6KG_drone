# Power and ESC Acceptance

## What the ESC Change Improves

Replacing one shared 55 A four-in-one ESC with four individual 40 A ESCs removes
a shared thermal and current bottleneck. With the observed motor current around
11 A and a conservative 15 A upper estimate per motor, each 40 A unit has useful
nominal current margin.

This is a strong hardware improvement, but it does not prove that earlier
crashes are solved. Ratings depend on voltage, cooling, firmware, switching
frequency, motor timing, wiring, and burst duration. Loss of control can also
come from attitude tuning, CG, vibration, motor saturation, battery sag,
connector resistance, desync, estimator errors, or bad mode transitions.

## Wiring Gate

- One ESC per motor, no shared signal ground omission.
- Signal ground connected between Cube and every ESC.
- Correct battery voltage rating.
- Wire and connector current ratings exceed measured demand.
- Power joints mechanically supported and insulated.
- No motor wire can touch a propeller or frame edge.
- ESCs mounted in comparable airflow.
- Cube and Jetson power cannot brown out when motors step load.
- Current sensor measures total battery current and voltage at the battery bus.

## Bench Electrical Test

Use a restrained thrust stand and appropriate propeller safety equipment.

For each motor:

1. Verify direction without a propeller.
2. Fit the correct propeller only on the secured test stand.
3. Increase throttle in steps and record current, voltage, thrust, RPM if
   available, ESC temperature, and motor temperature.
4. Hold the expected hover load long enough to reach thermal equilibrium.
5. Apply short bursts to the maximum intended load.
6. Stop for noise, roughness, sync loss, abnormal heat, smell, or voltage drop.

Then test all four together on a restrained airframe at hover-equivalent load.
Individual tests do not expose total battery, connector, and power-distribution
limits.

## Acceptance Data

Record:

- Current per motor and total current.
- Battery voltage at rest, hover-equivalent load, and burst load.
- Voltage at Cube and Jetson rails during load steps.
- ESC temperature before, during, and after the test.
- Motor temperature.
- Thrust at each current point.
- Any desync or audible discontinuity.
- Cube `BAT`, `POWR`, `RCOU`, `RATE`, `ATT`, `VIBE`, and motor/thrust-loss
  messages.

Do not accept the system merely because each ESC is rated 40 A. Accept it when
the complete installed power train repeats the required load without
overtemperature, sag, desync, resets, or output saturation.

## First Flight Review

After every early flight check:

- Minimum battery voltage and maximum current.
- Cube and Jetson reset/brownout evidence.
- Motor outputs for persistent saturation or one-motor imbalance.
- Vibration clipping and harmonic peaks.
- Desired versus achieved roll and pitch.
- Thrust-loss and failsafe messages.
- ESC and motor temperatures immediately after landing.

Three clean flights are the minimum gate before attributing the earlier crashes
to the removed four-in-one ESC.

