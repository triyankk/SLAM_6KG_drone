#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

exec /usr/bin/python3 -u realsense_optical_flow_to_cube.py \
  --ports /dev/ttyACM0 /dev/ttyACM1 \
  --qgc-udp-forward 127.0.0.1:14550 \
  --qgc-udp-forward 255.255.255.255:14550 \
  --qgc-udp-forward 172.20.127.255:14550 \
  --qgc-udp-forward 127.0.0.1:14551 \
  --qgc-udp-forward 255.255.255.255:14551 \
  --qgc-udp-forward 172.20.127.255:14551 \
  --qgc-udp-bind-port 14555 \
  --disable-gcs-failsafe \
  --flow-csv-log realsense_optical_flow_to_cube_of.csv \
  --post-home-ekf-source-set 2 \
  --gps-input-from-flow \
  --range-source external_or_realsense \
  --external-range-max-m 300 \
  --range-alt-consistency-max-m 2.0 \
  --range-alt-consistency-min-alt-m 1.5 \
  --flow-message rad \
  --flow-max-rate-rad-s 0.8 \
  --inertial-flow-gate-speed-m-s 1.2 \
  --inertial-flow-gate-lean-deg 4.0 \
  --inertial-flow-gate-accel-m-s2 0.7 \
  --flow-failsafe-min-height-m 0.6 \
  --flow-failsafe-min-height-margin-m 0.05 \
  --flow-failsafe-max-height-m 6.5 \
  --flow-failsafe-bad-seconds 0.5 \
  --flow-failsafe-land-seconds 0 \
  --poshold-failsafe-mode LOITER \
  --poshold-failsafe-bad-seconds 0.5 \
  --poshold-ready-min-health-seconds 5.0 \
  --poshold-ekf-variance-max 0.6 \
  --ekf-source-switch-after-sends 30 \
  --imu-port auto \
  --imu-baud auto \
  --external-imu-yaw-mode fc \
  --loiter-observer-interval 20 \
  --legacy-ready-min-health-s 10.0 \
  --legacy-profile-path runtime/legacy_flow_calibration.json \
  --legacy-brake-duration-s 45.0 \
  --legacy-brake-min-samples 160 \
  --crop-fraction 0.8 \
  --downscale 1 \
  --max-features 300 \
  --min-tracks 50 \
  --lk-fb-max-error-px 1.5 \
  --ransac-reproj-threshold-px 3 \
  "$@"
