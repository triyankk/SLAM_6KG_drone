#!/usr/bin/env python3
"""
Stationary SLAM/VIO calibration script.

Run this script while the drone is sitting still in one place (no flight, no movement required).
It will measure the alignment between SLAM/VIO output and flight controller reference,
and save the calibration profile for later use by the SLAM bridge.

Usage:
    python3 scripts/stationary_slam_calibrate.py --ports /dev/ttyACM1 /dev/ttyACM0 --duration 30
"""

import argparse
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from intellisense_slam.calibration import (
    CalibrationProfile,
    circular_mean_deg,
    circular_std_deg,
    linear_std,
    pose_yaw_deg,
    rotate_xy,
    save_calibration_profile,
    wrap_angle_deg,
)
from intellisense_slam.external_imu import Im10aReader
from intellisense_slam.fc_config import (
    FlightControllerTelemetry,
    configure_telemetry_streams,
    drain_fc_telemetry,
    gps_reference_valid,
    rangefinder_height_valid,
    send_gcs_event,
    send_statustext,
)
from intellisense_slam.mavlink_bridge import connect_to_cube
from intellisense_slam.pose_sources import make_pose_source


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate SLAM/VIO alignment while the drone is stationary on the ground."
    )
    parser.add_argument("--ports", nargs="+", default=["/dev/ttyACM1", "/dev/ttyACM0"])
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source", choices=["vio", "hover"], default="vio")
    parser.add_argument("--duration", type=float, default=30.0, help="Calibration duration in seconds")
    parser.add_argument("--imu-port", default="auto")
    parser.add_argument("--imu-baud", default="auto")
    parser.add_argument("--output", default="runtime/slam_calibration.json")
    parser.add_argument("--imu", choices=["on", "off"], default="on")
    return parser.parse_args()


def connect_to_cube_with_retry(ports: list[str], baud: int, max_retries: int = 3) -> object:
    """Connect to Cube with retries."""
    for attempt in range(max_retries):
        try:
            connection = connect_to_cube(ports, baud)
            return connection
        except Exception as exc:  # noqa: BLE001
            if attempt < max_retries - 1:
                print(f"Connection attempt {attempt + 1} failed: {exc}; retrying in 2s...")
                time.sleep(2.0)
            else:
                raise RuntimeError(f"Failed to connect to Cube after {max_retries} attempts") from exc


def open_imu_with_retry(imu_port: str, imu_baud: str) -> object:
    """Open IMU reader if available."""
    if imu_port.lower() == "off" or imu_port == "":
        return None
    try:
        return Im10aReader(port=imu_port, baud=imu_baud if imu_baud else None, verbose=False)
    except Exception as exc:  # noqa: BLE001
        print(f"Warning: could not open IMU: {exc}")
        return None


def run_stationary_calibration(args):
    """Run stationary calibration."""
    print(
        f"Stationary SLAM calibration\n"
        f"  ports={args.ports}\n"
        f"  baud={args.baud}\n"
        f"  source={args.source}\n"
        f"  duration={args.duration}s\n"
        f"  output={args.output}"
    )

    # Connect to Cube
    print("\nConnecting to flight controller...")
    connection = connect_to_cube_with_retry(args.ports, args.baud)
    send_statustext(connection.master, "Stationary SLAM calibration started")
    send_gcs_event(connection.master, "Stationary SLAM calibration started")
    print(f"Connected to Cube on {connection.port} @ {connection.baud} baud")

    # Open IMU if requested
    imu_reader = None
    if args.imu == "on":
        print("Opening external IMU...")
        imu_reader = open_imu_with_retry(args.imu_port, args.imu_baud)
        if imu_reader is not None:
            print(f"IMU opened on {imu_reader.port} @ {imu_reader.baud} baud")
        else:
            print("Warning: IMU not available; continuing without external IMU")

    # Configure telemetry
    configure_telemetry_streams(connection.master)

    # Open VIO/pose source
    print(f"Opening {args.source} pose source...")
    pose_source = make_pose_source(args.source, "")

    fc_state = FlightControllerTelemetry()
    calibration_start_s = time.time()
    calibration_duration_s = args.duration

    # Collection buffers
    collected_yaw_offsets_deg = []
    collected_x_offsets_m = []
    collected_y_offsets_m = []
    collected_ranges_m = []
    collected_poses = []
    collected_fc_positions = []
    last_gcs_update_s = calibration_start_s

    print(f"\nCalibration active for {calibration_duration_s} seconds")
    print("Position data collection started...\n")
    send_statustext(connection.master, "Stationary calibration active")
    send_gcs_event(connection.master, "Stationary calibration active")

    try:
        while time.time() - calibration_start_s < calibration_duration_s:
            # Drain FC telemetry
            drain_fc_telemetry(connection.master, fc_state, None)

            # Get current SLAM/VIO pose
            pose = pose_source.sample()
            if pose is None or pose.tracking_state != "ok":
                time.sleep(0.05)
                continue

            # Get FC reference (prefer local position for better precision)
            if fc_state.local_position is None:
                time.sleep(0.05)
                continue

            fc_x_m = float(getattr(fc_state.local_position, "x", 0.0))
            fc_y_m = float(getattr(fc_state.local_position, "y", 0.0))
            fc_yaw_deg = math.degrees(float(getattr(fc_state.attitude, "yaw", 0.0))) if fc_state.attitude else 0.0
            range_m = float(fc_state.rangefinder_distance_m or 0.0)

            # Collect offset measurement
            pose_yaw = pose_yaw_deg(pose)
            yaw_offset = wrap_angle_deg(fc_yaw_deg - pose_yaw)
            rotated_x_m, rotated_y_m = rotate_xy(pose.x_m, pose.y_m, yaw_offset)
            x_offset = fc_x_m - rotated_x_m
            y_offset = fc_y_m - rotated_y_m

            collected_yaw_offsets_deg.append(yaw_offset)
            collected_x_offsets_m.append(x_offset)
            collected_y_offsets_m.append(y_offset)
            if range_m > 0.0:
                collected_ranges_m.append(range_m)
            collected_poses.append(pose)
            collected_fc_positions.append((fc_x_m, fc_y_m, fc_yaw_deg))

            # Periodic GCS update every 3 seconds
            now_s = time.time()
            if now_s - last_gcs_update_s >= 3.0:
                remaining_s = calibration_duration_s - (now_s - calibration_start_s)
                send_gcs_event(connection.master, f"Stationary calibration active ({remaining_s:.1f}s remaining)")
                last_gcs_update_s = now_s

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user")
        send_gcs_event(connection.master, "Stationary calibration cancelled", severity=4)
        return False
    except Exception as exc:  # noqa: BLE001
        print(f"\nCalibration error: {exc}")
        send_gcs_event(connection.master, f"Stationary calibration error: {exc}", severity=4)
        return False
    finally:
        if imu_reader is not None:
            imu_reader.close()

    print(f"\n\nCalibration collection complete: {len(collected_yaw_offsets_deg)} samples")

    # Analyze collected data
    if len(collected_yaw_offsets_deg) < 10:
        failure_reason = f"Not enough samples: {len(collected_yaw_offsets_deg)} (need at least 10)"
        print(f"FAIL: {failure_reason}")
        send_gcs_event(connection.master, f"Stationary calibration failed: {failure_reason}", severity=4)
        return False

    # Compute statistics
    yaw_mean = circular_mean_deg(collected_yaw_offsets_deg)
    yaw_std = circular_std_deg(collected_yaw_offsets_deg, yaw_mean)
    x_mean = sum(collected_x_offsets_m) / len(collected_x_offsets_m)
    x_std = linear_std(collected_x_offsets_m)
    y_mean = sum(collected_y_offsets_m) / len(collected_y_offsets_m)
    y_std = linear_std(collected_y_offsets_m)

    # Check for excessive drift by looking at position change over time
    if len(collected_poses) >= 2:
        first_pose = collected_poses[0]
        last_pose = collected_poses[-1]
        pose_drift_x = abs(last_pose.x_m - first_pose.x_m)
        pose_drift_y = abs(last_pose.y_m - first_pose.y_m)
        pose_drift = math.sqrt(pose_drift_x * pose_drift_x + pose_drift_y * pose_drift_y)
    else:
        pose_drift = 9999.0

    # Calculate SLAM update rate
    slam_rate_hz = len(collected_poses) / calibration_duration_s if calibration_duration_s > 0 else 0.0

    # Failure checks
    failure_reason = None
    if yaw_std > 15.0:
        failure_reason = f"yaw noise too high: {yaw_std:.1f} deg (limit: 15 deg)"
    elif x_std > 1.0:
        failure_reason = f"position X noise too high: {x_std:.3f} m (limit: 1.0 m)"
    elif y_std > 1.0:
        failure_reason = f"position Y noise too high: {y_std:.3f} m (limit: 1.0 m)"
    elif pose_drift > 0.5:
        failure_reason = f"SLAM drift too high: {pose_drift:.3f} m (limit: 0.5 m)"
    elif slam_rate_hz < 5.0:
        failure_reason = f"SLAM update rate too low: {slam_rate_hz:.1f} Hz (need: 5+ Hz)"

    if failure_reason is not None:
        print(f"\nFAIL: {failure_reason}")
        print("\nDebug info:")
        print(f"  Yaw offset: {yaw_mean:+.2f} deg +/- {yaw_std:.2f} deg")
        print(f"  X offset: {x_mean:+.3f} m +/- {x_std:.3f} m")
        print(f"  Y offset: {y_mean:+.3f} m +/- {y_std:.3f} m")
        print(f"  SLAM drift: {pose_drift:.3f} m")
        print(f"  SLAM rate: {slam_rate_hz:.1f} Hz")
        send_gcs_event(
            connection.master,
            f"Stationary calibration failed: {failure_reason}",
            severity=4,
        )
        return False

    # Success: build and save profile
    print("\nPASS: Calibration data is stable and usable")
    print("\nCalibration results:")
    print(f"  Samples: {len(collected_yaw_offsets_deg)}")
    print(f"  Yaw offset: {yaw_mean:+.2f} deg +/- {yaw_std:.2f} deg")
    print(f"  X offset: {x_mean:+.3f} m +/- {x_std:.3f} m")
    print(f"  Y offset: {y_mean:+.3f} m +/- {y_std:.3f} m")
    print(f"  SLAM drift: {pose_drift:.3f} m")
    print(f"  SLAM rate: {slam_rate_hz:.1f} Hz")
    if collected_ranges_m:
        range_mean = sum(collected_ranges_m) / len(collected_ranges_m)
        print(f"  Rangefinder height: {range_mean:.2f} m")
    else:
        range_mean = 0.0

    profile = CalibrationProfile(
        valid=True,
        calibration_mode="STATIONARY",
        sample_count=len(collected_yaw_offsets_deg),
        yaw_offset_deg=yaw_mean,
        x_offset_m=x_mean,
        y_offset_m=y_mean,
        yaw_std_deg=yaw_std,
        x_std_m=x_std,
        y_std_m=y_std,
        range_mean_m=range_mean,
        saved_at_epoch_s=time.time(),
    )

    # Save calibration
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    save_calibration_profile(output_path, profile)
    print(f"\nCalibration saved to: {output_path}")

    send_statustext(connection.master, "Stationary calibration completed")
    send_gcs_event(connection.master, "Stationary calibration completed")

    return True


def main():
    args = parse_args()
    try:
        success = run_stationary_calibration(args)
        sys.exit(0 if success else 1)
    except Exception as exc:  # noqa: BLE001
        print(f"\nFatal error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
