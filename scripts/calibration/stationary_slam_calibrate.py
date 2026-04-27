#!/usr/bin/env python3
"""
Stationary SLAM/VIO calibration and preflight health check.

Run this while the drone is disarmed, sitting still, and pointed in the
normal takeoff direction. The script stops the flight bridge if it is using
the camera, checks the required sensors, resets the local VIO origin, measures
stationary drift, saves a calibration profile, and restarts the bridge.

Typical use:
    python3 scripts/stationary_slam_calibrate.py --config config/autostart.yaml
"""

import argparse
import math
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.bridge_config import SlamBridgeConfig, load_bridge_config
from slam_core.calibration import (
    CalibrationProfile,
    circular_mean_deg,
    circular_std_deg,
    linear_std,
    pose_yaw_deg,
    rotate_xy,
    save_calibration_profile,
    wrap_angle_deg,
)
from slam_core.external_imu import Im10aReader, apply_imu_sample_to_pose
from slam_core.fc_config import (
    FlightControllerTelemetry,
    configure_telemetry_streams,
    drain_fc_telemetry,
    mavlink_heartbeat_valid,
    rangefinder_height_valid,
    request_active_source_set,
    send_gcs_event,
)
from slam_core.mavlink_bridge import connect_to_cube
from slam_core.pose_sources import make_pose_source


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run stationary SLAM/VIO calibration with sensor health checks."
    )
    parser.add_argument("--config", default="config/autostart.yaml")
    parser.add_argument("--ports", nargs="+")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--source", choices=["vio", "hover"], default=None)
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--warmup-seconds", type=float, default=3.0)
    parser.add_argument("--rate-hz", type=float, default=15.0)
    parser.add_argument("--imu", choices=["on", "off"], default=None)
    parser.add_argument("--imu-port", default=None)
    parser.add_argument("--imu-baud", default=None)
    parser.add_argument("--imu-scan-seconds", type=float, default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--manage-service", choices=["on", "off"], default="on")
    parser.add_argument("--restart-service", choices=["auto", "always", "never"], default="auto")
    parser.add_argument(
        "--conflict-service",
        action="append",
        default=["vio-flight.service", "intellisense_slam_bridge.service"],
        help="systemd service to stop while the camera/VIO pipeline is calibrated.",
    )
    parser.add_argument("--min-samples", type=int, default=80)
    parser.add_argument("--min-pose-quality", type=int, default=45)
    parser.add_argument("--max-stationary-drift-m", type=float, default=0.30)
    parser.add_argument("--max-pose-noise-m", type=float, default=0.18)
    parser.add_argument("--max-yaw-noise-deg", type=float, default=12.0)
    parser.add_argument("--min-range-m", type=float, default=0.05)
    parser.add_argument("--max-stationary-range-m", type=float, default=2.0)
    parser.add_argument("--max-range-noise-m", type=float, default=0.15)
    parser.add_argument("--max-height-error-m", type=float, default=0.75)
    parser.add_argument("--max-imu-gyro-deg-s", type=float, default=5.0)
    parser.add_argument("--max-imu-angle-std-deg", type=float, default=2.0)
    parser.add_argument("--verbose", action="store_true", help="Keep detailed per-step logs enabled.")
    parser.add_argument("--indoor", action="store_true", help="Label this run as indoor/GPS-denied stationary calibration.")
    parser.add_argument("--no-gps", action="store_true", help="Do not require GPS. This is already the default behavior.")
    return parser.parse_args()


def resolve_config(args) -> SlamBridgeConfig:
    config_path = Path(args.config).expanduser()
    config = load_bridge_config(config_path) if config_path.exists() else SlamBridgeConfig()
    if args.ports is not None:
        config.ports = args.ports
    if args.baud is not None:
        config.baud = args.baud
    if args.source is not None:
        config.source = args.source
    elif config.source == "standby":
        config.source = "vio"
    if args.imu is not None:
        config.imu_enabled = args.imu == "on"
    if args.imu_port is not None:
        config.imu_port = args.imu_port
    if args.imu_baud is not None:
        config.imu_baud = args.imu_baud
    if args.imu_scan_seconds is not None:
        config.imu_scan_seconds = args.imu_scan_seconds
    if args.output is not None:
        config.calibration.profile_path = args.output
    return config


def timestamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def format_context(
    stage: str,
    fc_state: FlightControllerTelemetry | None = None,
    pose=None,
    imu_status: str = "unknown",
    reason: str = "",
) -> str:
    mode = "unknown" if fc_state is None else fc_state.flight_mode
    armed = "unknown" if fc_state is None else ("yes" if fc_state.armed else "no")
    range_text = "unknown"
    mavlink = "unknown"
    if fc_state is not None:
        if fc_state.rangefinder_distance_m is not None:
            range_text = f"{fc_state.rangefinder_distance_m:.2f}m"
        mavlink = "ok" if mavlink_heartbeat_valid(fc_state) else "timeout"
    vio = "unknown"
    if pose is not None:
        vio = f"{pose.tracking_state}/q{pose.pose_quality}"
    reason_text = "" if not reason else f" reason={reason}"
    return (
        f"{timestamp()} | stage={stage}"
        f" mode={mode}"
        f" armed={armed}"
        f" rangefinder={range_text}"
        f" vio={vio}"
        f" imu={imu_status}"
        f" mavlink={mavlink}"
        f"{reason_text}"
    )


def log_step(
    stage: str,
    message: str,
    fc_state: FlightControllerTelemetry | None = None,
    pose=None,
    imu_status: str = "unknown",
    reason: str = "",
) -> None:
    print(format_context(stage, fc_state, pose, imu_status, reason))
    print(f"  {message}", flush=True)


def service_is_active(service_name: str) -> bool:
    result = subprocess.run(
        ["systemctl", "is-active", "--quiet", service_name],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def run_systemctl(action: str, service_name: str) -> bool:
    commands = [
        ["systemctl", action, service_name],
        ["sudo", "-n", "systemctl", action, service_name],
    ]
    for command in commands:
        result = subprocess.run(command, check=False, capture_output=True, text=True)
        if result.returncode == 0:
            return True
    return False


def stop_conflicting_services(args) -> list[str]:
    stopped_services: list[str] = []
    if args.manage_service != "on":
        return stopped_services
    for service_name in args.conflict_service:
        if not service_is_active(service_name):
            log_step("service", f"{service_name} is not active; no stop needed.")
            continue
        log_step("service", f"Stopping active VIO service... {service_name}")
        if run_systemctl("stop", service_name):
            stopped_services.append(service_name)
            log_step("service", f"Stopped {service_name}.")
        else:
            log_step(
                "service",
                f"Could not stop {service_name}; run: sudo systemctl stop {service_name}",
                reason="service stop failed",
            )
    return stopped_services


def restart_services(args, stopped_services: list[str]) -> None:
    if args.manage_service != "on" or args.restart_service == "never":
        return
    services = args.conflict_service if args.restart_service == "always" else stopped_services
    for service_name in services:
        log_step("service", f"Restarting flight VIO service... {service_name}")
        if run_systemctl("restart", service_name):
            log_step("service", f"Restarted {service_name}.")
        else:
            log_step(
                "service",
                f"Could not restart {service_name}; run: sudo systemctl restart {service_name}",
                reason="service restart failed",
            )


def connect_to_cube_with_retry(config: SlamBridgeConfig, max_retries: int = 3):
    for attempt in range(max_retries):
        try:
            return connect_to_cube(config.ports, config.baud)
        except Exception as exc:  # noqa: BLE001
            if attempt >= max_retries - 1:
                raise RuntimeError(f"Failed to connect to Cube: {exc}") from exc
            log_step("mavlink", f"Cube connection attempt {attempt + 1} failed: {exc}; retrying.")
            time.sleep(2.0)
    raise RuntimeError("Failed to connect to Cube")


def open_imu(config: SlamBridgeConfig):
    if not config.imu_enabled:
        return None
    return Im10aReader.open(config.imu_port, config.imu_baud, config.imu_scan_seconds)


def wait_for_mavlink(connection, fc_state: FlightControllerTelemetry, timeout_s: float = 5.0) -> bool:
    deadline_s = time.time() + timeout_s
    while time.time() <= deadline_s:
        drain_fc_telemetry(connection.master, fc_state)
        if mavlink_heartbeat_valid(fc_state):
            return True
        time.sleep(0.05)
    return False


def wait_for_attitude(connection, fc_state: FlightControllerTelemetry, timeout_s: float = 5.0) -> bool:
    deadline_s = time.time() + timeout_s
    while time.time() <= deadline_s:
        drain_fc_telemetry(connection.master, fc_state)
        if fc_state.attitude is not None:
            return True
        time.sleep(0.05)
    return False


def check_rangefinder(connection, fc_state: FlightControllerTelemetry, args) -> tuple[bool, list[float], str]:
    log_step("rangefinder", "Checking rangefinder height...", fc_state)
    samples: list[float] = []
    deadline_s = time.time() + 3.0
    while time.time() <= deadline_s:
        drain_fc_telemetry(connection.master, fc_state)
        if rangefinder_height_valid(fc_state):
            samples.append(float(fc_state.rangefinder_distance_m))
        time.sleep(0.05)
    if len(samples) < 8:
        return False, samples, "rangefinder unhealthy"
    mean_m = statistics.fmean(samples)
    noise_m = statistics.pstdev(samples) if len(samples) > 1 else 0.0
    if not args.min_range_m <= mean_m <= args.max_stationary_range_m:
        return False, samples, f"bad rangefinder data mean={mean_m:.2f}m"
    if noise_m > args.max_range_noise_m:
        return False, samples, f"bad rangefinder data noise={noise_m:.2f}m"
    return True, samples, f"rangefinder mean={mean_m:.2f}m noise={noise_m:.2f}m"


def check_imu_stability(imu_reader, args) -> tuple[bool, str, str]:
    if imu_reader is None:
        return False, "missing", "IMU missing"
    log_step("imu", "Checking IMU stability...", imu_status="warming")
    samples = []
    deadline_s = time.time() + 3.0
    while time.time() <= deadline_s:
        sample = imu_reader.poll(duration_s=0.03)
        if sample is not None:
            samples.append(sample)
    if len(samples) < 10:
        return False, "missing", "IMU not transmitting"

    gyro_norms = [
        math.sqrt(sample.gx_deg_s**2 + sample.gy_deg_s**2 + sample.gz_deg_s**2)
        for sample in samples
    ]
    roll_std = statistics.pstdev(sample.roll_deg for sample in samples)
    pitch_std = statistics.pstdev(sample.pitch_deg for sample in samples)
    gyro_mean = statistics.fmean(gyro_norms)
    if gyro_mean > args.max_imu_gyro_deg_s:
        return False, "unstable", f"IMU gyro too noisy {gyro_mean:.2f}deg/s"
    if max(roll_std, pitch_std) > args.max_imu_angle_std_deg:
        return False, "unstable", f"IMU attitude noise too high {max(roll_std, pitch_std):.2f}deg"
    return True, "stable", f"IMU stable gyro={gyro_mean:.2f}deg/s angle_std={max(roll_std, pitch_std):.2f}deg"


def sample_vio_stationary(
    pose_source,
    connection,
    fc_state: FlightControllerTelemetry,
    imu_reader,
    args,
    range_samples: list[float],
) -> tuple[bool, list[dict], str]:
    log_step("vio", "Checking RealSense frames...", fc_state)
    if hasattr(pose_source, "reset_origin"):
        log_step("vio", "Resetting SLAM origin...", fc_state)
        pose_source.reset_origin()

    warmup_deadline_s = time.time() + max(args.warmup_seconds, 0.0)
    while time.time() <= warmup_deadline_s:
        drain_fc_telemetry(connection.master, fc_state)
        if rangefinder_height_valid(fc_state) and hasattr(pose_source, "set_external_height_m"):
            pose_source.set_external_height_m(float(fc_state.rangefinder_distance_m))
        pose_source.sample()

    log_step("vio", "Checking VIO/SLAM output while stationary...", fc_state)
    records: list[dict] = []
    deadline_s = time.time() + args.duration
    period_s = 1.0 / max(args.rate_hz, 0.1)
    while time.time() <= deadline_s:
        loop_s = time.time()
        drain_fc_telemetry(connection.master, fc_state)
        if rangefinder_height_valid(fc_state) and hasattr(pose_source, "set_external_height_m"):
            pose_source.set_external_height_m(float(fc_state.rangefinder_distance_m))

        raw_pose = pose_source.sample()
        imu_sample = imu_reader.poll(duration_s=min(0.02, period_s)) if imu_reader is not None else None
        pose = apply_imu_sample_to_pose(raw_pose, imu_sample) if imu_sample is not None else raw_pose
        range_m = float(fc_state.rangefinder_distance_m or 0.0)
        raw_height_error_m = abs(abs(float(raw_pose.z_m)) - range_m) if range_m > 0.0 else 0.0
        if rangefinder_height_valid(fc_state):
            pose.z_m = -float(fc_state.rangefinder_distance_m)
            pose.tracking_state = f"{pose.tracking_state}+rng"
            pose.source_name = f"{pose.source_name}+rng" if pose.source_name else "rng"

        records.append(
            {
                "pose": pose,
                "timestamp_us": int(pose.timestamp_us),
                "range_m": range_m,
                "raw_height_error_m": raw_height_error_m,
                "fc_x_m": 0.0 if fc_state.local_position is None else float(getattr(fc_state.local_position, "x", 0.0)),
                "fc_y_m": 0.0 if fc_state.local_position is None else float(getattr(fc_state.local_position, "y", 0.0)),
                "fc_yaw_deg": 0.0
                if fc_state.attitude is None
                else math.degrees(float(getattr(fc_state.attitude, "yaw", 0.0))),
            }
        )

        if len(records) % max(int(args.rate_hz * 3.0), 1) == 0:
            drift_m = horizontal_drift_m(records)
            log_step(
                "vio",
                f"Stationary drift detected: {drift_m * 100.0:.1f} cm over {len(records) / max(args.rate_hz, 0.1):.1f} seconds",
                fc_state,
                pose,
            )

        remaining_s = period_s - (time.time() - loop_s)
        if remaining_s > 0:
            time.sleep(remaining_s)

    if len(records) < args.min_samples:
        return False, records, f"no frames or too few frames: {len(records)}"

    timestamps = [record["timestamp_us"] for record in records]
    if len(set(timestamps)) < max(3, len(timestamps) // 3):
        return False, records, "frozen timestamps"

    valid_records = [
        record
        for record in records
        if record["pose"].tracking_state.startswith("ok")
        and int(record["pose"].pose_quality) >= args.min_pose_quality
    ]
    if len(valid_records) < args.min_samples:
        return False, records, f"VIO/SLAM output unhealthy: {len(valid_records)} valid samples"

    drift_m = horizontal_drift_m(valid_records)
    if drift_m > args.max_stationary_drift_m:
        return False, valid_records, f"SLAM drift while stationary {drift_m:.2f}m"

    x_noise_m = linear_std([record["pose"].x_m for record in valid_records])
    y_noise_m = linear_std([record["pose"].y_m for record in valid_records])
    if max(x_noise_m, y_noise_m) > args.max_pose_noise_m:
        return False, valid_records, f"noisy pose {max(x_noise_m, y_noise_m):.2f}m"

    if range_samples:
        median_height_error_m = statistics.median(record["raw_height_error_m"] for record in valid_records)
        if median_height_error_m > args.max_height_error_m:
            return False, valid_records, f"bad scale height_error={median_height_error_m:.2f}m"

    return True, valid_records, "VIO/SLAM output plausible"


def horizontal_drift_m(records: list[dict]) -> float:
    if len(records) < 2:
        return 0.0
    first_pose = records[0]["pose"]
    last_pose = records[-1]["pose"]
    return math.hypot(float(last_pose.x_m) - float(first_pose.x_m), float(last_pose.y_m) - float(first_pose.y_m))


def build_profile(records: list[dict]) -> CalibrationProfile:
    yaw_offsets_deg: list[float] = []
    x_offsets_m: list[float] = []
    y_offsets_m: list[float] = []
    ranges_m: list[float] = []
    for record in records:
        pose = record["pose"]
        yaw_offset = wrap_angle_deg(float(record["fc_yaw_deg"]) - pose_yaw_deg(pose))
        rotated_x_m, rotated_y_m = rotate_xy(float(pose.x_m), float(pose.y_m), yaw_offset)
        yaw_offsets_deg.append(yaw_offset)
        x_offsets_m.append(float(record["fc_x_m"]) - rotated_x_m)
        y_offsets_m.append(float(record["fc_y_m"]) - rotated_y_m)
        if float(record["range_m"]) > 0.0:
            ranges_m.append(float(record["range_m"]))

    yaw_mean = circular_mean_deg(yaw_offsets_deg)
    return CalibrationProfile(
        valid=True,
        calibration_mode="STATIONARY",
        sample_count=len(records),
        yaw_offset_deg=yaw_mean,
        x_offset_m=statistics.fmean(x_offsets_m),
        y_offset_m=statistics.fmean(y_offsets_m),
        yaw_std_deg=circular_std_deg(yaw_offsets_deg, yaw_mean),
        x_std_m=linear_std(x_offsets_m),
        y_std_m=linear_std(y_offsets_m),
        range_mean_m=statistics.fmean(ranges_m) if ranges_m else 0.0,
        saved_at_epoch_s=time.time(),
    )


def profile_is_usable(profile: CalibrationProfile, args) -> tuple[bool, str]:
    if profile.sample_count < args.min_samples:
        return False, "not enough calibration samples"
    if profile.yaw_std_deg > args.max_yaw_noise_deg:
        return False, f"yaw noise too high {profile.yaw_std_deg:.1f}deg"
    if max(profile.x_std_m, profile.y_std_m) > args.max_pose_noise_m:
        return False, f"position offset noise too high {max(profile.x_std_m, profile.y_std_m):.2f}m"
    return True, "profile stable"


def close_connection(connection) -> None:
    master = getattr(connection, "master", None)
    if master is not None and hasattr(master, "close"):
        try:
            master.close()
        except Exception:  # noqa: BLE001
            pass


def run_stationary_calibration(args) -> bool:
    config = resolve_config(args)
    if args.verbose:
        log_step("start", "Verbose stationary calibration logs enabled.")
    if args.indoor or args.no_gps:
        log_step("start", "Indoor/no-GPS stationary calibration selected. GPS will not be required.")
    stopped_services = stop_conflicting_services(args)
    connection = None
    imu_reader = None
    pose_source = None
    fc_state = FlightControllerTelemetry()

    try:
        log_step("mavlink", "Checking MAVLink heartbeat...")
        connection = connect_to_cube_with_retry(config)
        configure_telemetry_streams(connection.master)
        fc_state.active_source_set = request_active_source_set(connection.master)
        if not wait_for_mavlink(connection, fc_state):
            raise RuntimeError("missing MAVLink heartbeat")
        if not wait_for_attitude(connection, fc_state):
            raise RuntimeError("FC attitude telemetry missing")
        send_gcs_event(connection.master, "Stationary SLAM calibration started")
        log_step("mavlink", f"MAVLink heartbeat present on {connection.port}@{connection.baud}.", fc_state)

        log_step("imu", "Opening IMU...")
        if config.imu_enabled:
            imu_reader = open_imu(config)
        imu_ok, imu_status, imu_reason = check_imu_stability(imu_reader, args)
        log_step("imu", imu_reason, fc_state, imu_status=imu_status)
        if not imu_ok:
            raise RuntimeError(imu_reason)

        range_ok, range_samples, range_reason = check_rangefinder(connection, fc_state, args)
        log_step("rangefinder", range_reason, fc_state, imu_status=imu_status)
        if not range_ok:
            raise RuntimeError(range_reason)

        log_step("vio", "Opening RealSense/VIO pipeline...", fc_state, imu_status=imu_status)
        pose_source = make_pose_source(config.source, config.csv_path)
        vio_ok, records, vio_reason = sample_vio_stationary(
            pose_source,
            connection,
            fc_state,
            imu_reader,
            args,
            range_samples,
        )
        last_pose = records[-1]["pose"] if records else None
        log_step("vio", vio_reason, fc_state, last_pose, imu_status)
        if not vio_ok:
            raise RuntimeError(vio_reason)

        profile = build_profile(records)
        profile_ok, profile_reason = profile_is_usable(profile, args)
        log_step(
            "profile",
            (
                f"{profile_reason}: samples={profile.sample_count}"
                f" yaw={profile.yaw_offset_deg:+.2f}+/-{profile.yaw_std_deg:.2f}deg"
                f" xy=({profile.x_offset_m:+.2f},{profile.y_offset_m:+.2f})"
                f" noise=({profile.x_std_m:.2f},{profile.y_std_m:.2f})m"
            ),
            fc_state,
            last_pose,
            imu_status,
        )
        if not profile_ok:
            raise RuntimeError(profile_reason)

        output_path = Path(config.calibration.profile_path).expanduser()
        save_calibration_profile(output_path, profile)
        log_step("complete", f"Calibration passed. Saved profile to {output_path}.", fc_state, last_pose, imu_status)
        send_gcs_event(connection.master, "Stationary SLAM calibration passed")
        log_step("complete", "System is flight ready.", fc_state, last_pose, imu_status)
        send_gcs_event(connection.master, "System is flight ready")
        return True
    except Exception as exc:  # noqa: BLE001
        reason = str(exc)
        log_step("failed", "Calibration failed.", fc_state, imu_status="failed", reason=reason)
        if connection is not None:
            send_gcs_event(connection.master, f"Stationary SLAM calibration failed: {reason}", severity=4)
        return False
    finally:
        if pose_source is not None and hasattr(pose_source, "close"):
            try:
                pose_source.close()
            except Exception:  # noqa: BLE001
                pass
        if imu_reader is not None:
            try:
                imu_reader.close()
            except Exception:  # noqa: BLE001
                pass
        if connection is not None:
            close_connection(connection)
        restart_services(args, stopped_services)


def main():
    args = parse_args()
    success = run_stationary_calibration(args)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
