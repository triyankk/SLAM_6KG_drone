#!/usr/bin/env python3

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.external_imu import Im10aReader, apply_imu_sample_to_pose
from slam_core.bridge_config import SlamBridgeConfig, load_bridge_config
from slam_core.calibration import (
    CalibrationAccumulator,
    CalibrationProfile,
    apply_calibration_profile,
    load_calibration_profile,
    pose_yaw_deg,
    save_calibration_profile,
)
from slam_core.fc_config import (
    BRIDGE_STATE_IDLE,
    BRIDGE_STATE_JETSON_BOOT,
    BRIDGE_STATE_SENSOR_CHECK_PASSED,
    BRIDGE_STATE_SLAM_STARTED,
    BRIDGE_STATE_SOURCE_SET_ACTIVE,
    BRIDGE_STATE_POSHOLD_READY,
    BRIDGE_STATE_CALIBRATION_WAITING_ARM,
    BRIDGE_STATE_CALIBRATION_ACTIVE,
    BRIDGE_STATE_CALIBRATION_COMPLETE_RTL,
    BRIDGE_STATE_SLAM_FLIGHT_ACTIVE,
    BRIDGE_STATE_SOURCE_SWITCH_FAILED,
    BRIDGE_STATE_SOURCE_SWITCH_NO_ACK,
    FlightControllerTelemetry,
    apply_fc_setup,
    configure_telemetry_streams,
    drain_fc_telemetry,
    publish_bridge_state,
    rangefinder_height_valid,
    mavlink_heartbeat_valid,
    recent_status_blocks_slam,
    request_active_source_set,
    rc_link_valid,
    gps_reference_valid,
    send_calibration_active_beeps,
    send_calibration_complete_beeps,
    send_calibration_failed_beeps,
    send_distance_sensor,
    send_fixed_gps_input,
    send_gcs_event,
    send_gps_input_from_pose,
    send_ground_calibration_warning_beeps,
    send_obstacle_distance,
    send_body_velocity_nudge,
    send_body_yaw_rate_nudge,
    send_companion_heartbeat,
    send_ready_beeps,
    send_sensor_check_beep,
    send_slam_flight_ping,
    send_startup_beeps,
    send_statustext,
    set_vehicle_mode,
    set_ekf_source_set,
)
from slam_core.lidar import LidarReader
from slam_core.mavlink_bridge import connect_to_cube, send_odometry
from slam_core.pose_sources import make_pose_source
from slam_core.qgc_bridge import QgcUdpBridge


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Send MAVLink ODOMETRY to the Cube from a pose source. "
            "This is the clean starting point for plugging a real SLAM backend into the drone."
        )
    )
    parser.add_argument("--config", default="")
    parser.add_argument("--ports", nargs="+")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--source", choices=["standby", "hover", "circle", "csv", "vio"])
    parser.add_argument("--csv-path", default=None)
    parser.add_argument("--rate-hz", type=float)
    parser.add_argument("--imu", choices=["on", "off"])
    parser.add_argument("--imu-port", default=None)
    parser.add_argument("--imu-baud", default=None)
    parser.add_argument("--imu-scan-seconds", type=float)
    parser.add_argument("--cube-retry-seconds", type=float)
    parser.add_argument("--reconnect-delay-seconds", type=float)
    parser.add_argument("--standby-log-seconds", type=float)
    parser.add_argument("--status-log-seconds", type=float)
    parser.add_argument("--heartbeat-timeout-seconds", type=float)
    parser.add_argument("--connect-in-standby", choices=["on", "off"])
    parser.add_argument("--fc-setup", choices=["on", "off"])
    parser.add_argument("--fc-source-set", type=int)
    parser.add_argument("--fc-switch-after-sends", type=int)
    parser.add_argument("--fc-select-source", choices=["on", "off"])
    return parser.parse_args()


def resolve_config(args) -> SlamBridgeConfig:
    config = load_bridge_config(args.config) if args.config else SlamBridgeConfig()
    if args.ports is not None:
        config.ports = args.ports
    if args.baud is not None:
        config.baud = args.baud
    if args.source is not None:
        config.source = args.source
    if args.csv_path is not None:
        config.csv_path = args.csv_path
    if args.rate_hz is not None:
        config.rate_hz = args.rate_hz
    if args.imu is not None:
        config.imu_enabled = args.imu == "on"
    if args.imu_port is not None:
        config.imu_port = args.imu_port
    if args.imu_baud is not None:
        config.imu_baud = args.imu_baud
    if args.imu_scan_seconds is not None:
        config.imu_scan_seconds = args.imu_scan_seconds
    if args.cube_retry_seconds is not None:
        config.cube_retry_seconds = args.cube_retry_seconds
    if args.reconnect_delay_seconds is not None:
        config.reconnect_delay_seconds = args.reconnect_delay_seconds
    if args.standby_log_seconds is not None:
        config.standby_log_seconds = args.standby_log_seconds
    if args.status_log_seconds is not None:
        config.status_log_seconds = args.status_log_seconds
    if args.heartbeat_timeout_seconds is not None:
        config.heartbeat_timeout_seconds = args.heartbeat_timeout_seconds
    if args.connect_in_standby is not None:
        config.connect_in_standby = args.connect_in_standby == "on"
    if args.fc_setup is not None:
        config.fc_setup.enabled = args.fc_setup == "on"
    if args.fc_source_set is not None:
        config.fc_setup.slam_source_set = args.fc_source_set
    if args.fc_switch_after_sends is not None:
        config.fc_setup.switch_after_sends = args.fc_switch_after_sends
    if args.fc_select_source is not None:
        config.fc_setup.select_source_set_on_stream = args.fc_select_source == "on"
    return config


def sleep_with_floor(seconds: float) -> None:
    time.sleep(max(seconds, 0.2))


def sleep_until_boot_delay(config: SlamBridgeConfig) -> None:
    if config.boot_delay_seconds <= 0:
        return
    try:
        uptime_s = float(Path("/proc/uptime").read_text(encoding="utf-8").split()[0])
    except Exception:  # noqa: BLE001
        return
    remaining_s = config.boot_delay_seconds - uptime_s
    if remaining_s > 0:
        print(f"Waiting {remaining_s:.1f}s for Jetson boot sensors to settle before SLAM start.")
        time.sleep(remaining_s)


def connect_to_cube_with_retry(config: SlamBridgeConfig):
    while True:
        try:
            return connect_to_cube(
                config.ports,
                config.baud,
                heartbeat_timeout_s=config.heartbeat_timeout_seconds,
            )
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(
                "Cube connection not ready:"
                f" ports={config.ports}"
                f" baud={config.baud}"
                f" error={exc}"
                f" | retrying in {config.cube_retry_seconds:.1f}s"
            )
            sleep_with_floor(config.cube_retry_seconds)


def open_imu_with_retry(config: SlamBridgeConfig):
    if not config.imu_enabled:
        return None

    while True:
        try:
            return Im10aReader.open(config.imu_port, config.imu_baud, config.imu_scan_seconds)
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(
                "External IMU not ready:"
                f" port={config.imu_port}"
                f" baud={config.imu_baud}"
                f" error={exc}"
                f" | retrying in {config.reconnect_delay_seconds:.1f}s"
            )
            sleep_with_floor(config.reconnect_delay_seconds)


def open_lidar_with_retry(config: SlamBridgeConfig):
    if not config.obstacle.enabled or not config.obstacle.lidar_enabled:
        return None

    while True:
        try:
            return LidarReader.open(
                config.obstacle.lidar_port,
                config.obstacle.lidar_baud,
                config.obstacle.sector_count,
                config.obstacle.filter_samples,
                config.obstacle.min_valid_distance_m,
                config.obstacle.max_distance_m,
            )
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(
                "JT lidar not ready:"
                f" port={config.obstacle.lidar_port}"
                f" baud={config.obstacle.lidar_baud}"
                f" error={exc}"
                f" | retrying in {config.reconnect_delay_seconds:.1f}s"
            )
            sleep_with_floor(config.reconnect_delay_seconds)


def open_qgc_bridge(config: SlamBridgeConfig):
    if not config.qgc.enabled:
        return None
    return QgcUdpBridge(
        config.qgc.forward_host,
        config.qgc.forward_port,
        config.qgc.bind_host,
        config.qgc.bind_port,
    )


def close_cube_connection(connection) -> None:
    master = getattr(connection, "master", None)
    if master is not None and hasattr(master, "close"):
        try:
            master.close()
        except Exception:  # noqa: BLE001
            pass


def ensure_fc_setup(connection, config: SlamBridgeConfig) -> None:
    if not config.fc_setup.enabled:
        return

    report = apply_fc_setup(connection.master, config.fc_setup)
    if report.changed:
        changed_summary = ", ".join(f"{item.name}={item.new_value:g}" for item in report.changed)
        print(
            "Flight controller SLAM setup applied:"
            f" source_set={config.fc_setup.slam_source_set}"
            f" changed={changed_summary}"
        )
        send_gcs_event(connection.master, f"FC setup changed: {changed_summary}")
    else:
        print(
            "Flight controller SLAM setup already matched:"
            f" source_set={config.fc_setup.slam_source_set}"
        )
        send_gcs_event(
            connection.master,
            f"FC SLAM setup already matched source {config.fc_setup.slam_source_set}",
        )

    if report.reboot_recommended:
        print(
            "Flight controller reboot is recommended because EKF/visual-odometry"
            " parameters changed on this boot."
        )
        send_gcs_event(
            connection.master,
            "FC reboot recommended: EKF/GPS parameter changes need reboot to fully apply.",
            severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
    if config.fc_setup.gps2_type == 0:
        send_gcs_event(
            connection.master,
            "GPS2 disabled: no Jetson GPS_INPUT stream is configured; GPS2 bad-fix warnings should clear after reboot.",
        )
    send_gcs_event(
        connection.master,
        f"FC prepared for ExternalNav source {config.fc_setup.slam_source_set}, avoidance margin {config.fc_setup.avoid_margin_m:.1f}m",
    )


def format_fc_position(state: FlightControllerTelemetry) -> str:
    if state.local_position is None:
        return "waiting"
    return (
        f"x={float(getattr(state.local_position, 'x', 0.0)):+.2f}"
        f" y={float(getattr(state.local_position, 'y', 0.0)):+.2f}"
        f" z={float(getattr(state.local_position, 'z', 0.0)):+.2f}"
    )


def mode_wants_slam(fc_state: FlightControllerTelemetry, config: SlamBridgeConfig) -> bool:
    target_mode = config.fc_setup.activate_mode.strip().upper()
    if target_mode in {"", "ANY"}:
        return True
    return fc_state.flight_mode.upper() == target_mode


def using_gps_input_bridge(config: SlamBridgeConfig) -> bool:
    return (
        config.gps_input.enabled
        and config.fc_setup.viso_type <= 0
        and config.fc_setup.gps2_type not in (None, 0)
    )


def apply_cube_rangefinder_height(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
):
    if not config.fc_setup.require_rangefinder_height:
        return pose
    if not rangefinder_height_valid(fc_state):
        return pose

    pose.z_m = -float(fc_state.rangefinder_distance_m)
    pose.tracking_state = f"{pose.tracking_state}+rng"
    if pose.source_name:
        pose.source_name = f"{pose.source_name}+rng"
    else:
        pose.source_name = "rng"
    return pose


def slam_poshold_ready(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    calibration_profile: CalibrationProfile,
) -> bool:
    return (
        mode_wants_slam(fc_state, config)
        and fc_state.active_source_set == config.fc_setup.slam_source_set
        and bridge_ready_for_poshold(pose, fc_state, config, calibration_profile)
    )


def quaternion_norm(pose) -> float:
    return math.sqrt(pose.qw * pose.qw + pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz)


def pose_safe_for_fc(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    min_quality: int | None = None,
) -> bool:
    return pose_safety_reason(pose, fc_state, config, min_quality=min_quality) is None


def pose_safety_reason(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    min_quality: int | None = None,
) -> str | None:
    if min_quality is None:
        min_quality = max(35, min(config.fc_setup.ready_min_quality, 45))
    if not pose.tracking_state.startswith("ok"):
        return f"tracking={pose.tracking_state}"
    if pose.pose_quality < min_quality:
        return f"quality {pose.pose_quality} < {min_quality}"
    if not all(
        math.isfinite(value)
        for value in (
            pose.x_m,
            pose.y_m,
            pose.z_m,
            pose.vx_m_s,
            pose.vy_m_s,
            pose.vz_m_s,
            pose.qw,
            pose.qx,
            pose.qy,
            pose.qz,
        )
    ):
        return "pose has non-finite values"
    if not 0.7 <= quaternion_norm(pose) <= 1.3:
        return f"bad quaternion norm {quaternion_norm(pose):.2f}"
    speed_m_s = math.sqrt(pose.vx_m_s * pose.vx_m_s + pose.vy_m_s * pose.vy_m_s + pose.vz_m_s * pose.vz_m_s)
    if speed_m_s > 8.0:
        return f"VIO speed too high {speed_m_s:.1f}m/s"
    if config.fc_setup.require_rangefinder_height:
        if not rangefinder_height_valid(fc_state):
            return "rangefinder height missing"
        if abs(abs(pose.z_m) - float(fc_state.rangefinder_distance_m or 0.0)) > 1.0:
            return "VIO height disagrees with rangefinder"
    return None


def sensor_quick_check_ok(
    pose,
    imu_sample,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
) -> bool:
    if config.imu_enabled and imu_sample is None:
        return False
    return pose_safe_for_fc(pose, fc_state, config, min_quality=max(40, config.calibration.min_pose_quality - 10))


def bridge_ready_for_poshold(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    calibration_profile: CalibrationProfile,
) -> bool:
    if config.fc_setup.viso_type <= 0 and not using_gps_input_bridge(config):
        return False
    if using_gps_input_bridge(config) and not calibration_profile.valid:
        return False
    if not pose_safe_for_fc(pose, fc_state, config, min_quality=config.fc_setup.ready_min_quality):
        return False
    if recent_status_blocks_slam(fc_state):
        return False
    return True


def bridge_not_ready_reason(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    calibration_profile: CalibrationProfile,
) -> str | None:
    if config.fc_setup.viso_type <= 0 and not using_gps_input_bridge(config):
        return "VISO_TYPE is disabled; FC will ignore ExternalNav ODOMETRY"
    if using_gps_input_bridge(config) and not calibration_profile.valid:
        return "GPS2 bridge is disabled until Brake calibration completes"
    pose_reason = pose_safety_reason(pose, fc_state, config, min_quality=config.fc_setup.ready_min_quality)
    if pose_reason is not None:
        return pose_reason
    if recent_status_blocks_slam(fc_state):
        return f"FC warning active: {fc_state.status_text or 'status text'}"
    return None


def mode_event_message(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    calibration_profile: CalibrationProfile,
) -> tuple[str, int]:
    mode = fc_state.flight_mode.upper()
    if mode == config.calibration.mode:
        return (
            "Mode BRAKE: SLAM calibration monitor engaged; Brake should hold while Jetson checks safety gates.",
            mavutil.mavlink.MAV_SEVERITY_NOTICE,
        )
    if mode == config.fc_setup.activate_mode.strip().upper():
        reason = bridge_not_ready_reason(pose, fc_state, config, calibration_profile)
        if reason is None:
            if not calibration_profile.valid:
                return (
                    "Mode POSHOLD: raw SLAM/VIO stream ready but not calibrated yet; using ExternalNav only for cautious setup.",
                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                )
            return (
                "Mode POSHOLD: SLAM/VIO ready; ExternalNav source can be used for GPS-denied hold.",
                mavutil.mavlink.MAV_SEVERITY_NOTICE,
            )
        return (
            f"Mode POSHOLD: SLAM not active yet; reason: {reason}. Keeping current EKF source.",
            mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
    if mode == "LOITER":
        return ("Mode LOITER: GPS/EKF position hold active; SLAM bridge monitoring only.", mavutil.mavlink.MAV_SEVERITY_INFO)
    if mode == "RTL":
        return ("Mode RTL: return-to-launch active; SLAM bridge monitoring and source-release only.", mavutil.mavlink.MAV_SEVERITY_NOTICE)
    if mode == "LAND":
        return ("Mode LAND: landing active; SLAM bridge will not command navigation.", mavutil.mavlink.MAV_SEVERITY_NOTICE)
    if mode in {"ALTHOLD", "ALT_HOLD"}:
        return ("Mode AltHold: pilot controls horizontal motion; SLAM bridge monitoring only.", mavutil.mavlink.MAV_SEVERITY_INFO)
    if mode == "STABILIZE":
        return ("Mode STABILIZE: manual attitude mode; SLAM bridge monitoring only.", mavutil.mavlink.MAV_SEVERITY_INFO)
    return (f"Mode {mode}: SLAM bridge monitoring; no autonomous action for this mode.", mavutil.mavlink.MAV_SEVERITY_INFO)


def calibration_profile_stable(profile: CalibrationProfile) -> bool:
    return (
        profile.valid
        and profile.sample_count > 0
        and profile.yaw_std_deg <= 10.0
        and profile.x_std_m <= 0.75
        and profile.y_std_m <= 0.75
    )


def calibration_block_reason(
    raw_pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
) -> str | None:
    if not fc_state.armed:
        return "vehicle is not armed"
    if not mavlink_heartbeat_valid(fc_state):
        return "MAVLink timeout"
    if config.calibration.require_rc_link and not rc_link_valid(fc_state):
        return "RC failsafe"
    if config.calibration.movement_commands_enabled and not config.calibration.kill_switch_confirmed:
        return "kill switch/failsafe not confirmed"
    if not gps_reference_valid(
        fc_state,
        config.calibration.min_gps_fix_type,
        config.calibration.min_gps_satellites,
    ):
        return (
            "GPS reference unhealthy"
            f" fix={fc_state.gps_fix_type}"
            f" sats={fc_state.gps_satellites}"
        )
    if fc_state.local_position is None:
        return "GPS/EKF local position reference missing"
    if not battery_healthy_for_calibration(fc_state, config):
        return (
            f"battery too low {fc_state.battery_remaining_pct}%"
            f" < {config.calibration.min_battery_remaining_pct}%"
        )
    if fc_state.attitude is None:
        return "FC attitude telemetry is missing"
    if fc_state.ekf_flags is None:
        return "EKF telemetry missing"
    if not rangefinder_height_valid(fc_state):
        return "rangefinder unhealthy"
    if recent_status_blocks_slam(fc_state):
        status_text = (fc_state.status_text or "").lower()
        if "ekf" in status_text and (
            "vision" in status_text or "external" in status_text or "nav" in status_text
        ):
            return "EKF rejected external navigation data"
        return f"FC warning active: {fc_state.status_text or 'status text'}"
    if not pose_safe_for_fc(raw_pose, fc_state, config, min_quality=config.calibration.min_pose_quality):
        return "SLAM/VIO lost tracking"

    if fc_state.local_position is not None:
        speed_xy_m_s = math.hypot(
            float(getattr(fc_state.local_position, "vx", 0.0)),
            float(getattr(fc_state.local_position, "vy", 0.0)),
        )
    else:
        speed_xy_m_s = math.hypot(float(raw_pose.vx_m_s), float(raw_pose.vy_m_s))
    if speed_xy_m_s > config.calibration.max_horizontal_speed_m_s:
        return f"vehicle still moving at {speed_xy_m_s:.2f}m/s"

    roll_deg = math.degrees(float(getattr(fc_state.attitude, "roll", 0.0)))
    pitch_deg = math.degrees(float(getattr(fc_state.attitude, "pitch", 0.0)))
    if abs(roll_deg) > config.calibration.max_roll_deg or abs(pitch_deg) > config.calibration.max_pitch_deg:
        return f"vehicle not level enough roll={roll_deg:+.1f} pitch={pitch_deg:+.1f}"
    return None


def active_slam_flight(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    calibration_profile: CalibrationProfile,
) -> bool:
    return fc_state.armed and slam_poshold_ready(pose, fc_state, config, calibration_profile)


def odometry_stream_requested(
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    calibration_mode_requested: bool,
) -> bool:
    if config.calibration.dry_run:
        return False
    if mode_wants_slam(fc_state, config):
        return True
    return (
        calibration_mode_requested
        and fc_state.armed
        and rangefinder_height_valid(fc_state)
        and not vehicle_on_ground_for_calibration(fc_state, config)
    )


def gps_input_stream_requested(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
    calibration_profile: CalibrationProfile,
    calibration_mode_requested: bool,
) -> bool:
    if config.calibration.dry_run or not using_gps_input_bridge(config):
        return False
    if calibration_mode_requested:
        return False
    if not mode_wants_slam(fc_state, config):
        return False
    if not calibration_profile.valid:
        return False
    return pose_safe_for_fc(pose, fc_state, config, min_quality=config.fc_setup.ready_min_quality)


def in_brake_calibration_mode(fc_state: FlightControllerTelemetry, config: SlamBridgeConfig) -> bool:
    return config.calibration.enabled and fc_state.flight_mode == config.calibration.mode


def vehicle_on_ground_for_calibration(fc_state: FlightControllerTelemetry, config: SlamBridgeConfig) -> bool:
    if fc_state.landed_state is not None:
        landed_on_ground = getattr(mavutil.mavlink, "MAV_LANDED_STATE_ON_GROUND", 1)
        if fc_state.landed_state == landed_on_ground:
            return True
    if not rangefinder_height_valid(fc_state):
        return False
    return float(fc_state.rangefinder_distance_m or 0.0) <= config.calibration.ground_max_height_m


def battery_healthy_for_calibration(fc_state: FlightControllerTelemetry, config: SlamBridgeConfig) -> bool:
    if fc_state.battery_remaining_pct is None or fc_state.battery_last_update_s <= 0.0:
        return True
    if time.time() - fc_state.battery_last_update_s > 10.0:
        return True
    return fc_state.battery_remaining_pct >= config.calibration.min_battery_remaining_pct


def landed_state_text(fc_state: FlightControllerTelemetry) -> str:
    if fc_state.landed_state is None:
        return "unknown"
    landed_map = {
        0: "undefined",
        1: "on_ground",
        2: "in_air",
        3: "takeoff",
        4: "landing",
    }
    return landed_map.get(fc_state.landed_state, str(fc_state.landed_state))


def calibration_state_name(stage: str) -> str:
    mapping = {
        "idle": "IDLE",
        "brake_detected": "BRAKE_DETECTED",
        "waiting_arm": "WAITING_FOR_ARM",
        "ground_precheck": "PREFLIGHT_CHECK",
        "waiting_takeoff": "TAKEOFF_WARNING",
        "waiting_height": "ASCEND_TO_5M",
        "hold_height": "HOLD_5M",
        "pitch": "CALIBRATING",
        "roll": "CALIBRATING",
        "yaw": "CALIBRATING",
        "altitude": "CALIBRATING",
        "complete": "SUCCESS",
        "failed": "FAILURE",
    }
    return mapping.get(stage, "READY")


def rangefinder_at_calibration_height(fc_state: FlightControllerTelemetry, config: SlamBridgeConfig) -> bool:
    if not rangefinder_height_valid(fc_state):
        return False
    return (
        abs(float(fc_state.rangefinder_distance_m or 0.0) - config.calibration.target_height_m)
        <= config.calibration.target_height_tolerance_m
    )


def mode_available(master, mode_name: str) -> bool:
    try:
        mapping = master.mode_mapping()
    except Exception:
        return False
    return bool(mapping and mode_name.upper() in mapping)


def calibration_drift_m(reference_pose, current_pose) -> float:
    if reference_pose is None:
        return 0.0
    dx_m = float(current_pose.x_m) - float(reference_pose.x_m)
    dy_m = float(current_pose.y_m) - float(reference_pose.y_m)
    return math.hypot(dx_m, dy_m)


def calibration_health_summary(
    stage: str,
    fc_state: FlightControllerTelemetry,
    pose,
    imu_sample,
    config: SlamBridgeConfig,
    action: str = "",
    reason: str = "",
) -> dict:
    mavlink_ok = mavlink_heartbeat_valid(fc_state)
    range_ok = rangefinder_height_valid(fc_state)
    vio_ok = bool(pose is not None and str(pose.tracking_state).startswith("ok"))
    imu_ok = (not config.imu_enabled) or imu_sample is not None
    if using_gps_input_bridge(config):
        nav_status = f"gps{config.gps_input.gps_id + 1}_bridge"
    else:
        ekf_external_nav = (
            fc_state.active_source_set == config.fc_setup.slam_source_set
            if config.fc_setup.slam_source_set > 0
            else False
        )
        nav_status = "accepted" if ekf_external_nav else "pending_or_rejected"
    return {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
        "state": calibration_state_name(stage),
        "stage": stage,
        "mode": fc_state.flight_mode,
        "armed": fc_state.armed,
        "landed_state": landed_state_text(fc_state),
        "on_ground": vehicle_on_ground_for_calibration(fc_state, config),
        "rangefinder_height_m": fc_state.rangefinder_distance_m,
        "rangefinder_healthy": range_ok,
        "gps1_fix_type": fc_state.gps_fix_type,
        "gps1_satellites": fc_state.gps_satellites,
        "gps2_fix_type": fc_state.gps2_fix_type,
        "gps2_satellites": fc_state.gps2_satellites,
        "gps2_lat": fc_state.gps2_lat,
        "gps2_lon": fc_state.gps2_lon,
        "gps2_alt_mm": fc_state.gps2_alt_mm,
        "vio_health": "ok" if vio_ok else "bad",
        "vio_tracking": None if pose is None else pose.tracking_state,
        "vio_quality": None if pose is None else pose.pose_quality,
        "imu_stability": "stable" if imu_ok else "missing",
        "ekf_external_nav_status": nav_status,
        "mavlink_status": "ok" if mavlink_ok else "timeout",
        "rc_link": "ok" if rc_link_valid(fc_state) else "missing",
        "battery_remaining_pct": fc_state.battery_remaining_pct,
        "current_action": action,
        "failure_reason": reason,
        "dry_run": config.calibration.dry_run,
        "movement_enabled": config.calibration.movement_commands_enabled,
    }


def append_calibration_log(config: SlamBridgeConfig, summary: dict) -> None:
    if not config.calibration.log_path:
        return
    path = Path(config.calibration.log_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    line = (
        f"{summary['timestamp']} | state={summary['state']}"
        f" mode={summary['mode']}"
        f" armed={'yes' if summary['armed'] else 'no'}"
        f" landed={summary['landed_state']}"
        f" on_ground={'yes' if summary['on_ground'] else 'no'}"
        f" rng={summary['rangefinder_height_m']}"
        f" gps1={summary['gps1_fix_type']}/{summary['gps1_satellites']}"
        f" gps2={summary['gps2_fix_type']}/{summary['gps2_satellites']}"
        f" vio={summary['vio_health']}/{summary['vio_quality']}"
        f" imu={summary['imu_stability']}"
        f" ekf_extnav={summary['ekf_external_nav_status']}"
        f" mavlink={summary['mavlink_status']}"
        f" action={summary['current_action']}"
        f" reason={summary['failure_reason']}"
    )
    with path.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def write_calibration_status(config: SlamBridgeConfig, summary: dict) -> None:
    if not config.calibration.status_path:
        return
    path = Path(config.calibration.status_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")


def record_calibration_status(
    config: SlamBridgeConfig,
    stage: str,
    fc_state: FlightControllerTelemetry,
    pose,
    imu_sample,
    action: str = "",
    reason: str = "",
) -> None:
    summary = calibration_health_summary(stage, fc_state, pose, imu_sample, config, action, reason)
    write_calibration_status(config, summary)
    append_calibration_log(config, summary)


def print_calibration_status(
    stage: str,
    fc_state: FlightControllerTelemetry,
    pose,
    imu_sample,
    config: SlamBridgeConfig,
    sent_count: int = 0,
    reason: str = "",
) -> None:
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    range_text = "unknown"
    if fc_state.rangefinder_distance_m is not None:
        range_text = f"{fc_state.rangefinder_distance_m:.2f}m"
    imu_text = "stable" if imu_sample is not None else "missing"
    mav_text = "ok" if mavlink_heartbeat_valid(fc_state) else "timeout"
    print(
        f"{timestamp} | mode={fc_state.flight_mode}"
        f" armed={'yes' if fc_state.armed else 'no'}"
        f" rng={range_text}"
        f" vio={pose.tracking_state}/q{pose.pose_quality}"
        f" odom_sent={sent_count}"
        f" imu={imu_text}"
        f" mavlink={mav_text}"
        f" stage={stage}"
        f"{'' if not reason else f' reason={reason}'}"
    )
    record_calibration_status(config, stage, fc_state, pose, imu_sample, action=f"status odom_sent={sent_count}", reason=reason)


def send_calibration_gcs_message(master, text: str, severity: int | None = None) -> None:
    if severity is None:
        send_statustext(master, text)
    else:
        send_statustext(master, text, severity=severity)


def send_calibration_failure(connection, config: SlamBridgeConfig, reason: str, change_mode: bool = True) -> None:
    send_calibration_failed_beeps(connection.master)
    send_calibration_gcs_message(
        connection.master,
        f"Calibration failed: not finished. Reason: {reason}",
        severity=4,
    )
    if not change_mode:
        return

    fallback = (config.calibration.fallback_mode or "").strip().upper()
    if not fallback:
        return
    if config.fc_setup.idle_source_set > 0:
        set_ekf_source_set(connection.master, config.fc_setup.idle_source_set, timeout_s=1.5)
    if not mode_available(connection.master, fallback):
        return

    set_vehicle_mode(connection.master, fallback)


def calibration_altitude_hold_vz(fc_state: FlightControllerTelemetry, config: SlamBridgeConfig) -> float:
    if not rangefinder_height_valid(fc_state):
        return 0.0
    error_m = config.calibration.target_height_m - float(fc_state.rangefinder_distance_m or 0.0)
    if abs(error_m) <= config.calibration.altitude_hold_deadband_m:
        return 0.0

    max_vertical_m_s = max(0.0, min(config.calibration.vertical_speed_m_s, 0.2))
    command_m_s = config.calibration.altitude_hold_gain * error_m
    command_m_s = max(-max_vertical_m_s, min(max_vertical_m_s, command_m_s))
    # MAV_FRAME_BODY_NED uses positive Z velocity downward, so positive height
    # error requires a negative vertical command to climb.
    return -command_m_s


def send_calibration_axis_motion(
    master,
    axis: str,
    elapsed_s: float,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
) -> None:
    if not config.calibration.movement_commands_enabled:
        return

    half_period_s = max(config.calibration.axis_stage_duration_s / 4.0, 1.0)
    direction = 1.0 if int(elapsed_s / half_period_s) % 2 == 0 else -1.0
    speed_m_s = max(0.0, min(config.calibration.movement_speed_m_s, 0.25))
    yaw_rate_deg_s = max(0.0, min(config.calibration.yaw_rate_deg_s, 12.0))
    vertical_m_s = calibration_altitude_hold_vz(fc_state, config)

    if axis == "pitch":
        send_body_velocity_nudge(master, direction * speed_m_s, 0.0, vertical_m_s)
    elif axis == "roll":
        send_body_velocity_nudge(master, 0.0, direction * speed_m_s, vertical_m_s)
    elif axis == "yaw":
        if abs(vertical_m_s) > 0.0:
            send_body_velocity_nudge(master, 0.0, 0.0, vertical_m_s)
        send_body_yaw_rate_nudge(master, direction * yaw_rate_deg_s)
    elif axis == "altitude":
        # Altitude calibration is a gentle height-hold check. It corrects back
        # toward the 5m rangefinder target instead of deliberately pumping the
        # throttle up and down.
        send_body_velocity_nudge(master, 0.0, 0.0, vertical_m_s)


def compute_lidar_body_nudge(snapshot, config):
    distance_m = snapshot.filtered_distance_m or snapshot.min_distance_m
    if distance_m <= 0.0 or distance_m >= config.trigger_distance_m:
        return None
    angle_rad = math.radians(snapshot.min_azimuth_deg + config.angle_offset_deg)
    strength = min(
        config.max_speed_m_s,
        config.max_speed_m_s * (config.trigger_distance_m - distance_m) / max(config.trigger_distance_m, 0.1),
    )
    return -math.cos(angle_rad) * strength, -math.sin(angle_rad) * strength, distance_m


def run_standby(config: SlamBridgeConfig) -> None:
    while True:
        connection = connect_to_cube_with_retry(config)
        imu_reader = open_imu_with_retry(config)
        lidar_reader = open_lidar_with_retry(config)
        qgc_bridge = open_qgc_bridge(config)
        send_companion_heartbeat(connection.master)
        ensure_fc_setup(connection, config)

        try:
            while True:
                msg = connection.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
                if qgc_bridge is not None:
                    qgc_bridge.forward_downlink(msg)
                    qgc_bridge.forward_uplink_to_cube(connection.master)
        except Exception:
            sleep_with_floor(2.0)
        finally:
            close_cube_connection(connection)


def run_bridge(config: SlamBridgeConfig) -> None:
    period_s = 1.0 / max(config.rate_hz, 0.1)

    while True:
        source = None
        connection = connect_to_cube_with_retry(config)
        imu_reader = open_imu_with_retry(config)
        lidar_reader = open_lidar_with_retry(config)
        qgc_bridge = open_qgc_bridge(config)
        send_companion_heartbeat(connection.master)
        ensure_fc_setup(connection, config)
        configure_telemetry_streams(connection.master)
        fc_state = FlightControllerTelemetry(
            active_source_set=request_active_source_set(connection.master),
            last_heartbeat_s=time.time(),
        )

        try:
            source = make_pose_source(config.source, config.csv_path)
        except Exception as exc:  # noqa: BLE001
            reason = f"VIO/camera unavailable: {exc}"
            print(reason)
            record_calibration_status(
                config,
                "failed",
                fc_state,
                None,
                None,
                action="source startup failed",
                reason=reason,
            )
            send_calibration_failed_beeps(connection.master)
            send_calibration_gcs_message(
                connection.master,
                f"Calibration failed: not finished. Reason: {reason}",
                severity=mavutil.mavlink.MAV_SEVERITY_ERROR,
            )
            publish_bridge_state(connection.master, BRIDGE_STATE_IDLE, config.fc_setup.idle_source_set)
            close_cube_connection(connection)
            sleep_with_floor(config.reconnect_delay_seconds)
            continue

        calibration_profile = (
            load_calibration_profile(config.calibration.profile_path)
            if config.calibration.profile_path
            else CalibrationProfile()
        )
        calibration_accumulator = CalibrationAccumulator(
            config.calibration.mode,
            config.calibration.duration_s,
            config.calibration.min_samples,
        )

        sent_count = 0
        last_switch_attempt_s = 0.0
        last_release_attempt_s = 0.0
        last_heartbeat_s = 0.0
        last_status_s = 0.0
        last_obstacle_publish_s = 0.0
        last_gps_input_s = 0.0
        last_waiting_message_s = 0.0
        last_active_beep_s = 0.0
        last_slam_flight_ping_s = 0.0
        last_announced_mode = ""
        last_announced_status_text = ""
        last_status_notice_s = 0.0
        mavlink_detected_s = time.time()
        startup_announced = False
        quick_check_announced = False
        ready_announced = False
        gps_input_announced = False
        gps_input_origin_missing_announced = False
        brake_announced = False
        ground_warning_announced = False
        height_announced = False
        calibration_complete_this_mode = False
        calibration_stage = "idle"
        calibration_axes = ("pitch", "roll", "yaw", "altitude")
        calibration_axis_index = 0
        calibration_stage_start_s = 0.0
        calibration_total_start_s = 0.0
        calibration_reference_pose = None
        latest_imu_sample = None
        source_started_published = False

        send_gcs_event(connection.master, "VIO service ready. Brake calibration monitor active.")
        if config.calibration.movement_commands_enabled:
            send_gcs_event(
                connection.master,
                "Active calibration motion ENABLED: gentle pitch/roll/yaw and 5m rangefinder height hold may be commanded.",
                severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
        else:
            send_gcs_event(
                connection.master,
                "Active calibration motion disabled: Jetson will observe Brake hold but will not command nudges.",
            )
        publish_bridge_state(connection.master, BRIDGE_STATE_JETSON_BOOT, config.fc_setup.idle_source_set)
        if config.calibration.dry_run:
            send_gcs_event(
                connection.master,
                "SLAM dry-run monitor active: no odometry, motion, fallback, or RTL commands.",
            )

        def fail_calibration(reason: str, change_mode: bool = True) -> None:
            nonlocal calibration_stage
            nonlocal calibration_complete_this_mode
            nonlocal calibration_reference_pose
            calibration_stage = "failed"
            calibration_complete_this_mode = True
            calibration_reference_pose = None
            record_calibration_status(
                config,
                calibration_stage,
                fc_state,
                pose,
                latest_imu_sample,
                action="failure",
                reason=reason,
            )
            send_calibration_failure(
                connection,
                config,
                reason,
                change_mode=(change_mode and not config.calibration.dry_run),
            )
            publish_bridge_state(connection.master, BRIDGE_STATE_IDLE, config.fc_setup.idle_source_set)

        def announce_waiting(text: str, severity=mavutil.mavlink.MAV_SEVERITY_INFO) -> None:
            nonlocal last_waiting_message_s
            if time.time() - last_waiting_message_s >= 10.0:
                send_calibration_gcs_message(connection.master, text, severity=severity)
                last_waiting_message_s = time.time()

        def start_axis(axis: str) -> None:
            nonlocal calibration_stage_start_s
            calibration_stage_start_s = time.time()
            send_calibration_gcs_message(connection.master, f"{axis.capitalize()} axis calibration started.")

        try:
            while True:
                loop_started_s = time.time()
                drain_fc_telemetry(connection.master, fc_state, qgc_bridge)
                if qgc_bridge is not None:
                    qgc_bridge.forward_uplink_to_cube(connection.master)
                now_s = time.time()
                heartbeat_age_s = now_s - float(fc_state.last_heartbeat_s or 0.0)
                if heartbeat_age_s > max(config.heartbeat_timeout_seconds, 1.0):
                    raise RuntimeError(
                        "MAVLink heartbeat timeout"
                        f" age={heartbeat_age_s:.1f}s port={connection.port}; reconnecting"
                    )

                if hasattr(source, "set_external_height_m"):
                    source.set_external_height_m(
                        fc_state.rangefinder_distance_m if rangefinder_height_valid(fc_state) else None
                    )

                raw_pose = source.sample()
                if imu_reader is not None:
                    imu_sample = imu_reader.poll(duration_s=min(0.02, period_s))
                    if imu_sample is not None:
                        latest_imu_sample = imu_sample
                        raw_pose = apply_imu_sample_to_pose(raw_pose, imu_sample)
                raw_pose = apply_cube_rangefinder_height(raw_pose, fc_state, config)
                pose = apply_calibration_profile(raw_pose, calibration_profile)
                now_s = time.time()
                calibration_mode_requested = in_brake_calibration_mode(fc_state, config)

                if (
                    gps_input_stream_requested(
                        pose,
                        fc_state,
                        config,
                        calibration_profile,
                        calibration_mode_requested,
                    )
                    and now_s - last_gps_input_s >= 0.2
                ):
                    if config.gps_input.fixed_fix:
                        send_fixed_gps_input(connection.master, config.gps_input)
                        if not gps_input_announced:
                            send_gcs_event(
                                connection.master,
                                f"diagnostic fake GPS{config.gps_input.gps_id + 1} fix streaming.",
                                severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
                            )
                            gps_input_announced = True
                    else:
                        sent_gps_input = send_gps_input_from_pose(connection.master, pose, config.gps_input)
                        if sent_gps_input and not gps_input_announced:
                            send_gcs_event(
                                connection.master,
                                f"VIO mirrored to GPS{config.gps_input.gps_id + 1} GPS_INPUT.",
                            )
                            gps_input_announced = True
                        elif not sent_gps_input and not gps_input_origin_missing_announced:
                            send_gcs_event(
                                connection.master,
                                "GPS_INPUT disabled: set gps_input origin lat/lon first.",
                                severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
                            )
                            gps_input_origin_missing_announced = True
                    last_gps_input_s = now_s

                if fc_state.flight_mode != last_announced_mode:
                    message, severity = mode_event_message(pose, fc_state, config, calibration_profile)
                    send_gcs_event(connection.master, message, severity=severity)
                    last_announced_mode = fc_state.flight_mode

                if (
                    fc_state.status_text
                    and fc_state.status_text != last_announced_status_text
                    and now_s - last_status_notice_s >= 5.0
                ):
                    status_lower = fc_state.status_text.lower()
                    if "gps2" in status_lower and "bad fix" in status_lower:
                        if config.gps_input.enabled and config.gps_input.gps_id == 1:
                            send_gcs_event(
                                connection.master,
                                "FC reports GPS2 bad fix while Jetson GPS2 is streaming; reboot FC if GPS2_TYPE just changed.",
                                severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
                            )
                        else:
                            send_gcs_event(
                                connection.master,
                                "FC reports GPS2 bad fix. GPS2 is disabled by this config; reboot FC if the warning persists.",
                                severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
                            )
                    elif "visodom" in status_lower and "out of memory" in status_lower:
                        send_gcs_event(
                            connection.master,
                            "FC reports VisOdom out of memory: true GPS-less PosHold cannot be enabled until the FC accepts VISO_TYPE.",
                            severity=mavutil.mavlink.MAV_SEVERITY_ERROR,
                        )
                    elif "visodom" in status_lower and "unhealthy" in status_lower:
                        send_gcs_event(
                            connection.master,
                            "FC reports VisOdom unhealthy: holding current EKF source; check ODOMETRY stream and VISO params.",
                            severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
                        )
                    else:
                        send_gcs_event(
                            connection.master,
                            f"FC status: {fc_state.status_text}",
                            severity=fc_state.status_severity or mavutil.mavlink.MAV_SEVERITY_INFO,
                        )
                    last_announced_status_text = fc_state.status_text
                    last_status_notice_s = now_s

                if not startup_announced and now_s - mavlink_detected_s >= 60.0:
                    send_startup_beeps(connection.master)
                    startup_announced = True

                if not quick_check_announced and sensor_quick_check_ok(
                    pose,
                    latest_imu_sample,
                    fc_state,
                    config,
                ):
                    send_sensor_check_beep(connection.master)
                    publish_bridge_state(connection.master, BRIDGE_STATE_SENSOR_CHECK_PASSED, config.fc_setup.idle_source_set)
                    quick_check_announced = True

                if (
                    not ready_announced
                    and bridge_ready_for_poshold(pose, fc_state, config, calibration_profile)
                ):
                    send_ready_beeps(connection.master)
                    publish_bridge_state(connection.master, BRIDGE_STATE_POSHOLD_READY, config.fc_setup.slam_source_set)
                    ready_announced = True

                active_stage = calibration_stage in calibration_axes

                if not calibration_mode_requested:
                    if active_stage and not calibration_complete_this_mode:
                        fail_calibration("mode changed by pilot", change_mode=False)
                    elif brake_announced:
                        send_calibration_gcs_message(
                            connection.master,
                            "Brake calibration monitor disengaged; normal flight modes restored.",
                        )
                        publish_bridge_state(
                            connection.master,
                            BRIDGE_STATE_IDLE,
                            config.fc_setup.idle_source_set,
                        )
                    brake_announced = False
                    ground_warning_announced = False
                    height_announced = False
                    calibration_accumulator.reset()
                    if calibration_stage not in {"complete", "failed"}:
                        calibration_stage = "idle"
                    calibration_reference_pose = None
                    calibration_axis_index = 0
                else:
                    if not brake_announced:
                        calibration_stage = "brake_detected"
                        send_calibration_gcs_message(
                            connection.master,
                            "Brake mode: SLAM calibration fused with Brake mode is active.",
                        )
                        if not config.calibration.dry_run and config.fc_setup.idle_source_set > 0:
                            if set_ekf_source_set(connection.master, config.fc_setup.idle_source_set, timeout_s=1.5) is True:
                                fc_state.active_source_set = config.fc_setup.idle_source_set
                                send_gcs_event(
                                    connection.master,
                                    f"GPS/EKF source set {config.fc_setup.idle_source_set} selected for calibration reference.",
                                )
                        publish_bridge_state(
                            connection.master,
                            BRIDGE_STATE_CALIBRATION_WAITING_ARM,
                            config.fc_setup.idle_source_set,
                        )
                        brake_announced = True

                    if not fc_state.armed:
                        calibration_stage = "waiting_arm"
                        calibration_accumulator.reset()
                        announce_waiting(
                            "Brake mode detected. Waiting for arm to start SLAM calibration."
                        )
                    elif vehicle_on_ground_for_calibration(fc_state, config):
                        calibration_stage = "waiting_takeoff"
                        calibration_accumulator.reset()
                        if not ground_warning_announced:
                            send_ground_calibration_warning_beeps(connection.master)
                            send_calibration_gcs_message(
                                connection.master,
                                "Armed in Brake mode. SLAM calibration takeoff sequence active.",
                            )
                            send_calibration_gcs_message(
                                connection.master,
                                "No automatic takeoff: pilot must take off and enter Brake near 5m.",
                            )
                            ground_warning_announced = True
                    elif calibration_complete_this_mode:
                        pass
                    else:
                        block_reason = calibration_block_reason(raw_pose, fc_state, config)
                        if block_reason is not None:
                            if calibration_accumulator.active():
                                fail_calibration(block_reason)
                            else:
                                calibration_stage = "ground_precheck"
                                announce_waiting(
                                    f"SLAM calibration waiting: {block_reason}",
                                    severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
                                )
                        elif not rangefinder_at_calibration_height(fc_state, config):
                            calibration_stage = "waiting_height"
                            current_height = (
                                "unknown"
                                if fc_state.rangefinder_distance_m is None
                                else f"{fc_state.rangefinder_distance_m:.2f}m"
                            )
                            announce_waiting(
                                "Brake calibration waiting for rangefinder"
                                f" {config.calibration.target_height_m:.1f}m AGL; current={current_height}."
                            )
                        else:
                            if not calibration_accumulator.active():
                                calibration_stage = "hold_height"
                                calibration_axis_index = 0
                                calibration_total_start_s = now_s
                                calibration_reference_pose = raw_pose
                                calibration_accumulator.start()
                                send_calibration_gcs_message(
                                    connection.master,
                                    "Reached 5 meters by rangefinder. Holding altitude for SLAM calibration.",
                                )
                                publish_bridge_state(
                                    connection.master,
                                    BRIDGE_STATE_CALIBRATION_ACTIVE,
                                    config.fc_setup.idle_source_set,
                                )
                                if config.calibration.movement_commands_enabled and not config.calibration.dry_run:
                                    send_calibration_gcs_message(
                                        connection.master,
                                        "Active calibration nudges enabled: gentle pitch, roll, yaw, and height-hold checks starting.",
                                        severity=mavutil.mavlink.MAV_SEVERITY_WARNING,
                                    )
                                send_calibration_active_beeps(connection.master)
                                last_active_beep_s = now_s
                                height_announced = True
                                calibration_stage = calibration_axes[calibration_axis_index]
                                start_axis(calibration_stage)

                            if calibration_stage in calibration_axes:
                                stage_elapsed_s = now_s - calibration_stage_start_s
                                total_elapsed_s = now_s - calibration_total_start_s
                                drift_m = calibration_drift_m(calibration_reference_pose, raw_pose)

                                if total_elapsed_s > config.calibration.total_timeout_s:
                                    fail_calibration("calibration timeout")
                                elif drift_m > config.calibration.max_stage_drift_m:
                                    fail_calibration(f"excessive drift {drift_m:.2f}m")
                                elif stage_elapsed_s > config.calibration.axis_stage_timeout_s:
                                    fail_calibration(f"{calibration_stage} axis timeout")
                                else:
                                    if now_s - last_active_beep_s >= 10.0:
                                        send_calibration_active_beeps(connection.master)
                                        publish_bridge_state(
                                            connection.master,
                                            BRIDGE_STATE_CALIBRATION_ACTIVE,
                                            config.fc_setup.idle_source_set,
                                        )
                                        send_calibration_gcs_message(connection.master, "SLAM calibration active.")
                                        last_active_beep_s = now_s

                                    reference_yaw_deg = math.degrees(float(getattr(fc_state.attitude, "yaw", 0.0)))
                                    calibration_accumulator.collect(
                                        float(getattr(fc_state.local_position, "x", 0.0)),
                                        float(getattr(fc_state.local_position, "y", 0.0)),
                                        reference_yaw_deg,
                                        float(fc_state.rangefinder_distance_m or 0.0),
                                        raw_pose,
                                    )

                                    if (
                                        config.calibration.movement_commands_enabled
                                        and not config.calibration.dry_run
                                    ):
                                        send_calibration_axis_motion(
                                            connection.master,
                                            calibration_stage,
                                            stage_elapsed_s,
                                            fc_state,
                                            config,
                                        )

                                    if stage_elapsed_s >= config.calibration.axis_stage_duration_s:
                                        completed_axis = calibration_stage
                                        send_calibration_gcs_message(
                                            connection.master,
                                            f"{completed_axis.capitalize()} axis calibration complete.",
                                        )
                                        calibration_axis_index += 1

                                        if calibration_axis_index < len(calibration_axes):
                                            calibration_stage = calibration_axes[calibration_axis_index]
                                            start_axis(calibration_stage)
                                        else:
                                            candidate_profile = calibration_accumulator.build_profile()
                                            if not calibration_profile_stable(candidate_profile):
                                                fail_calibration(
                                                    "calibration unstable"
                                                    f" yaw_std={candidate_profile.yaw_std_deg:.1f}deg"
                                                    f" x_std={candidate_profile.x_std_m:.2f}m"
                                                    f" y_std={candidate_profile.y_std_m:.2f}m"
                                                )
                                            else:
                                                final_reason = calibration_block_reason(raw_pose, fc_state, config)
                                                if final_reason is not None:
                                                    fail_calibration(final_reason)
                                                elif config.calibration.dry_run:
                                                    calibration_complete_this_mode = True
                                                    calibration_stage = "complete"
                                                    record_calibration_status(
                                                        config,
                                                        calibration_stage,
                                                        fc_state,
                                                        pose,
                                                        latest_imu_sample,
                                                        action="dry-run success",
                                                    )
                                                    send_calibration_complete_beeps(connection.master)
                                                    send_calibration_gcs_message(
                                                        connection.master,
                                                        "Calibration successful: SLAM PosHold calibration complete. Initiating RTL.",
                                                    )
                                                    send_calibration_gcs_message(
                                                        connection.master,
                                                        "Dry-run active: profile not saved and RTL not commanded.",
                                                    )
                                                elif not config.calibration.auto_rtl_after_complete:
                                                    save_calibration_profile(
                                                        config.calibration.profile_path,
                                                        candidate_profile,
                                                    )
                                                    calibration_profile = candidate_profile
                                                    calibration_complete_this_mode = True
                                                    calibration_stage = "complete"
                                                    record_calibration_status(
                                                        config,
                                                        calibration_stage,
                                                        fc_state,
                                                        pose,
                                                        latest_imu_sample,
                                                        action="success; rtl disabled",
                                                    )
                                                    send_calibration_complete_beeps(connection.master)
                                                    send_calibration_gcs_message(
                                                        connection.master,
                                                        "Calibration successful: SLAM PosHold calibration complete. Initiating RTL.",
                                                    )
                                                    send_calibration_gcs_message(
                                                        connection.master,
                                                        "Auto RTL disabled; pilot should exit Brake manually.",
                                                    )
                                                elif not mode_available(connection.master, "RTL"):
                                                    fail_calibration("RTL mode is not available", change_mode=False)
                                                else:
                                                    save_calibration_profile(
                                                        config.calibration.profile_path,
                                                        candidate_profile,
                                                    )
                                                    calibration_profile = candidate_profile
                                                    calibration_complete_this_mode = True
                                                    calibration_stage = "complete"
                                                    record_calibration_status(
                                                        config,
                                                        calibration_stage,
                                                        fc_state,
                                                        pose,
                                                        latest_imu_sample,
                                                        action="success; commanding RTL",
                                                    )
                                                    send_calibration_complete_beeps(connection.master)
                                                    publish_bridge_state(
                                                        connection.master,
                                                        BRIDGE_STATE_CALIBRATION_COMPLETE_RTL,
                                                        config.fc_setup.idle_source_set,
                                                    )
                                                    send_calibration_gcs_message(
                                                        connection.master,
                                                        "Calibration successful: SLAM PosHold calibration complete. Initiating RTL.",
                                                    )
                                                    if set_vehicle_mode(connection.master, "RTL") is not True:
                                                        fail_calibration("RTL mode change not accepted", change_mode=False)

                pose_safe_now = pose_safe_for_fc(pose, fc_state, config)
                slam_stream_requested = odometry_stream_requested(
                    fc_state,
                    config,
                    calibration_mode_requested,
                )
                if slam_stream_requested and pose_safe_now:
                    if not config.calibration.dry_run:
                        send_odometry(connection, pose)
                        sent_count += 1
                    if not source_started_published:
                        if config.calibration.dry_run:
                            send_gcs_event(
                                connection.master,
                                "SLAM dry-run pose stream healthy; ODOMETRY not sent.",
                            )
                        else:
                            send_gcs_event(connection.master, "SLAM ODOMETRY stream healthy.")
                        publish_bridge_state(connection.master, BRIDGE_STATE_SLAM_STARTED, config.fc_setup.idle_source_set)
                        source_started_published = True
                elif pose_safe_now and not source_started_published:
                    send_gcs_event(
                        connection.master,
                        "SLAM pose estimate healthy; monitoring only until PosHold or Brake requests it.",
                    )
                    source_started_published = True

                wants_poshold_slam = (
                    not config.calibration.dry_run
                    and mode_wants_slam(fc_state, config)
                    and bridge_ready_for_poshold(pose, fc_state, config, calibration_profile)
                    and sent_count >= config.fc_setup.switch_after_sends
                )
                if (
                    wants_poshold_slam
                    and config.fc_setup.select_source_set_on_stream
                    and fc_state.active_source_set != config.fc_setup.slam_source_set
                    and now_s - last_switch_attempt_s >= 2.0
                ):
                    result = set_ekf_source_set(connection.master, config.fc_setup.slam_source_set)
                    if result is True:
                        fc_state.active_source_set = config.fc_setup.slam_source_set
                        publish_bridge_state(connection.master, BRIDGE_STATE_SOURCE_SET_ACTIVE, config.fc_setup.slam_source_set)
                        send_gcs_event(
                            connection.master,
                            f"SLAM/ExternalNav source set {config.fc_setup.slam_source_set} active.",
                        )
                    elif result is False:
                        publish_bridge_state(connection.master, BRIDGE_STATE_SOURCE_SWITCH_FAILED, config.fc_setup.slam_source_set)
                    else:
                        publish_bridge_state(connection.master, BRIDGE_STATE_SOURCE_SWITCH_NO_ACK, config.fc_setup.slam_source_set)
                    last_switch_attempt_s = now_s

                if (
                    not config.calibration.dry_run
                    and not mode_wants_slam(fc_state, config)
                    and not calibration_mode_requested
                    and fc_state.active_source_set == config.fc_setup.slam_source_set
                    and now_s - last_release_attempt_s >= 2.0
                ):
                    if set_ekf_source_set(connection.master, config.fc_setup.idle_source_set) is True:
                        fc_state.active_source_set = config.fc_setup.idle_source_set
                        publish_bridge_state(connection.master, BRIDGE_STATE_SOURCE_SET_ACTIVE, config.fc_setup.idle_source_set)
                    last_release_attempt_s = now_s

                if active_slam_flight(pose, fc_state, config, calibration_profile):
                    if now_s - last_slam_flight_ping_s >= 6.0:
                        send_slam_flight_ping(connection.master)
                        publish_bridge_state(connection.master, BRIDGE_STATE_SLAM_FLIGHT_ACTIVE, config.fc_setup.slam_source_set)
                        last_slam_flight_ping_s = now_s

                if lidar_reader is not None and now_s - last_obstacle_publish_s >= 1.0 / max(config.obstacle.publish_rate_hz, 0.1):
                    try:
                        snapshot = lidar_reader.read_snapshot()
                        send_obstacle_distance(connection.master, snapshot.distances_m, config.obstacle.max_distance_m)
                    except Exception as exc:  # noqa: BLE001
                        print(f"LiDAR obstacle publish skipped: {exc}")
                    last_obstacle_publish_s = now_s

                if now_s - last_heartbeat_s >= 1.0:
                    send_companion_heartbeat(connection.master)
                    last_heartbeat_s = now_s

                if now_s - last_status_s >= max(config.status_log_seconds, 1.0):
                    print_calibration_status(
                        calibration_stage,
                        fc_state,
                        pose,
                        latest_imu_sample,
                        config,
                        sent_count=sent_count,
                        reason=pose_safety_reason(pose, fc_state, config) or "",
                    )
                    last_status_s = now_s

                remaining_s = period_s - (time.time() - loop_started_s)
                if remaining_s > 0:
                    time.sleep(remaining_s)
        except Exception as exc:
            print(f"Bridge loop interrupted: {exc}")
            sleep_with_floor(2.0)
        finally:
            try:
                publish_bridge_state(connection.master, BRIDGE_STATE_IDLE, config.fc_setup.idle_source_set)
            except Exception:
                pass
            if source is not None and hasattr(source, "close"):
                try:
                    source.close()
                except Exception:
                    pass
            close_cube_connection(connection)


def main() -> None:
    args = parse_args()
    config = resolve_config(args)
    sleep_until_boot_delay(config)
    if config.source == "standby" and not config.connect_in_standby:
        run_standby(config)
        return
    run_bridge(config)


if __name__ == "__main__":
    main()
