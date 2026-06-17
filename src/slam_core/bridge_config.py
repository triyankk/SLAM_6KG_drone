"""YAML-backed configuration model for the SLAM bridge.

The service loads `config/autostart.yaml`, maps it into these dataclasses, and
then passes the strongly-typed object through the bridge. A lot of flight safety
comes from defaults here: if a key is absent, the default should be cautious and
should not command motion unexpectedly.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from .fc_config import FlightControllerSetupConfig
from .gps_denied_readiness import GpsDeniedReadinessConfig


def _default_ports() -> list[str]:
    return ["/dev/ttyACM1", "/dev/ttyACM0"]


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Cannot interpret boolean value from {value!r}")


@dataclass
class ObstacleAvoidanceConfig:
    """LiDAR/proximity publisher settings.

    Publishing obstacle distances is separate from commanding movement. This
    default only publishes proximity/obstacle data to ArduPilot.
    """

    enabled: bool = True
    lidar_enabled: bool = True
    lidar_port: str = "auto"
    lidar_baud: int = 3000000
    safety_distance_m: float = 1.5
    publish_rate_hz: float = 5.0
    sensor_id: int = 20
    sector_count: int = 72
    max_distance_m: float = 40.0
    min_valid_distance_m: float = 0.15
    filter_samples: int = 15


@dataclass
class QgcBridgeConfig:
    enabled: bool = True
    forward_host: str = "127.0.0.1"
    forward_port: int = 14550
    bind_host: str = "0.0.0.0"
    bind_port: int = 14555


@dataclass
class GpsInputConfig:
    """MAVLink GPS_INPUT settings for the GPS2 bridge path.

    `gps_id=1` means ArduPilot receives this as GPS2. `fixed_fix` is diagnostic
    only; flight code should use an origin plus live pose or standby GPS mirror.
    """

    enabled: bool = False
    gps_id: int = 1
    fixed_fix: bool = False
    fixed_lat_deg: float = 37.7749
    fixed_lon_deg: float = -122.4194
    fixed_alt_m: float = 10.0
    origin_lat_deg: float = 0.0
    origin_lon_deg: float = 0.0
    origin_alt_m: float = 0.0
    satellites_visible: int = 14
    horiz_accuracy_m: float = 0.5
    vert_accuracy_m: float = 1.0
    speed_accuracy_m_s: float = 0.3
    update_rate_hz: float = 8.0


@dataclass
class ExternalPoseConfig:
    """UDP JSON pose input for a ROS/Cartographer/OpenVINS sidecar."""

    bind_host: str = "127.0.0.1"
    bind_port: int = 15560
    max_age_s: float = 0.35
    first_sample_timeout_s: float = 3.0


@dataclass
class LidarSteeringConfig:
    enabled: bool = False
    trigger_distance_m: float = 2.0
    clear_distance_m: float = 2.4
    max_speed_m_s: float = 0.5
    angle_offset_deg: float = 0.0
    allowed_modes: list[str] = field(default_factory=lambda: ["GUIDED"])


@dataclass
class CalibrationConfig:
    """Real Brake-mode calibration workflow.

    This workflow is deliberately separate from the LOITER observer. Brake mode
    is the explicit pilot trigger for real calibration, while LOITER only learns
    confidence and bounded soft-correction estimates in the background.
    """

    enabled: bool = True
    mode: str = "BRAKE"
    duration_s: float = 12.0
    min_samples: int = 80
    min_gps_fix_type: int = 3
    min_gps_satellites: int = 8
    max_horizontal_speed_m_s: float = 0.35
    max_roll_deg: float = 7.0
    max_pitch_deg: float = 7.0
    min_pose_quality: int = 55
    profile_path: str = "runtime/slam_calibration.json"
    auto_rtl_after_complete: bool = True
    fallback_mode: str = "BRAKE"
    target_height_m: float = 5.0
    target_height_tolerance_m: float = 0.35
    ground_max_height_m: float = 0.7
    height_stage_timeout_s: float = 120.0
    axis_stage_duration_s: float = 8.0
    axis_stage_timeout_s: float = 25.0
    total_timeout_s: float = 240.0
    max_stationary_drift_m: float = 0.35
    max_stage_drift_m: float = 1.0
    require_rc_link: bool = True
    movement_commands_enabled: bool = False
    movement_speed_m_s: float = 0.12
    vertical_speed_m_s: float = 0.08
    altitude_hold_gain: float = 0.25
    altitude_hold_deadband_m: float = 0.12
    yaw_rate_deg_s: float = 6.0
    dry_run: bool = False
    kill_switch_confirmed: bool = False
    min_battery_remaining_pct: int = 20
    status_path: str = "logs/slam_calibration_status.json"
    log_path: str = "logs/slam_calibration.log"


@dataclass
class SoftCalibrationConfig:
    enabled: bool = True
    mode: str = "LOITER"
    duration_s: float = 20.0
    min_samples: int = 120
    min_gps_fix_type: int = 3
    min_gps_satellites: int = 8
    min_pose_quality: int = 45
    announce_interval_s: float = 10.0
    ready_score_threshold: float = 7.0
    low_score_threshold: float = 5.0
    critical_score_threshold: float = 3.0
    save_improvement_threshold: float = 0.3
    profile_path: str = "runtime/loiter_soft_calibration.json"
    sample_log_path: str = "logs/loiter_soft_learning.jsonl"
    sample_log_hz: float = 1.0
    fallback_enabled: bool = True
    fallback_mode: str = "LOITER"


@dataclass
class SlamObserverConfig:
    """GPS-assisted LOITER observation and GPS2 gating settings."""

    enable_loiter_observation: bool = True
    observation_message_interval_sec: float = 20.0
    min_quality_for_poshold: float = 7.0
    weak_quality_threshold: float = 5.0
    critical_quality_threshold: float = 3.0
    quality_update_delta: float = 0.5
    enable_live_soft_correction: bool = False
    enable_auto_fallback_to_loiter: bool = False
    log_observation_data: bool = True
    log_path: str = "logs/slam_loiter_observer.log"
    status_path: str = "logs/slam_loiter_observer_status.json"


@dataclass
class SlamBridgeConfig:
    """Top-level configuration consumed by the long-running bridge service."""

    ports: list[str] = field(default_factory=_default_ports)
    baud: int = 115200
    source: str = "standby"
    csv_path: str = ""
    rate_hz: float = 15.0
    imu_enabled: bool = True
    imu_port: str = "auto"
    imu_baud: str = "auto"
    imu_scan_seconds: float = 0.8
    cube_retry_seconds: float = 3.0
    reconnect_delay_seconds: float = 2.0
    standby_log_seconds: float = 20.0
    status_log_seconds: float = 10.0
    heartbeat_timeout_seconds: float = 8.0
    connect_in_standby: bool = True
    boot_delay_seconds: float = 45.0
    fc_setup: FlightControllerSetupConfig = field(default_factory=FlightControllerSetupConfig)
    obstacle: ObstacleAvoidanceConfig = field(default_factory=ObstacleAvoidanceConfig)
    qgc: QgcBridgeConfig = field(default_factory=QgcBridgeConfig)
    gps_input: GpsInputConfig = field(default_factory=GpsInputConfig)
    external_pose: ExternalPoseConfig = field(default_factory=ExternalPoseConfig)
    gps_denied: GpsDeniedReadinessConfig = field(default_factory=GpsDeniedReadinessConfig)
    lidar_steering: LidarSteeringConfig = field(default_factory=LidarSteeringConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    soft_calibration: SoftCalibrationConfig = field(default_factory=SoftCalibrationConfig)
    slam_observer: SlamObserverConfig = field(default_factory=SlamObserverConfig)
    # Robustness settings for announcing the SLAM stream
    stream_stable_s: float = 2.0
    stream_loss_hysteresis_s: float = 5.0

    @classmethod
    def from_mapping(cls, data: dict[str, Any], base_dir: Path | None = None) -> "SlamBridgeConfig":
        base_dir = base_dir or Path.cwd()
        csv_path = str(data.get("csv_path", "") or "")
        if csv_path and not Path(csv_path).is_absolute():
            csv_path = str((base_dir / csv_path).resolve())

        ports = data.get("ports", _default_ports())
        if not isinstance(ports, list) or not ports:
            raise ValueError("config 'ports' must be a non-empty list")

        imu_value = data.get("imu_enabled", data.get("imu", True))
        fc_setup_data = data.get("fc_setup", {}) or {}
        if not isinstance(fc_setup_data, dict):
            raise ValueError("config 'fc_setup' must be a mapping")
        obstacle_data = data.get("obstacle", {}) or {}
        if not isinstance(obstacle_data, dict):
            raise ValueError("config 'obstacle' must be a mapping")
        qgc_data = data.get("qgc", {}) or {}
        if not isinstance(qgc_data, dict):
            raise ValueError("config 'qgc' must be a mapping")
        gps_input_data = data.get("gps_input", {}) or {}
        if not isinstance(gps_input_data, dict):
            raise ValueError("config 'gps_input' must be a mapping")
        external_pose_data = data.get("external_pose", {}) or {}
        if not isinstance(external_pose_data, dict):
            raise ValueError("config 'external_pose' must be a mapping")
        gps_denied_data = data.get("gps_denied", {}) or {}
        if not isinstance(gps_denied_data, dict):
            raise ValueError("config 'gps_denied' must be a mapping")
        lidar_steering_data = data.get("lidar_steering", {}) or {}
        if not isinstance(lidar_steering_data, dict):
            raise ValueError("config 'lidar_steering' must be a mapping")
        calibration_data = data.get("calibration", {}) or {}
        if not isinstance(calibration_data, dict):
            raise ValueError("config 'calibration' must be a mapping")
        soft_calibration_data = data.get("soft_calibration", {}) or {}
        if not isinstance(soft_calibration_data, dict):
            raise ValueError("config 'soft_calibration' must be a mapping")
        slam_observer_data = data.get("slam_observer", {}) or {}
        if not isinstance(slam_observer_data, dict):
            raise ValueError("config 'slam_observer' must be a mapping")
        profile_path = str(calibration_data.get("profile_path", "runtime/slam_calibration.json") or "")
        if profile_path and not Path(profile_path).is_absolute():
            profile_path = str((base_dir / profile_path).resolve())
        status_path = str(calibration_data.get("status_path", "logs/slam_calibration_status.json") or "")
        if status_path and not Path(status_path).is_absolute():
            status_path = str((base_dir / status_path).resolve())
        log_path = str(calibration_data.get("log_path", "logs/slam_calibration.log") or "")
        if log_path and not Path(log_path).is_absolute():
            log_path = str((base_dir / log_path).resolve())
        soft_profile_path = str(
            soft_calibration_data.get("profile_path", "runtime/loiter_soft_calibration.json") or ""
        )
        if soft_profile_path and not Path(soft_profile_path).is_absolute():
            soft_profile_path = str((base_dir / soft_profile_path).resolve())
        soft_sample_log_path = str(
            soft_calibration_data.get("sample_log_path", "logs/loiter_soft_learning.jsonl") or ""
        )
        if soft_sample_log_path and not Path(soft_sample_log_path).is_absolute():
            soft_sample_log_path = str((base_dir / soft_sample_log_path).resolve())
        observer_log_path = str(
            slam_observer_data.get("log_path", "logs/slam_loiter_observer.log") or ""
        )
        if observer_log_path and not Path(observer_log_path).is_absolute():
            observer_log_path = str((base_dir / observer_log_path).resolve())
        observer_status_path = str(
            slam_observer_data.get("status_path", "logs/slam_loiter_observer_status.json") or ""
        )
        if observer_status_path and not Path(observer_status_path).is_absolute():
            observer_status_path = str((base_dir / observer_status_path).resolve())
        gps_denied_status_path = str(
            gps_denied_data.get("status_path", "logs/gps_denied_readiness.json") or ""
        )
        if gps_denied_status_path and not Path(gps_denied_status_path).is_absolute():
            gps_denied_status_path = str((base_dir / gps_denied_status_path).resolve())
        return cls(
            ports=[str(port) for port in ports],
            baud=int(data.get("baud", 115200)),
            source=str(data.get("source", "standby")),
            csv_path=csv_path,
            rate_hz=float(data.get("rate_hz", 15.0)),
            imu_enabled=_as_bool(imu_value),
            imu_port=str(data.get("imu_port", "auto")),
            imu_baud=str(data.get("imu_baud", "auto")),
            imu_scan_seconds=float(data.get("imu_scan_seconds", 0.8)),
            cube_retry_seconds=float(data.get("cube_retry_seconds", 3.0)),
            reconnect_delay_seconds=float(data.get("reconnect_delay_seconds", 2.0)),
            standby_log_seconds=float(data.get("standby_log_seconds", 20.0)),
            status_log_seconds=float(data.get("status_log_seconds", 10.0)),
            heartbeat_timeout_seconds=float(data.get("heartbeat_timeout_seconds", 8.0)),
            connect_in_standby=_as_bool(data.get("connect_in_standby", True)),
            boot_delay_seconds=float(data.get("boot_delay_seconds", 45.0)),
            fc_setup=FlightControllerSetupConfig(
                enabled=_as_bool(fc_setup_data.get("enabled", True)),
                slam_source_set=int(fc_setup_data.get("slam_source_set", 3)),
                idle_source_set=int(fc_setup_data.get("idle_source_set", 1)),
                switch_after_sends=int(fc_setup_data.get("switch_after_sends", 30)),
                select_source_set_on_stream=_as_bool(
                    fc_setup_data.get("select_source_set_on_stream", True)
                ),
                activate_mode=str(fc_setup_data.get("activate_mode", "POSHOLD")).upper(),
                ready_min_quality=int(fc_setup_data.get("ready_min_quality", 60)),
                require_rangefinder_height=_as_bool(fc_setup_data.get("require_rangefinder_height", True)),
                ahrs_ekf_type=int(fc_setup_data.get("ahrs_ekf_type", 3)),
                avoid_enable=int(fc_setup_data.get("avoid_enable", 7)),
                avoid_margin_m=float(fc_setup_data.get("avoid_margin_m", 1.5)),
                ek2_enable=int(fc_setup_data.get("ek2_enable", 0)),
                ek3_enable=int(fc_setup_data.get("ek3_enable", 1)),
                ek3_src_options=int(fc_setup_data.get("ek3_src_options", 0)),
                ek3_ogn_hgt_mask=int(fc_setup_data.get("ek3_ogn_hgt_mask", 0)),
                viso_type=int(fc_setup_data.get("viso_type", 0)),
                viso_pos_x_m=float(fc_setup_data.get("viso_pos_x_m", 0.0)),
                viso_pos_y_m=float(fc_setup_data.get("viso_pos_y_m", 0.0)),
                viso_pos_z_m=float(fc_setup_data.get("viso_pos_z_m", 0.0)),
                viso_qual_min=int(fc_setup_data.get("viso_qual_min", 0)),
                posxy_source=int(fc_setup_data.get("posxy_source", 6)),
                velxy_source=int(fc_setup_data.get("velxy_source", 6)),
                posz_source=int(fc_setup_data.get("posz_source", 1)),
                velz_source=int(fc_setup_data.get("velz_source", 0)),
                yaw_source=int(fc_setup_data.get("yaw_source", 1)),
                prx1_type=int(fc_setup_data.get("prx1_type", 2)),
                gps2_type=(
                    int(fc_setup_data["gps2_type"])
                    if fc_setup_data.get("gps2_type") is not None
                    else None
                ),
                gps_auto_switch=(
                    int(fc_setup_data["gps_auto_switch"])
                    if fc_setup_data.get("gps_auto_switch") is not None
                    else None
                ),
            ),
            obstacle=ObstacleAvoidanceConfig(
                enabled=_as_bool(obstacle_data.get("enabled", True)),
                lidar_enabled=_as_bool(obstacle_data.get("lidar_enabled", True)),
                lidar_port=str(obstacle_data.get("lidar_port", "auto")),
                lidar_baud=int(obstacle_data.get("lidar_baud", 3000000)),
                safety_distance_m=float(obstacle_data.get("safety_distance_m", 1.5)),
                publish_rate_hz=float(obstacle_data.get("publish_rate_hz", 5.0)),
                sensor_id=int(obstacle_data.get("sensor_id", 20)),
                sector_count=int(obstacle_data.get("sector_count", 72)),
                max_distance_m=float(obstacle_data.get("max_distance_m", 40.0)),
                min_valid_distance_m=float(obstacle_data.get("min_valid_distance_m", 0.15)),
                filter_samples=int(obstacle_data.get("filter_samples", 15)),
            ),
            qgc=QgcBridgeConfig(
                enabled=_as_bool(qgc_data.get("enabled", True)),
                forward_host=str(qgc_data.get("forward_host", "127.0.0.1")),
                forward_port=int(qgc_data.get("forward_port", 14550)),
                bind_host=str(qgc_data.get("bind_host", "0.0.0.0")),
                bind_port=int(qgc_data.get("bind_port", 14555)),
            ),
            gps_input=GpsInputConfig(
                enabled=_as_bool(gps_input_data.get("enabled", False)),
                gps_id=int(gps_input_data.get("gps_id", 1)),
                fixed_fix=_as_bool(gps_input_data.get("fixed_fix", False)),
                fixed_lat_deg=float(gps_input_data.get("fixed_lat_deg", 37.7749)),
                fixed_lon_deg=float(gps_input_data.get("fixed_lon_deg", -122.4194)),
                fixed_alt_m=float(gps_input_data.get("fixed_alt_m", 10.0)),
                origin_lat_deg=float(gps_input_data.get("origin_lat_deg", 0.0)),
                origin_lon_deg=float(gps_input_data.get("origin_lon_deg", 0.0)),
                origin_alt_m=float(gps_input_data.get("origin_alt_m", 0.0)),
                satellites_visible=int(gps_input_data.get("satellites_visible", 14)),
                horiz_accuracy_m=float(gps_input_data.get("horiz_accuracy_m", 0.5)),
                vert_accuracy_m=float(gps_input_data.get("vert_accuracy_m", 1.0)),
                speed_accuracy_m_s=float(gps_input_data.get("speed_accuracy_m_s", 0.3)),
                update_rate_hz=float(gps_input_data.get("update_rate_hz", 8.0)),
            ),
            external_pose=ExternalPoseConfig(
                bind_host=str(external_pose_data.get("bind_host", "127.0.0.1")),
                bind_port=int(external_pose_data.get("bind_port", 15560)),
                max_age_s=float(external_pose_data.get("max_age_s", 0.35)),
                first_sample_timeout_s=float(external_pose_data.get("first_sample_timeout_s", 3.0)),
            ),
            gps_denied=GpsDeniedReadinessConfig(
                enabled=_as_bool(gps_denied_data.get("enabled", True)),
                require_imu=_as_bool(gps_denied_data.get("require_imu", True)),
                require_rc_link=_as_bool(gps_denied_data.get("require_rc_link", True)),
                require_rangefinder=_as_bool(gps_denied_data.get("require_rangefinder", True)),
                require_ekf_status=_as_bool(gps_denied_data.get("require_ekf_status", True)),
                require_attitude=_as_bool(gps_denied_data.get("require_attitude", True)),
                require_local_position=_as_bool(gps_denied_data.get("require_local_position", True)),
                require_origin_for_gps_input=_as_bool(
                    gps_denied_data.get("require_origin_for_gps_input", True)
                ),
                require_calibration_or_observer=_as_bool(
                    gps_denied_data.get("require_calibration_or_observer", True)
                ),
                stable_seconds=float(gps_denied_data.get("stable_seconds", 3.0)),
                max_pose_dt_s=float(gps_denied_data.get("max_pose_dt_s", 0.35)),
                max_pose_jump_m=float(gps_denied_data.get("max_pose_jump_m", 1.25)),
                max_velocity_m_s=float(gps_denied_data.get("max_velocity_m_s", 5.0)),
                max_rangefinder_disagreement_m=float(
                    gps_denied_data.get("max_rangefinder_disagreement_m", 1.0)
                ),
                min_observer_score=float(gps_denied_data.get("min_observer_score", 7.0)),
                announce_interval_s=float(gps_denied_data.get("announce_interval_s", 10.0)),
                status_write_interval_s=float(gps_denied_data.get("status_write_interval_s", 1.0)),
                status_path=gps_denied_status_path,
            ),
            lidar_steering=LidarSteeringConfig(
                enabled=_as_bool(lidar_steering_data.get("enabled", False)),
                trigger_distance_m=float(lidar_steering_data.get("trigger_distance_m", 2.0)),
                clear_distance_m=float(lidar_steering_data.get("clear_distance_m", 2.4)),
                max_speed_m_s=float(lidar_steering_data.get("max_speed_m_s", 0.5)),
                angle_offset_deg=float(lidar_steering_data.get("angle_offset_deg", 0.0)),
                allowed_modes=[
                    str(mode).upper()
                    for mode in lidar_steering_data.get("allowed_modes", ["GUIDED"])
                ],
            ),
            calibration=CalibrationConfig(
                enabled=_as_bool(calibration_data.get("enabled", True)),
                mode=str(calibration_data.get("mode", "BRAKE")).upper(),
                duration_s=float(calibration_data.get("duration_s", 12.0)),
                min_samples=int(calibration_data.get("min_samples", 80)),
                min_gps_fix_type=int(calibration_data.get("min_gps_fix_type", 3)),
                min_gps_satellites=int(calibration_data.get("min_gps_satellites", 8)),
                max_horizontal_speed_m_s=float(
                    calibration_data.get("max_horizontal_speed_m_s", 0.35)
                ),
                max_roll_deg=float(calibration_data.get("max_roll_deg", 7.0)),
                max_pitch_deg=float(calibration_data.get("max_pitch_deg", 7.0)),
                min_pose_quality=int(calibration_data.get("min_pose_quality", 55)),
                profile_path=profile_path,
                auto_rtl_after_complete=_as_bool(
                    calibration_data.get("auto_rtl_after_complete", True)
                ),
                fallback_mode=str(calibration_data.get("fallback_mode", "BRAKE")).upper(),
                target_height_m=float(calibration_data.get("target_height_m", 5.0)),
                target_height_tolerance_m=float(calibration_data.get("target_height_tolerance_m", 0.35)),
                ground_max_height_m=float(calibration_data.get("ground_max_height_m", 0.7)),
                height_stage_timeout_s=float(calibration_data.get("height_stage_timeout_s", 120.0)),
                axis_stage_duration_s=float(calibration_data.get("axis_stage_duration_s", 8.0)),
                axis_stage_timeout_s=float(calibration_data.get("axis_stage_timeout_s", 25.0)),
                total_timeout_s=float(calibration_data.get("total_timeout_s", 240.0)),
                max_stationary_drift_m=float(calibration_data.get("max_stationary_drift_m", 0.35)),
                max_stage_drift_m=float(calibration_data.get("max_stage_drift_m", 1.0)),
                require_rc_link=_as_bool(calibration_data.get("require_rc_link", True)),
                movement_commands_enabled=_as_bool(
                    calibration_data.get("movement_commands_enabled", False)
                ),
                movement_speed_m_s=float(calibration_data.get("movement_speed_m_s", 0.12)),
                vertical_speed_m_s=float(calibration_data.get("vertical_speed_m_s", 0.08)),
                altitude_hold_gain=float(calibration_data.get("altitude_hold_gain", 0.25)),
                altitude_hold_deadband_m=float(calibration_data.get("altitude_hold_deadband_m", 0.12)),
                yaw_rate_deg_s=float(calibration_data.get("yaw_rate_deg_s", 6.0)),
                dry_run=_as_bool(calibration_data.get("dry_run", False)),
                kill_switch_confirmed=_as_bool(calibration_data.get("kill_switch_confirmed", False)),
                min_battery_remaining_pct=int(calibration_data.get("min_battery_remaining_pct", 20)),
                status_path=status_path,
                log_path=log_path,
            ),
            soft_calibration=SoftCalibrationConfig(
                enabled=_as_bool(soft_calibration_data.get("enabled", True)),
                mode=str(soft_calibration_data.get("mode", "LOITER")).upper(),
                duration_s=float(soft_calibration_data.get("duration_s", 20.0)),
                min_samples=int(soft_calibration_data.get("min_samples", 120)),
                min_gps_fix_type=int(soft_calibration_data.get("min_gps_fix_type", 3)),
                min_gps_satellites=int(soft_calibration_data.get("min_gps_satellites", 8)),
                min_pose_quality=int(soft_calibration_data.get("min_pose_quality", 45)),
                announce_interval_s=float(soft_calibration_data.get("announce_interval_s", 10.0)),
                ready_score_threshold=float(soft_calibration_data.get("ready_score_threshold", 7.0)),
                low_score_threshold=float(soft_calibration_data.get("low_score_threshold", 5.0)),
                critical_score_threshold=float(soft_calibration_data.get("critical_score_threshold", 3.0)),
                save_improvement_threshold=float(
                    soft_calibration_data.get("save_improvement_threshold", 0.3)
                ),
                profile_path=soft_profile_path,
                sample_log_path=soft_sample_log_path,
                sample_log_hz=float(soft_calibration_data.get("sample_log_hz", 1.0)),
                fallback_enabled=_as_bool(soft_calibration_data.get("fallback_enabled", True)),
                fallback_mode=str(soft_calibration_data.get("fallback_mode", "LOITER")).upper(),
            ),
            slam_observer=SlamObserverConfig(
                enable_loiter_observation=_as_bool(
                    slam_observer_data.get("enable_loiter_observation", True)
                ),
                observation_message_interval_sec=float(
                    slam_observer_data.get("observation_message_interval_sec", 20.0)
                ),
                min_quality_for_poshold=float(
                    slam_observer_data.get("min_quality_for_poshold", 7.0)
                ),
                weak_quality_threshold=float(
                    slam_observer_data.get("weak_quality_threshold", 5.0)
                ),
                critical_quality_threshold=float(
                    slam_observer_data.get("critical_quality_threshold", 3.0)
                ),
                quality_update_delta=float(
                    slam_observer_data.get("quality_update_delta", 0.5)
                ),
                enable_live_soft_correction=_as_bool(
                    slam_observer_data.get("enable_live_soft_correction", False)
                ),
                enable_auto_fallback_to_loiter=_as_bool(
                    slam_observer_data.get("enable_auto_fallback_to_loiter", False)
                ),
                log_observation_data=_as_bool(
                    slam_observer_data.get("log_observation_data", True)
                ),
                log_path=observer_log_path,
                status_path=observer_status_path,
            ),
            stream_stable_s=float(data.get("stream_stable_s", 2.0)),
            stream_loss_hysteresis_s=float(data.get("stream_loss_hysteresis_s", 5.0)),
        )


def load_bridge_config(path: str | Path) -> SlamBridgeConfig:
    config_path = Path(path).expanduser().resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Bridge config must be a mapping: {config_path}")
    base_dir = config_path.parent
    if base_dir.name == "config":
        base_dir = base_dir.parent
    return SlamBridgeConfig.from_mapping(data, base_dir=base_dir)
