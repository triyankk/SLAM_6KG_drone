"""Load and validate the project configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Raised when a project configuration is incomplete or unsafe."""


@dataclass(frozen=True)
class CubeMountConfig:
    x_m: float
    y_m: float
    z_m: float
    yaw_ccw_deg: float
    ahrs_orientation: int
    ahrs_orientation_name: str


@dataclass(frozen=True)
class MavlinkRouterConfig:
    enabled: bool
    serial_endpoint: str
    bind_host: str
    bind_port: int
    client_host: str
    client_port: int
    status_file: str


@dataclass(frozen=True)
class FlightControllerConfig:
    endpoint: str
    baud: int
    system_id: int
    companion_system_id: int
    companion_component_id: int
    heartbeat_timeout_s: float
    sample_window_s: float
    hflow_min_bench_quality: int
    cube_mount: CubeMountConfig
    router: MavlinkRouterConfig


@dataclass(frozen=True)
class PositionConfig:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class RotationConfig:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class DepthCameraConfig:
    model: str
    backend: str
    mounting: str
    serial: str | None
    width: int
    height: int
    fps: int
    stream_host: str
    stream_port: int
    jpeg_quality: int
    position_from_cg_frd_m: PositionConfig = field(
        default_factory=lambda: PositionConfig(0.0, 0.0, 0.0)
    )
    rotation_from_forward_frd: RotationConfig = field(
        default_factory=lambda: RotationConfig(0.0, 0.0, 0.0)
    )


@dataclass(frozen=True)
class AxisSignsConfig:
    x: int
    y: int
    z: int


@dataclass(frozen=True)
class ImuConfig:
    model: str
    symlink: str
    usb_vid: int
    usb_pid: int
    baud: int
    expected_rate_hz: float
    sensor_time_enabled: bool
    body_axis_signs: AxisSignsConfig
    axis_map_verified: bool
    axis_map_verification: str
    position_from_cg_frd_m: PositionConfig
    position_verified: bool


@dataclass(frozen=True)
class LidarConfig:
    model: str
    transport: str
    symlink: str
    usb_vid: int
    usb_pid: int
    usb_serial: str
    baud: int
    legacy_baud: int
    baud_verified: bool
    packet_probe_s: float
    sdk_revision: str
    bridge_binary: str
    correction_file: str
    correction_verified: bool
    position_from_cg_frd_m: PositionConfig
    rotation_to_body_frd: RotationConfig


@dataclass(frozen=True)
class LioClockSyncConfig:
    window_samples: int
    maximum_imu_window_span_s: float
    maximum_lidar_window_span_s: float
    minimum_imu_samples: int
    minimum_lidar_samples: int
    minimum_span_s: float
    time_offset_lidar_to_imu_s: float
    maximum_drift_ppm: float
    maximum_imu_residual_p95_ms: float
    maximum_lidar_residual_p95_ms: float


@dataclass(frozen=True)
class LioValidationConfig:
    approved: bool
    report_path: str
    report_sha256: str
    minimum_duration_s: float
    minimum_odometry_rate_hz: float
    stationary_window_s: float
    maximum_stationary_drift_m: float
    maximum_return_to_start_error_m: float
    maximum_position_jump_m: float
    maximum_speed_mps: float
    maximum_attitude_jump_deg: float
    maximum_clock_resets: int
    minimum_cube_reference_samples: int
    minimum_cube_reference_path_m: float
    maximum_cube_horizontal_rmse_m: float
    maximum_cube_vertical_rmse_m: float
    maximum_cube_attitude_p95_deg: float
    minimum_cube_path_ratio: float
    maximum_cube_path_ratio: float


@dataclass(frozen=True)
class LidarInertialOdometryConfig:
    stage: str
    backend: str
    backend_revision: str
    runtime_dir: str
    pointcloud_topic: str
    imu_topic: str
    odometry_topic: str
    diagnostics_topic: str
    odometry_shadow_to_cube_enabled: bool
    pose_output_to_cube_enabled: bool
    map_output_enabled: bool
    required_imu_rate_hz: float
    clock_sync: LioClockSyncConfig
    validation: LioValidationConfig


@dataclass(frozen=True)
class ObstacleRcToggleConfig:
    channel: int
    engage_pwm: int
    disengage_pwm: int


@dataclass(frozen=True)
class ObstacleAlertConfig:
    enabled: bool
    only_when_armed: bool
    warning_distance_m: float
    escalation_distance_m: float
    warning_rate_hz: float
    keepout_rate_hz: float
    maximum_rate_hz: float


@dataclass(frozen=True)
class NativeAvoidanceConfig:
    proximity_type: int
    enable_mask: int
    behavior: int
    rc_option: int
    backup_speed_mps: float
    acceleration_max_mpss: float


@dataclass(frozen=True)
class ObstacleAvoidanceConfig:
    stage: str
    mavlink_output_enabled: bool
    depth_camera_enabled: bool
    lidar_enabled: bool
    hard_cg_clearance_m: float
    target_rate_hz: float
    source_stale_timeout_s: float
    sector_increment_deg: float
    min_distance_m: float
    max_distance_m: float
    body_z_min_m: float
    body_z_max_m: float
    depth_percentile: float
    depth_sample_stride: int
    minimum_points_per_sector: int
    temporal_window: int
    airframe_radius_m: float
    airframe_geometry_verified: bool
    rc_toggle: ObstacleRcToggleConfig
    alerts: ObstacleAlertConfig
    native: NativeAvoidanceConfig

    @property
    def sector_count(self) -> int:
        return round(360.0 / self.sector_increment_deg)


@dataclass(frozen=True)
class CalibrationConfig:
    camera_intrinsics_verified: bool
    camera_to_body_extrinsics_verified: bool
    imu_to_body_extrinsics_verified: bool
    lidar_to_body_extrinsics_verified: bool
    imu_noise_profile_verified: bool
    sensor_time_sync_verified: bool

    @property
    def complete(self) -> bool:
        return all(
            (
                self.camera_intrinsics_verified,
                self.camera_to_body_extrinsics_verified,
                self.imu_to_body_extrinsics_verified,
                self.lidar_to_body_extrinsics_verified,
                self.imu_noise_profile_verified,
                self.sensor_time_sync_verified,
            )
        )


@dataclass(frozen=True)
class SlamReturnConfig:
    stage: str
    live_control_enabled: bool
    approval_file: str
    status_file: str
    required_mode: str
    rc_channel: int
    land_rc_channel: int
    engage_pwm: int
    disengage_pwm: int
    ekf_source_set: int
    minimum_altitude_m: float
    maximum_altitude_m: float
    minimum_flow_quality: int
    telemetry_stale_timeout_s: float
    battery_stale_timeout_s: float
    minimum_voltage_v: float
    command_rate_hz: float
    maximum_horizontal_speed_mps: float
    maximum_horizontal_acceleration_mpss: float
    arrival_radius_m: float
    breadcrumb_spacing_m: float
    waypoint_radius_m: float
    visual_stale_timeout_s: float
    visual_disagreement_limit_m: float


@dataclass(frozen=True)
class NavigationConfig:
    autonomous_control_enabled: bool
    external_nav_to_cube_enabled: bool
    target_rate_hz: float
    initial_max_horizontal_speed_mps: float
    initial_max_vertical_speed_mps: float
    initial_max_yaw_rate_dps: float
    local_pose_stale_timeout_s: float
    command_stale_timeout_s: float
    slam_return: SlamReturnConfig


@dataclass(frozen=True)
class SafetyConfig:
    forbidden_modes: tuple[str, ...]
    standard_rtl_allowed_without_global_position: bool
    rc_disarm_switch_configured: bool
    automatic_ekf_source_switching_enabled: bool


@dataclass(frozen=True)
class ProjectConfig:
    flight_controller: FlightControllerConfig
    depth_camera: DepthCameraConfig
    external_imu: ImuConfig
    lidar: LidarConfig
    lidar_inertial_odometry: LidarInertialOdometryConfig
    obstacle_avoidance: ObstacleAvoidanceConfig
    calibration: CalibrationConfig
    navigation: NavigationConfig
    safety: SafetyConfig


def _mapping(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ConfigError(f"{key} must be a mapping")
    return value


def _required(section: dict[str, Any], key: str) -> Any:
    if key not in section:
        raise ConfigError(f"missing required key: {key}")
    return section[key]


def _positive(value: Any, name: str) -> float:
    number = float(value)
    if number <= 0:
        raise ConfigError(f"{name} must be positive")
    return number


def _approved_lio_report(
    config_path: Path,
    validation: LioValidationConfig,
    backend_revision: str,
) -> dict[str, Any] | None:
    if not validation.approved:
        return None
    if not validation.report_path or not validation.report_sha256:
        raise ConfigError(
            "approved LIO validation requires report_path and report_sha256"
        )
    report_path = Path(validation.report_path)
    if not report_path.is_absolute():
        report_path = config_path.parent.parent / report_path
    if not report_path.is_file():
        raise ConfigError(f"approved LIO validation report is missing: {report_path}")
    report_bytes = report_path.read_bytes()
    actual_digest = hashlib.sha256(report_bytes).hexdigest()
    if actual_digest != validation.report_sha256.lower():
        raise ConfigError("approved LIO validation report digest does not match")
    try:
        report = json.loads(report_bytes)
    except json.JSONDecodeError as exc:
        raise ConfigError(f"approved LIO report is invalid JSON: {exc}") from exc
    if (
        not isinstance(report, dict)
        or report.get("result") != "pass"
        or report.get("eligible_for_cube_pose_approval") is not True
        or report.get("pose_sent_to_cube") is not False
    ):
        raise ConfigError("approved LIO validation report did not pass")
    if report.get("backend_revision") != backend_revision:
        raise ConfigError("approved LIO report used a different backend revision")
    return report


def load_config(path: str | Path) -> ProjectConfig:
    config_path = Path(path)
    raw = yaml.safe_load(config_path.read_text(encoding="ascii"))
    if not isinstance(raw, dict):
        raise ConfigError("configuration root must be a mapping")
    if raw.get("schema_version") != 1:
        raise ConfigError("unsupported schema_version")

    fc = _mapping(raw, "flight_controller")
    cube_mount = _mapping(fc, "cube_mount")
    router = _mapping(fc, "router")
    cube_position = _mapping(cube_mount, "position_from_cg_frd_m")
    sensors = _mapping(raw, "sensors")
    camera = _mapping(sensors, "depth_camera")
    camera_position = _mapping(camera, "position_from_cg_frd_m")
    camera_rotation = _mapping(camera, "rotation_from_forward_frd_deg")
    imu = _mapping(sensors, "external_imu")
    imu_axis_signs = _mapping(imu, "body_axis_signs")
    imu_position = _mapping(imu, "position_from_cg_frd_m")
    lidar = _mapping(sensors, "lidar")
    lidar_position = _mapping(lidar, "position_from_cg_frd_m")
    lidar_rotation = _mapping(lidar, "rotation_to_body_frd_deg")
    lio = _mapping(raw, "lidar_inertial_odometry")
    lio_clock = _mapping(lio, "clock_sync")
    lio_validation = _mapping(lio, "validation")
    obstacle = _mapping(raw, "obstacle_avoidance")
    obstacle_rc_toggle = _mapping(obstacle, "rc_toggle")
    obstacle_alerts = _mapping(obstacle, "alerts")
    native_avoidance = _mapping(obstacle, "native_avoidance")
    calibration = _mapping(raw, "calibration")
    navigation = _mapping(raw, "navigation")
    slam_return = _mapping(navigation, "slam_return")
    safety = _mapping(raw, "safety")

    router_enabled = bool(_required(router, "enabled"))
    router_serial_endpoint = str(_required(router, "serial_endpoint"))
    router_bind_host = str(_required(router, "bind_host"))
    router_client_host = str(_required(router, "client_host"))
    router_bind_port = int(_required(router, "bind_port"))
    router_client_port = int(_required(router, "client_port"))
    router_status_file = str(_required(router, "status_file"))
    if router_enabled:
        if router_bind_host != "127.0.0.1" or router_client_host != "127.0.0.1":
            raise ConfigError("Cube UART router must remain on localhost")
        if not router_serial_endpoint.startswith("/dev/"):
            raise ConfigError("Cube UART router serial endpoint must be under /dev")
        if not 1024 <= router_bind_port <= 65535:
            raise ConfigError("Cube UART router bind port is invalid")
        if not 1024 <= router_client_port <= 65535:
            raise ConfigError("Cube UART router client port is invalid")
        if router_bind_port == router_client_port:
            raise ConfigError("Cube UART router ports must be different")
        expected_endpoint = (
            f"udpin:{router_client_host}:{router_client_port}"
        )
        if str(_required(fc, "endpoint")) != expected_endpoint:
            raise ConfigError(
                "flight_controller.endpoint must match the UART router client"
            )

    forbidden_modes = tuple(str(mode).upper() for mode in _required(safety, "forbidden_modes"))
    if "STABILIZE" not in forbidden_modes:
        raise ConfigError("STABILIZE must remain forbidden")

    slam_return_stage = str(_required(slam_return, "stage")).lower()
    if slam_return_stage not in {"locked", "commissioning", "active"}:
        raise ConfigError(
            "navigation.slam_return.stage must be locked, commissioning, or active"
        )
    slam_return_rc_channel = int(_required(slam_return, "rc_channel"))
    if not 5 <= slam_return_rc_channel <= 16:
        raise ConfigError("slam-return RC channel must be between 5 and 16")
    slam_return_land_rc_channel = int(
        _required(slam_return, "land_rc_channel")
    )
    if not 5 <= slam_return_land_rc_channel <= 16:
        raise ConfigError("LAND RC channel must be between 5 and 16")
    if slam_return_land_rc_channel == slam_return_rc_channel:
        raise ConfigError("LAND and slam-return require separate RC channels")
    slam_return_engage_pwm = int(_required(slam_return, "engage_pwm"))
    slam_return_disengage_pwm = int(
        _required(slam_return, "disengage_pwm")
    )
    if slam_return_disengage_pwm >= slam_return_engage_pwm:
        raise ConfigError(
            "slam-return disengage PWM must be below engage PWM"
        )
    slam_return_minimum_altitude_m = _positive(
        _required(slam_return, "minimum_altitude_m"),
        "slam_return.minimum_altitude_m",
    )
    slam_return_maximum_altitude_m = _positive(
        _required(slam_return, "maximum_altitude_m"),
        "slam_return.maximum_altitude_m",
    )
    if slam_return_minimum_altitude_m >= slam_return_maximum_altitude_m:
        raise ConfigError(
            "slam-return minimum altitude must be below maximum altitude"
        )
    slam_return_minimum_flow_quality = int(
        _required(slam_return, "minimum_flow_quality")
    )
    if not 1 <= slam_return_minimum_flow_quality <= 255:
        raise ConfigError(
            "slam-return minimum flow quality must be between 1 and 255"
        )
    slam_return_ekf_source_set = int(
        _required(slam_return, "ekf_source_set")
    )
    if slam_return_ekf_source_set not in (1, 2, 3):
        raise ConfigError("slam-return EKF source set must be 1, 2, or 3")

    parsed_slam_return = SlamReturnConfig(
        stage=slam_return_stage,
        live_control_enabled=bool(
            _required(slam_return, "live_control_enabled")
        ),
        approval_file=str(_required(slam_return, "approval_file")),
        status_file=str(_required(slam_return, "status_file")),
        required_mode=str(_required(slam_return, "required_mode")).upper(),
        rc_channel=slam_return_rc_channel,
        land_rc_channel=slam_return_land_rc_channel,
        engage_pwm=slam_return_engage_pwm,
        disengage_pwm=slam_return_disengage_pwm,
        ekf_source_set=slam_return_ekf_source_set,
        minimum_altitude_m=slam_return_minimum_altitude_m,
        maximum_altitude_m=slam_return_maximum_altitude_m,
        minimum_flow_quality=slam_return_minimum_flow_quality,
        telemetry_stale_timeout_s=_positive(
            _required(slam_return, "telemetry_stale_timeout_s"),
            "slam_return.telemetry_stale_timeout_s",
        ),
        battery_stale_timeout_s=_positive(
            _required(slam_return, "battery_stale_timeout_s"),
            "slam_return.battery_stale_timeout_s",
        ),
        minimum_voltage_v=_positive(
            _required(slam_return, "minimum_voltage_v"),
            "slam_return.minimum_voltage_v",
        ),
        command_rate_hz=_positive(
            _required(slam_return, "command_rate_hz"),
            "slam_return.command_rate_hz",
        ),
        maximum_horizontal_speed_mps=_positive(
            _required(slam_return, "maximum_horizontal_speed_mps"),
            "slam_return.maximum_horizontal_speed_mps",
        ),
        maximum_horizontal_acceleration_mpss=_positive(
            _required(
                slam_return, "maximum_horizontal_acceleration_mpss"
            ),
            "slam_return.maximum_horizontal_acceleration_mpss",
        ),
        arrival_radius_m=_positive(
            _required(slam_return, "arrival_radius_m"),
            "slam_return.arrival_radius_m",
        ),
        breadcrumb_spacing_m=_positive(
            _required(slam_return, "breadcrumb_spacing_m"),
            "slam_return.breadcrumb_spacing_m",
        ),
        waypoint_radius_m=_positive(
            _required(slam_return, "waypoint_radius_m"),
            "slam_return.waypoint_radius_m",
        ),
        visual_stale_timeout_s=_positive(
            _required(slam_return, "visual_stale_timeout_s"),
            "slam_return.visual_stale_timeout_s",
        ),
        visual_disagreement_limit_m=_positive(
            _required(slam_return, "visual_disagreement_limit_m"),
            "slam_return.visual_disagreement_limit_m",
        ),
    )
    if parsed_slam_return.required_mode != "GUIDED":
        raise ConfigError("slam-return must use regular GUIDED mode")

    nav = NavigationConfig(
        autonomous_control_enabled=bool(
            _required(navigation, "autonomous_control_enabled")
        ),
        external_nav_to_cube_enabled=bool(
            _required(navigation, "external_nav_to_cube_enabled")
        ),
        target_rate_hz=_positive(
            _required(navigation, "target_rate_hz"), "target_rate_hz"
        ),
        initial_max_horizontal_speed_mps=_positive(
            _required(navigation, "initial_max_horizontal_speed_mps"),
            "initial_max_horizontal_speed_mps",
        ),
        initial_max_vertical_speed_mps=_positive(
            _required(navigation, "initial_max_vertical_speed_mps"),
            "initial_max_vertical_speed_mps",
        ),
        initial_max_yaw_rate_dps=_positive(
            _required(navigation, "initial_max_yaw_rate_dps"),
            "initial_max_yaw_rate_dps",
        ),
        local_pose_stale_timeout_s=_positive(
            _required(navigation, "local_pose_stale_timeout_s"),
            "local_pose_stale_timeout_s",
        ),
        command_stale_timeout_s=_positive(
            _required(navigation, "command_stale_timeout_s"),
            "command_stale_timeout_s",
        ),
        slam_return=parsed_slam_return,
    )
    if nav.external_nav_to_cube_enabled and not nav.autonomous_control_enabled:
        raise ConfigError(
            "external_nav_to_cube_enabled requires autonomous_control_enabled"
        )
    if (
        nav.slam_return.maximum_horizontal_speed_mps
        > nav.initial_max_horizontal_speed_mps
    ):
        raise ConfigError(
            "slam-return speed cannot exceed the navigation speed limit"
        )
    if nav.slam_return.maximum_horizontal_speed_mps > 0.75:
        raise ConfigError(
            "slam-return speed must never exceed the 0.75 m/s hard ceiling"
        )

    parsed_imu_axis_signs = AxisSignsConfig(
        x=int(_required(imu_axis_signs, "x")),
        y=int(_required(imu_axis_signs, "y")),
        z=int(_required(imu_axis_signs, "z")),
    )
    axis_sign_values = (
        parsed_imu_axis_signs.x,
        parsed_imu_axis_signs.y,
        parsed_imu_axis_signs.z,
    )
    if any(value not in (-1, 1) for value in axis_sign_values):
        raise ConfigError("external_imu body-axis signs must be -1 or 1")
    if math.prod(axis_sign_values) != 1:
        raise ConfigError(
            "external_imu body-axis signs must define a proper rotation"
        )

    lidar_transport = str(_required(lidar, "transport")).lower()
    if lidar_transport != "serial_rs485":
        raise ConfigError("JT16 transport must be serial_rs485")
    lidar_baud = int(_positive(_required(lidar, "baud"), "lidar.baud"))
    lidar_legacy_baud = int(
        _positive(_required(lidar, "legacy_baud"), "lidar.legacy_baud")
    )

    lio_clock_config = LioClockSyncConfig(
        window_samples=int(_required(lio_clock, "window_samples")),
        maximum_imu_window_span_s=_positive(
            _required(lio_clock, "maximum_imu_window_span_s"),
            (
                "lidar_inertial_odometry.clock_sync."
                "maximum_imu_window_span_s"
            ),
        ),
        maximum_lidar_window_span_s=_positive(
            _required(lio_clock, "maximum_lidar_window_span_s"),
            (
                "lidar_inertial_odometry.clock_sync."
                "maximum_lidar_window_span_s"
            ),
        ),
        minimum_imu_samples=int(
            _required(lio_clock, "minimum_imu_samples")
        ),
        minimum_lidar_samples=int(
            _required(lio_clock, "minimum_lidar_samples")
        ),
        minimum_span_s=_positive(
            _required(lio_clock, "minimum_span_s"),
            "lidar_inertial_odometry.clock_sync.minimum_span_s",
        ),
        time_offset_lidar_to_imu_s=float(
            _required(lio_clock, "time_offset_lidar_to_imu_s")
        ),
        maximum_drift_ppm=_positive(
            _required(lio_clock, "maximum_drift_ppm"),
            "lidar_inertial_odometry.clock_sync.maximum_drift_ppm",
        ),
        maximum_imu_residual_p95_ms=_positive(
            _required(lio_clock, "maximum_imu_residual_p95_ms"),
            (
                "lidar_inertial_odometry.clock_sync."
                "maximum_imu_residual_p95_ms"
            ),
        ),
        maximum_lidar_residual_p95_ms=_positive(
            _required(lio_clock, "maximum_lidar_residual_p95_ms"),
            (
                "lidar_inertial_odometry.clock_sync."
                "maximum_lidar_residual_p95_ms"
            ),
        ),
    )
    if (
        lio_clock_config.minimum_imu_samples < 20
        or lio_clock_config.minimum_lidar_samples < 10
        or lio_clock_config.window_samples
        < max(
            lio_clock_config.minimum_imu_samples,
            lio_clock_config.minimum_lidar_samples,
        )
    ):
        raise ConfigError("LIO clock sample limits are too small or inconsistent")
    if (
        lio_clock_config.maximum_imu_window_span_s
        < lio_clock_config.minimum_span_s
        or lio_clock_config.maximum_lidar_window_span_s
        < lio_clock_config.minimum_span_s
    ):
        raise ConfigError(
            "LIO clock windows must cover the minimum synchronization span"
        )
    if abs(lio_clock_config.time_offset_lidar_to_imu_s) > 0.2:
        raise ConfigError(
            "LIO fixed lidar-to-IMU time offset must be within 0.2 s"
        )

    lio_validation_config = LioValidationConfig(
        approved=bool(_required(lio_validation, "approved")),
        report_path=str(_required(lio_validation, "report_path")),
        report_sha256=str(_required(lio_validation, "report_sha256")),
        minimum_duration_s=_positive(
            _required(lio_validation, "minimum_duration_s"),
            "LIO validation minimum_duration_s",
        ),
        minimum_odometry_rate_hz=_positive(
            _required(lio_validation, "minimum_odometry_rate_hz"),
            "LIO validation minimum_odometry_rate_hz",
        ),
        stationary_window_s=_positive(
            _required(lio_validation, "stationary_window_s"),
            "LIO validation stationary_window_s",
        ),
        maximum_stationary_drift_m=_positive(
            _required(lio_validation, "maximum_stationary_drift_m"),
            "LIO validation maximum_stationary_drift_m",
        ),
        maximum_return_to_start_error_m=_positive(
            _required(lio_validation, "maximum_return_to_start_error_m"),
            "LIO validation maximum_return_to_start_error_m",
        ),
        maximum_position_jump_m=_positive(
            _required(lio_validation, "maximum_position_jump_m"),
            "LIO validation maximum_position_jump_m",
        ),
        maximum_speed_mps=_positive(
            _required(lio_validation, "maximum_speed_mps"),
            "LIO validation maximum_speed_mps",
        ),
        maximum_attitude_jump_deg=_positive(
            _required(lio_validation, "maximum_attitude_jump_deg"),
            "LIO validation maximum_attitude_jump_deg",
        ),
        maximum_clock_resets=int(
            _required(lio_validation, "maximum_clock_resets")
        ),
        minimum_cube_reference_samples=int(
            _required(lio_validation, "minimum_cube_reference_samples")
        ),
        minimum_cube_reference_path_m=_positive(
            _required(lio_validation, "minimum_cube_reference_path_m"),
            "LIO validation minimum_cube_reference_path_m",
        ),
        maximum_cube_horizontal_rmse_m=_positive(
            _required(lio_validation, "maximum_cube_horizontal_rmse_m"),
            "LIO validation maximum_cube_horizontal_rmse_m",
        ),
        maximum_cube_vertical_rmse_m=_positive(
            _required(lio_validation, "maximum_cube_vertical_rmse_m"),
            "LIO validation maximum_cube_vertical_rmse_m",
        ),
        maximum_cube_attitude_p95_deg=_positive(
            _required(lio_validation, "maximum_cube_attitude_p95_deg"),
            "LIO validation maximum_cube_attitude_p95_deg",
        ),
        minimum_cube_path_ratio=_positive(
            _required(lio_validation, "minimum_cube_path_ratio"),
            "LIO validation minimum_cube_path_ratio",
        ),
        maximum_cube_path_ratio=_positive(
            _required(lio_validation, "maximum_cube_path_ratio"),
            "LIO validation maximum_cube_path_ratio",
        ),
    )
    if lio_validation_config.maximum_clock_resets < 0:
        raise ConfigError("LIO maximum_clock_resets cannot be negative")
    if lio_validation_config.minimum_cube_reference_samples < 10:
        raise ConfigError(
            "LIO minimum_cube_reference_samples must be at least 10"
        )
    if (
        lio_validation_config.minimum_cube_path_ratio
        >= lio_validation_config.maximum_cube_path_ratio
    ):
        raise ConfigError(
            "LIO Cube path-ratio bounds must be increasing"
        )

    lio_config = LidarInertialOdometryConfig(
        stage=str(_required(lio, "stage")).lower(),
        backend=str(_required(lio, "backend")),
        backend_revision=str(_required(lio, "backend_revision")),
        runtime_dir=str(_required(lio, "runtime_dir")),
        pointcloud_topic=str(_required(lio, "pointcloud_topic")),
        imu_topic=str(_required(lio, "imu_topic")),
        odometry_topic=str(_required(lio, "odometry_topic")),
        diagnostics_topic=str(_required(lio, "diagnostics_topic")),
        odometry_shadow_to_cube_enabled=bool(
            _required(lio, "odometry_shadow_to_cube_enabled")
        ),
        pose_output_to_cube_enabled=bool(
            _required(lio, "pose_output_to_cube_enabled")
        ),
        map_output_enabled=bool(_required(lio, "map_output_enabled")),
        required_imu_rate_hz=_positive(
            _required(lio, "required_imu_rate_hz"),
            "lidar_inertial_odometry.required_imu_rate_hz",
        ),
        clock_sync=lio_clock_config,
        validation=lio_validation_config,
    )
    if lio_config.stage not in ("shadow", "active"):
        raise ConfigError(
            "lidar_inertial_odometry.stage must be shadow or active"
        )
    if lio_config.backend != "hesai_fast_lio2":
        raise ConfigError("LIO backend must remain the pinned Hesai FAST-LIO2")
    if len(lio_config.backend_revision) != 40:
        raise ConfigError("LIO backend revision must be a full Git commit")
    if (
        lio_config.odometry_shadow_to_cube_enabled
        and lio_config.stage != "shadow"
    ):
        raise ConfigError("Cube odometry shadow output requires shadow LIO stage")
    if (
        lio_config.odometry_shadow_to_cube_enabled
        and lio_config.pose_output_to_cube_enabled
    ):
        raise ConfigError(
            "Cube odometry shadow and active pose output cannot coexist"
        )
    _approved_lio_report(
        config_path,
        lio_validation_config,
        lio_config.backend_revision,
    )

    obstacle_config = ObstacleAvoidanceConfig(
        stage=str(_required(obstacle, "stage")).lower(),
        mavlink_output_enabled=bool(
            _required(obstacle, "mavlink_output_enabled")
        ),
        depth_camera_enabled=bool(
            _required(obstacle, "depth_camera_enabled")
        ),
        lidar_enabled=bool(_required(obstacle, "lidar_enabled")),
        hard_cg_clearance_m=_positive(
            _required(obstacle, "hard_cg_clearance_m"),
            "obstacle_avoidance.hard_cg_clearance_m",
        ),
        target_rate_hz=_positive(
            _required(obstacle, "target_rate_hz"),
            "obstacle_avoidance.target_rate_hz",
        ),
        source_stale_timeout_s=_positive(
            _required(obstacle, "source_stale_timeout_s"),
            "obstacle_avoidance.source_stale_timeout_s",
        ),
        sector_increment_deg=_positive(
            _required(obstacle, "sector_increment_deg"),
            "obstacle_avoidance.sector_increment_deg",
        ),
        min_distance_m=_positive(
            _required(obstacle, "min_distance_m"),
            "obstacle_avoidance.min_distance_m",
        ),
        max_distance_m=_positive(
            _required(obstacle, "max_distance_m"),
            "obstacle_avoidance.max_distance_m",
        ),
        body_z_min_m=float(_required(obstacle, "body_z_min_m")),
        body_z_max_m=float(_required(obstacle, "body_z_max_m")),
        depth_percentile=float(
            _required(obstacle, "depth_percentile")
        ),
        depth_sample_stride=int(
            _required(obstacle, "depth_sample_stride")
        ),
        minimum_points_per_sector=int(
            _required(obstacle, "minimum_points_per_sector")
        ),
        temporal_window=int(_required(obstacle, "temporal_window")),
        airframe_radius_m=_positive(
            _required(obstacle, "airframe_radius_m"),
            "obstacle_avoidance.airframe_radius_m",
        ),
        airframe_geometry_verified=bool(
            _required(obstacle, "airframe_geometry_verified")
        ),
        rc_toggle=ObstacleRcToggleConfig(
            channel=int(_required(obstacle_rc_toggle, "channel")),
            engage_pwm=int(_required(obstacle_rc_toggle, "engage_pwm")),
            disengage_pwm=int(
                _required(obstacle_rc_toggle, "disengage_pwm")
            ),
        ),
        alerts=ObstacleAlertConfig(
            enabled=bool(_required(obstacle_alerts, "enabled")),
            only_when_armed=bool(
                _required(obstacle_alerts, "only_when_armed")
            ),
            warning_distance_m=_positive(
                _required(obstacle_alerts, "warning_distance_m"),
                "obstacle_avoidance.alerts.warning_distance_m",
            ),
            escalation_distance_m=_positive(
                _required(obstacle_alerts, "escalation_distance_m"),
                "obstacle_avoidance.alerts.escalation_distance_m",
            ),
            warning_rate_hz=_positive(
                _required(obstacle_alerts, "warning_rate_hz"),
                "obstacle_avoidance.alerts.warning_rate_hz",
            ),
            keepout_rate_hz=_positive(
                _required(obstacle_alerts, "keepout_rate_hz"),
                "obstacle_avoidance.alerts.keepout_rate_hz",
            ),
            maximum_rate_hz=_positive(
                _required(obstacle_alerts, "maximum_rate_hz"),
                "obstacle_avoidance.alerts.maximum_rate_hz",
            ),
        ),
        native=NativeAvoidanceConfig(
            proximity_type=int(
                _required(native_avoidance, "proximity_type")
            ),
            enable_mask=int(_required(native_avoidance, "enable_mask")),
            behavior=int(_required(native_avoidance, "behavior")),
            rc_option=int(_required(native_avoidance, "rc_option")),
            backup_speed_mps=_positive(
                _required(native_avoidance, "backup_speed_mps"),
                "obstacle_avoidance.native_avoidance.backup_speed_mps",
            ),
            acceleration_max_mpss=_positive(
                _required(native_avoidance, "acceleration_max_mpss"),
                (
                    "obstacle_avoidance.native_avoidance."
                    "acceleration_max_mpss"
                ),
            ),
        ),
    )
    if obstacle_config.stage not in ("shadow", "active"):
        raise ConfigError("obstacle_avoidance.stage must be shadow or active")
    if obstacle_config.target_rate_hz < 10.0:
        raise ConfigError("obstacle avoidance target rate must be at least 10 Hz")
    if obstacle_config.source_stale_timeout_s >= 0.5:
        raise ConfigError(
            "obstacle source timeout must be below Cube's 0.5 s timeout"
        )
    sector_count = 360.0 / obstacle_config.sector_increment_deg
    if (
        not math.isclose(sector_count, round(sector_count), abs_tol=1.0e-9)
        or round(sector_count) > 72
    ):
        raise ConfigError(
            "obstacle sector increment must divide 360 into at most 72 bins"
        )
    if obstacle_config.min_distance_m >= obstacle_config.max_distance_m:
        raise ConfigError(
            "obstacle minimum distance must be below maximum distance"
        )
    if not (
        obstacle_config.min_distance_m
        < obstacle_config.hard_cg_clearance_m
        < obstacle_config.max_distance_m
    ):
        raise ConfigError(
            "hard CG clearance must be inside the obstacle measurement range"
        )
    if obstacle_config.body_z_min_m >= obstacle_config.body_z_max_m:
        raise ConfigError("obstacle body Z limits are reversed")
    if not 0.0 < obstacle_config.depth_percentile <= 50.0:
        raise ConfigError("obstacle depth percentile must be in (0, 50]")
    if obstacle_config.depth_sample_stride <= 0:
        raise ConfigError("obstacle depth sample stride must be positive")
    if obstacle_config.minimum_points_per_sector <= 0:
        raise ConfigError("minimum obstacle points must be positive")
    if (
        obstacle_config.temporal_window <= 0
        or obstacle_config.temporal_window % 2 == 0
    ):
        raise ConfigError("obstacle temporal window must be a positive odd value")
    if not (
        obstacle_config.depth_camera_enabled
        or obstacle_config.lidar_enabled
    ):
        raise ConfigError("at least one obstacle source must be enabled")
    if not 5 <= obstacle_config.rc_toggle.channel <= 18:
        raise ConfigError("obstacle RC toggle channel must be between 5 and 18")
    if not (
        800
        <= obstacle_config.rc_toggle.disengage_pwm
        < obstacle_config.rc_toggle.engage_pwm
        <= 2200
    ):
        raise ConfigError("obstacle RC toggle PWM thresholds are invalid")
    if not (
        max(
            obstacle_config.min_distance_m,
            obstacle_config.airframe_radius_m,
        )
        < obstacle_config.alerts.escalation_distance_m
        < obstacle_config.hard_cg_clearance_m
        < obstacle_config.alerts.warning_distance_m
        <= obstacle_config.max_distance_m
    ):
        raise ConfigError(
            "obstacle alert distances must increase from sensor minimum "
            "through escalation, hard clearance, warning, and sensor maximum"
        )
    if not (
        obstacle_config.alerts.warning_rate_hz
        <= obstacle_config.alerts.keepout_rate_hz
        <= obstacle_config.alerts.maximum_rate_hz
        <= obstacle_config.target_rate_hz
    ):
        raise ConfigError(
            "obstacle beep rates must be ordered and no faster than "
            "the obstacle update rate"
        )
    if obstacle_config.native.proximity_type != 2:
        raise ConfigError("Cube MAVLink proximity requires PRX1_TYPE=2")
    if obstacle_config.native.enable_mask <= 0:
        raise ConfigError("Cube native avoidance enable mask must be positive")
    if obstacle_config.native.behavior not in (0, 1):
        raise ConfigError("Cube native avoidance behavior must be slide or stop")
    if obstacle_config.native.rc_option != 40:
        raise ConfigError("Cube obstacle RC option must remain 40")
    if nav.slam_return.rc_channel == obstacle_config.rc_toggle.channel:
        raise ConfigError(
            "slam-return and obstacle avoidance require separate RC channels"
        )
    if nav.slam_return.land_rc_channel == obstacle_config.rc_toggle.channel:
        raise ConfigError(
            "LAND and obstacle avoidance require separate RC channels"
        )
    if not (
        800
        <= nav.slam_return.disengage_pwm
        < nav.slam_return.engage_pwm
        <= 2200
    ):
        raise ConfigError("slam-return RC PWM thresholds are invalid")
    if nav.slam_return.command_rate_hz < 5.0:
        raise ConfigError("slam-return command rate must be at least 5 Hz")
    if nav.slam_return.command_rate_hz > nav.target_rate_hz:
        raise ConfigError(
            "slam-return command rate cannot exceed navigation target rate"
        )
    if nav.slam_return.waypoint_radius_m > (
        nav.slam_return.breadcrumb_spacing_m
    ):
        raise ConfigError(
            "slam-return waypoint radius cannot exceed breadcrumb spacing"
        )
    if obstacle_config.mavlink_output_enabled:
        if obstacle_config.stage != "active":
            raise ConfigError(
                "MAVLink obstacle output requires active obstacle stage"
            )
        if not obstacle_config.airframe_geometry_verified:
            raise ConfigError(
                "MAVLink obstacle output requires verified airframe geometry"
            )
        if (
            obstacle_config.depth_camera_enabled
            and not bool(
                _required(
                    calibration, "camera_to_body_extrinsics_verified"
                )
            )
        ):
            raise ConfigError(
                "MAVLink obstacle output requires camera extrinsics"
            )
        if (
            obstacle_config.depth_camera_enabled
            and not bool(
                _required(calibration, "camera_intrinsics_verified")
            )
        ):
            raise ConfigError(
                "MAVLink obstacle output requires camera intrinsics"
            )
        if (
            obstacle_config.lidar_enabled
            and not bool(
                _required(
                    calibration, "lidar_to_body_extrinsics_verified"
                )
            )
        ):
            raise ConfigError(
                "MAVLink obstacle output requires lidar extrinsics"
            )
        if (
            obstacle_config.lidar_enabled
            and not bool(_required(lidar, "baud_verified"))
        ):
            raise ConfigError(
                "MAVLink obstacle output requires verified lidar baud"
            )
        if (
            obstacle_config.lidar_enabled
            and not bool(_required(lidar, "correction_verified"))
        ):
            raise ConfigError(
                "MAVLink obstacle output requires verified lidar correction"
            )
    if nav.external_nav_to_cube_enabled != (
        lio_config.pose_output_to_cube_enabled
    ):
        raise ConfigError(
            "Cube external-nav and LIO pose-output enables must change together"
        )
    if lio_config.pose_output_to_cube_enabled:
        if lio_config.stage != "active":
            raise ConfigError("Cube LIO pose output requires active LIO stage")
        if not lio_config.validation.approved:
            raise ConfigError(
                "Cube LIO pose output requires an approved trajectory report"
            )
        if not bool(_required(imu, "sensor_time_enabled")):
            raise ConfigError(
                "Cube LIO pose output requires IM10A sensor timestamps"
            )
        if (
            float(_required(imu, "expected_rate_hz"))
            < lio_config.required_imu_rate_hz
        ):
            raise ConfigError(
                "Cube LIO pose output requires the configured IMU LIO rate"
            )
        for key in (
            "imu_to_body_extrinsics_verified",
            "lidar_to_body_extrinsics_verified",
            "imu_noise_profile_verified",
            "sensor_time_sync_verified",
        ):
            if not bool(_required(calibration, key)):
                raise ConfigError(f"Cube LIO pose output requires {key}")

    if nav.slam_return.live_control_enabled:
        if nav.slam_return.stage != "active":
            raise ConfigError(
                "live SLAM return requires the active return stage"
            )
        if not nav.autonomous_control_enabled:
            raise ConfigError(
                "live SLAM return requires autonomous_control_enabled"
            )
        if not lio_config.validation.approved:
            raise ConfigError(
                "live SLAM return requires an approved LIO trajectory"
            )
        if obstacle_config.stage != "active":
            raise ConfigError(
                "live SLAM return requires active obstacle monitoring"
            )
        for key in (
            "camera_intrinsics_verified",
            "camera_to_body_extrinsics_verified",
            "imu_to_body_extrinsics_verified",
            "lidar_to_body_extrinsics_verified",
            "imu_noise_profile_verified",
            "sensor_time_sync_verified",
        ):
            if not bool(_required(calibration, key)):
                raise ConfigError(f"live SLAM return requires {key}")
        if not nav.slam_return.approval_file:
            raise ConfigError(
                "live SLAM return requires an approval marker path"
            )

    return ProjectConfig(
        flight_controller=FlightControllerConfig(
            endpoint=str(_required(fc, "endpoint")),
            baud=int(_required(fc, "baud")),
            system_id=int(_required(fc, "system_id")),
            companion_system_id=int(
                _required(fc, "companion_system_id")
            ),
            companion_component_id=int(
                _required(fc, "companion_component_id")
            ),
            heartbeat_timeout_s=_positive(
                _required(fc, "heartbeat_timeout_s"), "heartbeat_timeout_s"
            ),
            sample_window_s=_positive(
                _required(fc, "sample_window_s"), "sample_window_s"
            ),
            hflow_min_bench_quality=int(
                _required(fc, "hflow_min_bench_quality")
            ),
            cube_mount=CubeMountConfig(
                x_m=float(_required(cube_position, "x")),
                y_m=float(_required(cube_position, "y")),
                z_m=float(_required(cube_position, "z")),
                yaw_ccw_deg=float(_required(cube_mount, "yaw_ccw_deg")),
                ahrs_orientation=int(
                    _required(cube_mount, "ahrs_orientation")
                ),
                ahrs_orientation_name=str(
                    _required(cube_mount, "ahrs_orientation_name")
                ),
            ),
            router=MavlinkRouterConfig(
                enabled=router_enabled,
                serial_endpoint=router_serial_endpoint,
                bind_host=router_bind_host,
                bind_port=router_bind_port,
                client_host=router_client_host,
                client_port=router_client_port,
                status_file=router_status_file,
            ),
        ),
        depth_camera=DepthCameraConfig(
            model=str(_required(camera, "model")),
            backend=str(_required(camera, "backend")),
            mounting=str(_required(camera, "mounting")),
            serial=(
                None
                if camera.get("serial") is None
                else str(camera["serial"])
            ),
            width=int(_required(camera, "width")),
            height=int(_required(camera, "height")),
            fps=int(_required(camera, "fps")),
            stream_host=str(_required(camera, "stream_host")),
            stream_port=int(_required(camera, "stream_port")),
            jpeg_quality=int(_required(camera, "jpeg_quality")),
            position_from_cg_frd_m=PositionConfig(
                x=float(_required(camera_position, "x")),
                y=float(_required(camera_position, "y")),
                z=float(_required(camera_position, "z")),
            ),
            rotation_from_forward_frd=RotationConfig(
                roll_deg=float(_required(camera_rotation, "roll")),
                pitch_deg=float(_required(camera_rotation, "pitch")),
                yaw_deg=float(_required(camera_rotation, "yaw")),
            ),
        ),
        external_imu=ImuConfig(
            model=str(_required(imu, "model")),
            symlink=str(_required(imu, "symlink")),
            usb_vid=int(_required(imu, "usb_vid")),
            usb_pid=int(_required(imu, "usb_pid")),
            baud=int(_positive(_required(imu, "baud"), "external_imu.baud")),
            expected_rate_hz=_positive(
                _required(imu, "expected_rate_hz"),
                "external_imu.expected_rate_hz",
            ),
            sensor_time_enabled=bool(
                _required(imu, "sensor_time_enabled")
            ),
            body_axis_signs=parsed_imu_axis_signs,
            axis_map_verified=bool(_required(imu, "axis_map_verified")),
            axis_map_verification=str(
                _required(imu, "axis_map_verification")
            ),
            position_from_cg_frd_m=PositionConfig(
                x=float(_required(imu_position, "x")),
                y=float(_required(imu_position, "y")),
                z=float(_required(imu_position, "z")),
            ),
            position_verified=bool(_required(imu, "position_verified")),
        ),
        lidar=LidarConfig(
            model=str(_required(lidar, "model")),
            transport=lidar_transport,
            symlink=str(_required(lidar, "symlink")),
            usb_vid=int(_required(lidar, "usb_vid")),
            usb_pid=int(_required(lidar, "usb_pid")),
            usb_serial=str(_required(lidar, "usb_serial")),
            baud=lidar_baud,
            legacy_baud=lidar_legacy_baud,
            baud_verified=bool(_required(lidar, "baud_verified")),
            packet_probe_s=_positive(
                _required(lidar, "packet_probe_s"), "lidar.packet_probe_s"
            ),
            sdk_revision=str(_required(lidar, "sdk_revision")),
            bridge_binary=str(_required(lidar, "bridge_binary")),
            correction_file=str(_required(lidar, "correction_file")),
            correction_verified=bool(
                _required(lidar, "correction_verified")
            ),
            position_from_cg_frd_m=PositionConfig(
                x=float(_required(lidar_position, "x")),
                y=float(_required(lidar_position, "y")),
                z=float(_required(lidar_position, "z")),
            ),
            rotation_to_body_frd=RotationConfig(
                roll_deg=float(_required(lidar_rotation, "roll")),
                pitch_deg=float(_required(lidar_rotation, "pitch")),
                yaw_deg=float(_required(lidar_rotation, "yaw")),
            ),
        ),
        lidar_inertial_odometry=lio_config,
        obstacle_avoidance=obstacle_config,
        calibration=CalibrationConfig(
            **{
                key: bool(_required(calibration, key))
                for key in (
                    "camera_intrinsics_verified",
                    "camera_to_body_extrinsics_verified",
                    "imu_to_body_extrinsics_verified",
                    "lidar_to_body_extrinsics_verified",
                    "imu_noise_profile_verified",
                    "sensor_time_sync_verified",
                )
            }
        ),
        navigation=nav,
        safety=SafetyConfig(
            forbidden_modes=forbidden_modes,
            standard_rtl_allowed_without_global_position=bool(
                _required(
                    safety, "standard_rtl_allowed_without_global_position"
                )
            ),
            rc_disarm_switch_configured=bool(
                _required(safety, "rc_disarm_switch_configured")
            ),
            automatic_ekf_source_switching_enabled=bool(
                _required(
                    safety, "automatic_ekf_source_switching_enabled"
                )
            ),
        ),
    )
