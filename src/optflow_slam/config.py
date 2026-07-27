"""Load and validate the project configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Raised when a project configuration is incomplete or unsafe."""


@dataclass(frozen=True)
class FlightControllerConfig:
    endpoint: str
    baud: int
    system_id: int
    companion_component_id: int
    heartbeat_timeout_s: float
    sample_window_s: float
    hflow_min_bench_quality: int


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


@dataclass(frozen=True)
class ImuConfig:
    model: str
    symlink: str
    usb_vid: int
    usb_pid: int


@dataclass(frozen=True)
class LidarConfig:
    model: str
    lidar_ip: str
    jetson_ip: str
    udp_port: int
    packet_probe_s: float
    network_values_verified: bool


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
class NavigationConfig:
    autonomous_control_enabled: bool
    external_nav_to_cube_enabled: bool
    target_rate_hz: float
    initial_max_horizontal_speed_mps: float
    initial_max_vertical_speed_mps: float
    initial_max_yaw_rate_dps: float
    obstacle_stop_distance_m: float
    local_pose_stale_timeout_s: float
    command_stale_timeout_s: float


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


def load_config(path: str | Path) -> ProjectConfig:
    config_path = Path(path)
    raw = yaml.safe_load(config_path.read_text(encoding="ascii"))
    if not isinstance(raw, dict):
        raise ConfigError("configuration root must be a mapping")
    if raw.get("schema_version") != 1:
        raise ConfigError("unsupported schema_version")

    fc = _mapping(raw, "flight_controller")
    sensors = _mapping(raw, "sensors")
    camera = _mapping(sensors, "depth_camera")
    imu = _mapping(sensors, "external_imu")
    lidar = _mapping(sensors, "lidar")
    calibration = _mapping(raw, "calibration")
    navigation = _mapping(raw, "navigation")
    safety = _mapping(raw, "safety")

    forbidden_modes = tuple(str(mode).upper() for mode in _required(safety, "forbidden_modes"))
    if "STABILIZE" not in forbidden_modes:
        raise ConfigError("STABILIZE must remain forbidden")

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
        obstacle_stop_distance_m=_positive(
            _required(navigation, "obstacle_stop_distance_m"),
            "obstacle_stop_distance_m",
        ),
        local_pose_stale_timeout_s=_positive(
            _required(navigation, "local_pose_stale_timeout_s"),
            "local_pose_stale_timeout_s",
        ),
        command_stale_timeout_s=_positive(
            _required(navigation, "command_stale_timeout_s"),
            "command_stale_timeout_s",
        ),
    )
    if nav.external_nav_to_cube_enabled and not nav.autonomous_control_enabled:
        raise ConfigError(
            "external_nav_to_cube_enabled requires autonomous_control_enabled"
        )

    return ProjectConfig(
        flight_controller=FlightControllerConfig(
            endpoint=str(_required(fc, "endpoint")),
            baud=int(_required(fc, "baud")),
            system_id=int(_required(fc, "system_id")),
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
        ),
        external_imu=ImuConfig(
            model=str(_required(imu, "model")),
            symlink=str(_required(imu, "symlink")),
            usb_vid=int(_required(imu, "usb_vid")),
            usb_pid=int(_required(imu, "usb_pid")),
        ),
        lidar=LidarConfig(
            model=str(_required(lidar, "model")),
            lidar_ip=str(_required(lidar, "lidar_ip")),
            jetson_ip=str(_required(lidar, "jetson_ip")),
            udp_port=int(_required(lidar, "udp_port")),
            packet_probe_s=_positive(
                _required(lidar, "packet_probe_s"), "packet_probe_s"
            ),
            network_values_verified=bool(
                _required(lidar, "network_values_verified")
            ),
        ),
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
