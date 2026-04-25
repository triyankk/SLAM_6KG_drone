from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from .fc_config import FlightControllerSetupConfig


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
    enabled: bool = True
    lidar_enabled: bool = True
    lidar_port: str = "auto"
    lidar_baud: int = 3000000
    safety_distance_m: float = 2.0
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
    enabled: bool = False
    gps_id: int = 1
    origin_lat_deg: float = 0.0
    origin_lon_deg: float = 0.0
    origin_alt_m: float = 0.0
    satellites_visible: int = 14
    horiz_accuracy_m: float = 0.5
    vert_accuracy_m: float = 1.0
    speed_accuracy_m_s: float = 0.3


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


@dataclass
class SlamBridgeConfig:
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
    boot_delay_seconds: float = 60.0
    fc_setup: FlightControllerSetupConfig = field(default_factory=FlightControllerSetupConfig)
    obstacle: ObstacleAvoidanceConfig = field(default_factory=ObstacleAvoidanceConfig)
    qgc: QgcBridgeConfig = field(default_factory=QgcBridgeConfig)
    gps_input: GpsInputConfig = field(default_factory=GpsInputConfig)
    lidar_steering: LidarSteeringConfig = field(default_factory=LidarSteeringConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)

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
        lidar_steering_data = data.get("lidar_steering", {}) or {}
        if not isinstance(lidar_steering_data, dict):
            raise ValueError("config 'lidar_steering' must be a mapping")
        calibration_data = data.get("calibration", {}) or {}
        if not isinstance(calibration_data, dict):
            raise ValueError("config 'calibration' must be a mapping")
        profile_path = str(calibration_data.get("profile_path", "runtime/slam_calibration.json") or "")
        if profile_path and not Path(profile_path).is_absolute():
            profile_path = str((base_dir / profile_path).resolve())
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
            boot_delay_seconds=float(data.get("boot_delay_seconds", 60.0)),
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
                avoid_margin_m=float(fc_setup_data.get("avoid_margin_m", 2.0)),
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
                safety_distance_m=float(obstacle_data.get("safety_distance_m", 2.0)),
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
                origin_lat_deg=float(gps_input_data.get("origin_lat_deg", 0.0)),
                origin_lon_deg=float(gps_input_data.get("origin_lon_deg", 0.0)),
                origin_alt_m=float(gps_input_data.get("origin_alt_m", 0.0)),
                satellites_visible=int(gps_input_data.get("satellites_visible", 14)),
                horiz_accuracy_m=float(gps_input_data.get("horiz_accuracy_m", 0.5)),
                vert_accuracy_m=float(gps_input_data.get("vert_accuracy_m", 1.0)),
                speed_accuracy_m_s=float(gps_input_data.get("speed_accuracy_m_s", 0.3)),
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
            ),
        )


def load_bridge_config(path: str | Path) -> SlamBridgeConfig:
    config_path = Path(path).expanduser().resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Bridge config must be a mapping: {config_path}")
    return SlamBridgeConfig.from_mapping(data, base_dir=config_path.parent)
