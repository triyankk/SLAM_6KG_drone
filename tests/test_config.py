from pathlib import Path

import pytest

from optflow_slam.config import ConfigError, load_config


ROOT = Path(__file__).resolve().parents[1]


def test_default_config_is_conservative() -> None:
    config = load_config(ROOT / "config" / "system.yaml")

    assert config.flight_controller.endpoint == "/dev/ttyTHS1"
    assert config.flight_controller.baud == 460800
    assert not config.flight_controller.router.enabled
    assert config.flight_controller.router.serial_endpoint == "/dev/ttyTHS1"
    assert config.flight_controller.router.bind_port == 14600
    assert config.flight_controller.router.client_port == 14601
    assert config.flight_controller.system_id == 1
    assert config.flight_controller.companion_system_id == 1
    assert config.flight_controller.cube_mount.x_m == 0.08
    assert config.flight_controller.cube_mount.y_m == 0.0
    assert config.flight_controller.cube_mount.z_m == -0.08
    assert config.flight_controller.cube_mount.ahrs_orientation == 6
    assert config.depth_camera.model == "Intel RealSense D415"
    assert config.depth_camera.serial == "327322062285"
    assert config.depth_camera.stream_port == 8770
    assert config.depth_camera.position_from_cg_frd_m.x == 0.19
    assert config.depth_camera.position_from_cg_frd_m.y == 0.0
    assert config.depth_camera.position_from_cg_frd_m.z == 0.10
    assert config.lidar.transport == "serial_rs485"
    assert config.lidar.symlink == "/dev/jt16_usb"
    assert config.lidar.usb_vid == 0x067B
    assert config.lidar.usb_pid == 0x23A3
    assert config.lidar.usb_serial == "DCCEb114J19"
    assert config.lidar.baud == 3_000_000
    assert config.lidar.legacy_baud == 3_125_000
    assert config.lidar.baud_verified
    assert config.lidar.position_from_cg_frd_m.x == 0.0
    assert config.lidar.position_from_cg_frd_m.y == 0.0
    assert config.lidar.position_from_cg_frd_m.z == -0.10
    assert config.lidar.sdk_revision == (
        "534c707846a810e8211b93446f878dbf415f7000"
    )
    assert config.lidar.bridge_binary.endswith("optflow-jt16-bridge")
    assert config.lidar.correction_verified
    assert config.obstacle_avoidance.stage == "active"
    assert config.obstacle_avoidance.mavlink_output_enabled
    assert not config.obstacle_avoidance.depth_camera_enabled
    assert config.obstacle_avoidance.lidar_enabled
    assert config.obstacle_avoidance.target_rate_hz == 10.0
    assert config.obstacle_avoidance.sector_count == 72
    assert config.obstacle_avoidance.hard_cg_clearance_m == 1.5
    assert config.obstacle_avoidance.airframe_radius_m == 0.75
    assert config.obstacle_avoidance.airframe_geometry_verified
    assert config.obstacle_avoidance.rc_toggle.channel == 7
    assert config.obstacle_avoidance.rc_toggle.engage_pwm == 1700
    assert config.obstacle_avoidance.rc_toggle.disengage_pwm == 1300
    assert config.obstacle_avoidance.alerts.warning_distance_m == 2.0
    assert config.obstacle_avoidance.alerts.escalation_distance_m == 1.25
    assert config.obstacle_avoidance.alerts.warning_rate_hz == 1.0
    assert config.obstacle_avoidance.alerts.keepout_rate_hz == 3.0
    assert config.obstacle_avoidance.alerts.maximum_rate_hz == 10.0
    assert config.obstacle_avoidance.native.proximity_type == 2
    assert config.obstacle_avoidance.native.enable_mask == 7
    assert config.obstacle_avoidance.native.rc_option == 40
    assert not config.navigation.autonomous_control_enabled
    assert not config.navigation.external_nav_to_cube_enabled
    assert config.navigation.slam_return.rc_channel == 9
    assert config.navigation.slam_return.land_rc_channel == 10
    assert config.navigation.slam_return.maximum_altitude_m == 8.0
    assert not config.safety.standard_rtl_allowed_without_global_position
    assert "STABILIZE" in config.safety.forbidden_modes
    assert config.external_imu.baud == 230400
    assert config.external_imu.expected_rate_hz == 200
    assert config.external_imu.sensor_time_enabled
    assert config.external_imu.body_axis_signs.x == 1
    assert config.external_imu.body_axis_signs.y == -1
    assert config.external_imu.body_axis_signs.z == -1
    assert config.external_imu.axis_map_verified
    assert config.external_imu.position_from_cg_frd_m.x == 0.08
    assert config.external_imu.position_from_cg_frd_m.y == 0.0
    assert config.external_imu.position_from_cg_frd_m.z == -0.09
    assert config.external_imu.position_verified
    assert config.lidar_inertial_odometry.stage == "shadow"
    assert config.lidar_inertial_odometry.backend == "hesai_fast_lio2"
    assert config.lidar_inertial_odometry.odometry_shadow_to_cube_enabled
    assert not config.lidar_inertial_odometry.pose_output_to_cube_enabled
    assert (
        config.lidar_inertial_odometry.clock_sync
        .maximum_imu_window_span_s
        == 20.0
    )
    assert (
        config.lidar_inertial_odometry.clock_sync
        .maximum_lidar_window_span_s
        == 60.0
    )
    assert not config.lidar_inertial_odometry.validation.approved
    assert (
        config.lidar_inertial_odometry.validation
        .minimum_cube_reference_samples
        == 100
    )
    assert (
        config.lidar_inertial_odometry.validation
        .maximum_cube_horizontal_rmse_m
        == 0.5
    )


def test_hard_cg_clearance_must_be_inside_sensor_range(
    tmp_path: Path,
) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(
        encoding="ascii"
    )
    source = source.replace(
        "hard_cg_clearance_m: 1.50",
        "hard_cg_clearance_m: 9.00",
    )
    config_path = tmp_path / "invalid-clearance.yaml"
    config_path.write_text(source, encoding="ascii")

    with pytest.raises(ConfigError):
        load_config(config_path)


def test_land_and_return_require_separate_channels(tmp_path: Path) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(
        encoding="ascii"
    )
    source = source.replace("land_rc_channel: 10", "land_rc_channel: 9")
    config_path = tmp_path / "duplicate-rc-channel.yaml"
    config_path.write_text(source, encoding="ascii")

    with pytest.raises(ConfigError, match="separate RC channels"):
        load_config(config_path)


def test_external_nav_cannot_be_enabled_without_control(tmp_path: Path) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(encoding="ascii")
    source = source.replace(
        "external_nav_to_cube_enabled: false",
        "external_nav_to_cube_enabled: true",
    )
    config_path = tmp_path / "unsafe.yaml"
    config_path.write_text(source, encoding="ascii")

    with pytest.raises(ConfigError):
        load_config(config_path)


def test_lidar_transport_must_be_serial_rs485(
    tmp_path: Path,
) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(encoding="ascii")
    source = source.replace(
        "transport: serial_rs485",
        "transport: ethernet",
    )
    config_path = tmp_path / "invalid-lidar-transport.yaml"
    config_path.write_text(source, encoding="ascii")

    with pytest.raises(ConfigError):
        load_config(config_path)


def test_active_obstacle_output_requires_explicit_active_stage(
    tmp_path: Path,
) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(encoding="ascii")
    source = source.replace(
        "obstacle_avoidance:\n  stage: active",
        "obstacle_avoidance:\n  stage: shadow",
    )
    config_path = tmp_path / "unsafe-obstacle-output.yaml"
    config_path.write_text(source, encoding="ascii")

    with pytest.raises(ConfigError):
        load_config(config_path)


def test_active_lidar_output_is_independent_of_lio_trajectory_approval(
    tmp_path: Path,
) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(encoding="ascii")
    config_path = tmp_path / "verified-obstacle-output.yaml"
    config_path.write_text(source, encoding="ascii")

    config = load_config(config_path)

    assert config.obstacle_avoidance.stage == "active"
    assert config.obstacle_avoidance.mavlink_output_enabled
    assert not config.lidar_inertial_odometry.validation.approved


def test_active_lidar_output_still_requires_verified_extrinsics(
    tmp_path: Path,
) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(encoding="ascii")
    source = source.replace(
        "lidar_to_body_extrinsics_verified: true",
        "lidar_to_body_extrinsics_verified: false",
    )
    config_path = tmp_path / "unverified-lidar-output.yaml"
    config_path.write_text(source, encoding="ascii")

    with pytest.raises(ConfigError, match="requires lidar extrinsics"):
        load_config(config_path)
