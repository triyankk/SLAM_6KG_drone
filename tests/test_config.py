from pathlib import Path

import pytest

from optflow_slam.config import ConfigError, load_config


ROOT = Path(__file__).resolve().parents[1]


def test_default_config_is_conservative() -> None:
    config = load_config(ROOT / "config" / "system.yaml")

    assert config.flight_controller.endpoint == "/dev/ttyTHS1"
    assert config.flight_controller.baud == 921600
    assert config.flight_controller.cube_mount.x_m == 0.08
    assert config.flight_controller.cube_mount.y_m == 0.0
    assert config.flight_controller.cube_mount.z_m == -0.08
    assert config.flight_controller.cube_mount.ahrs_orientation == 6
    assert config.depth_camera.model == "Intel RealSense D415"
    assert config.depth_camera.serial == "327322062285"
    assert config.depth_camera.stream_port == 8770
    assert not config.navigation.autonomous_control_enabled
    assert not config.navigation.external_nav_to_cube_enabled
    assert not config.safety.standard_rtl_allowed_without_global_position
    assert "STABILIZE" in config.safety.forbidden_modes
    assert config.external_imu.baud == 9600
    assert config.external_imu.expected_rate_hz == 10


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
