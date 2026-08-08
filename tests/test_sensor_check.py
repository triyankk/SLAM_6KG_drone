from optflow_slam.config import load_config
from optflow_slam.paths import CONFIG_DIR
from optflow_slam.sensor_check import evaluate_live_imu


def test_live_imu_requires_connection_freshness_and_rate() -> None:
    config = load_config(CONFIG_DIR / "system.yaml")
    result = evaluate_live_imu(
        {
            "sensors": {
                "external_imu_connected": True,
                "external_imu_age_ms": 25,
                "external_imu_rate_hz": 199.8,
            }
        },
        config,
    )

    assert result.available
    assert result.metrics["rate_hz"] == 199.8


def test_live_imu_rejects_stale_measurement() -> None:
    config = load_config(CONFIG_DIR / "system.yaml")
    result = evaluate_live_imu(
        {
            "sensors": {
                "external_imu_connected": True,
                "external_imu_age_ms": 1000,
                "external_imu_rate_hz": 200.0,
            }
        },
        config,
    )

    assert not result.available
