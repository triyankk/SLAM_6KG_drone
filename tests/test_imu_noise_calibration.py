import numpy as np
import pytest

from optflow_slam.imu_noise_calibration import (
    allan_deviation,
    analyze_capture,
    noise_density_from_allan,
    stationary_metrics,
)


def test_allan_deviation_recovers_white_noise_density() -> None:
    random = np.random.default_rng(42)
    period_s = 0.005
    standard_deviation = np.asarray([0.02, 0.03, 0.04])
    samples = random.normal(size=(40_000, 3)) * standard_deviation

    taus, deviation = allan_deviation(samples, period_s)
    density = noise_density_from_allan(taus, deviation)

    assert density == pytest.approx(
        standard_deviation * np.sqrt(period_s),
        rel=0.12,
    )


def test_stationary_metrics_accepts_quiet_level_imu() -> None:
    random = np.random.default_rng(7)
    count = 24_000
    accel = random.normal(scale=0.01, size=(count, 3))
    accel[:, 2] += 9.80665
    gyro = random.normal(scale=0.001, size=(count, 3))

    metrics = stationary_metrics(accel, gyro, 200.0)

    assert metrics["stationary"]
    assert metrics["gravity_norm_mpss"] == pytest.approx(9.80665, abs=0.01)


def test_stationary_metrics_rejects_physical_motion() -> None:
    random = np.random.default_rng(9)
    count = 24_000
    accel = random.normal(scale=0.01, size=(count, 3))
    accel[:, 2] += 9.80665
    gyro = random.normal(scale=0.001, size=(count, 3))
    accel[10_000:11_000, 0] += 1.0
    gyro[10_000:11_000, 1] += 0.1

    metrics = stationary_metrics(accel, gyro, 200.0)

    assert not metrics["stationary"]
    assert metrics["accel_residual_norm_p999_mpss"] > 0.5
    assert metrics["gyro_residual_norm_p999_rads"] > 0.05


def test_capture_analysis_keeps_fast_lio_candidate_read_only() -> None:
    random = np.random.default_rng(11)
    count = 20_000
    sensor_time = np.arange(count, dtype=np.float64) * 0.005
    accel = random.normal(scale=0.01, size=(count, 3))
    accel[:, 2] += 9.80665
    gyro = random.normal(scale=0.001, size=(count, 3))
    capture = {
        "sensor_time_s": sensor_time,
        "accel_mss": accel,
        "gyro_rads": gyro,
        "decoder": {
            "valid_frames": count * 3,
            "checksum_errors": 0,
            "payload_errors": 0,
            "discarded_bytes": 0,
            "frame_type_counts": {
                "0x50": count,
                "0x51": count,
                "0x52": count,
            },
            "completed_samples": count,
            "incomplete_samples": 0,
        },
    }

    analysis = analyze_capture(capture)

    assert analysis["result"] == "pass"
    assert analysis["sample_rate_hz"] == pytest.approx(200.0)
    assert analysis["estimated_drops"] == 0
    assert analysis["fast_lio_candidate"]["apply_automatically"] is False
    assert analysis["fast_lio_candidate"]["b_acc_cov"] is None
    assert analysis["fast_lio_candidate"]["b_gyr_cov"] is None


def test_capture_analysis_applies_quantization_floor_to_zero_gyro() -> None:
    count = 20_000
    sensor_time = np.arange(count, dtype=np.float64) * 0.005
    accel = np.zeros((count, 3))
    accel[:, 2] = 9.80665
    gyro = np.zeros((count, 3))
    capture = {
        "sensor_time_s": sensor_time,
        "accel_mss": accel,
        "gyro_rads": gyro,
        "decoder": {
            "checksum_errors": 0,
            "payload_errors": 0,
            "incomplete_samples": 0,
        },
    }

    analysis = analyze_capture(capture)

    assert analysis["result"] == "pass"
    assert analysis["quantization"]["gyro_unique_levels"] == [1, 1, 1]
    assert analysis["quantization"]["gyro_allan_noise_observable"] == [
        False,
        False,
        False,
    ]
    assert analysis["allan"]["gyro_bias_instability_rads"] == [
        None,
        None,
        None,
    ]
    assert (
        analysis["fast_lio_candidate"]["gyr_cov_measurement_noise_candidate"]
        > 0.0
    )
