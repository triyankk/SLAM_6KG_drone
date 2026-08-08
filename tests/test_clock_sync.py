import pytest

from optflow_slam.clock_sync import AffineClockMapper


def test_affine_clock_mapper_rejects_outliers_and_maps_sensor_time() -> None:
    mapper = AffineClockMapper(
        minimum_samples=20,
        minimum_span_s=0.09,
        maximum_drift_ppm=2000.0,
        maximum_residual_p95_ms=2.0,
    )
    sensor_origin = 1_724_000_000.0
    host_origin = 12_345.0
    for index in range(30):
        sensor = sensor_origin + index * 0.005
        jitter = 0.0001 if index % 2 else -0.0001
        if index == 12:
            jitter = 0.003
        mapper.add(sensor, host_origin + index * 0.005 + jitter)

    assert mapper.ready
    assert mapper.fit.inliers == 29
    assert mapper.fit.residual_p95_ms < 1.0
    assert mapper.map(sensor_origin + 0.2) == pytest.approx(
        host_origin + 0.2,
        abs=0.001,
    )


def test_affine_clock_mapper_clears_after_sensor_clock_reset() -> None:
    mapper = AffineClockMapper(
        minimum_samples=5,
        minimum_span_s=0.03,
    )
    for index in range(8):
        mapper.add(100.0 + index * 0.01, 20.0 + index * 0.01)
    assert mapper.ready

    fit = mapper.add(1.0, 20.1)

    assert mapper.resets == 1
    assert fit.samples == 1
    assert not fit.ready


def test_affine_clock_mapper_tracks_slow_drift_with_time_bounded_window() -> None:
    mapper = AffineClockMapper(
        window_samples=2000,
        minimum_samples=20,
        minimum_span_s=2.0,
        maximum_drift_ppm=5000.0,
        maximum_residual_p95_ms=15.0,
        maximum_window_span_s=60.0,
    )
    host_origin = 10_000.0
    readiness_after_warmup = []
    for index in range(4500):
        sensor_s = index * 0.2
        changing_clock_offset_s = 1.0e-6 * sensor_s * sensor_s
        callback_jitter_s = 0.0004 if index % 2 else -0.0004
        host_s = (
            host_origin
            + sensor_s
            + changing_clock_offset_s
            + callback_jitter_s
        )
        fit = mapper.add(sensor_s, host_s)
        if sensor_s >= 60.0:
            readiness_after_warmup.append(fit.ready)

    expected_host_s = host_origin + 900.0 + 1.0e-6 * 900.0**2

    assert all(readiness_after_warmup)
    assert mapper.fit.span_s <= 60.2
    assert mapper.fit.residual_p95_ms is not None
    assert mapper.fit.residual_p95_ms < 2.0
    assert mapper.map(900.0) == pytest.approx(expected_host_s, abs=0.003)


def test_affine_clock_mapper_rejects_window_shorter_than_warmup() -> None:
    with pytest.raises(ValueError, match="must cover minimum_span_s"):
        AffineClockMapper(
            minimum_span_s=2.0,
            maximum_window_span_s=1.0,
        )
