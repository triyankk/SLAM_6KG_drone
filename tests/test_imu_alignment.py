import numpy as np
import pytest

from optflow_slam.imu_alignment import (
    cube_rate_alignment,
    odometry_rate_alignment,
)


def _body_rates(time_s: np.ndarray) -> np.ndarray:
    return np.column_stack(
        (
            0.12 * np.sin(0.73 * time_s) + 0.04 * np.sin(2.17 * time_s),
            0.10 * np.cos(0.61 * time_s) + 0.03 * np.sin(1.83 * time_s),
            0.18 * np.sin(0.47 * time_s) + 0.05 * np.cos(1.57 * time_s),
        )
    )


def test_cube_rate_alignment_recovers_delayed_imu_timestamps() -> None:
    delay_s = 0.018
    imu_time_s = np.arange(0.5, 30.0, 0.005)
    cube_time_s = np.arange(0.0, 30.5, 0.02)
    imu_rates = _body_rates(imu_time_s - delay_s)
    cube_rates = _body_rates(cube_time_s)

    result = cube_rate_alignment(imu_time_s, imu_rates, cube_time_s, cube_rates)

    for axis in ("x", "y", "z"):
        assert result[axis] is not None
        assert result[axis]["accepted"]
        assert result[axis]["correlation"] > 0.999
        assert result[axis]["cube_rate_per_imu_rate"] == pytest.approx(
            1.0,
            abs=0.01,
        )
        assert result[axis]["imu_timestamp_advance_crosscheck_s"] == (
            pytest.approx(delay_s, abs=0.0011)
        )


def test_odometry_rate_alignment_recovers_positive_fast_lio_offset() -> None:
    delay_s = 0.014
    dense_time_s = np.arange(0.0, 32.0, 0.001)
    yaw_rate = _body_rates(dense_time_s)[:, 2]
    yaw = np.concatenate(
        (
            np.zeros(1),
            np.cumsum((yaw_rate[:-1] + yaw_rate[1:]) * 0.0005),
        )
    )
    odometry_time_s = np.arange(1.0, 31.0, 0.2)
    odometry_yaw = np.interp(odometry_time_s, dense_time_s, yaw)
    odometry_quaternions = np.column_stack(
        (
            np.zeros_like(odometry_yaw),
            np.zeros_like(odometry_yaw),
            np.sin(odometry_yaw / 2.0),
            np.cos(odometry_yaw / 2.0),
        )
    )
    imu_time_s = np.arange(0.0, 32.0, 0.005)
    imu_rates = np.zeros((len(imu_time_s), 3))
    imu_rates[:, 2] = _body_rates(imu_time_s - delay_s)[:, 2]

    result = odometry_rate_alignment(
        imu_time_s,
        imu_rates,
        odometry_time_s,
        odometry_quaternions,
    )

    assert result["x"] is None
    assert result["y"] is None
    assert result["z"] is not None
    assert result["z"]["accepted"]
    assert result["z"]["correlation"] > 0.999
    assert result["z"]["time_offset_lidar_to_imu_candidate_s"] == (
        pytest.approx(delay_s, abs=0.0011)
    )
