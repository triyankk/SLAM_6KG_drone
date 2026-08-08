import math

import pytest

from optflow_slam.lio_validation import (
    _read_ndjson,
    cube_attitude_metrics,
    cube_reference_metrics,
    guided_translation_metrics,
    sensor_trace_metrics,
    trajectory_metrics,
)


def test_read_ndjson_ignores_power_loss_nul_padding(tmp_path) -> None:
    path = tmp_path / "odometry.ndjson"
    path.write_bytes(b'{"sample":1}\n' + b"\x00" * 4096)

    assert _read_ndjson(path) == [{"sample": 1}]


def odometry_row(time_s: float, x_m: float) -> dict:
    return {
        "host_monotonic_ns": int(time_s * 1.0e9),
        "position_m": [x_m, 0.0, 0.0],
        "linear_velocity_mps": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def test_trajectory_metrics_measure_closure_and_static_windows() -> None:
    rows = []
    for index in range(101):
        time_s = index * 0.1
        if time_s <= 2.0:
            x_m = 0.002 * math.sin(index)
        elif time_s >= 8.0:
            x_m = 0.002 * math.sin(index)
        else:
            x_m = min(time_s - 2.0, 8.0 - time_s)
        rows.append(odometry_row(time_s, x_m))

    metrics = trajectory_metrics(rows, stationary_window_s=2.0)

    assert metrics["duration_s"] == pytest.approx(10.0)
    assert metrics["odometry_rate_hz"] == pytest.approx(10.0)
    assert metrics["maximum_stationary_drift_m"] < 0.005
    assert metrics["return_to_start_error_m"] < 0.005
    assert metrics["non_monotonic_timestamps"] == 0


def test_trajectory_metrics_reject_non_finite_rows() -> None:
    rows = [odometry_row(1.0, 0.0), odometry_row(2.0, float("nan"))]

    metrics = trajectory_metrics(rows, stationary_window_s=1.0)

    assert metrics["samples"] == 1
    assert metrics["invalid_rows"] == 1


def test_trajectory_metrics_derives_speed_from_pose() -> None:
    rows = [odometry_row(1.0, 0.0), odometry_row(1.1, 0.5)]

    metrics = trajectory_metrics(rows, stationary_window_s=0.05)

    assert metrics["maximum_reported_speed_mps"] == 0.0
    assert metrics["maximum_derived_speed_mps"] == pytest.approx(5.0)


def test_cube_reference_metrics_align_yaw_but_preserve_scale() -> None:
    odometry = []
    cube = []
    for index in range(120):
        time_ns = index * 100_000_000
        x_m = index * 0.03
        y_m = 0.4 * math.sin(index * 0.08)
        odometry.append(
            {
                **odometry_row(time_ns / 1.0e9, x_m),
                "position_m": [x_m, y_m, index * 0.001],
            }
        )
        cube.append(
            {
                "host_monotonic_ns": time_ns + 5_000_000,
                "type": "LOCAL_POSITION_NED",
                "data": {
                    "x": -y_m,
                    "y": x_m,
                    "z": index * 0.001,
                },
            }
        )

    metrics = cube_reference_metrics(odometry, cube)

    assert metrics["available"] is True
    assert metrics["paired_samples"] == 120
    assert metrics["horizontal_aligned_rmse_m"] < 1.0e-9
    assert metrics["vertical_rmse_m"] < 1.0e-9
    assert metrics["path_length_ratio"] == pytest.approx(1.0)
    assert metrics["pairing_error_p95_ms"] == pytest.approx(5.0)


def test_cube_reference_metrics_reports_scale_error() -> None:
    odometry = []
    cube = []
    for index in range(40):
        time_ns = index * 100_000_000
        odometry.append(odometry_row(time_ns / 1.0e9, index * 0.2))
        cube.append(
            {
                "host_monotonic_ns": time_ns,
                "type": "LOCAL_POSITION_NED",
                "data": {"x": index * 0.1, "y": 0.0, "z": 0.0},
            }
        )

    metrics = cube_reference_metrics(odometry, cube)

    assert metrics["path_length_ratio"] == pytest.approx(2.0)
    assert metrics["horizontal_aligned_rmse_m"] > 1.0


def test_cube_attitude_metrics_removes_initial_world_yaw() -> None:
    odometry = []
    cube = []
    for index in range(120):
        time_ns = index * 100_000_000
        yaw = 0.5 * math.sin(index * 0.04)
        odometry.append(
            {
                **odometry_row(time_ns / 1.0e9, 0.0),
                "quaternion_xyzw": [
                    0.0,
                    0.0,
                    math.sin(yaw * 0.5),
                    math.cos(yaw * 0.5),
                ],
            }
        )
        cube.append(
            {
                "host_monotonic_ns": time_ns + 4_000_000,
                "type": "ATTITUDE",
                "data": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": yaw + 0.7,
                },
            }
        )

    metrics = cube_attitude_metrics(odometry, cube)

    assert metrics["available"] is True
    assert metrics["paired_samples"] == 120
    assert metrics["attitude_error_p95_deg"] < 1.0e-5


def test_sensor_trace_metrics_uses_sensor_stamps_for_rate() -> None:
    rows = [
        {
            "ros_time_ns": 1_700_000_000_000_000_000 + index * 5_000_000,
            "host_unix_ns": (
                1_700_000_000_002_000_000 + index * 5_000_000
            ),
        }
        for index in range(401)
    ]

    metrics = sensor_trace_metrics(rows)

    assert metrics["rate_hz"] == pytest.approx(200.0)
    assert metrics["non_monotonic_timestamps"] == 0
    assert metrics["delivery_offset_median_ms"] == pytest.approx(2.0)


def test_guided_translation_metrics_scores_tape_marked_axes() -> None:
    targets = {
        "settle": [0.0, 0.0, 0.0],
        "forward_1": [0.5, 0.0, 0.0],
        "center_1": [0.0, 0.0, 0.0],
        "right_1": [0.0, 0.5, 0.0],
        "center_2": [0.0, 0.0, 0.0],
        "final_still": [0.0, 0.0, 0.0],
    }
    observed = {
        **targets,
        "forward_1": [0.48, 0.02, 0.01],
        "right_1": [-0.01, 0.52, -0.02],
        "center_1": [0.01, -0.01, 0.0],
    }
    payload = {
        "guide_kind": "translation",
        "guide_complete": True,
        "reference": "operator_positioned_tape_marks",
        "cube_local_position_used_as_ground_truth": False,
        "captures": [
            {
                "phase_id": phase_id,
                "target_m": target,
                "observed_m": observed[phase_id],
                "samples": 12,
            }
            for phase_id, target in targets.items()
        ],
    }

    metrics = guided_translation_metrics(payload)

    assert metrics["available"]
    assert metrics["complete"]
    assert metrics["forward_scale"] == pytest.approx(0.96)
    assert metrics["right_scale"] == pytest.approx(1.04)
    assert metrics["maximum_cross_axis_error_m"] == pytest.approx(0.02)
    assert metrics["maximum_return_error_m"] == pytest.approx(math.sqrt(0.0002))
