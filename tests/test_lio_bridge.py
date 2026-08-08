import numpy as np
import pytest

from optflow_slam.config import RotationConfig
from optflow_slam.lio_bridge import (
    FAST_LIO_POINT_DTYPE,
    JT16_MAXIMUM_FRAME_SPAN_S,
    JT16_MAXIMUM_POINT_GAP_S,
    JT16_MINIMUM_FRAME_SPAN_S,
    JT16_INPUT_DTYPE,
    filter_fast_lio_points,
    jt16_frame_time_metrics,
    pack_fast_lio_points,
    rotation_matrix,
)


def test_fast_lio_point_layout_matches_hesai_registered_fields() -> None:
    assert FAST_LIO_POINT_DTYPE.itemsize == 32
    assert FAST_LIO_POINT_DTYPE.fields["x"][1] == 0
    assert FAST_LIO_POINT_DTYPE.fields["intensity"][1] == 16
    assert FAST_LIO_POINT_DTYPE.fields["ring"][1] == 20
    assert FAST_LIO_POINT_DTYPE.fields["timestamp"][1] == 24


def test_jt16_sdk_axes_are_body_aligned_without_changing_metadata() -> None:
    records = np.zeros(1, dtype=JT16_INPUT_DTYPE)
    records["x"] = 1.0
    records["y"] = 2.0
    records["z"] = 3.0
    records["timestamp"] = 123.456
    records["ring"] = 7
    records["intensity"] = 42
    rotation = rotation_matrix(RotationConfig(0.0, 0.0, 180.0))

    packed = pack_fast_lio_points(records, rotation)

    # Raw (X right, Y forward, Z up) first becomes FRD (Y, X, -Z),
    # then the calibrated 180 degree mount yaw is applied.
    assert packed["x"][0] == pytest.approx(-2.0)
    assert packed["y"][0] == pytest.approx(-1.0)
    assert packed["z"][0] == pytest.approx(-3.0)
    assert packed["timestamp"][0] == pytest.approx(123.456)
    assert packed["ring"][0] == 7
    assert packed["intensity"][0] == 42.0


def test_jt16_frame_time_metrics_accept_a_complete_scan() -> None:
    timestamps = np.linspace(10.0, 10.2, 100)

    span_s, maximum_gap_s = jt16_frame_time_metrics(timestamps)

    assert JT16_MINIMUM_FRAME_SPAN_S <= span_s <= JT16_MAXIMUM_FRAME_SPAN_S
    assert maximum_gap_s <= JT16_MAXIMUM_POINT_GAP_S


def test_jt16_frame_time_metrics_reject_non_monotonic_points() -> None:
    with pytest.raises(ValueError, match="monotonic"):
        jt16_frame_time_metrics(np.array((1.0, 1.2, 1.1)))


def test_fast_lio_filter_rejects_bad_coordinates_ranges_and_rings() -> None:
    points = np.zeros(7, dtype=FAST_LIO_POINT_DTYPE)
    points["x"] = (1.0, np.nan, np.inf, 0.1, 31.0, 2.0, 3.0)
    points["ring"] = (0, 1, 2, 3, 4, 16, 15)

    filtered, stats = filter_fast_lio_points(points)

    assert filtered["x"].tolist() == [1.0, 3.0]
    assert stats == {
        "input_points": 7,
        "accepted_points": 2,
        "non_finite_points": 2,
        "invalid_ring_points": 1,
        "out_of_range_points": 2,
    }


def test_fast_lio_filter_handles_finite_float32_extremes() -> None:
    points = np.zeros(2, dtype=FAST_LIO_POINT_DTYPE)
    points["x"] = (np.finfo(np.float32).max, 5.0)
    points["ring"] = 0

    filtered, stats = filter_fast_lio_points(points)

    assert filtered["x"].tolist() == [5.0]
    assert stats["out_of_range_points"] == 1


def test_shadow_bridge_has_no_mavlink_pose_sender() -> None:
    source = __import__("inspect").getsource(
        __import__("optflow_slam.lio_bridge", fromlist=["unused"])
    )

    assert "pymavlink" not in source
    assert "VISION_POSITION_ESTIMATE" not in source
    assert "ODOMETRY_SEND" not in source
