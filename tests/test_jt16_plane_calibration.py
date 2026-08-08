import numpy as np
import pytest

from optflow_slam.jt16_plane_calibration import (
    analyze_planes,
    robust_plane_fit,
)


def test_robust_plane_fit_rejects_outliers() -> None:
    random = np.random.default_rng(41)
    y = random.uniform(-1.0, 1.0, 5_000)
    z = random.uniform(-0.8, 0.8, 5_000)
    wall = np.column_stack(
        (2.5 + random.normal(0.0, 0.008, len(y)), y, z)
    )
    outliers = random.uniform(-3.0, 3.0, size=(500, 3))

    fit = robust_plane_fit(
        np.vstack((wall, outliers)),
        np.asarray((1.0, 0.0, 0.0)),
    )

    assert fit["metrics"]["plane_distance_m"] == pytest.approx(2.5, abs=0.01)
    assert fit["metrics"]["normal_error_deg"] < 0.2
    assert fit["metrics"]["residual_p95_m"] < 0.02
    assert fit["metrics"]["inlier_fraction"] < 0.95


def test_plane_analysis_validates_all_rings_and_level_mount() -> None:
    random = np.random.default_rng(73)
    wall_points = []
    wall_rings = []
    for ring in range(16):
        count = 500
        wall_points.append(
            np.column_stack(
                (
                    2.5 + random.normal(0.0, 0.006, count),
                    random.uniform(-0.75, 0.75, count),
                    random.normal(-0.9 + ring * 0.12, 0.01, count),
                )
            )
        )
        wall_rings.append(np.full(count, ring, dtype=np.uint16))
    floor_count = 12_000
    floor_bearing = random.uniform(np.radians(25), np.radians(335), floor_count)
    floor_radius = random.uniform(0.9, 4.5, floor_count)
    floor_points = np.column_stack(
        (
            np.cos(floor_bearing) * floor_radius,
            np.sin(floor_bearing) * floor_radius,
            0.62 + random.normal(0.0, 0.007, floor_count),
        )
    )
    floor_rings = random.integers(0, 16, floor_count, dtype=np.uint16)

    result = analyze_planes(
        np.vstack((*wall_points, floor_points)),
        np.concatenate((*wall_rings, floor_rings)),
        target_distance_m=2.5,
    )

    assert result["result"] == "pass"
    assert result["gates"]["correction_file_validated_for_unit"]
    assert result["gates"]["lidar_to_body_rotation_verified"]
    assert result["gates"]["all_16_rings_covered"]
    assert result["wall"]["residual_p95_m"] < 0.02
    assert result["floor"]["residual_p95_m"] < 0.02
    assert result["recommended_config"]["apply_automatically"] is False


def test_plane_analysis_preserves_wall_result_when_floor_is_blocked() -> None:
    random = np.random.default_rng(91)
    wall_points = []
    wall_rings = []
    for ring in range(16):
        count = 250
        wall_points.append(
            np.column_stack(
                (
                    2.5 + random.normal(0.0, 0.008, count),
                    random.uniform(-0.7, 0.7, count),
                    random.normal(-0.8 + ring * 0.10, 0.01, count),
                )
            )
        )
        wall_rings.append(np.full(count, ring, dtype=np.uint16))

    result = analyze_planes(
        np.vstack(wall_points),
        np.concatenate(wall_rings),
        target_distance_m=2.5,
    )

    assert result["result"] == "fail"
    assert result["gates"]["correction_file_validated_for_unit"]
    assert not result["gates"]["lidar_to_body_rotation_verified"]
    assert result["floor_error"]
