from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from optflow_slam.config import load_config
from optflow_slam.obstacle_check import (
    _BenchSession,
    evaluate_target,
    summarize_scan,
)
from optflow_slam.obstacles import (
    DepthObstacleExtractor,
    LidarObstacleExtractor,
    ObstacleFusion,
    ObstacleScan,
    PointObstacleExtractor,
    UNKNOWN_DISTANCE_CM,
    obstacle_alert_state,
)


ROOT = Path(__file__).resolve().parents[1]


def settings(**changes):
    configured = load_config(
        ROOT / "config" / "system.yaml"
    ).obstacle_avoidance
    overrides = {
        "depth_sample_stride": 1,
        "minimum_points_per_sector": 1,
        "temporal_window": 1,
    }
    overrides.update(changes)
    return replace(configured, **overrides)


def alert_state(distance_m):
    configured = settings()
    return obstacle_alert_state(
        distance_m,
        hard_clearance_m=configured.hard_cg_clearance_m,
        full_rate_distance_m=max(
            configured.min_distance_m,
            configured.airframe_radius_m,
        ),
        settings=configured.alerts,
    )


def test_obstacle_alert_profile_has_warning_only_outer_band() -> None:
    assert alert_state(None).zone == "clear"
    assert alert_state(2.01).beep_rate_hz == 0.0

    warning = alert_state(2.0)
    assert warning.zone == "warning"
    assert warning.beep_rate_hz == 1.0
    assert not warning.avoidance_required

    warning = alert_state(1.51)
    assert warning.zone == "warning"
    assert warning.beep_rate_hz == 1.0
    assert not warning.avoidance_required


def test_obstacle_alert_profile_beeps_three_hz_at_hard_boundary() -> None:
    for distance_m in (1.5, 1.25):
        state = alert_state(distance_m)
        assert state.beep_rate_hz == 3.0
        assert state.avoidance_required


def test_obstacle_alert_profile_rises_below_1p25m_and_is_capped() -> None:
    near = alert_state(1.0)
    closer = alert_state(0.9)
    airframe_boundary = alert_state(0.75)
    inside_airframe = alert_state(0.5)

    assert near.zone == "escalating"
    assert 3.0 < near.beep_rate_hz < closer.beep_rate_hz
    assert closer.beep_rate_hz < 10.0
    assert airframe_boundary.beep_rate_hz == 10.0
    assert inside_airframe.beep_rate_hz == 10.0


def test_depth_center_pixel_maps_to_forward_sector() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    extractor = DepthObstacleExtractor(settings(), config.depth_camera)
    depth = np.zeros((3, 5), dtype=np.uint16)
    depth[1, 2] = 2000

    scan = extractor.extract(
        depth,
        depth_scale_m=0.001,
        fx=2.0,
        fy=2.0,
        ppx=2.0,
        ppy=1.0,
        monotonic_ns=1,
    )

    assert scan.distances_cm[0] == 219
    assert scan.valid_sector_count == 1


def test_depth_right_pixel_uses_positive_body_yaw() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    extractor = DepthObstacleExtractor(settings(), config.depth_camera)
    depth = np.zeros((3, 5), dtype=np.uint16)
    depth[1, 4] = 2000

    scan = extractor.extract(
        depth,
        depth_scale_m=0.001,
        fx=2.0,
        fy=2.0,
        ppx=2.0,
        ppy=1.0,
        monotonic_ns=1,
    )

    assert scan.distances_cm[8] == 297


def test_depth_clearance_is_measured_from_cg_not_camera() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    extractor = DepthObstacleExtractor(settings(), config.depth_camera)
    depth = np.array([[1310]], dtype=np.uint16)

    scan = extractor.extract(
        depth,
        depth_scale_m=0.001,
        fx=1.0,
        fy=1.0,
        ppx=0.0,
        ppy=0.0,
        monotonic_ns=1,
    )
    clearance = scan.assess_clearance(
        config.obstacle_avoidance.hard_cg_clearance_m
    )

    assert scan.nearest_distance_m == 1.5
    assert clearance.status == "breach"
    assert clearance.margin_m == pytest.approx(0.0)


def test_depth_points_below_body_envelope_are_rejected() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    extractor = DepthObstacleExtractor(settings(), config.depth_camera)
    depth = np.zeros((3, 5), dtype=np.uint16)
    depth[2, 2] = 2000

    scan = extractor.extract(
        depth,
        depth_scale_m=0.001,
        fx=2.0,
        fy=2.0,
        ppx=2.0,
        ppy=1.0,
        monotonic_ns=1,
    )

    assert scan.valid_sector_count == 0


def test_temporal_filter_keeps_new_obstacle_for_full_window() -> None:
    extractor = PointObstacleExtractor(
        settings(temporal_window=3), source="test"
    )
    point = np.array([[2.0, 0.0, 0.0]])
    empty = np.empty((0, 3))

    assert extractor.extract(point, monotonic_ns=1).nearest_distance_m == 2.0
    assert extractor.extract(empty, monotonic_ns=2).nearest_distance_m == 2.0
    assert extractor.extract(empty, monotonic_ns=3).nearest_distance_m == 2.0
    assert extractor.extract(empty, monotonic_ns=4).nearest_distance_m is None


def test_airframe_returns_are_removed_before_clearance_check() -> None:
    extractor = PointObstacleExtractor(settings(), source="test")

    scan = extractor.extract(
        np.array(
            (
                (0.67, 0.0, 0.0),
                (0.76, 0.0, 0.0),
            )
        ),
        monotonic_ns=1,
    )

    assert scan.nearest_distance_m == 0.76


def test_lidar_flu_points_cover_left_and_rear_body_sectors() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    zero_yaw_lidar = replace(
        config.lidar,
        rotation_to_body_frd=replace(
            config.lidar.rotation_to_body_frd,
            yaw_deg=0.0,
        ),
    )
    extractor = LidarObstacleExtractor(settings(), zero_yaw_lidar)

    scan = extractor.extract(
        np.array(((-2.0, 0.0, 0.0), (0.0, -3.0, 0.0))),
        monotonic_ns=1,
    )

    assert scan.distances_cm[54] == 200
    assert scan.distances_cm[36] == 300


def test_lidar_calibrated_yaw_rotates_sensor_points_into_body() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    calibrated_lidar = replace(
        config.lidar,
        rotation_to_body_frd=replace(
            config.lidar.rotation_to_body_frd,
            yaw_deg=180.0,
        ),
    )
    extractor = LidarObstacleExtractor(settings(), calibrated_lidar)

    scan = extractor.extract(
        np.array(((-2.0, 0.0, 0.0), (0.0, -3.0, 0.0))),
        monotonic_ns=1,
    )

    assert scan.distances_cm[18] == 200
    assert scan.distances_cm[0] == 300


def test_fusion_uses_nearest_fresh_source_and_drops_stale_data() -> None:
    configured = settings(source_stale_timeout_s=0.25)
    fusion = ObstacleFusion(configured)
    unknown = [UNKNOWN_DISTANCE_CM] * configured.sector_count
    depth = list(unknown)
    lidar = list(unknown)
    depth[0] = 300
    lidar[0] = 200
    for source, values in (("depth", depth), ("lidar", lidar)):
        fusion.update(
            ObstacleScan(
                source=source,
                monotonic_ns=1_000_000_000,
                distances_cm=tuple(values),
                increment_deg=configured.sector_increment_deg,
                min_distance_cm=30,
                max_distance_cm=800,
            )
        )

    fused = fusion.fused(monotonic_ns=1_100_000_000)

    assert fused is not None
    assert fused.distances_cm[0] == 200
    assert fused.source == "depth+lidar"
    assert fusion.fused(monotonic_ns=1_300_000_001) is None


def test_clearance_assessment_treats_boundary_as_breach() -> None:
    configured = settings()
    distances = [UNKNOWN_DISTANCE_CM] * configured.sector_count
    distances[0] = 200
    distances[71] = 201
    scan = ObstacleScan(
        source="test",
        monotonic_ns=1,
        distances_cm=tuple(distances),
        increment_deg=configured.sector_increment_deg,
        min_distance_cm=30,
        max_distance_cm=800,
    )

    assessment = scan.assess_clearance(2.0)

    assert assessment.status == "breach"
    assert assessment.breached
    assert assessment.margin_m == 0.0
    assert assessment.violating_sector_indices == (0,)
    assert assessment.violating_sector_angles_deg == (0.0,)


def test_clearance_assessment_is_unknown_without_known_sectors() -> None:
    configured = settings()
    scan = ObstacleScan(
        source="test",
        monotonic_ns=1,
        distances_cm=tuple(
            [UNKNOWN_DISTANCE_CM] * configured.sector_count
        ),
        increment_deg=configured.sector_increment_deg,
        min_distance_cm=30,
        max_distance_cm=800,
    )

    assessment = scan.assess_clearance(2.0)

    assert assessment.status == "unknown"
    assert not assessment.breached
    assert assessment.nearest_distance_m is None
    assert assessment.margin_m is None


def test_bench_summary_reports_right_and_left_sector_angles() -> None:
    configured = settings()
    distances = [UNKNOWN_DISTANCE_CM] * configured.sector_count
    distances[1] = 100
    distances[-1] = 200
    scan = ObstacleScan(
        source="test",
        monotonic_ns=1,
        distances_cm=tuple(distances),
        increment_deg=configured.sector_increment_deg,
        min_distance_cm=30,
        max_distance_cm=800,
    )

    summary = summarize_scan(scan)

    assert summary["known_sectors"] == [
        {"angle_deg": 5.0, "distance_m": 1.0},
        {"angle_deg": -5.0, "distance_m": 2.0},
    ]


def test_bench_summary_includes_requested_cg_clearance() -> None:
    configured = settings()
    distances = [UNKNOWN_DISTANCE_CM] * configured.sector_count
    distances[0] = 175
    scan = ObstacleScan(
        source="test",
        monotonic_ns=1,
        distances_cm=tuple(distances),
        increment_deg=configured.sector_increment_deg,
        min_distance_cm=30,
        max_distance_cm=800,
    )

    summary = summarize_scan(scan, hard_cg_clearance_m=2.0)

    assert summary["clearance"]["reference"] == "aircraft_cg"
    assert summary["clearance"]["status"] == "breach"
    assert summary["clearance"]["margin_m"] == -0.25


def test_target_check_uses_cg_distance_and_neighboring_sector() -> None:
    configured = settings()
    distances = [UNKNOWN_DISTANCE_CM] * configured.sector_count
    distances[1] = 204
    scan = ObstacleScan(
        source="test",
        monotonic_ns=1,
        distances_cm=tuple(distances),
        increment_deg=configured.sector_increment_deg,
        min_distance_cm=30,
        max_distance_cm=800,
    )

    check = evaluate_target(
        scan,
        expected_distance_m=2.0,
        angle_deg=0.0,
        tolerance_m=0.05,
    )

    assert check["passed"]
    assert check["reference"] == "aircraft_cg"
    assert check["measured_distance_m"] == 2.04
    assert check["error_m"] == pytest.approx(0.04)
    assert check["measured_sector_angle_deg"] == 5.0


def test_target_check_fails_when_target_is_not_detected() -> None:
    configured = settings()
    scan = ObstacleScan(
        source="test",
        monotonic_ns=1,
        distances_cm=tuple(
            [UNKNOWN_DISTANCE_CM] * configured.sector_count
        ),
        increment_deg=configured.sector_increment_deg,
        min_distance_cm=30,
        max_distance_cm=800,
    )

    check = evaluate_target(
        scan,
        expected_distance_m=2.0,
        angle_deg=90.0,
        tolerance_m=0.08,
    )

    assert not check["passed"]
    assert check["status"] == "no_detection"


def test_bench_session_accepts_sensor_timing_samples(tmp_path) -> None:
    session = _BenchSession(tmp_path)

    session.record_sensor_timing({"source": "jt16_frame"})

    assert session.sensor_timing_sample_count == 1
