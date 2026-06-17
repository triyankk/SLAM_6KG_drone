import time
from types import SimpleNamespace

from slam_core.fc_config import FlightControllerTelemetry
from slam_core.gps_denied_readiness import GpsDeniedReadinessConfig, GpsDeniedReadinessTracker
from slam_core.types import PoseSample


def _healthy_pose(timestamp_us: int = 1_000_000) -> PoseSample:
    return PoseSample(
        timestamp_us=timestamp_us,
        x_m=0.0,
        y_m=0.0,
        z_m=-2.0,
        qw=1.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        pose_quality=90,
        tracking_state="ok_slam",
        source_name="test",
    )


def _healthy_fc_state() -> FlightControllerTelemetry:
    now_s = time.time()
    return FlightControllerTelemetry(
        local_position=SimpleNamespace(x=0.0, y=0.0, z=-2.0, vx=0.0, vy=0.0, vz=0.0),
        attitude=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
        ekf_flags=831,
        flight_mode="POSHOLD",
        armed=True,
        last_heartbeat_s=now_s,
        gps_fix_type=3,
        gps_satellites=12,
        gps2_fix_type=3,
        gps2_satellites=12,
        rc_channel_count=8,
        rc_last_update_s=now_s,
        rc_channels={1: 1500, 2: 1500, 3: 1500, 4: 1500, 7: 1800},
        rangefinder_distance_m=2.0,
        rangefinder_last_update_s=now_s,
    )


def _ready_observer() -> dict:
    return {"recommendation": "ready_for_no_gps_poshold", "score": 8.2}


def test_gps_denied_gate_reports_ready_when_all_inputs_are_coherent():
    tracker = GpsDeniedReadinessTracker(GpsDeniedReadinessConfig(stable_seconds=0.0, status_path=""))

    report = tracker.update(
        _healthy_pose(),
        imu_sample=object(),
        fc_state=_healthy_fc_state(),
        gps_input_enabled=True,
        gps_input_fixed_fix=False,
        gps_input_origin_valid=True,
        target_mode="POSHOLD",
        calibration_profile_valid=False,
        observer_summary=_ready_observer(),
        using_gps_input_bridge=True,
        slam_pose_gps2_recent=True,
    )

    assert report.ready is True
    assert report.active is True
    assert report.blockers == []
    assert report.score >= 90


def test_gps_denied_gate_blocks_without_rangefinder_origin_and_observer():
    tracker = GpsDeniedReadinessTracker(GpsDeniedReadinessConfig(stable_seconds=0.0, status_path=""))
    fc_state = _healthy_fc_state()
    fc_state.rangefinder_distance_m = None
    fc_state.rangefinder_last_update_s = 0.0
    fc_state.gps_fix_type = 1
    fc_state.gps_satellites = 0

    report = tracker.update(
        _healthy_pose(),
        imu_sample=object(),
        fc_state=fc_state,
        gps_input_enabled=True,
        gps_input_fixed_fix=False,
        gps_input_origin_valid=False,
        target_mode="POSHOLD",
        calibration_profile_valid=False,
        observer_summary={"recommendation": "weak", "score": 4.0},
        using_gps_input_bridge=True,
    )

    assert report.ready is False
    assert any("rangefinder" in reason for reason in report.blockers)
    assert any("GPS2 origin" in reason for reason in report.blockers)
    assert any("Brake calibration" in reason for reason in report.blockers)


def test_gps_denied_gate_blocks_pose_height_jump_against_rangefinder():
    tracker = GpsDeniedReadinessTracker(GpsDeniedReadinessConfig(stable_seconds=0.0, status_path=""))
    pose = _healthy_pose()
    pose.z_m = -4.0

    report = tracker.update(
        pose,
        imu_sample=object(),
        fc_state=_healthy_fc_state(),
        gps_input_enabled=True,
        gps_input_fixed_fix=False,
        gps_input_origin_valid=True,
        target_mode="POSHOLD",
        calibration_profile_valid=True,
        observer_summary=None,
        using_gps_input_bridge=True,
    )

    assert report.ready is False
    assert any("range/SLAM height mismatch" in reason for reason in report.blockers)
