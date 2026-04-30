import time
from pathlib import Path
from types import SimpleNamespace

from slam_core.bridge_config import load_bridge_config
from slam_core.fc_config import FlightControllerTelemetry
from slam_core.slam_observer import SlamLoiterObserver


class FakeMav:
    def __init__(self):
        self.status_texts = []

    def statustext_send(self, severity, text):
        self.status_texts.append((severity, text))


class FakeMaster:
    def __init__(self):
        self.mav = FakeMav()


def test_autostart_config_enables_guarded_slam_observer():
    config_path = Path(__file__).resolve().parents[1] / "config" / "autostart.yaml"
    config = load_bridge_config(config_path)

    assert config.slam_observer.enable_loiter_observation is True
    assert config.slam_observer.observation_message_interval_sec == 20
    assert config.slam_observer.enable_live_soft_correction is True
    assert config.slam_observer.enable_auto_fallback_to_loiter is True
    assert config.slam_observer.log_observation_data is True


def test_loiter_observer_scores_without_commanding_motion():
    config = load_bridge_config("config/autostart.yaml")
    config.slam_observer.log_observation_data = False
    config.slam_observer.status_path = ""
    observer = SlamLoiterObserver(config.slam_observer)
    master = FakeMaster()
    fc_state = FlightControllerTelemetry(
        local_position=SimpleNamespace(x=0.0, y=0.0, z=-3.0, vx=0.0, vy=0.0, vz=0.0),
        attitude=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
        ekf_flags=831,
        flight_mode="LOITER",
        armed=True,
        last_heartbeat_s=time.time(),
        gps_fix_type=3,
        gps_satellites=12,
    )
    pose = SimpleNamespace(
        timestamp_us=1000000,
        x_m=0.0,
        y_m=0.0,
        z_m=-3.0,
        vx_m_s=0.0,
        vy_m_s=0.0,
        vz_m_s=0.0,
        qw=1.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        pose_quality=70,
        tracking_state="ok_hold",
        feature_count=120,
        tracked_feature_count=100,
        inlier_count=80,
    )

    summary = observer.update(master, fc_state, pose, imu_sample=None, imu_expected=False)

    assert summary["active"] is True
    assert summary["score"] > 0
    assert master.mav.status_texts
    assert any(b"LOITER active" in text for _, text in master.mav.status_texts)


def test_live_soft_correction_is_bounded_and_applied_after_good_loiter_score():
    config = load_bridge_config("config/autostart.yaml")
    config.slam_observer.log_observation_data = False
    config.slam_observer.status_path = ""
    observer = SlamLoiterObserver(config.slam_observer)
    master = FakeMaster()
    fc_state = FlightControllerTelemetry(
        local_position=SimpleNamespace(x=1.0, y=0.0, z=-3.0, vx=0.0, vy=0.0, vz=0.0),
        attitude=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.1),
        ekf_flags=831,
        flight_mode="LOITER",
        armed=True,
        last_heartbeat_s=time.time(),
        gps_fix_type=3,
        gps_satellites=12,
    )
    pose = SimpleNamespace(
        timestamp_us=1000000,
        x_m=1.0,
        y_m=0.0,
        z_m=-3.0,
        vx_m_s=0.0,
        vy_m_s=0.0,
        vz_m_s=0.0,
        qw=1.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        pose_quality=90,
        tracking_state="ok_hold",
        feature_count=120,
        tracked_feature_count=100,
        inlier_count=80,
        source_name="test",
    )

    observer.update(master, fc_state, pose, imu_sample=None, imu_expected=False)
    observer.state.started_s = time.time() - 70.0
    summary = observer.update(master, fc_state, pose, imu_sample=None, imu_expected=False)
    corrected = observer.apply_soft_correction(pose)

    assert summary["score"] >= config.slam_observer.min_quality_for_poshold
    assert summary["correction"]["valid"] is True
    assert corrected.source_name.endswith("+soft")
    assert abs(corrected.x_m - pose.x_m) < 0.5
