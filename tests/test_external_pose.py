import json
import socket
import time

from slam_core.external_pose import ExternalPoseUdpSource, pose_sample_from_json


def test_pose_sample_from_json_accepts_compact_slam_packet():
    sample = pose_sample_from_json(
        {
            "timestamp_us": 123456,
            "position_m": [1.0, 2.0, -1.5],
            "velocity_m_s": [0.1, 0.2, 0.0],
            "yaw_deg": 90.0,
            "quality": 88,
            "tracking": "ok_slam",
            "source": "cartographer",
        }
    )

    assert sample.timestamp_us == 123456
    assert sample.x_m == 1.0
    assert sample.y_m == 2.0
    assert sample.z_m == -1.5
    assert sample.vx_m_s == 0.1
    assert sample.pose_quality == 88
    assert sample.tracking_state == "ok_slam"
    assert sample.source_name == "cartographer"


def test_external_pose_udp_source_returns_latest_packet():
    source = ExternalPoseUdpSource(bind_host="127.0.0.1", bind_port=0, first_sample_timeout_s=0.5)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(
            json.dumps(
                {
                    "x_m": 0.5,
                    "y_m": -0.2,
                    "z_m": -2.0,
                    "yaw_rad": 0.0,
                    "pose_quality": 95,
                    "tracking_state": "ok",
                }
            ).encode("utf-8"),
            ("127.0.0.1", source.bind_port),
        )

        sample = source.sample()

        assert sample.x_m == 0.5
        assert sample.y_m == -0.2
        assert sample.z_m == -2.0
        assert sample.pose_quality == 95
        assert sample.tracking_state == "ok"
    finally:
        sender.close()
        source.close()


def test_external_pose_udp_source_marks_stale_samples_unhealthy():
    source = ExternalPoseUdpSource(
        bind_host="127.0.0.1",
        bind_port=0,
        max_age_s=0.01,
        first_sample_timeout_s=0.5,
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(
            json.dumps(
                {
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "z_m": -1.0,
                    "pose_quality": 90,
                    "tracking_state": "ok",
                }
            ).encode("utf-8"),
            ("127.0.0.1", source.bind_port),
        )

        assert source.sample().tracking_state == "ok"
        time.sleep(0.03)
        stale = source.sample()

        assert stale.pose_quality == 0
        assert stale.tracking_state.startswith("stale_")
    finally:
        sender.close()
        source.close()
