import json
from pathlib import Path
import struct

import numpy as np

from optflow_slam.config import load_config
from optflow_slam.flight_analysis import analyze_session
from optflow_slam.flight_logger import (
    FlightSession,
    HesaiLidarRecorder,
    IdealHoldShadow,
    RawIpPcapWriter,
)
from optflow_slam.paths import CONFIG_DIR
from optflow_slam.pointcloud import (
    MapPose,
    VoxelMap,
    camera_optical_to_local,
    write_binary_ply,
)
from optflow_slam.slam_timing import summarize_timestamps


def telemetry_snapshot(
    *,
    armed: bool = True,
    mode: str = "POSHOLD",
    local_x: float = 0.0,
    local_vx: float = 0.0,
) -> dict:
    return {
        "vehicle": {"armed": armed, "mode": mode},
        "flow": {
            "comp_x_mps": local_vx,
            "comp_y_mps": 0.0,
            "quality": 150,
            "age_ms": 5,
        },
        "range": {"distance_m": 1.2, "age_ms": 5},
        "attitude": {
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
            "time_boot_ms": 1000,
        },
        "local_position": {
            "x_m": local_x,
            "y_m": 0.0,
            "z_down_m": -1.0,
            "vx_mps": local_vx,
            "vy_mps": 0.0,
            "vz_mps": 0.0,
            "age_ms": 5,
        },
        "imu": {
            "gyro_x_rads": 0.1,
            "gyro_y_rads": 0.2,
            "gyro_z_rads": 0.3,
            "accel_x_mss": 0.0,
            "accel_y_mss": 0.0,
            "accel_z_mss": -9.8,
            "age_ms": 5,
        },
        "ros_imu": {
            "body_preview": {
                "gyro_x_rads": 0.1,
                "gyro_y_rads": 0.2,
                "gyro_z_rads": 0.3,
                "accel_x_mss": 0.0,
                "accel_y_mss": 0.0,
                "accel_z_mss": -9.8,
            },
            "age_ms": 5,
        },
        "power": {
            "voltage_v": 24.0,
            "current_a": 10.0,
            "remaining_pct": 70,
        },
        "vibration": {
            "x_mss": 2.0,
            "y_mss": 3.0,
            "z_mss": 4.0,
            "clipping_0": 0,
            "clipping_1": 0,
            "clipping_2": 0,
        },
    }


def test_shadow_model_predicts_return_tilt_without_control_output() -> None:
    model = IdealHoldShadow(min_flow_quality=1)
    model.update(telemetry_snapshot(local_x=0.0), 1_000_000_000)

    prediction = model.update(
        telemetry_snapshot(local_x=1.0, local_vx=0.2),
        1_050_000_000,
    )

    hold = prediction["perfect_slam_stationary_hold"]
    assert hold["prediction_applicable"]
    assert hold["predicted_pitch_rad"] > 0.0
    assert hold["not_for_flight_control"]
    assert model.latest_pose is not None
    assert model.latest_pose.source == "cube_local_position"


def test_shadow_model_uses_fresh_guided_position_target() -> None:
    model = IdealHoldShadow(min_flow_quality=1)
    snapshot = telemetry_snapshot(mode="GUIDED", local_x=0.0)
    snapshot["position_target"] = {
        "x_m": 1.0,
        "y_m": 0.0,
        "vx_mps": 0.0,
        "vy_mps": 0.0,
        "type_mask": 0,
        "age_ms": 5,
    }

    prediction = model.update(snapshot, 1_000_000_000)
    hold = prediction["perfect_slam_stationary_hold"]

    assert hold["prediction_applicable"]
    assert hold["reference_source"] == "cube_position_target"
    assert hold["desired_x_m"] == 1.0
    assert hold["predicted_pitch_rad"] < 0.0


def test_forward_camera_optical_axis_maps_to_body_forward() -> None:
    pose = MapPose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, "test")
    camera_point = np.array([[0.0, 0.0, 2.0]], dtype=np.float32)

    local = camera_optical_to_local(camera_point, pose)

    assert np.allclose(local, [[3.0, 2.0, 3.0]])


def test_voxel_map_merges_nearby_points_and_writes_ply(
    tmp_path: Path,
) -> None:
    voxel_map = VoxelMap(voxel_size_m=0.1)
    points = np.array([[0.01, 0.01, 0.01], [0.04, 0.02, 0.03]])
    colors = np.array([[100, 120, 140], [200, 220, 240]], dtype=np.uint8)
    voxel_map.add(points, colors)

    output = tmp_path / "map.ply"
    voxel_map.write(output)

    merged, merged_colors = voxel_map.cloud()
    assert len(voxel_map) == 1
    assert np.allclose(merged[0], np.mean(points, axis=0))
    assert np.allclose(merged_colors[0], [150, 170, 190])
    assert b"element vertex 1" in output.read_bytes()[:300]


def test_binary_ply_rejects_mismatched_colors(tmp_path: Path) -> None:
    points = np.zeros((2, 3), dtype=np.float32)

    try:
        write_binary_ply(tmp_path / "bad.ply", points, np.zeros((1, 3)))
    except ValueError as exc:
        assert "colors_rgb" in str(exc)
    else:
        raise AssertionError("mismatched colors should fail")


def test_raw_ip_pcap_contains_udp_packet(tmp_path: Path) -> None:
    output = tmp_path / "lidar.pcap"
    writer = RawIpPcapWriter(output, "192.168.1.100", 2368)
    writer.write(b"jt16", "192.168.1.201", 5000, 1_700_000_000_123_000_000)
    writer.close()

    data = output.read_bytes()
    magic, major, minor, _, _, _, link_type = struct.unpack(
        "<IHHIIII", data[:24]
    )
    assert magic == 0xA1B2C3D4
    assert (major, minor) == (2, 4)
    assert link_type == 101
    _, _, included, original = struct.unpack("<IIII", data[24:40])
    assert included == original == 20 + 8 + len(b"jt16")
    assert data.endswith(b"jt16")


def test_jt16_bridge_protocol_preserves_slam_point_fields() -> None:
    point_format = HesaiLidarRecorder.POINT_DTYPE

    assert point_format.itemsize == 24
    assert point_format.names == (
        "x",
        "y",
        "z",
        "timestamp",
        "ring",
        "intensity",
        "confidence",
    )


def test_timestamp_summary_reports_rate_jitter_and_missing_samples() -> None:
    timestamps_ns = [
        0,
        100_000_000,
        200_000_000,
        400_000_000,
        500_000_000,
    ]

    summary = summarize_timestamps(
        timestamps_ns,
        units_per_second=1.0e9,
        expected_rate_hz=10.0,
    )

    assert summary["samples"] == 5
    assert summary["observed_rate_hz"] == 8.0
    assert summary["estimated_drops"] == 1
    assert summary["period_ms"]["max"] == 200.0
    assert summary["jitter_rms_ms"] == 50.0


def test_session_and_analysis_create_a_reusable_flight_folder(
    tmp_path: Path,
) -> None:
    config_path = CONFIG_DIR / "system.yaml"
    config = load_config(config_path)
    session = FlightSession(
        tmp_path,
        "bench observation",
        config,
        config_path,
        "http://127.0.0.1:8765/api/stream",
        "http://127.0.0.1:8765/api/events",
    )
    for index in range(12):
        snapshot = telemetry_snapshot(
            local_x=index * 0.01,
            local_vx=0.01,
        )
        snapshot["attitude"]["time_boot_ms"] = 1000 + index * 40
        session.record_snapshot(snapshot, 1_000_000_000 + index * 40_000_000)
    session.record_sensor_event(
        {
            "sequence": 1,
            "host_monotonic_ns": 1_000_000_000,
            "source": "external_imu",
            "type": "gyro_rads",
            "data": {"values": [0.0, 0.0, 0.0]},
        }
    )
    session.record_sensor_event(
        {
            "sequence": 2,
            "host_monotonic_ns": 1_100_000_000,
            "source": "external_imu",
            "type": "accel_mss",
            "data": {"values": [0.0, 0.0, 9.8]},
        }
    )
    session.close()

    report_path = analyze_session(session.path)
    report = json.loads(report_path.read_text())
    manifest = json.loads((session.path / "manifest.json").read_text())

    assert manifest["status"] == "complete"
    assert manifest["passive_only"]
    assert report["coverage"]["telemetry_rows"] == 12
    assert report["raw_sensor_events"]["rows"] == 2
    assert report["raw_sensor_events"]["counts"]["external_imu"] == {
        "accel_mss": 1,
        "gyro_rads": 1,
    }
    assert report["stationary_hold_shadow"]["applicable_samples"] == 12
    assert (session.path / "analysis" / "report.md").exists()
    assert (session.path / "analysis" / "timeline.csv").exists()


def test_analysis_builds_a_passive_slam_timing_gate(tmp_path: Path) -> None:
    config_path = CONFIG_DIR / "system.yaml"
    session = FlightSession(
        tmp_path,
        "slam timing",
        load_config(config_path),
        config_path,
        "direct://cube-uart",
        "direct://sensor-event-bus",
    )
    for index in range(5):
        host_ns = 1_000_000_000 + index * 100_000_000
        session.record_sensor_event(
            {
                "sequence": index + 1,
                "host_monotonic_ns": host_ns,
                "host_unix_ns": 2_000_000_000 + host_ns,
                "source": "external_imu",
                "type": "gyro_rads",
                "data": {"values": [0.0, 0.0, 0.0]},
            }
        )
        session.record_sensor_timing(
            {
                "source": "realsense_frameset",
                "host_monotonic_ns": host_ns,
                "host_unix_ns": 2_000_000_000 + host_ns,
                "depth_frame_number": index,
                "depth_sensor_timestamp_ms": index * 100.0,
                "depth_timestamp_domain": "hardware_clock",
                "color_frame_number": index,
                "color_sensor_timestamp_ms": index * 100.0,
                "color_timestamp_domain": "hardware_clock",
            }
        )
        session.record_sensor_timing(
            {
                "source": "jt16_frame",
                "host_receive_monotonic_ns": host_ns,
                "host_receive_unix_ns": 2_000_000_000 + host_ns,
                "bridge_callback_monotonic_ns": host_ns - 1_000_000,
                "frame_index": index,
                "point_count": 16,
                "point_timestamp_min_s": index * 0.1,
                "point_timestamp_max_s": index * 0.1 + 0.05,
                "point_timestamp_span_s": 0.05,
            }
        )
    session.record_sensor_timing(
        {
            "source": "realsense_frameset",
            "host_monotonic_ns": 1_450_000_000,
            "host_unix_ns": 3_450_000_000,
            "depth_frame_number": 4,
            "depth_sensor_timestamp_ms": 400.0,
            "depth_timestamp_domain": "hardware_clock",
            "color_frame_number": 4,
            "color_sensor_timestamp_ms": 400.0,
            "color_timestamp_domain": "hardware_clock",
        }
    )
    session.close()

    report_path = analyze_session(session.path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    manifest = json.loads(
        (session.path / "manifest.json").read_text(encoding="utf-8")
    )
    timing = report["slam_timing"]

    assert manifest["rows"]["sensor_timing"] == 16
    assert timing["d415"]["framesets"] == 6
    assert timing["d415"]["unique_depth_frames"] == 5
    assert timing["d415"]["repeated_depth_frames"] == 1
    assert timing["jt16"]["frames"] == 5
    assert timing["external_imu"]["selected_sample_type"] == "gyro_rads"
    assert timing["external_imu"]["host_arrival"]["observed_rate_hz"] == 10.0
    assert not timing["gates"]["ready_for_lidar_inertial_replay"]
    assert "IM10A frames do not yet carry sensor time." in timing["blockers"]
    assert (session.path / "analysis" / "slam_timing.json").exists()


def test_session_lifecycle_event_is_immediately_durable(
    tmp_path: Path,
) -> None:
    config_path = CONFIG_DIR / "system.yaml"
    session = FlightSession(
        tmp_path,
        "durability",
        load_config(config_path),
        config_path,
        "direct://cube-uart",
        "direct://sensor-event-bus",
    )

    events = (session.path / "events.ndjson").read_text().splitlines()

    assert len(events) == 1
    assert json.loads(events[0])["event"] == "recording_started"
    session.close(status="interrupted", reason="test_complete")
