import json
import os
from datetime import datetime, timezone
from pathlib import Path

from optflow_slam.config import load_config
from optflow_slam.flight_service import (
    ArmTriggeredRecorder,
    ServiceSettings,
    latest_service_session,
    recover_stale_service_sessions,
)
from optflow_slam.paths import CONFIG_DIR


def telemetry_snapshot(*, armed: bool) -> dict:
    return {
        "vehicle": {"armed": armed, "mode": "FLOWHOLD"},
        "flow": {
            "comp_x_mps": 0.0,
            "comp_y_mps": 0.0,
            "quality": 150,
            "age_ms": 5,
        },
        "range": {"distance_m": 1.0, "age_ms": 5},
        "attitude": {
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
            "time_boot_ms": 1000,
        },
        "local_position": {
            "x_m": 0.0,
            "y_m": 0.0,
            "z_down_m": -1.0,
            "vx_mps": 0.0,
            "vy_mps": 0.0,
            "vz_mps": 0.0,
            "age_ms": 5,
        },
        "imu": {
            "gyro_x_rads": 0.0,
            "gyro_y_rads": 0.0,
            "gyro_z_rads": 0.0,
            "accel_x_mss": 0.0,
            "accel_y_mss": 0.0,
            "accel_z_mss": -9.8,
            "age_ms": 5,
        },
        "ros_imu": {
            "body_preview": {
                "gyro_x_rads": 0.0,
                "gyro_y_rads": 0.0,
                "gyro_z_rads": 0.0,
                "accel_x_mss": 0.0,
                "accel_y_mss": 0.0,
                "accel_z_mss": -9.8,
            },
            "age_ms": 5,
        },
        "power": {
            "voltage_v": 24.0,
            "current_a": 0.0,
            "remaining_pct": 80,
        },
        "vibration": {
            "x_mss": 1.0,
            "y_mss": 1.0,
            "z_mss": 1.0,
            "clipping_0": 0,
            "clipping_1": 0,
            "clipping_2": 0,
        },
    }


def raw_event(sequence: int, timestamp_ns: int) -> dict:
    return {
        "sequence": sequence,
        "host_monotonic_ns": timestamp_ns,
        "host_unix_ns": timestamp_ns,
        "source": "cube_mavlink",
        "type": "HEARTBEAT",
        "data": {},
    }


def make_recorder(
    tmp_path: Path,
    *,
    post_disarm_s: float = 0.1,
    disk_free_gb=None,
) -> ArmTriggeredRecorder:
    config_path = CONFIG_DIR / "system.yaml"
    return ArmTriggeredRecorder(
        load_config(config_path),
        config_path,
        tmp_path,
        ServiceSettings(
            sample_rate_hz=30.0,
            pre_roll_s=1.0,
            post_disarm_s=post_disarm_s,
            min_free_gb=5.0,
            depth_enabled=False,
            lidar_enabled=False,
        ),
        disk_free_gb=disk_free_gb,
    )


def process(
    recorder: ArmTriggeredRecorder,
    *,
    armed: bool,
    timestamp_ns: int,
    sequence: int,
) -> None:
    recorder.process(
        telemetry_snapshot(armed=armed),
        timestamp_ns,
        f"2026-07-29T00:00:00.{sequence:03d}+00:00",
        [raw_event(sequence, timestamp_ns)],
    )


def test_arm_starts_session_and_disarm_finalizes_after_tail(
    tmp_path: Path,
) -> None:
    recorder = make_recorder(tmp_path)
    process(recorder, armed=False, timestamp_ns=1_000_000_000, sequence=1)
    process(recorder, armed=False, timestamp_ns=1_050_000_000, sequence=2)

    process(recorder, armed=True, timestamp_ns=1_100_000_000, sequence=3)
    assert recorder.state == "recording"
    session_path = recorder.current_session.path

    process(recorder, armed=False, timestamp_ns=1_150_000_000, sequence=4)
    assert recorder.state == "post_disarm_tail"
    process(recorder, armed=False, timestamp_ns=1_260_000_000, sequence=5)

    assert recorder.state == "waiting_for_arm"
    assert recorder.last_session == session_path
    manifest = json.loads((session_path / "manifest.json").read_text())
    telemetry = [
        json.loads(line)
        for line in (session_path / "telemetry.ndjson").read_text().splitlines()
    ]
    assert manifest["status"] == "complete"
    assert manifest["stop_reason"] == "post_disarm_tail_complete"
    assert manifest["rows"]["telemetry"] == 5
    assert telemetry[0]["host_time_utc"].endswith(".001+00:00")
    assert recorder.last_report == session_path / "analysis" / "report.json"


def test_rearm_during_tail_keeps_one_session(tmp_path: Path) -> None:
    recorder = make_recorder(tmp_path, post_disarm_s=0.2)
    process(recorder, armed=True, timestamp_ns=1_000_000_000, sequence=1)
    session_path = recorder.current_session.path
    process(recorder, armed=False, timestamp_ns=1_050_000_000, sequence=2)
    process(recorder, armed=True, timestamp_ns=1_100_000_000, sequence=3)

    assert recorder.state == "recording"
    assert recorder.current_session.path == session_path

    process(recorder, armed=False, timestamp_ns=1_200_000_000, sequence=4)
    process(recorder, armed=False, timestamp_ns=1_410_000_000, sequence=5)
    assert recorder.last_session == session_path
    assert len(list(tmp_path.iterdir())) == 1


def test_low_disk_inhibits_until_disarm(tmp_path: Path) -> None:
    free_gb = [4.0]
    recorder = make_recorder(
        tmp_path, disk_free_gb=lambda: free_gb[0]
    )

    process(recorder, armed=True, timestamp_ns=1_000_000_000, sequence=1)
    assert recorder.state == "inhibited_until_disarm"
    assert recorder.current_session is None
    assert not list(tmp_path.iterdir())

    free_gb[0] = 10.0
    process(recorder, armed=True, timestamp_ns=1_100_000_000, sequence=2)
    assert recorder.state == "inhibited_until_disarm"

    process(recorder, armed=False, timestamp_ns=1_200_000_000, sequence=3)
    process(recorder, armed=True, timestamp_ns=1_300_000_000, sequence=4)
    assert recorder.state == "recording"
    recorder.shutdown()

    manifest = json.loads(
        (recorder.last_session / "manifest.json").read_text()
    )
    assert manifest["status"] == "interrupted"
    assert manifest["stop_reason"] == "logger_service_stopped"


def test_low_disk_during_recording_finalizes_once(tmp_path: Path) -> None:
    free_gb = [10.0]
    recorder = make_recorder(
        tmp_path, disk_free_gb=lambda: free_gb[0]
    )
    process(recorder, armed=True, timestamp_ns=1_000_000_000, sequence=1)
    session_path = recorder.current_session.path

    free_gb[0] = 4.0
    process(recorder, armed=True, timestamp_ns=2_100_000_000, sequence=2)
    assert recorder.state == "inhibited_until_disarm"
    assert recorder.current_session is None
    assert recorder.last_session == session_path

    process(recorder, armed=True, timestamp_ns=3_200_000_000, sequence=3)
    assert len(list(tmp_path.iterdir())) == 1
    manifest = json.loads((session_path / "manifest.json").read_text())
    assert manifest["status"] == "interrupted"
    assert manifest["stop_reason"] == "minimum_free_space_reached"


def test_startup_recovers_stale_service_manifest(tmp_path: Path) -> None:
    session_path = tmp_path / "20260730T000000Z_armed"
    session_path.mkdir()
    manifest_path = session_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "status": "recording",
                "ended_utc": None,
                "telemetry_url": "direct://cube-uart",
            }
        ),
        encoding="utf-8",
    )
    telemetry_path = session_path / "telemetry.ndjson"
    telemetry_path.write_text(
        '{"sample":1}\n', encoding="utf-8"
    )
    expected_end = datetime.fromtimestamp(
        telemetry_path.stat().st_mtime, timezone.utc
    ).isoformat(timespec="milliseconds")
    analysis_path = session_path / "analysis"
    analysis_path.mkdir()
    later_report = analysis_path / "report.json"
    later_report.write_text("{}", encoding="utf-8")
    os.utime(later_report, (2_000_000_000, 2_000_000_000))

    recovered = recover_stale_service_sessions(tmp_path)

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert recovered == [session_path]
    assert manifest["status"] == "interrupted"
    assert manifest["stop_reason"] == "recovered_after_unclean_shutdown"
    assert manifest["ended_utc"] == expected_end
    assert manifest["recovery"]["files_preserved"]


def test_startup_leaves_manual_recording_manifest_alone(
    tmp_path: Path,
) -> None:
    session_path = tmp_path / "manual"
    session_path.mkdir()
    manifest_path = session_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "status": "recording",
                "telemetry_url": "http://127.0.0.1:8765/api/stream",
            }
        ),
        encoding="utf-8",
    )

    assert recover_stale_service_sessions(tmp_path) == []
    assert json.loads(manifest_path.read_text())["status"] == "recording"


def test_latest_service_session_ignores_manual_session(
    tmp_path: Path,
) -> None:
    service_path = tmp_path / "service"
    manual_path = tmp_path / "manual"
    service_path.mkdir()
    manual_path.mkdir()
    (service_path / "manifest.json").write_text(
        json.dumps(
            {
                "started_utc": "2026-07-30T01:00:00+00:00",
                "telemetry_url": "direct://cube-uart",
            }
        ),
        encoding="utf-8",
    )
    (manual_path / "manifest.json").write_text(
        json.dumps(
            {
                "started_utc": "2026-07-30T02:00:00+00:00",
                "telemetry_url": "http://127.0.0.1:8765/api/stream",
            }
        ),
        encoding="utf-8",
    )

    latest = latest_service_session(tmp_path)

    assert latest is not None
    assert latest[0] == service_path
