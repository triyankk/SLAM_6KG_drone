from __future__ import annotations

import json
from pathlib import Path

import pytest

from optflow_slam.config import load_config
from optflow_slam.rtl_shadow import (
    LocalReturnShadow,
    ReturnSettings,
    replay_session,
)


def test_shadow_return_generates_bounded_commands_and_arrives() -> None:
    supervisor = LocalReturnShadow()
    assert supervisor.capture_launch(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    for index, x_m in enumerate((0.0, 0.1, 0.2, 0.3, 0.4), start=1):
        timestamp_ns = index * 200_000_000
        supervisor.update_visual(
            timestamp_ns, [x_m, 0.0, 0.0], tracking=True
        )
        supervisor.observe_outbound(timestamp_ns, [x_m, 0.0, 0.0])
    assert supervisor.begin_return(1_000_000_000)

    for index, x_m in enumerate((0.35, 0.25, 0.15, 0.10), start=6):
        timestamp_ns = index * 200_000_000
        supervisor.update_visual(
            timestamp_ns, [x_m, 0.0, 0.0], tracking=True
        )
        supervisor.observe_return(timestamp_ns, [x_m, 0.0, 0.0])

    supervisor.finish()
    report = supervisor.report()

    assert report["result"] == "shadow_pass"
    assert report["state"] == "arrived"
    assert report["control_eligible"] is False
    assert report["pose_sent_to_cube"] is False
    assert report["velocity_sent_to_cube"] is False
    assert report["metrics"]["maximum_proposed_speed_mps"] <= 0.5
    assert report["metrics"]["maximum_proposed_acceleration_mpss"] <= (
        1.0 + 1.0e-6
    )
    assert all(report["gates"].values())


def test_shadow_return_stops_on_fresh_visual_disagreement() -> None:
    supervisor = LocalReturnShadow(
        ReturnSettings(visual_disagreement_limit_m=0.20)
    )
    supervisor.capture_launch(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    supervisor.observe_outbound(200_000_000, [0.4, 0.0, 0.0])
    supervisor.begin_return(200_000_000)
    supervisor.update_visual(
        400_000_000, [-0.2, 0.0, 0.0], tracking=True
    )

    row = supervisor.observe_return(400_000_000, [0.35, 0.0, 0.0])

    assert row is not None
    assert row["blocked_reason"] == "visual_lio_disagreement"
    assert row["proposed_speed_mps"] == 0.0
    assert row["velocity_sent_to_cube"] is False


def test_replay_session_writes_shadow_evidence(tmp_path: Path) -> None:
    session = tmp_path / "session"
    session.mkdir()
    lio_rows = []
    visual_rows = []
    for index in range(61):
        time_s = index * 0.2
        if time_s <= 5.0:
            x_m = 0.0
        elif time_s <= 8.0:
            x_m = (time_s - 5.0) * 0.15
        else:
            x_m = max(0.05, 0.45 - (time_s - 8.0) * 0.15)
        timestamp_ns = int(time_s * 1.0e9)
        lio_rows.append(
            {
                "host_monotonic_ns": timestamp_ns,
                "position_m": [x_m, 0.0, 0.0],
            }
        )
        visual_rows.append(
            {
                "host_monotonic_ns": timestamp_ns,
                "position_local_flu_m": [x_m * 0.98, 0.0, 0.0],
                "tracking": True,
            }
        )
    (session / "lio_odometry.ndjson").write_text(
        "".join(json.dumps(row) + "\n" for row in lio_rows),
        encoding="utf-8",
    )
    (session / "rgbd_odometry.ndjson").write_text(
        "".join(json.dumps(row) + "\n" for row in visual_rows),
        encoding="utf-8",
    )
    config = load_config(Path("config/system.yaml"))

    report_path, report, digest = replay_session(session, config)

    assert report["result"] == "shadow_pass"
    assert report["control_eligible"] is False
    assert report_path.is_file()
    assert len(digest) == 64
    commands_path = Path(report["artifacts"]["commands"])
    assert commands_path.is_file()
    assert report["artifacts"]["command_rows"] > 2
    assert report["metrics"]["final_home_distance_m"] == pytest.approx(0.05)
