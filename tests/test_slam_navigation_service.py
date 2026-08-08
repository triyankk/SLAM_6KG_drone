from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import sys
import time

import optflow_slam.slam_navigation_service as service


def config_for_status(path: Path) -> SimpleNamespace:
    return SimpleNamespace(
        navigation=SimpleNamespace(
            slam_return=SimpleNamespace(status_file=str(path))
        ),
        flight_controller=SimpleNamespace(
            endpoint="/dev/ttyTHS1",
            baud=921600,
        ),
    )


def test_prearm_status_reports_service_cached_uart_errors(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    status_path = tmp_path / "status.json"
    status_path.write_text(
        json.dumps(
            {
                "updated_unix_ns": time.time_ns(),
                "health_gates": {"cube_heartbeat_fresh": True},
                "cube": {
                    "status_text_window_s": 60.0,
                    "recent_status_texts": [
                        {
                            "severity": 4,
                            "age_s": 2.5,
                            "text": "PRX1: No Data",
                        }
                    ],
                    "prearm_errors": [
                        {
                            "severity": 4,
                            "age_s": 2.5,
                            "text": "PreArm: Compass not calibrated",
                        }
                    ],
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        service, "load_config", lambda _path: config_for_status(status_path)
    )
    monkeypatch.setattr(sys, "argv", ["optflow-prearm-status"])

    result = service.prearm_status_main()
    output = capsys.readouterr().out

    assert result == 0
    assert "SERVICE=live" in output
    assert "CUBE_LINK=/dev/ttyTHS1 baud=921600" in output
    assert "CUBE_TELEMETRY=live" in output
    assert "STATUSTEXT_STREAM=recent" in output
    assert "STATUSTEXT_MESSAGES=1 window=60s" in output
    assert "ARMING_RELEVANT_WARNINGS=1" in output
    assert "PRX1: No Data" in output


def test_prearm_status_never_calls_stale_telemetry_live(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    status_path = tmp_path / "status.json"
    status_path.write_text(
        json.dumps(
            {
                "updated_unix_ns": 0,
                "health_gates": {"cube_heartbeat_fresh": True},
                "cube": {
                    "status_text_window_s": 60.0,
                    "recent_status_texts": [],
                    "prearm_errors": [],
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        service, "load_config", lambda _path: config_for_status(status_path)
    )
    monkeypatch.setattr(sys, "argv", ["optflow-prearm-status"])

    result = service.prearm_status_main()
    output = capsys.readouterr().out

    assert result == 1
    assert "SERVICE=stale" in output
    assert "CUBE_TELEMETRY=stale_or_missing" in output
    assert "CUBE_TELEMETRY=live" not in output
