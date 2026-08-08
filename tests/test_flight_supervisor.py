import json

from optflow_slam.flight_supervisor import (
    _read_status,
    _shadow_completed_this_boot,
    _write_status,
)


def test_shadow_completion_marker_is_scoped_to_current_boot(tmp_path) -> None:
    status_path = tmp_path / "supervisor.json"
    _write_status(
        status_path,
        state="shadow_flight_complete",
        boot_id="boot-a",
        completed_arm_cycle=True,
    )

    status = _read_status(status_path)

    assert _shadow_completed_this_boot(status, "boot-a")
    assert not _shadow_completed_this_boot(status, "boot-b")
    assert json.loads(status_path.read_text())["schema_version"] == 1


def test_incomplete_arm_cycle_does_not_skip_next_shadow(tmp_path) -> None:
    status_path = tmp_path / "supervisor.json"
    _write_status(
        status_path,
        state="shadow_flight_complete",
        boot_id="boot-a",
        completed_arm_cycle=False,
    )

    assert not _shadow_completed_this_boot(
        _read_status(status_path), "boot-a"
    )
