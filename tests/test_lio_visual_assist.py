import math

import pytest

import optflow_slam.lio_visual_assist as visual_assist
from optflow_slam.lio_visual_assist import (
    GUIDE_DURATION_S,
    LioVisualState,
)


def advance_with_cube(
    state: LioVisualState,
    now_ns: list[int],
    duration_s: float,
    attitude: dict,
    position: dict,
) -> None:
    steps = max(1, math.ceil(duration_s / 0.25))
    step_ns = int(duration_s * 1.0e9 / steps)
    for _ in range(steps):
        now_ns[0] += step_ns
        state.update_cube("ATTITUDE", attitude)
        state.update_cube("LOCAL_POSITION_NED", position)


def diagnostics(*, synchronized: bool) -> dict:
    return {
        "synchronized": synchronized,
        "publishing": synchronized,
        "imu": {
            "connected": True,
            "rate_hz": 200.0,
            "queue_drops": 0,
            "clock": {
                "ready": synchronized,
                "residual_p95_ms": 2.0,
            },
        },
        "lidar": {
            "connected": True,
            "rate_hz": 5.0,
            "queue_drops": 0,
            "clock": {
                "ready": synchronized,
                "residual_p95_ms": 1.0,
            },
        },
    }


def test_visual_guide_auto_starts_only_after_synchronized_odometry(
    monkeypatch,
) -> None:
    now_ns = [1_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("test-session")

    state.update_odometry([1.0, 2.0, 3.0])
    state.update_diagnostics(diagnostics(synchronized=False))
    waiting = state.snapshot()

    assert not waiting["guide_started"]
    assert waiting["phase"]["id"] == "sync"
    assert waiting["pose_output_to_cube"] is False

    state.update_diagnostics(diagnostics(synchronized=True))
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_diagnostics(diagnostics(synchronized=True))
    started = state.snapshot()

    assert started["guide_started"]
    assert started["phase"]["id"] == "settle"
    assert started["path"] == [[0.0, 0.0, 0.0]]


def test_visual_guide_tracks_relative_path_and_cube_reference(
    monkeypatch,
) -> None:
    now_ns = [5_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState(
        "test-session",
        maximum_position_jump_m=2.0,
    )
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([10.0, 20.0, 1.0])
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([11.0, 20.0, 1.0])
    state.update_cube("HEARTBEAT")
    state.update_cube("LOCAL_POSITION_NED")

    snapshot = state.snapshot()

    assert snapshot["path"][-1] == [1.0, 0.0, 0.0]
    assert snapshot["distance_m"] == 1.0
    assert snapshot["return_error_m"] == 1.0
    assert snapshot["cube_messages"] == 2
    assert snapshot["cube_local_position_rows"] == 1


def test_visual_guide_stops_on_impossible_lio_motion(monkeypatch) -> None:
    now_ns = [7_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("test-session")
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0], timestamp_ns=1_000_000_000)
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_diagnostics(diagnostics(synchronized=True))

    state.update_odometry([0.1, 0.0, 0.0], timestamp_ns=1_100_000_000)
    state.update_odometry([1.0, 0.0, 0.0], timestamp_ns=1_200_000_000)

    snapshot = state.snapshot()
    assert state.should_stop()
    assert snapshot["failed"]
    assert snapshot["phase"]["id"] == "failed"
    assert snapshot["failure"]["code"] == "trajectory_divergence"
    assert snapshot["failure"]["measurements"]["step_m"] == 0.9


def test_visual_guide_stops_on_excessive_attitude_step(monkeypatch) -> None:
    now_ns = [9_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("test-session")
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry(
        [0.0, 0.0, 0.0],
        timestamp_ns=1_000_000_000,
        quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
    )
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_diagnostics(diagnostics(synchronized=True))
    half_angle = math.radians(11.0 / 2.0)

    state.update_odometry(
        [0.0, 0.0, 0.0],
        timestamp_ns=1_100_000_000,
        quaternion_xyzw=[
            0.0,
            0.0,
            math.sin(half_angle),
            math.cos(half_angle),
        ],
    )

    snapshot = state.snapshot()
    assert state.should_stop()
    assert snapshot["failure"]["code"] == "trajectory_divergence"
    assert snapshot["failure"]["measurements"]["attitude_jump_deg"] > 10.0


def test_visual_guide_stops_after_complete_sequence(monkeypatch) -> None:
    now_ns = [10_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("test-session")
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_diagnostics(diagnostics(synchronized=True))

    now_ns[0] += int((GUIDE_DURATION_S + 0.1) * 1.0e9)

    assert state.should_stop()
    assert state.snapshot()["guide_complete"]


def test_yaw_guide_uses_repeated_right_center_left_phases(monkeypatch) -> None:
    now_ns = [15_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("yaw-session", guide_kind="yaw")
    state.update_cube("ATTITUDE", {"yaw": 0.0, "yawspeed": 0.0})
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_cube("ATTITUDE", {"yaw": 0.0, "yawspeed": 0.0})
    state.update_diagnostics(diagnostics(synchronized=True))
    now_ns[0] += 16_000_000_000

    snapshot = state.snapshot()

    assert snapshot["guide_kind"] == "yaw"
    assert snapshot["phase"]["id"] == "yaw_right_1"
    assert len(snapshot["guide_phases"]) == 10
    assert [phase["timeline_label"] for phase in snapshot["guide_phases"]].count(
        "RIGHT"
    ) == 2


def test_yaw_guide_stops_when_cube_motion_exceeds_rate_limit(
    monkeypatch,
) -> None:
    now_ns = [18_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("yaw-session", guide_kind="yaw")
    state.update_cube("ATTITUDE", {"yaw": 0.0, "yawspeed": 0.0})
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_cube("ATTITUDE", {"yaw": 0.0, "yawspeed": 0.0})
    state.update_diagnostics(diagnostics(synchronized=True))

    state.update_cube("ATTITUDE", {"yaw": 0.1, "yawspeed": 0.7})

    snapshot = state.snapshot()
    assert state.should_stop()
    assert snapshot["failure"]["code"] == "excessive_test_motion"
    assert snapshot["yaw"]["cube_maximum_rate_dps"] > 30.0


def test_yaw_guide_pauses_when_cube_attitude_becomes_stale(
    monkeypatch,
) -> None:
    now_ns = [22_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("yaw-session", guide_kind="yaw")
    state.update_cube("ATTITUDE", {"yaw": 0.0, "yawspeed": 0.0})
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_cube("ATTITUDE", {"yaw": 0.0, "yawspeed": 0.0})
    assert state.snapshot()["guide_started"]

    now_ns[0] += int((visual_assist.CUBE_ATTITUDE_STALE_S + 0.1) * 1.0e9)
    state.update_odometry([0.0, 0.0, 0.0])

    snapshot = state.snapshot()
    assert snapshot["paused"]
    assert not snapshot["cube_attitude_fresh"]


def test_translation_guide_reports_cube_position_in_start_body_frame(
    monkeypatch,
) -> None:
    now_ns = [26_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("translation-session", guide_kind="translation")
    cube_attitude = {"yaw": math.pi / 2.0, "yawspeed": 0.0}
    cube_position = {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
    }
    state.update_cube("ATTITUDE", cube_attitude)
    state.update_cube("LOCAL_POSITION_NED", cube_position)
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    advance_with_cube(
        state,
        now_ns,
        visual_assist.STABLE_LOCK_S + 0.1,
        cube_attitude,
        cube_position,
    )
    advance_with_cube(
        state,
        now_ns,
        16.0,
        cube_attitude,
        cube_position,
    )
    state.update_cube(
        "LOCAL_POSITION_NED",
        {"x": 1.0, "y": 2.5, "z": 3.0, "vx": 0.0, "vy": 0.05, "vz": 0.0},
    )

    snapshot = state.snapshot()
    assert snapshot["phase"]["id"] == "forward_1"
    assert snapshot["translation"]["cube_body_delta_m"] == pytest.approx(
        [0.5, 0.0, 0.0]
    )
    assert snapshot["cube_local_position_fresh"]


def test_translation_guide_treats_cube_position_as_diagnostic(monkeypatch) -> None:
    now_ns = [30_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("translation-session", guide_kind="translation")
    cube_attitude = {"yaw": 0.0, "yawspeed": 0.0}
    cube_position = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
    }
    state.update_cube("ATTITUDE", cube_attitude)
    state.update_cube("LOCAL_POSITION_NED", cube_position)
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    advance_with_cube(
        state,
        now_ns,
        visual_assist.STABLE_LOCK_S + 0.1,
        cube_attitude,
        cube_position,
    )

    state.update_cube(
        "LOCAL_POSITION_NED",
        {**cube_position, "x": 0.1, "vx": 0.6},
    )

    snapshot = state.snapshot()
    assert not state.should_stop()
    assert snapshot["failure"] is None
    assert snapshot["translation"][
        "cube_maximum_horizontal_speed_mps"
    ] == pytest.approx(0.6)


def test_translation_guide_captures_tape_marked_phase_positions(
    monkeypatch,
) -> None:
    now_ns = [34_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("translation-session", guide_kind="translation")
    cube_attitude = {"yaw": 0.0, "yawspeed": 0.0}
    state.update_cube("ATTITUDE", cube_attitude)
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_cube("ATTITUDE", cube_attitude)
    assert state.snapshot()["guide_started"]
    guide_start_ns = now_ns[0]

    for phase in visual_assist.TRANSLATION_GUIDE_PHASES:
        target = visual_assist.TRANSLATION_CAPTURE_TARGETS[phase["id"]]
        for before_end_s in (2.75, 2.25, 1.75, 1.25, 0.75, 0.25):
            elapsed_s = float(phase["end_s"]) - before_end_s
            now_ns[0] = guide_start_ns + int(elapsed_s * 1.0e9)
            state.update_cube("ATTITUDE", cube_attitude)
            state.update_odometry(target, timestamp_ns=now_ns[0])

    now_ns[0] = guide_start_ns + int((GUIDE_DURATION_S + 0.1) * 1.0e9)
    state.update_cube("ATTITUDE", cube_attitude)
    result = state.guide_result()

    assert result["guide_complete"]
    assert result["cube_local_position_used_as_ground_truth"] is False
    assert all(capture["samples"] == 6 for capture in result["captures"])
    captures = {
        capture["phase_id"]: capture for capture in result["captures"]
    }
    assert captures["forward_1"]["observed_m"] == pytest.approx(
        [0.5, 0.0, 0.0]
    )
    assert captures["right_1"]["observed_m"] == pytest.approx(
        [0.0, 0.5, 0.0]
    )
    assert captures["final_still"]["observed_m"] == pytest.approx(
        [0.0, 0.0, 0.0]
    )


def test_visual_guide_pauses_until_lock_is_stable_again(monkeypatch) -> None:
    now_ns = [20_000_000_000]
    monkeypatch.setattr(
        visual_assist.time,
        "monotonic_ns",
        lambda: now_ns[0],
    )
    state = LioVisualState("test-session")
    state.update_diagnostics(diagnostics(synchronized=True))
    state.update_odometry([0.0, 0.0, 0.0])
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_diagnostics(diagnostics(synchronized=True))
    now_ns[0] += 2_000_000_000

    state.update_diagnostics(diagnostics(synchronized=False))
    paused_elapsed = state.snapshot()["elapsed_s"]
    now_ns[0] += 20_000_000_000

    paused = state.snapshot()
    assert paused["paused"]
    assert paused["phase"]["id"] == "paused"
    assert paused["elapsed_s"] == paused_elapsed

    state.update_diagnostics(diagnostics(synchronized=True))
    now_ns[0] += int((visual_assist.STABLE_LOCK_S + 0.1) * 1.0e9)
    state.update_diagnostics(diagnostics(synchronized=True))
    assert not state.snapshot()["paused"]
