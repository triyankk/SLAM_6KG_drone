from types import SimpleNamespace

from optflow_slam.jt16_calibration import (
    CubeCalibrationLink,
    DirectionResult,
    direction_result,
    ordered_directions,
    run_guided_sequence,
)


def passing_result(direction, distance=2.5):
    return DirectionResult(
        direction=direction,
        passed=True,
        measured_distance_m=distance,
        measured_angle_deg=direction.bearing_deg,
        payload={},
    )


def test_ordered_directions_can_start_from_current_left_side() -> None:
    directions = ordered_directions("left")

    assert [direction.key for direction in directions] == [
        "left",
        "forward",
        "right",
        "rear",
    ]


def test_guided_sequence_beeps_then_shows_and_waits_for_next() -> None:
    directions = ordered_directions("left")
    events = []

    success, results = run_guided_sequence(
        directions,
        check_direction=lambda direction: (
            events.append(("check", direction.key))
            or passing_result(direction)
        ),
        beep=lambda: events.append(("beep", None)),
        show_direction=lambda direction, _index, _total, previous: (
            events.append(
                (
                    "show",
                    direction.key,
                    None if previous is None else previous.direction.key,
                )
            )
        ),
        wait_for_positioning=lambda: events.append(("wait", 10)),
        show_failure=lambda result: events.append(
            ("failure", result.direction.key)
        ),
        show_complete=lambda completed: events.append(
            ("complete", len(completed))
        ),
    )

    assert success
    assert len(results) == 4
    assert events == [
        ("show", "left", None),
        ("wait", 10),
        ("check", "left"),
        ("beep", None),
        ("show", "forward", "left"),
        ("wait", 10),
        ("check", "forward"),
        ("beep", None),
        ("show", "right", "forward"),
        ("wait", 10),
        ("check", "right"),
        ("beep", None),
        ("show", "rear", "right"),
        ("wait", 10),
        ("check", "rear"),
        ("beep", None),
        ("complete", 4),
    ]


def test_guided_sequence_stops_without_beep_after_failure() -> None:
    directions = ordered_directions("forward")
    events = []

    def check(direction):
        events.append(("check", direction.key))
        return DirectionResult(
            direction=direction,
            passed=False,
            measured_distance_m=1.0,
            measured_angle_deg=45.0,
            payload={},
        )

    success, results = run_guided_sequence(
        directions,
        check_direction=check,
        beep=lambda: events.append(("beep", None)),
        show_direction=lambda direction, _index, _total, _previous: (
            events.append(("show", direction.key))
        ),
        wait_for_positioning=lambda: events.append(("wait", 10)),
        show_failure=lambda result: events.append(
            ("failure", result.direction.key)
        ),
        show_complete=lambda completed: events.append(
            ("complete", len(completed))
        ),
    )

    assert not success
    assert len(results) == 1
    assert events == [
        ("show", "forward"),
        ("wait", 10),
        ("check", "forward"),
        ("failure", "forward"),
    ]


def test_direction_result_reads_lidar_target_measurement() -> None:
    direction = ordered_directions("right")[0]
    result = direction_result(
        direction,
        {
            "target_check": {
                "passed": True,
                "sources": {
                    "lidar": {
                        "measured_distance_m": 2.44,
                        "measured_sector_angle_deg": 90.0,
                    }
                },
            }
        },
    )

    assert result.passed
    assert result.measured_distance_m == 2.44
    assert result.measured_angle_deg == 90.0


def test_cube_link_repairs_pymavlink_mixed_instance_cache() -> None:
    cached_message = SimpleNamespace(
        _instance_field="instance",
        instance=None,
        _instances=None,
    )
    heartbeat = SimpleNamespace(
        autopilot=3,
        get_srcComponent=lambda: 1,
    )

    class Master:
        def __init__(self) -> None:
            self.calls = 0
            self.sysid_state = {
                1: SimpleNamespace(messages={"SENSOR": cached_message})
            }

        def recv_match(self, **_kwargs):
            self.calls += 1
            if self.calls == 1:
                raise TypeError(
                    "'NoneType' object does not support item assignment"
                )
            return heartbeat

    link = CubeCalibrationLink(
        SimpleNamespace(heartbeat_timeout_s=1.0, system_id=1)
    )
    link.master = Master()
    link.mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_AUTOPILOT_ARDUPILOTMEGA=3,
            MAV_COMP_ID_AUTOPILOT1=1,
        )
    )

    assert link._wait_for_heartbeat() is heartbeat
    assert cached_message._instances == {}
