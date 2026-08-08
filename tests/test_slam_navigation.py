from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

import optflow_slam.slam_navigation as slam_navigation
from optflow_slam.config import ConfigError, load_config
from optflow_slam.cube_odometry import OdometryPacket
from optflow_slam.obstacles import ObstacleScan
from optflow_slam.slam_navigation import (
    CubeGuidedVelocityLink,
    SlamReturnController,
    audit_cube_parameters,
)


ROOT = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self) -> None:
        self.now_ns = 1_000_000_000

    def __call__(self) -> int:
        return self.now_ns

    def advance(self, seconds: float) -> None:
        self.now_ns += round(seconds * 1.0e9)


class RowCollector:
    def __init__(self) -> None:
        self.rows: list[dict] = []

    def write(self, row: dict) -> None:
        self.rows.append(row)


class PoseState:
    def __init__(self, value: OdometryPacket) -> None:
        self.value = value

    def latest_healthy(
        self, _now_ns: int
    ) -> tuple[OdometryPacket, str]:
        return self.value, "ready"

    def latest_observed(self):
        return None


def ready_parameters(controller: SlamReturnController) -> dict[str, float]:
    source = controller.settings.ekf_source_set
    return {
        "AHRS_EKF_TYPE": 3.0,
        "EK3_ENABLE": 1.0,
        "FLOW_TYPE": 6.0,
        "RNGFND1_TYPE": 24.0,
        "FLTMODE_CH": 5.0,
        "GUID_OPTIONS": 64.0,
        "GUID_TIMEOUT": 0.5,
        "BATT_MONITOR": 4.0,
        f"RC{controller.settings.rc_channel}_OPTION": 0.0,
        f"RC{controller.settings.land_rc_channel}_OPTION": 18.0,
        f"EK3_SRC{source}_POSXY": 0.0,
        f"EK3_SRC{source}_VELXY": 5.0,
        f"EK3_SRC{source}_POSZ": 1.0,
        f"EK3_SRC{source}_VELZ": 0.0,
        f"EK3_SRC{source}_YAW": 1.0,
    }


def packet(sequence: int, x_m: float) -> OdometryPacket:
    return OdometryPacket(
        sequence=sequence,
        time_usec=sequence * 100_000,
        position_local_frd_m=(x_m, 0.0, 0.0),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        velocity_body_frd_mps=(0.0, 0.0, 0.0),
        angular_velocity_body_frd_rads=(0.0, 0.0, 0.0),
        pose_error=(0.0,) * 21,
        velocity_covariance=(0.0,) * 21,
        reset_counter=0,
        quality=50,
    )


def obstacle(clock: Clock, distance_m: float) -> ObstacleScan:
    return ObstacleScan(
        source="lidar",
        monotonic_ns=clock(),
        distances_cm=tuple([round(distance_m * 100)] * 72),
        increment_deg=5.0,
        min_distance_cm=30,
        max_distance_cm=800,
    )


def update_flight_inputs(
    controller: SlamReturnController,
    clock: Clock,
    *,
    rc_pwm: int,
    yaw_rad: float = 0.0,
    obstacle_distance_m: float = 3.0,
    mode_name: str = "GUIDED",
) -> None:
    controller.observe_cube(
        "ATTITUDE", {"yaw": yaw_rad}
    )
    controller.observe_cube(
        "HEARTBEAT", {"base_mode": 128, "_mode_name": mode_name}
    )
    controller.observe_cube("OPTICAL_FLOW", {"quality": 150})
    controller.observe_cube(
        "DISTANCE_SENSOR",
        {"orientation": 25, "current_distance": 150},
    )
    controller.observe_cube(
        "RC_CHANNELS",
        {f"chan{controller.settings.rc_channel}_raw": rc_pwm},
    )
    controller.observe_cube(
        "SYS_STATUS", {"voltage_battery": 24_000}
    )
    controller.observe_cube(
        "LOCAL_POSITION_NED", {"x": 0.0, "y": 0.0, "z": -1.5}
    )
    controller.observe_cube("GPS_GLOBAL_ORIGIN", {})
    controller.observe_obstacle(obstacle(clock, obstacle_distance_m))


def prepare_return(
    *, control_permitted: bool, yaw_rad: float = 0.0
) -> tuple[SlamReturnController, Clock]:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=control_permitted,
        approval_reason="test",
        clock_ns=clock,
    )
    controller.set_parameter_audit({"test": True}, "ready")
    controller.observe_cube(
        "HEARTBEAT", {"base_mode": 0, "_mode_name": "ALT_HOLD"}
    )
    controller.observe_pose(packet(1, 0.0), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [0.0, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(controller, clock, rc_pwm=1000, yaw_rad=yaw_rad)
    clock.advance(0.10)
    controller.observe_pose(packet(2, 0.40), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [0.40, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(controller, clock, rc_pwm=1000, yaw_rad=yaw_rad)
    clock.advance(0.05)
    update_flight_inputs(controller, clock, rc_pwm=1800, yaw_rad=yaw_rad)
    waiting = controller.step(clock())
    assert waiting.reason == "waiting_for_return_odometry"
    clock.advance(0.10)
    controller.observe_pose(packet(3, 0.35), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [0.35, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(controller, clock, rc_pwm=1800, yaw_rad=yaw_rad)
    return controller, clock


def test_live_return_generates_bounded_velocity_toward_launch() -> None:
    controller, clock = prepare_return(control_permitted=True)

    decision = controller.step(clock())

    assert decision.transmit
    assert decision.state == "returning_live"
    assert decision.reason is None
    assert decision.velocity_local_flu_mps[0] < 0.0
    assert abs(decision.velocity_local_flu_mps[0]) <= 0.30
    assert decision.velocity_local_ned_mps[0] < 0.0


def test_raw_monitor_trajectory_does_not_feed_guarded_navigation() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=False,
        approval_reason="test",
        clock_ns=clock,
    )

    controller.observe_monitor_pose((0.0, 0.0, 0.0), 1, clock())
    clock.advance(0.1)
    controller.observe_monitor_pose((0.4, 0.1, -0.2), 2, clock())
    snapshot = controller.snapshot(clock())

    assert snapshot["estimator"]["position_local_flu_m"] is None
    assert snapshot["estimator"]["monitor_position_local_flu_m"] == [
        0.4,
        -0.1,
        0.2,
    ]
    assert snapshot["trajectories"]["lio"] == []
    assert snapshot["trajectories"]["lio_monitor"] == [
        [0.0, -0.0, -0.0],
        [0.4, -0.1, 0.2],
    ]


def test_recent_cube_prearm_errors_are_deduplicated_and_expire() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=False,
        approval_reason="test",
        clock_ns=clock,
    )

    controller.observe_cube(
        "STATUSTEXT",
        {"severity": 4, "text": b"PreArm: Compass not calibrated\x00"},
    )
    clock.advance(1.0)
    controller.observe_cube(
        "STATUSTEXT",
        {"severity": 4, "text": "PreArm: Compass not calibrated"},
    )
    controller.observe_cube(
        "STATUSTEXT",
        {"severity": 5, "text": "EKF3 IMU0 is using GPS"},
    )

    snapshot = controller.snapshot(clock())

    assert snapshot["cube"]["latest_status_text"]["text"] == (
        "EKF3 IMU0 is using GPS"
    )
    assert snapshot["cube"]["recent_status_texts"] == [
        {
            "text": "EKF3 IMU0 is using GPS",
            "severity": 5,
            "age_s": 0.0,
        },
        {
            "text": "PreArm: Compass not calibrated",
            "severity": 4,
            "age_s": 0.0,
        },
    ]
    assert snapshot["cube"]["prearm_errors"] == [
        {
            "text": "PreArm: Compass not calibrated",
            "severity": 4,
            "age_s": 0.0,
        }
    ]

    clock.advance(slam_navigation.STATUS_TEXT_FRESHNESS_S + 0.1)
    assert controller.snapshot(clock())["cube"]["prearm_errors"] == []


def test_locked_return_never_transmits_a_velocity() -> None:
    controller, clock = prepare_return(control_permitted=False)

    decision = controller.step(clock())

    assert decision.state == "returning_locked"
    assert not decision.transmit
    assert decision.velocity_local_flu_mps[0] < 0.0


def test_locked_rehearsal_keeps_computing_in_flowhold() -> None:
    controller, clock = prepare_return(control_permitted=False)
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1800,
        mode_name="FLOWHOLD",
    )

    decision = controller.step(clock())
    snapshot = controller.snapshot(clock())

    assert decision.state == "returning_locked"
    assert not decision.transmit
    assert not snapshot["abort_latched"]
    assert not snapshot["health_gates"]["regular_guided_mode"]


def test_five_metre_flowhold_rehearsal_prompts_rc10_then_shadow_return() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=False,
        approval_reason="test",
        clock_ns=clock,
    )
    controller.set_parameter_audit({"test": True}, "ready")
    controller.observe_cube(
        "HEARTBEAT", {"base_mode": 0, "_mode_name": "FLOWHOLD"}
    )
    controller.observe_pose(packet(1, 0.0), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [0.0, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1000,
        mode_name="FLOWHOLD",
    )
    initial_hold = controller.rehearsal_status(clock())
    assert initial_hold["phase"] == "initial_hold"
    assert "1-8M" in initial_hold["instruction"]
    clock.advance(10.1)
    controller.observe_pose(packet(2, 0.0), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [0.0, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1000,
        mode_name="FLOWHOLD",
    )
    clock.advance(0.1)
    controller.observe_pose(packet(3, 5.0), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [5.0, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1000,
        mode_name="FLOWHOLD",
    )
    controller.rehearsal_status(clock())
    clock.advance(3.1)
    controller.observe_pose(packet(4, 5.0), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [5.0, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1000,
        mode_name="FLOWHOLD",
    )

    ready = controller.rehearsal_status(clock())

    assert ready["phase"] == "ready_for_return_switch"
    assert ready["ready_for_return_switch"]
    assert ready["return_rc_channel"] == 9
    assert ready["line_ready"]

    clock.advance(0.05)
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1800,
        mode_name="FLOWHOLD",
    )
    decision = controller.step(clock())

    assert decision.state == "returning_locked"
    assert not decision.transmit
    assert controller.rehearsal_status(clock())["phase"] == "shadow_return"

    clock.advance(0.1)
    controller.observe_pose(packet(5, 0.1), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [0.1, 0.0, 0.0],
            "tracking": True,
        }
    )
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1800,
        mode_name="FLOWHOLD",
    )
    clock.advance(0.1)
    controller.observe_pose(packet(6, 0.1), clock())
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1800,
        mode_name="FLOWHOLD",
    )
    arrived = controller.step(clock())
    result = controller.rehearsal_status(clock())

    assert arrived.state == "arrived"
    assert result["phase"] == "arrival"
    assert result["profile_pass"]


def test_live_return_rotates_launch_frame_velocity_into_ned() -> None:
    controller, clock = prepare_return(
        control_permitted=True, yaw_rad=1.5707963267948966
    )

    decision = controller.step(clock())

    assert decision.velocity_local_ned_mps[0] == pytest.approx(0.0, abs=1e-6)
    assert decision.velocity_local_ned_mps[1] < 0.0


def test_obstacle_breach_latches_abort_and_zeroes_output() -> None:
    controller, clock = prepare_return(control_permitted=True)
    assert controller.step(clock()).transmit
    clock.advance(0.05)
    update_flight_inputs(
        controller,
        clock,
        rc_pwm=1800,
        obstacle_distance_m=1.0,
    )

    decision = controller.step(clock())

    assert decision.state == "aborted"
    assert decision.transmit
    assert decision.hold_zero
    assert decision.velocity_local_ned_mps == (0.0, 0.0, 0.0)
    assert decision.reason == "hard_clearance_clear"


def test_low_rc_switch_cancels_and_sends_a_zero_handoff() -> None:
    controller, clock = prepare_return(control_permitted=True)
    assert controller.step(clock()).transmit
    clock.advance(0.05)
    update_flight_inputs(controller, clock, rc_pwm=1000)

    decision = controller.step(clock())

    assert decision.state == "aborted"
    assert decision.reason == "pilot_cancelled_return"
    assert decision.transmit
    assert decision.hold_zero


def test_starting_while_cube_is_already_armed_latches_abort() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=True,
        approval_reason="test",
        clock_ns=clock,
    )
    controller.observe_pose(packet(1, 0.0), clock())
    controller.observe_visual(
        {
            "host_monotonic_ns": clock(),
            "position_local_flu_m": [0.0, 0.0, 0.0],
            "tracking": True,
        }
    )

    update_flight_inputs(controller, clock, rc_pwm=1000)
    decision = controller.step(clock())

    assert decision.state == "aborted"
    assert decision.reason == "armed_before_disarmed_boot_gate"
    assert not decision.transmit


def test_config_rejects_return_speed_above_hard_ceiling(
    tmp_path: Path,
) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(encoding="ascii")
    source = source.replace(
        "maximum_horizontal_speed_mps: 0.30",
        "maximum_horizontal_speed_mps: 0.80",
    ).replace(
        "initial_max_horizontal_speed_mps: 0.5",
        "initial_max_horizontal_speed_mps: 0.8",
    )
    path = tmp_path / "too-fast.yaml"
    path.write_text(source, encoding="ascii")

    with pytest.raises(ConfigError, match="0.75 m/s"):
        load_config(path)


def test_cube_parameter_audit_requires_bounded_guided_timeout() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    controller = SlamReturnController(
        config,
        control_permitted=False,
        approval_reason="test",
    )
    parameters = ready_parameters(controller)

    gates, detail = audit_cube_parameters(
        parameters, controller.settings
    )

    assert detail == "ready"
    assert all(gates.values())
    parameters["GUID_TIMEOUT"] = 3.0
    gates, detail = audit_cube_parameters(
        parameters, controller.settings
    )
    assert detail == "cube_parameter_gate_failed"
    assert not gates["guided_timeout_bounded"]


def test_transport_sends_only_bounded_horizontal_velocity(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    controller, clock = prepare_return(control_permitted=True)
    sent_targets: list[tuple] = []
    command_requests: list[tuple] = []
    mav = SimpleNamespace(
        command_long_send=lambda *args: command_requests.append(args),
        set_position_target_local_ned_send=(
            lambda *args: sent_targets.append(args)
        ),
        statustext_send=lambda *_args: None,
        play_tune_send=lambda *_args: None,
    )
    connection = SimpleNamespace(mav=mav)
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_CMD_REQUEST_MESSAGE=512,
            MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN=49,
            MAV_FRAME_LOCAL_NED=1,
            MAV_SEVERITY_NOTICE=5,
            MAV_SEVERITY_WARNING=4,
        )
    )
    output = RowCollector()
    link = CubeGuidedVelocityLink(
        controller,
        PoseState(packet(4, 0.30)),
        output,
        tmp_path / "status.json",
        heartbeat_timeout_s=1.0,
    )
    link.parameters.update(ready_parameters(controller))
    monkeypatch.setattr(slam_navigation.time, "monotonic_ns", clock)
    clock.advance(0.10)

    link.tick(connection, mavutil, 1, 1)

    assert command_requests
    assert len(sent_targets) == 1
    target = sent_targets[0]
    assert target[3] == mavutil.mavlink.MAV_FRAME_LOCAL_NED
    assert target[4] == slam_navigation.VELOCITY_ONLY_TYPE_MASK
    assert target[10] == 0.0
    assert (target[8] ** 2 + target[9] ** 2) ** 0.5 <= 0.30
    assert target[11:] == (0.0, 0.0, 0.0, 0.0, 0.0)
    assert link.commands_sent == 1
    assert any(row["event"] == "slam_return_decision" for row in output.rows)


def test_parameter_audit_allows_only_one_outstanding_request(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=False,
        approval_reason="test",
        clock_ns=clock,
    )
    link = CubeGuidedVelocityLink(
        controller,
        PoseState(packet(1, 0.0)),
        RowCollector(),
        tmp_path / "status.json",
        heartbeat_timeout_s=1.0,
    )
    requests: list[tuple] = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            param_request_read_send=lambda *args: requests.append(args)
        )
    )
    monkeypatch.setattr(slam_navigation.time, "monotonic_ns", clock)

    link._request_parameters(connection, 1, 1, clock())
    clock.advance(1.0)
    link._request_parameters(connection, 1, 1, clock())

    assert len(requests) == 1
    pending = requests[0][2].decode("ascii")
    link.observe_message(
        "PARAM_VALUE",
        {"param_id": pending, "param_value": 3.0},
    )
    clock.advance(0.05)
    link._request_parameters(connection, 1, 1, clock())
    assert len(requests) == 2
    assert requests[1][2] != requests[0][2]


def test_parameter_audit_falls_back_to_one_full_list_request(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=False,
        approval_reason="test",
        clock_ns=clock,
    )
    link = CubeGuidedVelocityLink(
        controller,
        PoseState(packet(1, 0.0)),
        RowCollector(),
        tmp_path / "status.json",
        heartbeat_timeout_s=1.0,
    )
    named_requests: list[tuple] = []
    list_requests: list[tuple] = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            param_request_read_send=lambda *args: named_requests.append(args),
            param_request_list_send=lambda *args: list_requests.append(args),
        )
    )
    monkeypatch.setattr(slam_navigation.time, "monotonic_ns", clock)
    link.observe_message("HEARTBEAT", {"base_mode": 0})
    clock.advance(5.1)

    link._request_parameters(connection, 1, 1, clock())
    link._request_parameters(connection, 1, 1, clock())

    assert len(list_requests) == 1
    assert len(named_requests) == 1
    assert link.parameter_list_requests_sent == 1


def test_qgc_rehearsal_prompt_repeats_for_late_connection(
    tmp_path: Path,
) -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    clock = Clock()
    controller = SlamReturnController(
        config,
        control_permitted=False,
        approval_reason="test",
        clock_ns=clock,
    )
    link = CubeGuidedVelocityLink(
        controller,
        PoseState(packet(1, 0.0)),
        RowCollector(),
        tmp_path / "status.json",
        heartbeat_timeout_s=1.0,
    )
    messages: list[tuple] = []
    tunes: list[tuple] = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            statustext_send=lambda *args: messages.append(args),
            play_tune_send=lambda *args: tunes.append(args),
        )
    )
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_SEVERITY_NOTICE=5,
            MAV_SEVERITY_WARNING=4,
        )
    )

    link._announce_rehearsal(connection, mavutil, 1, 1, clock())
    clock.advance(4.0)
    link._announce_rehearsal(connection, mavutil, 1, 1, clock())
    clock.advance(1.1)
    link._announce_rehearsal(connection, mavutil, 1, 1, clock())

    assert len(messages) == 2
    assert not tunes
    assert b"GPS ON, RC9 LOW" in messages[-1][1]


def test_transport_disconnect_invalidates_cube_inputs_and_aborts() -> None:
    controller, clock = prepare_return(control_permitted=False)

    controller.observe_transport_disconnect("cube_transport_disconnected")
    decision = controller.step(clock())
    snapshot = controller.snapshot(clock())

    assert decision.state == "aborted"
    assert decision.reason == "cube_transport_disconnected"
    assert not snapshot["health_gates"]["cube_heartbeat_fresh"]
    assert not snapshot["health_gates"]["rc_input_fresh"]
