from __future__ import annotations

import math
from types import SimpleNamespace
import time

import pytest

from optflow_slam.cube_odometry import (
    ARMED_FLAG,
    AUDIT_PARAMETERS,
    CubeOdometryShadowLink,
    MAV_ESTIMATOR_TYPE_LIDAR,
    MAV_FRAME_BODY_FRD,
    MAV_FRAME_LOCAL_FRD,
    OdometryShadowState,
)


class Writer:
    def __init__(self) -> None:
        self.rows: list[dict] = []

    def write(self, row: dict) -> None:
        self.rows.append(row)


class Mav:
    def __init__(self) -> None:
        self.parameter_requests: list[str] = []
        self.parameter_list_requests = 0
        self.odometry: list[tuple] = []

    def param_request_read_send(
        self, _system: int, _component: int, name: bytes, _index: int
    ) -> None:
        self.parameter_requests.append(name.decode("ascii"))

    def param_request_list_send(self, _system: int, _component: int) -> None:
        self.parameter_list_requests += 1

    def odometry_send(self, *values) -> None:
        self.odometry.append(values)


def _healthy_state(*, yaw_deg: float = 0.0) -> OdometryShadowState:
    state = OdometryShadowState(
        stale_timeout_s=0.5,
        maximum_position_jump_m=0.5,
        maximum_attitude_jump_deg=10.0,
        minimum_samples=2,
    )
    now_ns = time.monotonic_ns()
    state.update_diagnostics(
        now_ns,
        {
            "synchronized": True,
            "publishing": True,
            "imu": {
                "connected": True,
                "error": None,
                "checksum_errors": 0,
                "payload_errors": 0,
            },
            "lidar": {
                "connected": True,
                "error": None,
                "non_monotonic_frames": 0,
            },
        },
    )
    state.update_imu(now_ns, (0.01, -0.02, 0.03))
    yaw = math.radians(yaw_deg) * 0.5
    quaternion = (0.0, 0.0, math.sin(yaw), math.cos(yaw))
    state.update_odometry(
        host_monotonic_ns=now_ns - 100_000_000,
        ros_time_ns=1_000_000_000,
        frame_id="camera_init",
        child_frame_id="body",
        position_m=(1.0, 2.0, 3.0),
        quaternion_xyzw=quaternion,
        pose_covariance=(0.0,) * 36,
    )
    state.update_odometry(
        host_monotonic_ns=now_ns,
        ros_time_ns=1_100_000_000,
        frame_id="camera_init",
        child_frame_id="body",
        position_m=(1.0, 2.1, 3.0),
        quaternion_xyzw=quaternion,
        pose_covariance=(0.0,) * 36,
    )
    return state


def _audited_link(state: OdometryShadowState) -> tuple[
    CubeOdometryShadowLink, Writer
]:
    writer = Writer()
    link = CubeOdometryShadowLink(
        state,
        writer,
        heartbeat_timeout_s=2.0,
    )
    link.observe_message("HEARTBEAT", {"base_mode": 0})
    for name in AUDIT_PARAMETERS:
        value = 3.0 if name == "AHRS_EKF_TYPE" else 1.0 if name == "EK3_ENABLE" else 0.0
        link.observe_message(
            "PARAM_VALUE",
            {"param_id": name, "param_value": value},
        )
    return link, writer


def test_pose_is_rebased_into_initial_local_frd_heading() -> None:
    state = _healthy_state(yaw_deg=90.0)

    packet, reason = state.candidate(time.monotonic_ns())

    assert reason == "ready"
    assert packet is not None
    assert packet.position_local_frd_m == pytest.approx((0.1, 0.0, 0.0))
    assert packet.quaternion_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert packet.pose_error[0] == pytest.approx(0.25)
    assert packet.pose_error[6] == pytest.approx(0.25)
    assert packet.pose_error[11] == pytest.approx(0.35)


def test_shadow_link_sends_mavlink2_odometry_after_read_only_audit() -> None:
    state = _healthy_state()
    link, writer = _audited_link(state)
    mav = Mav()
    connection = SimpleNamespace(mav=mav)
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_FRAME_LOCAL_FRD=MAV_FRAME_LOCAL_FRD,
            MAV_FRAME_BODY_FRD=MAV_FRAME_BODY_FRD,
            MAV_ESTIMATOR_TYPE_LIDAR=MAV_ESTIMATOR_TYPE_LIDAR,
        )
    )

    link.tick(connection, mavutil, 1, 1)

    assert len(mav.odometry) == 1
    sent = mav.odometry[0]
    assert sent[1] == MAV_FRAME_LOCAL_FRD
    assert sent[2] == MAV_FRAME_BODY_FRD
    assert sent[-2] == MAV_ESTIMATOR_TYPE_LIDAR
    assert sent[-1] == 50
    assert writer.rows[-1]["event"] == "odometry_sent"
    assert writer.rows[-1]["position_local_frd_m"] == pytest.approx(
        (0.0, 0.1, 0.0)
    )


def test_shadow_link_refuses_any_external_nav_source_configuration() -> None:
    state = _healthy_state()
    link, _writer = _audited_link(state)
    link.parameters["EK3_SRC2_POSXY"] = 6.0
    connection = SimpleNamespace(mav=Mav())
    mavutil = SimpleNamespace(mavlink=SimpleNamespace())

    with pytest.raises(RuntimeError, match="ExternalNav is configured"):
        link.tick(connection, mavutil, 1, 1)

    assert not connection.mav.odometry


def test_armed_heartbeat_latches_the_output_interlock() -> None:
    state = _healthy_state()
    link, _writer = _audited_link(state)
    link.observe_message("HEARTBEAT", {"base_mode": ARMED_FLAG})

    with pytest.raises(RuntimeError, match="Cube armed"):
        link.tick(
            SimpleNamespace(mav=Mav()),
            SimpleNamespace(mavlink=SimpleNamespace()),
            1,
            1,
        )

    assert link.armed_interlock_triggered


def test_stale_odometry_is_not_repeated() -> None:
    state = _healthy_state()
    packet, reason = state.candidate(time.monotonic_ns() + 1_000_000_000)

    assert packet is None
    assert reason == "odometry_stale"


def test_raw_monitor_pose_continues_after_guarded_attitude_fault() -> None:
    state = _healthy_state()
    now_ns = time.monotonic_ns()
    yaw = math.radians(20.0) * 0.5
    quaternion = (0.0, 0.0, math.sin(yaw), math.cos(yaw))

    state.update_odometry(
        host_monotonic_ns=now_ns,
        ros_time_ns=1_200_000_000,
        frame_id="camera_init",
        child_frame_id="body",
        position_m=(1.0, 2.2, 3.0),
        quaternion_xyzw=quaternion,
        pose_covariance=(0.0,) * 36,
    )
    packet, reason = state.latest_healthy(now_ns)
    assert packet is None
    assert reason.startswith("attitude_jump:")
    fault_observation = state.latest_observed()
    assert fault_observation is not None

    state.update_odometry(
        host_monotonic_ns=now_ns + 100_000_000,
        ros_time_ns=1_300_000_000,
        frame_id="camera_init",
        child_frame_id="body",
        position_m=(1.0, 2.3, 3.0),
        quaternion_xyzw=quaternion,
        pose_covariance=(0.0,) * 36,
    )
    latest_observation = state.latest_observed()

    assert latest_observation is not None
    assert latest_observation.sequence == fault_observation.sequence + 1
    assert latest_observation.position_local_frd_m == pytest.approx(
        (0.0, 0.3, 0.0)
    )
    packet, repeated_reason = state.latest_healthy(now_ns + 100_000_000)
    assert packet is None
    assert repeated_reason == reason


def test_imu_supported_attitude_motion_does_not_latch_false_jump() -> None:
    state = OdometryShadowState(
        stale_timeout_s=0.5,
        maximum_position_jump_m=0.5,
        maximum_attitude_jump_deg=10.0,
        minimum_samples=2,
    )
    now_ns = time.monotonic_ns()
    diagnostics = {
        "synchronized": True,
        "publishing": True,
        "imu": {
            "connected": True,
            "error": None,
            "checksum_errors": 0,
            "payload_errors": 0,
        },
        "lidar": {
            "connected": True,
            "error": None,
            "non_monotonic_frames": 0,
        },
    }
    state.update_diagnostics(now_ns, diagnostics)
    angular_rate = math.radians(120.0)
    state.update_imu(now_ns, (0.0, 0.0, angular_rate))
    state.update_odometry(
        host_monotonic_ns=now_ns,
        ros_time_ns=1_000_000_000,
        frame_id="camera_init",
        child_frame_id="body",
        position_m=(0.0, 0.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        pose_covariance=(0.0,) * 36,
    )
    state.update_imu(now_ns + 50_000_000, (0.0, 0.0, angular_rate))
    state.update_imu(now_ns + 100_000_000, (0.0, 0.0, angular_rate))
    yaw = math.radians(12.0) * 0.5
    state.update_odometry(
        host_monotonic_ns=now_ns + 100_000_000,
        ros_time_ns=1_100_000_000,
        frame_id="camera_init",
        child_frame_id="body",
        position_m=(0.0, 0.1, 0.0),
        quaternion_xyzw=(0.0, 0.0, math.sin(yaw), math.cos(yaw)),
        pose_covariance=(0.0,) * 36,
    )

    packet, reason = state.latest_healthy(now_ns + 100_000_000)

    assert packet is not None
    assert reason == "ready"
    status = state.status(now_ns + 100_000_000)
    assert status["last_attitude_jump_deg"] == pytest.approx(12.0)
    assert status["last_gyro_motion_deg"] == pytest.approx(12.0)
    assert status["last_attitude_limit_deg"] > 12.0


def test_unexplained_attitude_motion_still_latches_jump() -> None:
    state = _healthy_state()
    now_ns = time.monotonic_ns()
    yaw = math.radians(12.0) * 0.5
    state.update_odometry(
        host_monotonic_ns=now_ns,
        ros_time_ns=1_200_000_000,
        frame_id="camera_init",
        child_frame_id="body",
        position_m=(1.0, 2.2, 3.0),
        quaternion_xyzw=(0.0, 0.0, math.sin(yaw), math.cos(yaw)),
        pose_covariance=(0.0,) * 36,
    )

    packet, reason = state.latest_healthy(now_ns)

    assert packet is None
    assert reason.startswith("attitude_jump:")
