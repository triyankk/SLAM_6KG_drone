"""Disarmed-only MAVLink ODOMETRY shadow output for Cube bench proving."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import threading
import time
from typing import Any, Protocol, Sequence

import numpy as np


MAV_FRAME_BODY_FRD = 12
MAV_FRAME_LOCAL_FRD = 20
MAV_ESTIMATOR_TYPE_LIDAR = 7
EXTERNAL_NAV_SOURCE = 6
ARMED_FLAG = 128
GYRO_ATTITUDE_MARGIN_DEG = 3.0
GYRO_ATTITUDE_SCALE = 1.5
MAXIMUM_GYRO_SUPPORTED_ATTITUDE_JUMP_DEG = 30.0

SOURCE_AXES = ("POSXY", "VELXY", "POSZ", "VELZ", "YAW")
SOURCE_PARAMETERS = tuple(
    f"EK3_SRC{source_set}_{axis}"
    for source_set in (1, 2, 3)
    for axis in SOURCE_AXES
)
AUDIT_PARAMETERS = ("AHRS_EKF_TYPE", "EK3_ENABLE", *SOURCE_PARAMETERS)


class RowWriter(Protocol):
    def write(self, row: dict[str, Any]) -> None: ...


@dataclass(frozen=True)
class OdometryPacket:
    sequence: int
    time_usec: int
    position_local_frd_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    velocity_body_frd_mps: tuple[float, float, float]
    angular_velocity_body_frd_rads: tuple[float, float, float]
    pose_error: tuple[float, ...]
    velocity_covariance: tuple[float, ...]
    reset_counter: int
    quality: int


@dataclass(frozen=True)
class OdometryObservation:
    """Raw local pose for monitoring; never eligible for Cube output."""

    sequence: int
    host_monotonic_ns: int
    position_local_frd_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


def _vector(values: Sequence[float], length: int, label: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {length} finite values")
    return result


def _quaternion_matrix_xyzw(values: Sequence[float]) -> np.ndarray:
    quaternion = _vector(values, 4, "quaternion")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        raise ValueError("quaternion norm is zero")
    x_value, y_value, z_value, w_value = quaternion / norm
    return np.asarray(
        (
            (
                1.0 - 2.0 * (y_value * y_value + z_value * z_value),
                2.0 * (x_value * y_value - z_value * w_value),
                2.0 * (x_value * z_value + y_value * w_value),
            ),
            (
                2.0 * (x_value * y_value + z_value * w_value),
                1.0 - 2.0 * (x_value * x_value + z_value * z_value),
                2.0 * (y_value * z_value - x_value * w_value),
            ),
            (
                2.0 * (x_value * z_value - y_value * w_value),
                2.0 * (y_value * z_value + x_value * w_value),
                1.0 - 2.0 * (x_value * x_value + y_value * y_value),
            ),
        ),
        dtype=np.float64,
    )


def _matrix_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = np.asarray(
            (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            )
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(
                max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            ) * 2.0
            values = np.asarray(
                (
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                )
            )
        elif index == 1:
            scale = math.sqrt(
                max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            ) * 2.0
            values = np.asarray(
                (
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                )
            )
        else:
            scale = math.sqrt(
                max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            ) * 2.0
            values = np.asarray(
                (
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                )
            )
    norm = float(np.linalg.norm(values))
    if norm < 1.0e-9:
        raise ValueError("rotation produced a zero quaternion")
    values /= norm
    if values[3] < 0.0:
        values *= -1.0
    return values


def _yaw_rotation_to_local(rotation_world_body: np.ndarray) -> np.ndarray:
    yaw = math.atan2(rotation_world_body[1, 0], rotation_world_body[0, 0])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.asarray(
        ((cosine, sine, 0.0), (-sine, cosine, 0.0), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )


def _rotation_difference_deg(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(
        np.clip((np.trace(first.T @ second) - 1.0) * 0.5, -1.0, 1.0)
    )
    return math.degrees(math.acos(cosine))


def _pose_error(
    covariance: Sequence[float],
    world_to_local: np.ndarray,
) -> tuple[float, ...]:
    values = np.asarray(covariance, dtype=np.float64)
    if values.shape != (36,) or not np.all(np.isfinite(values)):
        values = np.zeros(36, dtype=np.float64)
    matrix = values.reshape(6, 6)
    position_covariance = world_to_local @ matrix[:3, :3] @ world_to_local.T
    attitude_covariance = world_to_local @ matrix[3:, 3:] @ world_to_local.T
    position_floor_m = (0.25, 0.25, 0.35)
    attitude_floor_rad = (0.08, 0.08, 0.15)
    errors = [0.0] * 21
    for index, diagonal, floor in zip(
        (0, 6, 11), np.diag(position_covariance), position_floor_m
    ):
        errors[index] = max(float(floor), math.sqrt(max(0.0, float(diagonal))))
    for index, diagonal, floor in zip(
        (15, 18, 20), np.diag(attitude_covariance), attitude_floor_rad
    ):
        errors[index] = max(float(floor), math.sqrt(max(0.0, float(diagonal))))
    return tuple(errors)


class OdometryShadowState:
    """Convert healthy FAST-LIO samples into a guarded local-FRD stream."""

    def __init__(
        self,
        *,
        stale_timeout_s: float,
        maximum_position_jump_m: float,
        maximum_attitude_jump_deg: float,
        minimum_samples: int = 5,
        quality: int = 50,
    ) -> None:
        if stale_timeout_s <= 0.0:
            raise ValueError("stale timeout must be positive")
        if minimum_samples < 2:
            raise ValueError("minimum samples must be at least two")
        if not 1 <= quality <= 100:
            raise ValueError("quality must be between 1 and 100")
        self.stale_timeout_ns = int(stale_timeout_s * 1.0e9)
        self.maximum_position_jump_m = maximum_position_jump_m
        self.maximum_attitude_jump_deg = maximum_attitude_jump_deg
        self.minimum_samples = minimum_samples
        self.quality = quality
        self._lock = threading.Lock()
        self._origin_position: np.ndarray | None = None
        self._world_to_local: np.ndarray | None = None
        self._previous_position: np.ndarray | None = None
        self._previous_rotation: np.ndarray | None = None
        self._previous_ros_time_ns: int | None = None
        self._filtered_velocity_local = np.zeros(3, dtype=np.float64)
        self._imu_rates = np.zeros(3, dtype=np.float64)
        self._imu_host_ns: int | None = None
        self._previous_imu_rates: np.ndarray | None = None
        self._gyro_motion_since_odometry_rad = 0.0
        self._last_attitude_jump_deg: float | None = None
        self._last_gyro_motion_deg = 0.0
        self._last_attitude_limit_deg = maximum_attitude_jump_deg
        self._diagnostics_host_ns: int | None = None
        self._diagnostics_healthy = False
        self._diagnostics_detail = "waiting_for_diagnostics"
        self._latest: OdometryPacket | None = None
        self._latest_host_ns: int | None = None
        self._samples = 0
        self._sequence = 0
        self._last_sent_sequence = 0
        self._fault: str | None = None
        self._observed_sequence = 0
        self._observed_latest: OdometryObservation | None = None

    def update_imu(
        self, host_monotonic_ns: int, angular_velocity_rads: Sequence[float]
    ) -> None:
        rates = _vector(angular_velocity_rads, 3, "angular velocity")
        sample_ns = int(host_monotonic_ns)
        with self._lock:
            if self._imu_host_ns is not None and self._previous_imu_rates is not None:
                interval_s = (sample_ns - self._imu_host_ns) / 1.0e9
                if 0.0 < interval_s <= 0.25:
                    mean_rate_rads = 0.5 * (
                        float(np.linalg.norm(self._previous_imu_rates))
                        + float(np.linalg.norm(rates))
                    )
                    self._gyro_motion_since_odometry_rad += (
                        mean_rate_rads * interval_s
                    )
            self._imu_rates = rates
            self._previous_imu_rates = rates.copy()
            self._imu_host_ns = sample_ns

    def update_diagnostics(
        self, host_monotonic_ns: int, diagnostics: dict[str, Any]
    ) -> None:
        imu = diagnostics.get("imu", {})
        lidar = diagnostics.get("lidar", {})
        healthy = bool(
            diagnostics.get("synchronized")
            and diagnostics.get("publishing")
            and imu.get("connected")
            and lidar.get("connected")
            and not imu.get("error")
            and not lidar.get("error")
            and int(imu.get("checksum_errors", 0)) == 0
            and int(imu.get("payload_errors", 0)) == 0
            and int(lidar.get("non_monotonic_frames", 0)) == 0
        )
        detail = "healthy" if healthy else "sensor_or_clock_health_failed"
        with self._lock:
            self._diagnostics_host_ns = int(host_monotonic_ns)
            self._diagnostics_healthy = healthy
            self._diagnostics_detail = detail

    def update_odometry(
        self,
        *,
        host_monotonic_ns: int,
        ros_time_ns: int,
        frame_id: str,
        child_frame_id: str,
        position_m: Sequence[float],
        quaternion_xyzw: Sequence[float],
        pose_covariance: Sequence[float],
    ) -> None:
        if frame_id != "camera_init" or child_frame_id != "body":
            raise ValueError(
                "FAST-LIO odometry must use camera_init -> body frames"
            )
        position = _vector(position_m, 3, "position")
        rotation_world_body = _quaternion_matrix_xyzw(quaternion_xyzw)
        host_ns = int(host_monotonic_ns)
        stamp_ns = int(ros_time_ns)
        with self._lock:
            if self._origin_position is None:
                self._origin_position = position.copy()
                self._world_to_local = _yaw_rotation_to_local(
                    rotation_world_body
                )
            assert self._world_to_local is not None
            assert self._origin_position is not None
            local_position = self._world_to_local @ (
                position - self._origin_position
            )
            local_rotation = self._world_to_local @ rotation_world_body
            quaternion_local = _matrix_quaternion_xyzw(local_rotation)
            self._observed_sequence += 1
            self._observed_latest = OdometryObservation(
                sequence=self._observed_sequence,
                host_monotonic_ns=host_ns,
                position_local_frd_m=tuple(
                    float(value) for value in local_position
                ),
                quaternion_wxyz=(
                    float(quaternion_local[3]),
                    float(quaternion_local[0]),
                    float(quaternion_local[1]),
                    float(quaternion_local[2]),
                ),
            )
            if self._fault is not None:
                return

            if self._previous_position is not None:
                jump_m = float(
                    np.linalg.norm(local_position - self._previous_position)
                )
                attitude_jump_deg = _rotation_difference_deg(
                    self._previous_rotation, local_rotation
                )
                gyro_motion_deg = math.degrees(
                    self._gyro_motion_since_odometry_rad
                )
                attitude_limit_deg = min(
                    MAXIMUM_GYRO_SUPPORTED_ATTITUDE_JUMP_DEG,
                    max(
                        self.maximum_attitude_jump_deg,
                        GYRO_ATTITUDE_MARGIN_DEG
                        + GYRO_ATTITUDE_SCALE * gyro_motion_deg,
                    ),
                )
                self._last_attitude_jump_deg = attitude_jump_deg
                self._last_gyro_motion_deg = gyro_motion_deg
                self._last_attitude_limit_deg = attitude_limit_deg
                if jump_m > self.maximum_position_jump_m:
                    self._fault = f"position_jump:{jump_m:.3f}m"
                    return
                if attitude_jump_deg > attitude_limit_deg:
                    self._fault = (
                        f"attitude_jump:{attitude_jump_deg:.2f}deg"
                    )
                    return
                assert self._previous_ros_time_ns is not None
                interval_s = (stamp_ns - self._previous_ros_time_ns) / 1.0e9
                if interval_s <= 0.0:
                    self._fault = "non_monotonic_odometry_timestamp"
                    return
                if interval_s <= 0.5:
                    measured_velocity = (
                        local_position - self._previous_position
                    ) / interval_s
                    self._filtered_velocity_local = (
                        0.4 * measured_velocity
                        + 0.6 * self._filtered_velocity_local
                    )

            body_velocity = local_rotation.T @ self._filtered_velocity_local
            imu_fresh = bool(
                self._imu_host_ns is not None
                and 0 <= host_ns - self._imu_host_ns <= 100_000_000
            )
            body_rates = self._imu_rates if imu_fresh else np.zeros(3)
            self._sequence += 1
            self._samples += 1
            self._latest = OdometryPacket(
                sequence=self._sequence,
                time_usec=max(0, stamp_ns // 1000),
                position_local_frd_m=tuple(float(value) for value in local_position),
                quaternion_wxyz=(
                    float(quaternion_local[3]),
                    float(quaternion_local[0]),
                    float(quaternion_local[1]),
                    float(quaternion_local[2]),
                ),
                velocity_body_frd_mps=tuple(
                    float(value) for value in body_velocity
                ),
                angular_velocity_body_frd_rads=tuple(
                    float(value) for value in body_rates
                ),
                pose_error=_pose_error(
                    pose_covariance, self._world_to_local
                ),
                velocity_covariance=(math.nan,) + (0.0,) * 20,
                reset_counter=0,
                quality=self.quality,
            )
            self._latest_host_ns = host_ns
            self._previous_position = local_position
            self._previous_rotation = local_rotation
            self._previous_ros_time_ns = stamp_ns
            self._gyro_motion_since_odometry_rad = 0.0

    def latest_observed(self) -> OdometryObservation | None:
        """Return display-only pose data even after a safety fault latches."""
        with self._lock:
            return self._observed_latest

    def candidate(
        self, now_monotonic_ns: int
    ) -> tuple[OdometryPacket | None, str]:
        packet, reason = self.latest_healthy(now_monotonic_ns)
        if packet is None:
            return None, reason
        with self._lock:
            if packet.sequence <= self._last_sent_sequence:
                return None, "waiting_for_new_odometry"
        return packet, "ready"

    def latest_healthy(
        self, now_monotonic_ns: int
    ) -> tuple[OdometryPacket | None, str]:
        """Return the latest fresh estimate without consuming its sequence."""
        now_ns = int(now_monotonic_ns)
        with self._lock:
            if self._fault is not None:
                return None, self._fault
            if self._samples < self.minimum_samples or self._latest is None:
                return None, "odometry_warmup"
            if self._latest_host_ns is None:
                return None, "odometry_missing"
            if now_ns - self._latest_host_ns > self.stale_timeout_ns:
                return None, "odometry_stale"
            if self._diagnostics_host_ns is None:
                return None, "diagnostics_missing"
            if now_ns - self._diagnostics_host_ns > 1_000_000_000:
                return None, "diagnostics_stale"
            if not self._diagnostics_healthy:
                return None, self._diagnostics_detail
            return self._latest, "ready"

    def mark_sent(self, sequence: int) -> None:
        with self._lock:
            self._last_sent_sequence = max(self._last_sent_sequence, sequence)

    def status(self, now_monotonic_ns: int | None = None) -> dict[str, Any]:
        now_ns = (
            time.monotonic_ns()
            if now_monotonic_ns is None
            else int(now_monotonic_ns)
        )
        with self._lock:
            age_ms = (
                None
                if self._latest_host_ns is None
                else (now_ns - self._latest_host_ns) / 1.0e6
            )
            return {
                "samples": self._samples,
                "latest_age_ms": age_ms,
                "diagnostics_healthy": self._diagnostics_healthy,
                "diagnostics_detail": self._diagnostics_detail,
                "fault": self._fault,
                "last_attitude_jump_deg": self._last_attitude_jump_deg,
                "last_gyro_motion_deg": self._last_gyro_motion_deg,
                "last_attitude_limit_deg": self._last_attitude_limit_deg,
                "last_sent_sequence": self._last_sent_sequence,
                "observed_sequence": self._observed_sequence,
                "observed_age_ms": (
                    None
                    if self._observed_latest is None
                    else (
                        now_ns - self._observed_latest.host_monotonic_ns
                    )
                    / 1.0e6
                ),
            }


def _parameter_name(message_data: dict[str, Any]) -> str:
    value = message_data.get("param_id", "")
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")


class CubeOdometryShadowLink:
    """Audit Cube sources and send ODOMETRY that cannot be fused."""

    def __init__(
        self,
        state: OdometryShadowState,
        output: RowWriter,
        *,
        heartbeat_timeout_s: float,
        parameter_timeout_s: float = 15.0,
    ) -> None:
        self.state = state
        self.output = output
        self.heartbeat_timeout_ns = int(heartbeat_timeout_s * 1.0e9)
        self.parameter_timeout_ns = int(parameter_timeout_s * 1.0e9)
        self.parameters: dict[str, float] = {}
        self.last_heartbeat_ns: int | None = None
        self.first_heartbeat_ns: int | None = None
        self.next_parameter_request_ns = 0
        self.full_parameter_list_requested = False
        self.last_block_reason: str | None = None
        self.fatal_error: str | None = None
        self.armed_interlock_triggered = False
        self.packets_sent = 0
        self.first_packet_ns: int | None = None
        self.last_packet_ns: int | None = None
        self.maximum_packet_gap_s = 0.0

    def observe_message(
        self, message_type: str, message_data: dict[str, Any]
    ) -> None:
        now_ns = time.monotonic_ns()
        if message_type == "HEARTBEAT":
            self.last_heartbeat_ns = now_ns
            self.first_heartbeat_ns = self.first_heartbeat_ns or now_ns
            if int(message_data.get("base_mode", 0)) & ARMED_FLAG:
                self.armed_interlock_triggered = True
                self.fatal_error = (
                    "Cube armed; ODOMETRY shadow output stopped immediately"
                )
        elif message_type == "PARAM_VALUE":
            name = _parameter_name(message_data)
            if name in AUDIT_PARAMETERS:
                try:
                    self.parameters[name] = float(message_data["param_value"])
                except (KeyError, TypeError, ValueError):
                    return

    def _audit_error(self) -> str | None:
        if len(self.parameters) < len(AUDIT_PARAMETERS):
            return None
        if not math.isclose(
            self.parameters["AHRS_EKF_TYPE"], 3.0, abs_tol=0.01
        ):
            return "AHRS_EKF_TYPE is not EKF3"
        if self.parameters["EK3_ENABLE"] < 0.5:
            return "EKF3 is disabled"
        external = sorted(
            name
            for name in SOURCE_PARAMETERS
            if math.isclose(
                self.parameters[name], EXTERNAL_NAV_SOURCE, abs_tol=0.01
            )
        )
        if external:
            return (
                "ExternalNav is configured in Cube source sets: "
                + ", ".join(external)
            )
        return ""

    def _log_transition(self, reason: str) -> None:
        if reason == self.last_block_reason:
            return
        self.last_block_reason = reason
        self.output.write(
            {
                "schema_version": 1,
                "event": "state",
                "host_monotonic_ns": time.monotonic_ns(),
                "host_unix_ns": time.time_ns(),
                "state": reason,
            }
        )

    def _request_parameters(
        self,
        connection: Any,
        target_system: int,
        target_component: int,
        now_ns: int,
    ) -> None:
        if now_ns < self.next_parameter_request_ns:
            return
        for name in AUDIT_PARAMETERS:
            if name in self.parameters:
                continue
            connection.mav.param_request_read_send(
                target_system,
                target_component,
                name.encode("ascii"),
                -1,
            )
        self.next_parameter_request_ns = now_ns + 1_000_000_000

    def tick(
        self,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
    ) -> None:
        now_ns = time.monotonic_ns()
        if self.fatal_error is not None:
            raise RuntimeError(self.fatal_error)
        if self.last_heartbeat_ns is None:
            self._log_transition("waiting_for_cube_heartbeat")
            return
        if now_ns - self.last_heartbeat_ns > self.heartbeat_timeout_ns:
            self._log_transition("cube_heartbeat_stale")
            return
        audit_error = self._audit_error()
        if audit_error is None:
            self._request_parameters(
                connection,
                target_system,
                target_component,
                now_ns,
            )
            if (
                self.first_heartbeat_ns is not None
                and now_ns - self.first_heartbeat_ns > 5_000_000_000
                and not self.full_parameter_list_requested
            ):
                # Some ArduPilot builds occasionally omit one response to
                # repeated named reads. A single full-list request is still
                # read-only and gives the audit a deterministic fallback.
                connection.mav.param_request_list_send(
                    target_system,
                    target_component,
                )
                self.full_parameter_list_requested = True
            if (
                self.first_heartbeat_ns is not None
                and now_ns - self.first_heartbeat_ns
                > self.parameter_timeout_ns
            ):
                missing = sorted(set(AUDIT_PARAMETERS) - self.parameters.keys())
                self.fatal_error = (
                    "Cube source audit timed out: " + ", ".join(missing)
                )
                raise RuntimeError(self.fatal_error)
            self._log_transition("auditing_cube_ekf_sources")
            return
        if audit_error:
            self.fatal_error = audit_error
            raise RuntimeError(audit_error)

        packet, reason = self.state.candidate(now_ns)
        if packet is None:
            if reason != "waiting_for_new_odometry":
                self._log_transition(reason)
            return
        if not hasattr(connection.mav, "odometry_send"):
            self.fatal_error = "pymavlink MAVLink 2 ODOMETRY support is absent"
            raise RuntimeError(self.fatal_error)
        connection.mav.odometry_send(
            packet.time_usec,
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            *packet.position_local_frd_m,
            packet.quaternion_wxyz,
            *packet.velocity_body_frd_mps,
            *packet.angular_velocity_body_frd_rads,
            packet.pose_error,
            packet.velocity_covariance,
            packet.reset_counter,
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_LIDAR,
            packet.quality,
        )
        self.state.mark_sent(packet.sequence)
        if self.last_packet_ns is not None:
            self.maximum_packet_gap_s = max(
                self.maximum_packet_gap_s,
                (now_ns - self.last_packet_ns) / 1.0e9,
            )
        self.first_packet_ns = self.first_packet_ns or now_ns
        self.last_packet_ns = now_ns
        self.packets_sent += 1
        self.last_block_reason = None
        self.output.write(
            {
                "schema_version": 1,
                "event": "odometry_sent",
                "host_monotonic_ns": now_ns,
                "host_unix_ns": time.time_ns(),
                "sequence": packet.sequence,
                "time_usec": packet.time_usec,
                "frame_id": MAV_FRAME_LOCAL_FRD,
                "child_frame_id": MAV_FRAME_BODY_FRD,
                "position_local_frd_m": packet.position_local_frd_m,
                "quaternion_wxyz": packet.quaternion_wxyz,
                "velocity_body_frd_mps": packet.velocity_body_frd_mps,
                "angular_velocity_body_frd_rads": (
                    packet.angular_velocity_body_frd_rads
                ),
                "pose_error": packet.pose_error,
                "reset_counter": packet.reset_counter,
                "quality": packet.quality,
            }
        )

    def report(self) -> dict[str, Any]:
        duration_s = (
            None
            if self.first_packet_ns is None or self.last_packet_ns is None
            else (self.last_packet_ns - self.first_packet_ns) / 1.0e9
        )
        rate_hz = (
            None
            if duration_s is None or duration_s <= 0.0 or self.packets_sent < 2
            else (self.packets_sent - 1) / duration_s
        )
        audit_complete = len(self.parameters) == len(AUDIT_PARAMETERS)
        cube_ignores_external_nav = audit_complete and self._audit_error() == ""
        passed = bool(
            self.fatal_error is None
            and not self.armed_interlock_triggered
            and cube_ignores_external_nav
            and self.packets_sent >= 10
            and rate_hz is not None
            and rate_hz >= 4.0
        )
        return {
            "schema_version": 1,
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "result": "pass" if passed else "fail",
            "mode": "disarmed_odometry_shadow",
            "mavlink_version": 2,
            "frame_id": "MAV_FRAME_LOCAL_FRD",
            "child_frame_id": "MAV_FRAME_BODY_FRD",
            "estimator_type": "MAV_ESTIMATOR_TYPE_LIDAR",
            "cube_external_nav_fusion_enabled": False,
            "cube_ignores_external_nav_proven": cube_ignores_external_nav,
            "parameter_writes": 0,
            "full_parameter_list_fallback": (
                self.full_parameter_list_requested
            ),
            "source_parameter_audit": dict(sorted(self.parameters.items())),
            "missing_audit_parameters": sorted(
                set(AUDIT_PARAMETERS) - self.parameters.keys()
            ),
            "packets_sent": self.packets_sent,
            "packet_rate_hz": rate_hz,
            "maximum_packet_gap_s": self.maximum_packet_gap_s,
            "armed_interlock_triggered": self.armed_interlock_triggered,
            "fatal_error": self.fatal_error,
            "latest_block_reason": self.last_block_reason,
            "odometry": self.state.status(self.last_packet_ns),
        }
