"""Guarded local-SLAM breadcrumb return over Copter Guided velocity targets."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Protocol, Sequence

import numpy as np

from .config import ProjectConfig
from .cube_odometry import ARMED_FLAG, OdometryPacket, OdometryShadowState
from .obstacles import ObstacleFusion, ObstacleScan
from .paths import PROJECT_ROOT
from .rtl_shadow import LocalReturnShadow, ReturnSettings


VELOCITY_ONLY_TYPE_MASK = 3527
ZERO_COMMAND_HOLD_S = 1.0
STATUS_WRITE_RATE_HZ = 5.0
LAND_RC_OPTION = 18.0
MAXIMUM_TRAJECTORY_POINTS = 2400
STATUS_TEXT_HISTORY_LIMIT = 32
STATUS_TEXT_FRESHNESS_S = 60.0
RETURN_TUNE = "MFT200L16CE"
ABORT_TUNE = "MFT180L16C"
ARRIVAL_TUNE = "MFT200L16EG"
REHEARSAL_READY_TUNE = "MFT180L16CE"
REHEARSAL_OUTBOUND_TARGET_M = 5.0
REHEARSAL_OUTBOUND_TOLERANCE_M = 0.5
REHEARSAL_MAXIMUM_OUTBOUND_M = 6.0
REHEARSAL_MAXIMUM_LATERAL_ERROR_M = 0.75
REHEARSAL_MAXIMUM_VERTICAL_ERROR_M = 0.75
REHEARSAL_INITIAL_HOLD_S = 10.0
REHEARSAL_OUTBOUND_HOLD_S = 3.0
REHEARSAL_PROMPT_REPEAT_S = 5.0
BASE_AUDIT_PARAMETERS = (
    "AHRS_EKF_TYPE",
    "EK3_ENABLE",
    "FLOW_TYPE",
    "RNGFND1_TYPE",
    "FLTMODE_CH",
    "GUID_OPTIONS",
    "GUID_TIMEOUT",
    "BATT_MONITOR",
)


class RowWriter(Protocol):
    def write(self, row: dict[str, Any]) -> None: ...


@dataclass(frozen=True)
class ControlDecision:
    timestamp_ns: int
    state: str
    transmit: bool
    velocity_local_flu_mps: tuple[float, float, float]
    velocity_local_ned_mps: tuple[float, float, float]
    reason: str | None
    hold_zero: bool = False


def _point(values: Sequence[float]) -> np.ndarray | None:
    try:
        point = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if point.shape != (3,) or not np.isfinite(point).all():
        return None
    return point


def _quaternion_yaw_wxyz(values: Sequence[float]) -> float | None:
    try:
        quaternion = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        return None
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        return None
    w_value, x_value, y_value, z_value = quaternion / norm
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _age_ok(now_ns: int, sample_ns: int | None, timeout_s: float) -> bool:
    return bool(
        sample_ns is not None
        and 0 <= now_ns - sample_ns <= round(timeout_s * 1.0e9)
    )


def _append_path(path: deque[list[float]], point: np.ndarray) -> None:
    values = [float(value) for value in point]
    if not path or math.dist(path[-1], values) >= 0.025:
        path.append(values)


def _decode_status_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (bytes, bytearray, memoryview)):
        text = bytes(value).decode("utf-8", errors="replace")
    else:
        text = str(value)
    return text.rstrip("\x00").strip()


def _resolved_project_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _altitude_envelope_label(minimum_m: float, maximum_m: float) -> str:
    def format_value(value: float) -> str:
        return str(int(value)) if value.is_integer() else f"{value:g}"

    return f"{format_value(minimum_m)}-{format_value(maximum_m)}M"


def live_control_approval(
    config: ProjectConfig,
    config_path: Path,
) -> tuple[bool, str]:
    """Require both conservative config gates and an explicit approval file."""
    settings = config.navigation.slam_return
    if not config.navigation.autonomous_control_enabled:
        return False, "navigation_output_locked"
    if not settings.live_control_enabled or settings.stage != "active":
        return False, "slam_return_output_locked"
    approval_path = _resolved_project_path(settings.approval_file)
    try:
        approval = json.loads(approval_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return False, "approval_file_missing_or_invalid"
    if not isinstance(approval, dict) or approval.get("approved") is not True:
        return False, "approval_not_granted"
    digest = hashlib.sha256(config_path.resolve().read_bytes()).hexdigest()
    if approval.get("config_sha256") != digest:
        return False, "approval_config_digest_mismatch"
    flight_digest = str(approval.get("flight_report_sha256", ""))
    if len(flight_digest) != 64:
        return False, "approval_flight_report_missing"
    return True, "approved"


def return_settings(config: ProjectConfig) -> ReturnSettings:
    settings = config.navigation.slam_return
    return ReturnSettings(
        maximum_horizontal_speed_mps=settings.maximum_horizontal_speed_mps,
        maximum_horizontal_acceleration_mpss=(
            settings.maximum_horizontal_acceleration_mpss
        ),
        arrival_radius_m=settings.arrival_radius_m,
        breadcrumb_spacing_m=settings.breadcrumb_spacing_m,
        waypoint_radius_m=settings.waypoint_radius_m,
        visual_stale_timeout_s=settings.visual_stale_timeout_s,
        visual_disagreement_limit_m=(
            settings.visual_disagreement_limit_m
        ),
        command_timeout_s=config.navigation.command_stale_timeout_s,
    )


def audit_parameter_names(settings: Any) -> tuple[str, ...]:
    source = settings.ekf_source_set
    return (
        *BASE_AUDIT_PARAMETERS,
        f"RC{settings.rc_channel}_OPTION",
        f"RC{settings.land_rc_channel}_OPTION",
        f"EK3_SRC{source}_POSXY",
        f"EK3_SRC{source}_VELXY",
        f"EK3_SRC{source}_POSZ",
        f"EK3_SRC{source}_VELZ",
        f"EK3_SRC{source}_YAW",
    )


def audit_cube_parameters(
    parameters: dict[str, float], settings: Any
) -> tuple[dict[str, bool], str]:
    names = audit_parameter_names(settings)
    missing = sorted(set(names) - parameters.keys())
    if missing:
        return {}, "waiting_for_parameters:" + ",".join(missing)
    source = settings.ekf_source_set
    parameter = lambda axis: parameters[f"EK3_SRC{source}_{axis}"]
    rc_option = parameters[f"RC{settings.rc_channel}_OPTION"]
    land_rc_option = parameters[f"RC{settings.land_rc_channel}_OPTION"]
    guid_options = int(round(parameters["GUID_OPTIONS"]))
    gates = {
        "ekf3_selected": math.isclose(
            parameters["AHRS_EKF_TYPE"], 3.0, abs_tol=0.01
        ),
        "ekf3_enabled": parameters["EK3_ENABLE"] >= 0.5,
        "horizontal_position_none": math.isclose(
            parameter("POSXY"), 0.0, abs_tol=0.01
        ),
        "horizontal_velocity_optical_flow": math.isclose(
            parameter("VELXY"), 5.0, abs_tol=0.01
        ),
        "vertical_position_barometer": math.isclose(
            parameter("POSZ"), 1.0, abs_tol=0.01
        ),
        "vertical_velocity_none": math.isclose(
            parameter("VELZ"), 0.0, abs_tol=0.01
        ),
        "yaw_compass": math.isclose(
            parameter("YAW"), 1.0, abs_tol=0.01
        ),
        "flow_driver_enabled": parameters["FLOW_TYPE"] > 0.0,
        "rangefinder_enabled": parameters["RNGFND1_TYPE"] > 0.0,
        "return_rc_channel_unassigned": math.isclose(
            rc_option, 0.0, abs_tol=0.01
        ),
        "land_rc_channel_assigned": math.isclose(
            land_rc_option, LAND_RC_OPTION, abs_tol=0.01
        ),
        "return_rc_not_flight_mode": not math.isclose(
            parameters["FLTMODE_CH"],
            float(settings.rc_channel),
            abs_tol=0.01,
        ),
        "land_rc_not_flight_mode": not math.isclose(
            parameters["FLTMODE_CH"],
            float(settings.land_rc_channel),
            abs_tol=0.01,
        ),
        "guided_xy_stabilization_enabled": (guid_options & 48) == 0,
        "guided_timeout_bounded": 0.1 <= parameters["GUID_TIMEOUT"] <= 1.0,
        "battery_monitor_enabled": parameters["BATT_MONITOR"] > 0.0,
    }
    detail = "ready" if all(gates.values()) else "cube_parameter_gate_failed"
    return gates, detail


class SlamReturnController:
    """Own the arm-cycle, RC-intent, health, and breadcrumb-return state."""

    def __init__(
        self,
        config: ProjectConfig,
        *,
        control_permitted: bool,
        approval_reason: str,
        clock_ns: Any = time.monotonic_ns,
    ) -> None:
        self.config = config
        self.settings = config.navigation.slam_return
        self.control_permitted = bool(control_permitted)
        self.approval_reason = approval_reason
        self._clock_ns = clock_ns
        self._lock = threading.RLock()
        self._planner_settings = return_settings(config)
        self._planner = LocalReturnShadow(self._planner_settings)
        self._obstacle_fusion = ObstacleFusion(config.obstacle_avoidance)
        self._state = "waiting_for_cube"
        self._abort_reason: str | None = None
        self._abort_latched = False
        self._zero_until_ns = 0
        self._command_started = False
        self._armed = False
        self._armed_once = False
        self._boot_disarmed_seen = False
        self._mode = "UNKNOWN"
        self._heartbeat_ns: int | None = None
        self._yaw_ned_rad: float | None = None
        self._roll_rad: float | None = None
        self._pitch_rad: float | None = None
        self._attitude_ns: int | None = None
        self._flow_quality: int | None = None
        self._flow_ns: int | None = None
        self._range_m: float | None = None
        self._range_ns: int | None = None
        self._rc_pwm: int | None = None
        self._rc_ns: int | None = None
        self._rc_low_seen = False
        self._battery_voltage_v: float | None = None
        self._battery_ns: int | None = None
        self._cube_status_texts: deque[dict[str, Any]] = deque(
            maxlen=STATUS_TEXT_HISTORY_LIMIT
        )
        self._last_arm_ns: int | None = None
        self._cube_local_ned: np.ndarray | None = None
        self._cube_local_ns: int | None = None
        self._cube_origin_ned: np.ndarray | None = None
        self._ekf_origin_seen = False
        self._pose: np.ndarray | None = None
        self._pose_ns: int | None = None
        self._pose_sequence = 0
        self._pose_reason = "odometry_warmup"
        self._lio_local_yaw_rad: float | None = None
        self._frame_yaw_ned_rad: float | None = None
        self._monitor_pose: np.ndarray | None = None
        self._monitor_pose_ns: int | None = None
        self._monitor_pose_sequence = 0
        self._visual: np.ndarray | None = None
        self._visual_ns: int | None = None
        self._visual_tracking = False
        self._obstacle_ns: int | None = None
        self._nearest_obstacle_m: float | None = None
        self._clearance_breached = False
        self._parameter_gates: dict[str, bool] = {}
        self._parameter_detail = "parameter_audit_pending"
        self._last_row: dict[str, Any] | None = None
        self._last_decision: ControlDecision | None = None
        self._rehearsal_airborne_since_ns: int | None = None
        self._rehearsal_outbound_hold_since_ns: int | None = None
        self._rehearsal_trigger_ready: bool | None = None
        self._lio_path: deque[list[float]] = deque(
            maxlen=MAXIMUM_TRAJECTORY_POINTS
        )
        self._monitor_lio_path: deque[list[float]] = deque(
            maxlen=MAXIMUM_TRAJECTORY_POINTS
        )
        self._visual_path: deque[list[float]] = deque(
            maxlen=MAXIMUM_TRAJECTORY_POINTS
        )
        self._cube_path: deque[list[float]] = deque(
            maxlen=MAXIMUM_TRAJECTORY_POINTS
        )

    def set_parameter_audit(
        self, gates: dict[str, bool], detail: str
    ) -> None:
        with self._lock:
            self._parameter_gates = dict(gates)
            self._parameter_detail = detail

    def set_pose_health(self, reason: str) -> None:
        with self._lock:
            self._pose_reason = reason

    def observe_transport_disconnect(self, reason: str) -> None:
        now_ns = self._clock_ns()
        with self._lock:
            self._heartbeat_ns = None
            self._attitude_ns = None
            self._flow_ns = None
            self._range_ns = None
            self._rc_ns = None
            self._battery_ns = None
            self._cube_local_ns = None
            if self._armed:
                self._abort_locked(reason, now_ns)

    def observe_pose(self, packet: OdometryPacket, host_ns: int) -> None:
        position_frd = np.asarray(
            packet.position_local_frd_m, dtype=np.float64
        )
        point_flu = np.asarray(
            (position_frd[0], -position_frd[1], -position_frd[2]),
            dtype=np.float64,
        )
        yaw = _quaternion_yaw_wxyz(packet.quaternion_wxyz)
        with self._lock:
            if packet.sequence <= self._pose_sequence:
                return
            self._pose = point_flu
            self._pose_ns = int(host_ns)
            self._pose_sequence = packet.sequence
            self._pose_reason = "ready"
            self._lio_local_yaw_rad = yaw
            _append_path(self._lio_path, point_flu)
            self._capture_launch_locked(int(host_ns))
            if not self._armed or self._planner.launch_lio is None:
                return
            if self._planner.state == "recording_outbound":
                self._planner.observe_outbound(host_ns, point_flu)
            elif self._planner.state in {"returning", "arrived"}:
                self._last_row = self._planner.observe_return(
                    host_ns, point_flu
                )
                if self._planner.state == "arrived":
                    self._state = "arrived"

    def observe_monitor_pose(
        self,
        position_local_frd_m: Sequence[float],
        sequence: int,
        host_ns: int,
    ) -> None:
        """Record raw LIO motion for display without feeding navigation."""
        position_frd = np.asarray(position_local_frd_m, dtype=np.float64)
        if position_frd.shape != (3,) or not np.isfinite(position_frd).all():
            return
        point_flu = np.asarray(
            (position_frd[0], -position_frd[1], -position_frd[2]),
            dtype=np.float64,
        )
        with self._lock:
            if int(sequence) <= self._monitor_pose_sequence:
                return
            self._monitor_pose = point_flu
            self._monitor_pose_ns = int(host_ns)
            self._monitor_pose_sequence = int(sequence)
            _append_path(self._monitor_lio_path, point_flu)

    def observe_visual(self, row: dict[str, Any]) -> None:
        point = _point(row.get("position_local_flu_m", ()))
        if point is None:
            return
        now_ns = self._clock_ns()
        sample_ns = row.get("host_monotonic_ns")
        if not isinstance(sample_ns, int):
            sample_ns = now_ns
        tracking = bool(row.get("tracking"))
        with self._lock:
            self._visual = point
            self._visual_ns = int(sample_ns)
            self._visual_tracking = tracking
            if tracking:
                _append_path(self._visual_path, point)
            self._planner.update_visual(
                sample_ns, point, tracking=tracking
            )

    def observe_obstacle(self, scan: ObstacleScan) -> None:
        with self._lock:
            self._obstacle_fusion.update(scan)
            fused = self._obstacle_fusion.fused(
                monotonic_ns=scan.monotonic_ns
            )
            if fused is None:
                return
            clearance = fused.assess_clearance(
                self.config.obstacle_avoidance.hard_cg_clearance_m
            )
            self._obstacle_ns = int(fused.monotonic_ns)
            self._nearest_obstacle_m = fused.nearest_distance_m
            self._clearance_breached = clearance.breached

    def observe_cube(
        self, message_type: str, data: dict[str, Any]
    ) -> None:
        now_ns = self._clock_ns()
        with self._lock:
            if message_type == "HEARTBEAT":
                armed = bool(int(data.get("base_mode", 0)) & ARMED_FLAG)
                self._heartbeat_ns = now_ns
                self._mode = str(data.get("_mode_name", self._mode)).upper()
                if not armed:
                    self._boot_disarmed_seen = True
                if armed and not self._armed:
                    self._last_arm_ns = now_ns
                    self._begin_arm_cycle_locked()
                    if not self._boot_disarmed_seen:
                        self._abort_locked(
                            "armed_before_disarmed_boot_gate", now_ns
                        )
                elif not armed and self._armed:
                    self._state = "disarmed_after_flight"
                    self._zero_until_ns = 0
                    self._command_started = False
                self._armed = armed
                self._armed_once = self._armed_once or armed
                self._capture_launch_locked(now_ns)
            elif message_type == "ATTITUDE":
                try:
                    self._yaw_ned_rad = float(data["yaw"])
                    self._attitude_ns = now_ns
                except (KeyError, TypeError, ValueError):
                    pass
                try:
                    self._roll_rad = float(data["roll"])
                    self._pitch_rad = float(data["pitch"])
                except (KeyError, TypeError, ValueError):
                    pass
                self._capture_launch_locked(now_ns)
            elif message_type == "OPTICAL_FLOW":
                try:
                    self._flow_quality = int(data["quality"])
                    self._flow_ns = now_ns
                except (KeyError, TypeError, ValueError):
                    pass
            elif message_type == "DISTANCE_SENSOR":
                try:
                    if int(data.get("orientation", -1)) == 25:
                        self._range_m = float(data["current_distance"]) / 100.0
                        self._range_ns = now_ns
                except (KeyError, TypeError, ValueError):
                    pass
            elif message_type == "RC_CHANNELS":
                key = f"chan{self.settings.rc_channel}_raw"
                try:
                    value = int(data[key])
                except (KeyError, TypeError, ValueError):
                    return
                if 800 <= value <= 2200:
                    self._rc_pwm = value
                    self._rc_ns = now_ns
                    if self._armed and value <= self.settings.disengage_pwm:
                        self._rc_low_seen = True
            elif message_type == "SYS_STATUS":
                try:
                    millivolts = int(data.get("voltage_battery", 0))
                except (TypeError, ValueError):
                    millivolts = 0
                if 0 < millivolts < 65535:
                    self._battery_voltage_v = millivolts / 1000.0
                    self._battery_ns = now_ns
            elif message_type == "BATTERY_STATUS":
                voltages = data.get("voltages")
                if isinstance(voltages, (list, tuple)):
                    valid = []
                    for value in voltages:
                        try:
                            millivolts = int(value)
                        except (TypeError, ValueError):
                            continue
                        if 0 < millivolts < 65535:
                            valid.append(millivolts)
                    if valid:
                        self._battery_voltage_v = sum(valid) / 1000.0
                        self._battery_ns = now_ns
            elif message_type == "STATUSTEXT":
                text = _decode_status_text(data.get("text"))
                if text:
                    try:
                        severity = int(data.get("severity"))
                    except (TypeError, ValueError):
                        severity = None
                    row = {
                        "received_monotonic_ns": now_ns,
                        "severity": severity,
                        "text": text,
                    }
                    if (
                        self._cube_status_texts
                        and self._cube_status_texts[-1]["text"] == text
                    ):
                        self._cube_status_texts[-1] = row
                    else:
                        self._cube_status_texts.append(row)
            elif message_type == "LOCAL_POSITION_NED":
                point = _point(
                    (data.get("x"), data.get("y"), data.get("z"))
                )
                if point is not None:
                    self._cube_local_ned = point
                    self._cube_local_ns = now_ns
                    self._append_cube_path_locked(point)
            elif message_type == "GPS_GLOBAL_ORIGIN":
                self._ekf_origin_seen = True

    def _begin_arm_cycle_locked(self) -> None:
        self._planner = LocalReturnShadow(self._planner_settings)
        self._state = "armed_waiting_for_launch_pose"
        self._abort_reason = None
        self._abort_latched = False
        self._zero_until_ns = 0
        self._command_started = False
        self._rc_low_seen = bool(
            self._rc_pwm is not None
            and self._rc_pwm <= self.settings.disengage_pwm
        )
        self._frame_yaw_ned_rad = None
        self._cube_origin_ned = None
        self._last_row = None
        self._rehearsal_airborne_since_ns = None
        self._rehearsal_outbound_hold_since_ns = None
        self._rehearsal_trigger_ready = None

    def _capture_launch_locked(self, now_ns: int) -> None:
        if (
            not self._armed
            or self._abort_latched
            or self._planner.launch_lio is not None
            or self._pose is None
            or self._yaw_ned_rad is None
            or self._lio_local_yaw_rad is None
            or not _age_ok(
                now_ns,
                self._pose_ns,
                self.config.navigation.local_pose_stale_timeout_s,
            )
            or not _age_ok(
                now_ns,
                self._attitude_ns,
                self.settings.telemetry_stale_timeout_s,
            )
        ):
            return
        if not self._planner.capture_launch(
            now_ns, self._pose, self._visual
        ):
            return
        self._frame_yaw_ned_rad = (
            self._yaw_ned_rad - self._lio_local_yaw_rad
        )
        if self._cube_local_ned is not None:
            self._cube_origin_ned = self._cube_local_ned.copy()
        self._state = "recording_outbound"

    def _append_cube_path_locked(self, point_ned: np.ndarray) -> None:
        if self._cube_origin_ned is None or self._frame_yaw_ned_rad is None:
            return
        delta = point_ned - self._cube_origin_ned
        cosine = math.cos(self._frame_yaw_ned_rad)
        sine = math.sin(self._frame_yaw_ned_rad)
        forward = cosine * delta[0] + sine * delta[1]
        right = -sine * delta[0] + cosine * delta[1]
        _append_path(
            self._cube_path,
            np.asarray((forward, -right, -delta[2]), dtype=np.float64),
        )

    def _rc_high_locked(self) -> bool:
        return bool(
            self._rc_pwm is not None
            and self._rc_pwm >= self.settings.engage_pwm
        )

    def _health_gates_locked(self, now_ns: int) -> dict[str, bool]:
        telemetry_timeout = self.settings.telemetry_stale_timeout_s
        visual_fresh = _age_ok(
            now_ns, self._visual_ns, self.settings.visual_stale_timeout_s
        )
        obstacle_fresh = _age_ok(
            now_ns,
            self._obstacle_ns,
            self.config.obstacle_avoidance.source_stale_timeout_s,
        )
        parameter_audit = bool(
            self._parameter_gates and all(self._parameter_gates.values())
        )
        return {
            "armed": self._armed,
            "disarmed_seen_since_start": self._boot_disarmed_seen,
            "regular_guided_mode": self._mode == self.settings.required_mode,
            "cube_heartbeat_fresh": _age_ok(
                now_ns,
                self._heartbeat_ns,
                self.config.flight_controller.heartbeat_timeout_s,
            ),
            "rc_input_fresh": _age_ok(
                now_ns, self._rc_ns, telemetry_timeout
            ),
            "rc_return_switch_high": self._rc_high_locked(),
            "rc_low_seen_after_arm": self._rc_low_seen,
            "lio_pose_fresh": bool(
                self._pose_reason == "ready"
                and _age_ok(
                    now_ns,
                    self._pose_ns,
                    self.config.navigation.local_pose_stale_timeout_s,
                )
            ),
            "visual_tracking_fresh": bool(
                self._visual_tracking and visual_fresh
            ),
            "optical_flow_fresh": _age_ok(
                now_ns, self._flow_ns, telemetry_timeout
            ),
            "optical_flow_quality": bool(
                self._flow_quality is not None
                and self._flow_quality >= self.settings.minimum_flow_quality
            ),
            "downward_range_fresh": _age_ok(
                now_ns, self._range_ns, telemetry_timeout
            ),
            "altitude_in_envelope": bool(
                self._range_m is not None
                and self.settings.minimum_altitude_m
                <= self._range_m
                <= self.settings.maximum_altitude_m
            ),
            "cube_local_position_fresh": _age_ok(
                now_ns, self._cube_local_ns, telemetry_timeout
            ),
            "ekf_origin_seen": self._ekf_origin_seen,
            "battery_fresh": _age_ok(
                now_ns,
                self._battery_ns,
                self.settings.battery_stale_timeout_s,
            ),
            "battery_voltage": bool(
                self._battery_voltage_v is not None
                and self._battery_voltage_v >= self.settings.minimum_voltage_v
            ),
            "obstacle_scan_fresh": obstacle_fresh,
            "hard_clearance_clear": bool(
                obstacle_fresh and not self._clearance_breached
            ),
            "cube_parameter_audit": parameter_audit,
            "launch_pose_captured": self._planner.launch_lio is not None,
            "frame_alignment_captured": self._frame_yaw_ned_rad is not None,
            "abort_not_latched": not self._abort_latched,
        }

    def _rehearsal_status_locked(self, now_ns: int) -> dict[str, Any]:
        launch = self._planner.launch_lio
        delta = None if launch is None or self._pose is None else self._pose - launch
        forward_m = None if delta is None else float(delta[0])
        lateral_m = None if delta is None else float(delta[1])
        vertical_m = None if delta is None else float(delta[2])
        health = self._health_gates_locked(now_ns)
        required_health = {
            name: health[name]
            for name in (
                "cube_heartbeat_fresh",
                "rc_input_fresh",
                "rc_low_seen_after_arm",
                "lio_pose_fresh",
                "visual_tracking_fresh",
                "optical_flow_fresh",
                "optical_flow_quality",
                "downward_range_fresh",
                "altitude_in_envelope",
                "battery_fresh",
                "battery_voltage",
                "obstacle_scan_fresh",
                "hard_clearance_clear",
            )
        }
        failed_health = [
            name for name, passed in required_health.items() if not passed
        ]
        line_ready = bool(
            forward_m is not None
            and REHEARSAL_OUTBOUND_TARGET_M
            - REHEARSAL_OUTBOUND_TOLERANCE_M
            <= forward_m
            <= REHEARSAL_MAXIMUM_OUTBOUND_M
            and lateral_m is not None
            and abs(lateral_m) <= REHEARSAL_MAXIMUM_LATERAL_ERROR_M
            and vertical_m is not None
            and abs(vertical_m) <= REHEARSAL_MAXIMUM_VERTICAL_ERROR_M
        )
        if not self._armed or not health["altitude_in_envelope"]:
            self._rehearsal_airborne_since_ns = None
        elif self._rehearsal_airborne_since_ns is None:
            self._rehearsal_airborne_since_ns = now_ns
        initial_hold_elapsed_s = (
            0.0
            if self._rehearsal_airborne_since_ns is None
            else max(
                0.0,
                (now_ns - self._rehearsal_airborne_since_ns) / 1.0e9,
            )
        )
        initial_hold_complete = (
            initial_hold_elapsed_s >= REHEARSAL_INITIAL_HOLD_S
        )
        if (
            self._planner.state != "recording_outbound"
            or not line_ready
            or failed_health
        ):
            self._rehearsal_outbound_hold_since_ns = None
        elif self._rehearsal_outbound_hold_since_ns is None:
            self._rehearsal_outbound_hold_since_ns = now_ns
        outbound_hold_elapsed_s = (
            0.0
            if self._rehearsal_outbound_hold_since_ns is None
            else max(
                0.0,
                (now_ns - self._rehearsal_outbound_hold_since_ns) / 1.0e9,
            )
        )
        outbound_hold_complete = (
            outbound_hold_elapsed_s >= REHEARSAL_OUTBOUND_HOLD_S
        )
        trigger_conditions_ready = bool(
            self._planner.state == "recording_outbound"
            and line_ready
            and initial_hold_complete
            and outbound_hold_complete
            and not failed_health
        )
        ready_for_return_switch = bool(
            trigger_conditions_ready
            and not self._rc_high_locked()
        )

        phase = "preflight"
        return_switch = f"RC{self.settings.rc_channel}"
        altitude_envelope = _altitude_envelope_label(
            self.settings.minimum_altitude_m,
            self.settings.maximum_altitude_m,
        )
        instruction = (
            f"SLAM TEST: GPS ON, {return_switch} LOW, USE FLOWHOLD"
        )
        if self._armed:
            if self._abort_latched:
                phase = "blocked"
                instruction = "SLAM TEST BLOCKED: KEEP CONTROL AND LAND"
            elif launch is None or not health["altitude_in_envelope"]:
                phase = "launch_hold"
                instruction = (
                    f"SLAM TEST: HOLD {altitude_envelope} IN FLOWHOLD"
                )
            elif not initial_hold_complete:
                phase = "initial_hold"
                instruction = (
                    f"SLAM TEST: HOLD LEVEL 10S AT {altitude_envelope}"
                )
            elif self._planner.state == "recording_outbound":
                if self._rc_high_locked():
                    phase = "reset_return_switch"
                    instruction = (
                        f"SLAM TEST: {return_switch} LOW - NOT READY"
                    )
                elif forward_m is None or forward_m < (
                    REHEARSAL_OUTBOUND_TARGET_M
                    - REHEARSAL_OUTBOUND_TOLERANCE_M
                ):
                    phase = "outbound"
                    instruction = "SLAM TEST: FLY FORWARD SLOWLY TO 5M"
                elif not line_ready:
                    phase = "correct_line"
                    instruction = "SLAM TEST: CORRECT LINE AND HOLD AT 5M"
                elif failed_health:
                    phase = "hold_for_health"
                    instruction = "SLAM TEST: HOLD - WAIT FOR SENSOR READY"
                elif not outbound_hold_complete:
                    phase = "outbound_hold"
                    instruction = "SLAM TEST: HOLD LEVEL AT 5M FOR 3S"
                else:
                    phase = "ready_for_return_switch"
                    instruction = (
                        f"SLAM TEST: HOLD, THEN {return_switch} HIGH"
                    )
            elif self._planner.state == "returning":
                if self._rehearsal_trigger_ready is False:
                    phase = "invalid_trigger"
                    instruction = (
                        f"TEST INVALID: {return_switch} WAS EARLY - LAND"
                    )
                else:
                    phase = "shadow_return"
                    instruction = "SHADOW ONLY: FLY BACK IN FLOWHOLD"
            elif self._planner.state == "arrived":
                phase = "arrival"
                instruction = "AT START: HOLD, LAND MANUALLY"
        elif self._armed_once:
            phase = "complete"
            instruction = "SLAM TEST SAVED: DISARM CONFIRMED"

        return {
            "profile": "flowhold_5m_shadow_return",
            "phase": phase,
            "instruction": instruction,
            "outbound_target_m": REHEARSAL_OUTBOUND_TARGET_M,
            "outbound_maximum_m": REHEARSAL_MAXIMUM_OUTBOUND_M,
            "forward_m": forward_m,
            "lateral_m": lateral_m,
            "vertical_m": vertical_m,
            "line_ready": line_ready,
            "initial_hold_elapsed_s": initial_hold_elapsed_s,
            "initial_hold_complete": initial_hold_complete,
            "outbound_hold_elapsed_s": outbound_hold_elapsed_s,
            "outbound_hold_complete": outbound_hold_complete,
            "trigger_conditions_ready": trigger_conditions_ready,
            "return_rc_channel": self.settings.rc_channel,
            "ready_for_return_switch": ready_for_return_switch,
            "triggered_when_ready": self._rehearsal_trigger_ready,
            "profile_pass": bool(
                self._planner.state == "arrived"
                and self._rehearsal_trigger_ready is True
                and self._planner.maximum_excursion_m
                >= REHEARSAL_OUTBOUND_TARGET_M
                - REHEARSAL_OUTBOUND_TOLERANCE_M
            ),
            "required_health_gates": required_health,
            "failed_health_gates": failed_health,
            "movement_commands_sent": False,
            "automatic_landing_enabled": False,
        }

    def rehearsal_status(self, now_ns: int | None = None) -> dict[str, Any]:
        timestamp_ns = self._clock_ns() if now_ns is None else int(now_ns)
        with self._lock:
            return self._rehearsal_status_locked(timestamp_ns)

    def _abort_locked(self, reason: str, now_ns: int) -> None:
        if self._abort_latched:
            return
        self._abort_latched = True
        self._abort_reason = reason
        self._state = "aborted"
        if self._command_started:
            self._zero_until_ns = now_ns + round(
                ZERO_COMMAND_HOLD_S * 1.0e9
            )

    def _to_ned_locked(self, velocity_flu: np.ndarray) -> np.ndarray:
        if self._frame_yaw_ned_rad is None:
            return np.zeros(3, dtype=np.float64)
        cosine = math.cos(self._frame_yaw_ned_rad)
        sine = math.sin(self._frame_yaw_ned_rad)
        forward = float(velocity_flu[0])
        right = -float(velocity_flu[1])
        return np.asarray(
            (
                cosine * forward - sine * right,
                sine * forward + cosine * right,
                0.0,
            ),
            dtype=np.float64,
        )

    def step(self, now_ns: int | None = None) -> ControlDecision:
        timestamp_ns = self._clock_ns() if now_ns is None else int(now_ns)
        with self._lock:
            self._capture_launch_locked(timestamp_ns)
            zero = np.zeros(3, dtype=np.float64)
            if not self._armed:
                decision = ControlDecision(
                    timestamp_ns,
                    self._state,
                    False,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    "vehicle_disarmed",
                )
                self._last_decision = decision
                return decision

            if (
                self._planner.state == "recording_outbound"
                and self._rc_high_locked()
                and self._rc_low_seen
                and not self._abort_latched
            ):
                rehearsal = self._rehearsal_status_locked(timestamp_ns)
                self._rehearsal_trigger_ready = bool(
                    rehearsal["trigger_conditions_ready"]
                )
                if self._planner.begin_return(timestamp_ns):
                    self._state = (
                        "returning_live"
                        if self.control_permitted
                        else "returning_locked"
                    )

            if (
                self._planner.state in {"returning", "arrived"}
                and not self._rc_high_locked()
            ):
                self._abort_locked("pilot_cancelled_return", timestamp_ns)

            gates = self._health_gates_locked(timestamp_ns)
            failed = [name for name, passed in gates.items() if not passed]
            if (
                self.control_permitted
                and self._planner.state in {"returning", "arrived"}
                and failed
            ):
                self._abort_locked(failed[0], timestamp_ns)

            if self._abort_latched:
                transmit_zero = bool(
                    self.control_permitted
                    and self._command_started
                    and timestamp_ns <= self._zero_until_ns
                )
                decision = ControlDecision(
                    timestamp_ns,
                    self._state,
                    transmit_zero,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    self._abort_reason,
                    hold_zero=transmit_zero,
                )
                self._last_decision = decision
                return decision

            if self._planner.state not in {"returning", "arrived"}:
                decision = ControlDecision(
                    timestamp_ns,
                    self._state,
                    False,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    "waiting_for_return_request",
                )
                self._last_decision = decision
                return decision

            if self._last_row is None:
                decision = ControlDecision(
                    timestamp_ns,
                    self._state,
                    False,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    "waiting_for_return_odometry",
                )
                self._last_decision = decision
                return decision
            else:
                valid_until_ns = int(self._last_row.get("valid_until_ns", 0))
                reason = self._last_row.get("blocked_reason")
                if timestamp_ns > valid_until_ns:
                    reason = "return_command_stale"
                velocity_flu = np.asarray(
                    self._last_row.get(
                        "proposed_velocity_local_flu_mps", zero
                    ),
                    dtype=np.float64,
                )
                if reason is not None:
                    velocity_flu = zero
            if reason is not None:
                if not self.control_permitted:
                    decision = ControlDecision(
                        timestamp_ns,
                        self._state,
                        False,
                        tuple(float(value) for value in velocity_flu),
                        (0.0, 0.0, 0.0),
                        str(reason),
                    )
                    self._last_decision = decision
                    return decision
                self._abort_locked(str(reason), timestamp_ns)
                decision = ControlDecision(
                    timestamp_ns,
                    self._state,
                    False,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    str(reason),
                )
                self._last_decision = decision
                return decision

            velocity_ned = self._to_ned_locked(velocity_flu)
            transmit = bool(self.control_permitted)
            self._command_started = self._command_started or transmit
            if self._planner.state == "arrived":
                self._state = "arrived"
                velocity_flu = zero
                velocity_ned = zero
            decision = ControlDecision(
                timestamp_ns,
                self._state,
                transmit,
                tuple(float(value) for value in velocity_flu),
                tuple(float(value) for value in velocity_ned),
                None,
                hold_zero=self._planner.state == "arrived",
            )
            self._last_decision = decision
            return decision

    def snapshot(self, now_ns: int | None = None) -> dict[str, Any]:
        timestamp_ns = self._clock_ns() if now_ns is None else int(now_ns)
        with self._lock:
            gates = self._health_gates_locked(timestamp_ns)
            latest = self._last_row
            target = None if latest is None else latest.get("target_local_flu_m")
            recent_status_text_rows: list[tuple[dict[str, Any], int]] = []
            seen_status_texts: set[str] = set()
            freshness_ns = round(STATUS_TEXT_FRESHNESS_S * 1.0e9)
            for row in reversed(self._cube_status_texts):
                received_ns = int(row["received_monotonic_ns"])
                age_ns = timestamp_ns - received_ns
                if age_ns < 0 or age_ns > freshness_ns:
                    continue
                text = str(row["text"])
                key = text.casefold()
                if key in seen_status_texts:
                    continue
                seen_status_texts.add(key)
                recent_status_text_rows.append(
                    (
                        {
                            "text": text,
                            "severity": row["severity"],
                            "age_s": round(age_ns / 1.0e9, 2),
                        },
                        received_ns,
                    )
                )
            recent_status_texts = [
                row for row, _received_ns in recent_status_text_rows
            ]
            prearm_errors = [
                row
                for row, received_ns in recent_status_text_rows
                if row["text"].casefold().startswith(("prearm:", "arm:"))
                and (
                    self._last_arm_ns is None
                    or received_ns > self._last_arm_ns
                )
            ]
            return {
                "schema_version": 1,
                "updated_monotonic_ns": timestamp_ns,
                "updated_unix_ns": time.time_ns(),
                "kind": "gps_denied_slam_return",
                "stage": self.settings.stage,
                "state": self._state,
                "shadow_only": not self.control_permitted,
                "live_control_permitted": self.control_permitted,
                "approval_reason": self.approval_reason,
                "velocity_output_started": self._command_started,
                "abort_latched": self._abort_latched,
                "abort_reason": self._abort_reason,
                "cube": {
                    "armed": self._armed,
                    "armed_once": self._armed_once,
                    "disarmed_seen_since_start": self._boot_disarmed_seen,
                    "mode": self._mode,
                    "rc_channel": self.settings.rc_channel,
                    "rc_pwm": self._rc_pwm,
                    "rc_low_seen_after_arm": self._rc_low_seen,
                    "flow_quality": self._flow_quality,
                    "downward_range_m": self._range_m,
                    "battery_voltage_v": self._battery_voltage_v,
                    "status_text_window_s": STATUS_TEXT_FRESHNESS_S,
                    "latest_status_text": (
                        recent_status_texts[0]
                        if recent_status_texts
                        else None
                    ),
                    "recent_status_texts": recent_status_texts,
                    "prearm_errors": prearm_errors,
                    "ekf_origin_seen": self._ekf_origin_seen,
                    "roll_rad": self._roll_rad,
                    "pitch_rad": self._pitch_rad,
                    "yaw_rad": self._yaw_ned_rad,
                    "local_position_ned_m": (
                        None
                        if self._cube_local_ned is None
                        else self._cube_local_ned.tolist()
                    ),
                },
                "estimator": {
                    "pose_reason": self._pose_reason,
                    "pose_sequence": self._pose_sequence,
                    "position_local_flu_m": (
                        None if self._pose is None else self._pose.tolist()
                    ),
                    "frame_yaw_ned_rad": self._frame_yaw_ned_rad,
                    "visual_tracking": self._visual_tracking,
                    "monitor_pose_sequence": self._monitor_pose_sequence,
                    "monitor_position_local_flu_m": (
                        None
                        if self._monitor_pose is None
                        else self._monitor_pose.tolist()
                    ),
                    "monitor_pose_age_ms": (
                        None
                        if self._monitor_pose_ns is None
                        else (timestamp_ns - self._monitor_pose_ns) / 1.0e6
                    ),
                },
                "obstacles": {
                    "nearest_distance_m": self._nearest_obstacle_m,
                    "hard_clearance_m": (
                        self.config.obstacle_avoidance.hard_cg_clearance_m
                    ),
                    "clearance_breached": self._clearance_breached,
                },
                "parameter_audit": {
                    "detail": self._parameter_detail,
                    "gates": deepcopy(self._parameter_gates),
                },
                "health_gates": gates,
                "failed_health_gates": [
                    name for name, passed in gates.items() if not passed
                ],
                "command": (
                    None
                    if self._last_decision is None
                    else {
                        "transmit": self._last_decision.transmit,
                        "velocity_local_flu_mps": list(
                            self._last_decision.velocity_local_flu_mps
                        ),
                        "velocity_local_ned_mps": list(
                            self._last_decision.velocity_local_ned_mps
                        ),
                        "reason": self._last_decision.reason,
                        "hold_zero": self._last_decision.hold_zero,
                    }
                ),
                "flight_test": self._rehearsal_status_locked(timestamp_ns),
                "trajectories": {
                    "frame": "launch_local_flu",
                    "lio": list(self._lio_path),
                    "lio_monitor": list(self._monitor_lio_path),
                    "rgbd": list(self._visual_path),
                    "cube": list(self._cube_path),
                    "breadcrumbs": [
                        point.tolist() for point in self._planner.breadcrumbs
                    ],
                    "target": deepcopy(target),
                    "launch": (
                        None
                        if self._planner.launch_lio is None
                        else self._planner.launch_lio.tolist()
                    ),
                },
            }


class CubeGuidedVelocityLink:
    """Read-only audit plus an explicitly approved Guided velocity transport."""

    def __init__(
        self,
        controller: SlamReturnController,
        pose_state: OdometryShadowState,
        output: RowWriter,
        status_path: Path,
        *,
        heartbeat_timeout_s: float,
    ) -> None:
        self.controller = controller
        self.pose_state = pose_state
        self.output = output
        self.status_path = status_path
        self.heartbeat_timeout_ns = round(heartbeat_timeout_s * 1.0e9)
        self.audit_parameters = audit_parameter_names(controller.settings)
        self.parameters: dict[str, float] = {}
        self.last_heartbeat_ns: int | None = None
        self.first_heartbeat_ns: int | None = None
        self.next_parameter_request_ns = 0
        self.next_origin_request_ns = 0
        self._parameter_request_index = 0
        self._pending_parameter_name: str | None = None
        self._pending_parameter_since_ns: int | None = None
        self.parameter_requests_sent = 0
        self.origin_seen = False
        self._last_pose_sequence = 0
        self._last_monitor_pose_sequence = 0
        self._last_command_ns = 0
        self._last_status_write_ns = 0
        self._last_announced_state: str | None = None
        self._last_rehearsal_phase: str | None = None
        self._last_rehearsal_announce_ns = 0
        self._last_audit_detail: str | None = None
        self.commands_sent = 0
        self.zero_commands_sent = 0
        self.send_errors = 0
        self.full_parameter_list_requested = False
        self.parameter_list_requests_sent = 0

    @staticmethod
    def _parameter_name(data: dict[str, Any]) -> str:
        value = data.get("param_id", "")
        if isinstance(value, bytes):
            value = value.decode("ascii", errors="replace")
        return str(value).rstrip("\x00")

    def observe_message(
        self, message_type: str, message_data: dict[str, Any]
    ) -> None:
        if message_type == "PARAM_VALUE":
            name = self._parameter_name(message_data)
            if name in self.audit_parameters:
                try:
                    self.parameters[name] = float(message_data["param_value"])
                except (KeyError, TypeError, ValueError):
                    pass
                else:
                    if name == self._pending_parameter_name:
                        self._pending_parameter_name = None
                        self._pending_parameter_since_ns = None
                        self.next_parameter_request_ns = (
                            time.monotonic_ns() + 50_000_000
                        )
        elif message_type == "HEARTBEAT":
            now_ns = time.monotonic_ns()
            self.last_heartbeat_ns = now_ns
            self.first_heartbeat_ns = self.first_heartbeat_ns or now_ns
        elif message_type == "GPS_GLOBAL_ORIGIN":
            self.origin_seen = True
        self.controller.observe_cube(message_type, message_data)

    def observe_visual(self, row: dict[str, Any]) -> None:
        self.controller.observe_visual(row)

    def mark_disconnected(self, reason: str) -> None:
        self.controller.observe_transport_disconnect(reason)

    def observe_obstacle(self, scan: ObstacleScan) -> None:
        self.controller.observe_obstacle(scan)

    def _audit(self) -> tuple[dict[str, bool], str]:
        return audit_cube_parameters(
            self.parameters, self.controller.settings
        )

    def ready_for_stream_request(self) -> bool:
        return all(name in self.parameters for name in self.audit_parameters)

    def _request_parameters(
        self,
        connection: Any,
        target_system: int,
        target_component: int,
        now_ns: int,
    ) -> None:
        if (
            not self.full_parameter_list_requested
            and self.first_heartbeat_ns is not None
            and now_ns - self.first_heartbeat_ns >= 5_000_000_000
            and len(self.parameters) < len(self.audit_parameters)
        ):
            connection.mav.param_request_list_send(
                target_system, target_component
            )
            self.full_parameter_list_requested = True
            self.parameter_list_requests_sent += 1
        if now_ns < self.next_parameter_request_ns:
            return
        if self._pending_parameter_name is not None:
            pending_since_ns = self._pending_parameter_since_ns or now_ns
            if now_ns - pending_since_ns < 2_000_000_000:
                return
            self._pending_parameter_name = None
            self._pending_parameter_since_ns = None
        count = len(self.audit_parameters)
        for offset in range(count):
            index = (self._parameter_request_index + offset) % count
            name = self.audit_parameters[index]
            if name in self.parameters:
                continue
            connection.mav.param_request_read_send(
                target_system,
                target_component,
                name.encode("ascii"),
                -1,
            )
            self._parameter_request_index = (index + 1) % count
            self._pending_parameter_name = name
            self._pending_parameter_since_ns = now_ns
            self.parameter_requests_sent += 1
            break
        self.next_parameter_request_ns = now_ns + 50_000_000

    def _request_origin(
        self,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
        now_ns: int,
    ) -> None:
        if self.origin_seen or now_ns < self.next_origin_request_ns:
            return
        connection.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self.next_origin_request_ns = now_ns + 2_000_000_000

    def _write_status(self, now_ns: int, *, force: bool = False) -> None:
        period_ns = round(1.0e9 / STATUS_WRITE_RATE_HZ)
        if not force and now_ns - self._last_status_write_ns < period_ns:
            return
        payload = self.controller.snapshot(now_ns)
        payload["transport"] = {
            "kind": "SET_POSITION_TARGET_LOCAL_NED",
            "frame": "MAV_FRAME_LOCAL_NED",
            "type_mask": VELOCITY_ONLY_TYPE_MASK,
            "commands_sent": self.commands_sent,
            "zero_commands_sent": self.zero_commands_sent,
            "send_errors": self.send_errors,
            "parameter_writes": 0,
            "parameter_requests_sent": self.parameter_requests_sent,
            "parameter_list_requests_sent": self.parameter_list_requests_sent,
        }
        guard_status = getattr(self.pose_state, "status", None)
        payload["odometry_guard"] = (
            guard_status(now_ns) if callable(guard_status) else {}
        )
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.status_path.with_suffix(
            self.status_path.suffix + ".tmp"
        )
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.status_path)
        self._last_status_write_ns = now_ns

    def _announce(
        self,
        decision: ControlDecision,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
    ) -> None:
        if decision.state == self._last_announced_state:
            return
        self._last_announced_state = decision.state
        text = None
        tune = None
        severity = mavutil.mavlink.MAV_SEVERITY_NOTICE
        if decision.state == "returning_live":
            text = (
                "SLAM RETURN ACTIVE - "
                f"RC{self.controller.settings.rc_channel} LOW CANCELS"
            )
            tune = RETURN_TUNE
        elif decision.state == "returning_locked":
            text = "SLAM RETURN LOCKED - PROPOSALS ONLY"
        elif decision.state == "arrived":
            text = "SLAM RETURN ARRIVED - TAKE CONTROL AND LAND"
            tune = ARRIVAL_TUNE
        elif decision.state == "aborted":
            text = f"SLAM RETURN ABORT: {decision.reason or 'health gate'}"
            severity = mavutil.mavlink.MAV_SEVERITY_WARNING
            tune = ABORT_TUNE
        if text is None:
            return
        try:
            connection.mav.statustext_send(
                severity, text[:50].encode("ascii", errors="replace")
            )
            if tune is not None:
                connection.mav.play_tune_send(
                    target_system,
                    target_component,
                    tune.encode("ascii"),
                )
        except Exception:
            self.send_errors += 1

    def _announce_rehearsal(
        self,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
        now_ns: int,
    ) -> None:
        rehearsal = self.controller.rehearsal_status(now_ns)
        phase = str(rehearsal["phase"])
        phase_changed = phase != self._last_rehearsal_phase
        repeat_due = (
            now_ns - self._last_rehearsal_announce_ns
            >= round(REHEARSAL_PROMPT_REPEAT_S * 1.0e9)
        )
        if not phase_changed and not repeat_due:
            return
        self._last_rehearsal_phase = phase
        self._last_rehearsal_announce_ns = now_ns
        severity = mavutil.mavlink.MAV_SEVERITY_NOTICE
        if phase in {
            "blocked",
            "reset_return_switch",
            "hold_for_health",
        }:
            severity = mavutil.mavlink.MAV_SEVERITY_WARNING
        tune = (
            REHEARSAL_READY_TUNE
            if phase_changed and phase == "ready_for_return_switch"
            else None
        )
        try:
            connection.mav.statustext_send(
                severity,
                str(rehearsal["instruction"])[0:50].encode(
                    "ascii", errors="replace"
                ),
            )
            if tune is not None:
                connection.mav.play_tune_send(
                    target_system,
                    target_component,
                    tune.encode("ascii"),
                )
        except Exception:
            self.send_errors += 1

    def tick(
        self,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
    ) -> None:
        now_ns = time.monotonic_ns()
        self._request_parameters(
            connection, target_system, target_component, now_ns
        )
        self._request_origin(
            connection,
            mavutil,
            target_system,
            target_component,
            now_ns,
        )
        gates, detail = self._audit()
        if detail != self._last_audit_detail:
            self._last_audit_detail = detail
        self.controller.set_parameter_audit(gates, detail)

        observed = self.pose_state.latest_observed()
        if (
            observed is not None
            and observed.sequence > self._last_monitor_pose_sequence
        ):
            self._last_monitor_pose_sequence = observed.sequence
            self.controller.observe_monitor_pose(
                observed.position_local_frd_m,
                observed.sequence,
                observed.host_monotonic_ns,
            )

        packet, pose_reason = self.pose_state.latest_healthy(now_ns)
        if packet is None:
            self.controller.set_pose_health(pose_reason)
        elif packet.sequence > self._last_pose_sequence:
            self._last_pose_sequence = packet.sequence
            self.controller.observe_pose(packet, now_ns)

        command_period_ns = round(
            1.0e9 / self.controller.settings.command_rate_hz
        )
        if now_ns - self._last_command_ns < command_period_ns:
            self._write_status(now_ns)
            return
        self._last_command_ns = now_ns
        decision = self.controller.step(now_ns)
        self._announce(
            decision,
            connection,
            mavutil,
            target_system,
            target_component,
        )
        self._announce_rehearsal(
            connection,
            mavutil,
            target_system,
            target_component,
            now_ns,
        )
        if decision.transmit:
            try:
                connection.mav.set_position_target_local_ned_send(
                    (now_ns // 1_000_000) & 0xFFFFFFFF,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    VELOCITY_ONLY_TYPE_MASK,
                    0.0,
                    0.0,
                    0.0,
                    *decision.velocity_local_ned_mps,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
                self.commands_sent += 1
                if decision.hold_zero:
                    self.zero_commands_sent += 1
            except Exception as exc:
                self.send_errors += 1
                self.output.write(
                    {
                        "schema_version": 1,
                        "event": "guided_velocity_send_error",
                        "host_monotonic_ns": now_ns,
                        "error": str(exc),
                    }
                )
        self.output.write(
            {
                "schema_version": 1,
                "event": "slam_return_decision",
                "host_monotonic_ns": now_ns,
                "state": decision.state,
                "transmit": decision.transmit,
                "velocity_local_flu_mps": (
                    decision.velocity_local_flu_mps
                ),
                "velocity_local_ned_mps": (
                    decision.velocity_local_ned_mps
                ),
                "reason": decision.reason,
                "hold_zero": decision.hold_zero,
            }
        )
        self._write_status(now_ns)

    def close(self) -> None:
        gates, detail = self._audit()
        self.controller.set_parameter_audit(gates, detail)
        self._write_status(time.monotonic_ns(), force=True)

    def report(self) -> dict[str, Any]:
        gates, detail = self._audit()
        self.controller.set_parameter_audit(gates, detail)
        snapshot = self.controller.snapshot()
        return {
            "schema_version": 1,
            "kind": "gps_denied_slam_return_runtime",
            "result": (
                "arrived"
                if snapshot["state"] == "arrived"
                else "not_completed"
            ),
            "control_permitted": self.controller.control_permitted,
            "approval_reason": self.controller.approval_reason,
            "commands_sent": self.commands_sent,
            "zero_commands_sent": self.zero_commands_sent,
            "send_errors": self.send_errors,
            "parameter_writes": 0,
            "parameter_requests_sent": self.parameter_requests_sent,
            "parameter_list_requests_sent": self.parameter_list_requests_sent,
            "parameter_audit": dict(sorted(self.parameters.items())),
            "snapshot": snapshot,
        }
