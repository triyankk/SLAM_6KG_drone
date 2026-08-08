"""Browser-based visual guidance for a hardware-owning LIO shadow session."""

from __future__ import annotations

from collections import deque
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import statistics
import threading
import time
from typing import Any
import webbrowser


GUIDE_DURATION_S = 110.0
STABLE_LOCK_S = 5.0
DEFAULT_MAXIMUM_POSITION_JUMP_M = 0.50
DEFAULT_MAXIMUM_SPEED_MPS = 3.0
DEFAULT_MAXIMUM_ATTITUDE_JUMP_DEG = 10.0
YAW_GUIDE_MAXIMUM_RATE_DPS = 35.0
CUBE_ATTITUDE_STALE_S = 0.5
CUBE_LOCAL_POSITION_STALE_S = 0.5
TRANSLATION_GUIDE_MAXIMUM_SPEED_MPS = 0.5
TRANSLATION_GUIDE_MAXIMUM_YAW_DEVIATION_DEG = 10.0
TRANSLATION_CAPTURE_WINDOW_S = 3.0
TRANSLATION_CAPTURE_TARGETS = {
    "settle": (0.0, 0.0, 0.0),
    "forward_1": (0.5, 0.0, 0.0),
    "center_1": (0.0, 0.0, 0.0),
    "right_1": (0.0, 0.5, 0.0),
    "center_2": (0.0, 0.0, 0.0),
    "final_still": (0.0, 0.0, 0.0),
}
GUIDE_PHASES = (
    {
        "id": "settle",
        "start_s": 0.0,
        "end_s": 20.0,
        "label": "HOLD STILL",
        "timeline_label": "STILL",
        "instruction": "Leave the aircraft untouched on the marked start pose.",
    },
    {
        "id": "axes",
        "start_s": 20.0,
        "end_s": 35.0,
        "label": "SLOW AXIS MOTION",
        "timeline_label": "AXES",
        "instruction": (
            "Roll, pitch, then yaw one at a time; stay under 30 deg/s."
        ),
    },
    {
        "id": "compact_translation",
        "start_s": 35.0,
        "end_s": 70.0,
        "label": "COMPACT OUT-AND-BACK",
        "timeline_label": "OUT/BACK",
        "instruction": (
            "Move 0.5 m forward and back twice, then 0.5 m sideways and back."
        ),
    },
    {
        "id": "return",
        "start_s": 70.0,
        "end_s": 90.0,
        "label": "RETURN TO START",
        "timeline_label": "RETURN",
        "instruction": "Return to the exact marked position and orientation.",
    },
    {
        "id": "final_still",
        "start_s": 90.0,
        "end_s": GUIDE_DURATION_S,
        "label": "FINAL STILLNESS",
        "timeline_label": "STILL",
        "instruction": "Set the aircraft down and keep hands off.",
    },
)

YAW_GUIDE_PHASES = (
    {
        "id": "settle",
        "start_s": 0.0,
        "end_s": 15.0,
        "label": "HOLD STILL",
        "timeline_label": "STILL",
        "instruction": "Leave the aircraft untouched on the marked heading.",
    },
    {
        "id": "yaw_right_1",
        "start_s": 15.0,
        "end_s": 25.0,
        "label": "YAW RIGHT 20 DEG",
        "timeline_label": "RIGHT",
        "instruction": "Turn the nose right slowly; stay under 30 deg/s.",
    },
    {
        "id": "yaw_center_1",
        "start_s": 25.0,
        "end_s": 35.0,
        "label": "RETURN TO CENTER",
        "timeline_label": "CENTER",
        "instruction": "Return slowly to the exact marked heading.",
    },
    {
        "id": "yaw_left_1",
        "start_s": 35.0,
        "end_s": 45.0,
        "label": "YAW LEFT 20 DEG",
        "timeline_label": "LEFT",
        "instruction": "Turn the nose left slowly; stay under 30 deg/s.",
    },
    {
        "id": "yaw_center_2",
        "start_s": 45.0,
        "end_s": 55.0,
        "label": "RETURN TO CENTER",
        "timeline_label": "CENTER",
        "instruction": "Return slowly to the exact marked heading.",
    },
    {
        "id": "yaw_right_2",
        "start_s": 55.0,
        "end_s": 65.0,
        "label": "REPEAT YAW RIGHT",
        "timeline_label": "RIGHT",
        "instruction": "Repeat the same slow 20 degree right turn.",
    },
    {
        "id": "yaw_center_3",
        "start_s": 65.0,
        "end_s": 75.0,
        "label": "RETURN TO CENTER",
        "timeline_label": "CENTER",
        "instruction": "Return slowly to the exact marked heading.",
    },
    {
        "id": "yaw_left_2",
        "start_s": 75.0,
        "end_s": 85.0,
        "label": "REPEAT YAW LEFT",
        "timeline_label": "LEFT",
        "instruction": "Repeat the same slow 20 degree left turn.",
    },
    {
        "id": "yaw_center_4",
        "start_s": 85.0,
        "end_s": 95.0,
        "label": "FINAL CENTER",
        "timeline_label": "CENTER",
        "instruction": "Return to the marked heading and set it down.",
    },
    {
        "id": "final_still",
        "start_s": 95.0,
        "end_s": GUIDE_DURATION_S,
        "label": "FINAL STILLNESS",
        "timeline_label": "STILL",
        "instruction": "Keep the aircraft untouched while yaw is checked.",
    },
)

TRANSLATION_GUIDE_PHASES = (
    {
        "id": "settle",
        "start_s": 0.0,
        "end_s": 15.0,
        "label": "HOLD STILL",
        "timeline_label": "STILL",
        "instruction": "Leave the aircraft untouched on the marked start pose.",
    },
    {
        "id": "forward_1",
        "start_s": 15.0,
        "end_s": 35.0,
        "label": "MOVE FORWARD 0.5 M",
        "timeline_label": "FWD",
        "instruction": "Slide forward to +0.50 m; keep heading and level fixed.",
    },
    {
        "id": "center_1",
        "start_s": 35.0,
        "end_s": 55.0,
        "label": "RETURN TO CENTER",
        "timeline_label": "CENTER",
        "instruction": "Slide back until forward and right both read near zero.",
    },
    {
        "id": "right_1",
        "start_s": 55.0,
        "end_s": 75.0,
        "label": "MOVE RIGHT 0.5 M",
        "timeline_label": "RIGHT",
        "instruction": "Slide right to +0.50 m without rotating the aircraft.",
    },
    {
        "id": "center_2",
        "start_s": 75.0,
        "end_s": 95.0,
        "label": "RETURN TO CENTER",
        "timeline_label": "CENTER",
        "instruction": "Slide back until forward and right both read near zero.",
    },
    {
        "id": "final_still",
        "start_s": 95.0,
        "end_s": GUIDE_DURATION_S,
        "label": "FINAL STILLNESS",
        "timeline_label": "STILL",
        "instruction": "Keep the aircraft untouched while scale is checked.",
    },
)


def _finite_vector(values: list[float] | tuple[float, ...]) -> bool:
    return len(values) == 3 and all(math.isfinite(float(value)) for value in values)


def _normalized_quaternion(
    values: list[float] | tuple[float, ...] | None,
) -> tuple[float, float, float, float] | None:
    if values is None or len(values) != 4:
        return None
    quaternion = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in quaternion):
        return None
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-9:
        return None
    return tuple(value / norm for value in quaternion)


def _quaternion_delta_deg(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    dot = min(1.0, abs(sum(a * b for a, b in zip(first, second))))
    return math.degrees(2.0 * math.acos(dot))


def _quaternion_yaw_rad(
    quaternion: tuple[float, float, float, float] | None,
) -> float | None:
    if quaternion is None:
        return None
    x_value, y_value, z_value, w_value = quaternion
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _angle_delta_deg(value_rad: float | None, origin_rad: float | None) -> float | None:
    if value_rad is None or origin_rad is None:
        return None
    delta = math.atan2(
        math.sin(value_rad - origin_rad),
        math.cos(value_rad - origin_rad),
    )
    return math.degrees(delta)


def _ned_delta_to_start_body_frd(
    current_ned_m: tuple[float, float, float] | None,
    origin_ned_m: tuple[float, float, float] | None,
    origin_yaw_rad: float | None,
) -> tuple[float, float, float] | None:
    if (
        current_ned_m is None
        or origin_ned_m is None
        or origin_yaw_rad is None
    ):
        return None
    north = current_ned_m[0] - origin_ned_m[0]
    east = current_ned_m[1] - origin_ned_m[1]
    down = current_ned_m[2] - origin_ned_m[2]
    cosine = math.cos(origin_yaw_rad)
    sine = math.sin(origin_yaw_rad)
    return (
        cosine * north + sine * east,
        -sine * north + cosine * east,
        down,
    )


class LioVisualState:
    """Thread-safe state shared by ROS callbacks and the assist HTTP server."""

    def __init__(
        self,
        session_name: str,
        *,
        auto_start: bool = True,
        maximum_path_points: int = 1200,
        maximum_position_jump_m: float = DEFAULT_MAXIMUM_POSITION_JUMP_M,
        maximum_speed_mps: float = DEFAULT_MAXIMUM_SPEED_MPS,
        maximum_attitude_jump_deg: float = (
            DEFAULT_MAXIMUM_ATTITUDE_JUMP_DEG
        ),
        guide_kind: str = "full",
    ) -> None:
        if (
            maximum_position_jump_m <= 0.0
            or maximum_speed_mps <= 0.0
            or maximum_attitude_jump_deg <= 0.0
        ):
            raise ValueError("trajectory safety limits must be positive")
        if guide_kind not in {"full", "yaw", "translation"}:
            raise ValueError(
                "guide_kind must be 'full', 'yaw', or 'translation'"
            )
        self.session_name = session_name
        self.auto_start = auto_start
        self.guide_kind = guide_kind
        self.guide_phases = {
            "full": GUIDE_PHASES,
            "yaw": YAW_GUIDE_PHASES,
            "translation": TRANSLATION_GUIDE_PHASES,
        }[guide_kind]
        self.guide_duration_s = float(self.guide_phases[-1]["end_s"])
        self.maximum_position_jump_m = maximum_position_jump_m
        self.maximum_speed_mps = maximum_speed_mps
        self.maximum_attitude_jump_deg = maximum_attitude_jump_deg
        self._lock = threading.Lock()
        self._path: deque[tuple[float, float, float]] = deque(
            maxlen=maximum_path_points
        )
        self._translation_samples: deque[
            tuple[float, tuple[float, float, float]]
        ] = deque(maxlen=2000)
        self._guide_started = False
        self._guide_active_since_ns: int | None = None
        self._guide_accumulated_ns = 0
        self._raw_ready = False
        self._ready_since_ns: int | None = None
        self._start_position: tuple[float, float, float] | None = None
        self._current_position: tuple[float, float, float] | None = None
        self._current_odometry_time_ns: int | None = None
        self._current_quaternion: tuple[float, float, float, float] | None = None
        self._current_lio_yaw_rad: float | None = None
        self._start_lio_yaw_rad: float | None = None
        self._last_path_time_ns: int | None = None
        self._last_path_quaternion: (
            tuple[float, float, float, float] | None
        ) = None
        self._distance_m = 0.0
        self._odometry_rows = 0
        self._cube_messages = 0
        self._cube_local_position_rows = 0
        self._current_cube_yaw_rad: float | None = None
        self._cube_attitude_monotonic_ns: int | None = None
        self._start_cube_yaw_rad: float | None = None
        self._cube_yaw_rate_dps: float | None = None
        self._cube_maximum_yaw_rate_dps = 0.0
        self._cube_maximum_yaw_deviation_deg = 0.0
        self._current_cube_position_ned_m: (
            tuple[float, float, float] | None
        ) = None
        self._start_cube_position_ned_m: (
            tuple[float, float, float] | None
        ) = None
        self._cube_local_position_monotonic_ns: int | None = None
        self._cube_horizontal_speed_mps: float | None = None
        self._cube_maximum_horizontal_speed_mps = 0.0
        self._diagnostics: dict[str, Any] = {}
        self._stop_requested = False
        self._failure: dict[str, Any] | None = None

    def update_diagnostics(self, diagnostics: dict[str, Any]) -> None:
        with self._lock:
            self._diagnostics = dict(diagnostics)
            self._refresh_readiness_locked(time.monotonic_ns())

    def update_odometry(
        self,
        position_m: list[float] | tuple[float, ...],
        *,
        timestamp_ns: int | None = None,
        quaternion_xyzw: list[float] | tuple[float, ...] | None = None,
    ) -> None:
        if not _finite_vector(position_m):
            return
        position = tuple(float(value) for value in position_m)
        odometry_time_ns = (
            int(timestamp_ns)
            if timestamp_ns is not None
            else time.monotonic_ns()
        )
        quaternion = _normalized_quaternion(quaternion_xyzw)
        now_ns = time.monotonic_ns()
        with self._lock:
            self._current_position = position
            self._current_odometry_time_ns = odometry_time_ns
            self._current_quaternion = quaternion
            self._current_lio_yaw_rad = _quaternion_yaw_rad(quaternion)
            self._odometry_rows += 1
            self._refresh_readiness_locked(now_ns)
            if not self._guide_started or self._failure is not None:
                return
            if self._path:
                previous = self._path[-1]
                step_m = math.dist(previous, position)
                attitude_jump_deg = None
                if (
                    self._last_path_quaternion is not None
                    and quaternion is not None
                ):
                    attitude_jump_deg = _quaternion_delta_deg(
                        self._last_path_quaternion,
                        quaternion,
                    )
                speed_mps = None
                if (
                    self._last_path_time_ns is not None
                    and odometry_time_ns > self._last_path_time_ns
                ):
                    speed_mps = step_m / (
                        (odometry_time_ns - self._last_path_time_ns) / 1.0e9
                    )
                if (
                    step_m > self.maximum_position_jump_m
                    or (
                        speed_mps is not None
                        and speed_mps > self.maximum_speed_mps
                    )
                    or (
                        attitude_jump_deg is not None
                        and attitude_jump_deg
                        > self.maximum_attitude_jump_deg
                    )
                ):
                    self._fail_locked(
                        "trajectory_divergence",
                        (
                            f"LIO step {step_m:.2f} m"
                            + (
                                f" at {speed_mps:.2f} m/s"
                                if speed_mps is not None
                                else ""
                            )
                            + (
                                f", attitude step {attitude_jump_deg:.1f} deg"
                                if attitude_jump_deg is not None
                                else ""
                            )
                            + " exceeded the shadow safety limit"
                        ),
                        {
                            "step_m": step_m,
                            "speed_mps": speed_mps,
                            "maximum_position_jump_m": (
                                self.maximum_position_jump_m
                            ),
                            "maximum_speed_mps": self.maximum_speed_mps,
                            "attitude_jump_deg": attitude_jump_deg,
                            "maximum_attitude_jump_deg": (
                                self.maximum_attitude_jump_deg
                            ),
                        },
                    )
                    return
                self._distance_m += step_m
            self._path.append(position)
            self._last_path_time_ns = odometry_time_ns
            self._last_path_quaternion = quaternion
            self._record_translation_sample_locked(now_ns, position)

    def update_cube(
        self,
        message_type: str,
        data: dict[str, Any] | None = None,
    ) -> None:
        with self._lock:
            self._cube_messages += 1
            now_ns = time.monotonic_ns()
            if message_type == "LOCAL_POSITION_NED":
                self._cube_local_position_rows += 1
                if not isinstance(data, dict):
                    return
                try:
                    position = tuple(
                        float(data[key]) for key in ("x", "y", "z")
                    )
                    velocity = tuple(
                        float(data[key]) for key in ("vx", "vy", "vz")
                    )
                except (KeyError, TypeError, ValueError):
                    return
                if not all(math.isfinite(value) for value in position + velocity):
                    return
                self._current_cube_position_ned_m = position
                self._cube_local_position_monotonic_ns = now_ns
                self._cube_horizontal_speed_mps = math.hypot(
                    velocity[0], velocity[1]
                )
                if (
                    self._guide_started
                    and self._start_cube_position_ned_m is None
                ):
                    self._start_cube_position_ned_m = position
                self._refresh_readiness_locked(now_ns)
                if self._guide_started and self.guide_kind == "translation":
                    self._cube_maximum_horizontal_speed_mps = max(
                        self._cube_maximum_horizontal_speed_mps,
                        self._cube_horizontal_speed_mps,
                    )
                return
            if message_type != "ATTITUDE" or not isinstance(data, dict):
                return
            try:
                yaw_rad = float(data["yaw"])
                yaw_rate_dps = math.degrees(float(data["yawspeed"]))
            except (KeyError, TypeError, ValueError):
                return
            if not math.isfinite(yaw_rad) or not math.isfinite(yaw_rate_dps):
                return
            self._current_cube_yaw_rad = yaw_rad
            self._cube_attitude_monotonic_ns = now_ns
            self._cube_yaw_rate_dps = yaw_rate_dps
            if self._guide_started and self._start_cube_yaw_rad is None:
                self._start_cube_yaw_rad = yaw_rad
            if not self._guide_started:
                self._refresh_readiness_locked(now_ns)
                return
            self._refresh_readiness_locked(now_ns)
            yaw_deviation_deg = abs(
                _angle_delta_deg(yaw_rad, self._start_cube_yaw_rad) or 0.0
            )
            self._cube_maximum_yaw_deviation_deg = max(
                self._cube_maximum_yaw_deviation_deg,
                yaw_deviation_deg,
            )
            if self.guide_kind == "translation":
                if (
                    self._failure is None
                    and yaw_deviation_deg
                    > TRANSLATION_GUIDE_MAXIMUM_YAW_DEVIATION_DEG
                ):
                    self._fail_locked(
                        "excessive_test_motion",
                        (
                            f"Cube yaw changed {yaw_deviation_deg:.1f} deg "
                            "during the translation-only test"
                        ),
                        {
                            "cube_yaw_deviation_deg": yaw_deviation_deg,
                            "maximum_yaw_deviation_deg": (
                                TRANSLATION_GUIDE_MAXIMUM_YAW_DEVIATION_DEG
                            ),
                        },
                    )
                return
            if self.guide_kind != "yaw":
                return
            absolute_rate_dps = abs(yaw_rate_dps)
            self._cube_maximum_yaw_rate_dps = max(
                self._cube_maximum_yaw_rate_dps,
                absolute_rate_dps,
            )
            if (
                self._failure is None
                and absolute_rate_dps > YAW_GUIDE_MAXIMUM_RATE_DPS
            ):
                self._fail_locked(
                    "excessive_test_motion",
                    (
                        f"Cube yaw rate {absolute_rate_dps:.1f} deg/s exceeded "
                        f"the {YAW_GUIDE_MAXIMUM_RATE_DPS:.0f} deg/s test limit"
                    ),
                    {
                        "cube_yaw_rate_dps": yaw_rate_dps,
                        "maximum_yaw_rate_dps": YAW_GUIDE_MAXIMUM_RATE_DPS,
                    },
                )

    def start_guide(self) -> tuple[bool, str]:
        with self._lock:
            now_ns = time.monotonic_ns()
            if not self._stable_ready_locked(now_ns):
                return False, "Waiting for five seconds of stable LIO odometry"
            self._start_guide_locked(now_ns)
            return True, f"Guided {self.guide_kind} sequence started"

    def request_stop(self) -> None:
        with self._lock:
            self._stop_requested = True

    def should_stop(self) -> bool:
        with self._lock:
            if self._stop_requested:
                return True
            if not self._guide_started:
                return False
            elapsed_s = self._elapsed_s_locked(time.monotonic_ns())
            return elapsed_s >= self.guide_duration_s

    def failure_detail(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._failure) if self._failure is not None else None

    def guide_result(self) -> dict[str, Any]:
        """Return compact, phase-aware evidence for offline validation."""
        now_ns = time.monotonic_ns()
        with self._lock:
            elapsed_s = self._elapsed_s_locked(now_ns)
            payload: dict[str, Any] = {
                "schema_version": 1,
                "guide_kind": self.guide_kind,
                "guide_complete": (
                    self._guide_started
                    and elapsed_s >= self.guide_duration_s
                    and self._failure is None
                ),
                "failure": self._failure,
                "pose_output_to_cube": False,
                "cube_local_position_used_as_ground_truth": False,
            }
            if self.guide_kind != "translation":
                return payload

            raw_captures: list[dict[str, Any]] = []
            for phase in self.guide_phases:
                phase_id = str(phase["id"])
                target = TRANSLATION_CAPTURE_TARGETS.get(phase_id)
                if target is None:
                    continue
                end_s = float(phase["end_s"])
                window_start_s = max(
                    float(phase["start_s"]),
                    end_s - TRANSLATION_CAPTURE_WINDOW_S,
                )
                samples = [
                    position
                    for sample_elapsed_s, position in self._translation_samples
                    if window_start_s <= sample_elapsed_s < end_s
                ]
                observed = (
                    tuple(
                        statistics.median(sample[axis] for sample in samples)
                        for axis in range(3)
                    )
                    if samples
                    else None
                )
                raw_captures.append(
                    {
                        "phase_id": phase_id,
                        "target_m": list(target),
                        "raw_observed_m": (
                            list(observed) if observed is not None else None
                        ),
                        "samples": len(samples),
                    }
                )

            settle = next(
                (
                    capture["raw_observed_m"]
                    for capture in raw_captures
                    if capture["phase_id"] == "settle"
                ),
                None,
            )
            captures: list[dict[str, Any]] = []
            for capture in raw_captures:
                raw_observed = capture.pop("raw_observed_m")
                observed = (
                    [
                        float(raw_observed[axis]) - float(settle[axis])
                        for axis in range(3)
                    ]
                    if raw_observed is not None and settle is not None
                    else None
                )
                captures.append({**capture, "observed_m": observed})
            payload.update(
                {
                    "reference": "operator_positioned_tape_marks",
                    "capture_window_s": TRANSLATION_CAPTURE_WINDOW_S,
                    "captures": captures,
                }
            )
            return payload

    def snapshot(self) -> dict[str, Any]:
        now_ns = time.monotonic_ns()
        with self._lock:
            diagnostics = self._diagnostics
            synchronized = bool(diagnostics.get("synchronized"))
            publishing = bool(diagnostics.get("publishing"))
            elapsed_s = self._elapsed_s_locked(now_ns)
            phase = self._phase_locked(elapsed_s)
            paused = self._guide_started and self._guide_active_since_ns is None
            if self._failure is not None:
                paused = False
                phase = {
                    "id": "failed",
                    "label": "TRAJECTORY REJECTED",
                    "instruction": (
                        "Stop moving. The recorder detected impossible LIO motion."
                    ),
                    "remaining_s": None,
                }
            elif paused:
                phase = {
                    "id": "paused",
                    "label": "GUIDE PAUSED",
                    "instruction": (
                        "Keep the aircraft still while synchronized lock recovers."
                    ),
                    "remaining_s": phase.get("remaining_s"),
                }
            start = self._start_position
            current = self._current_position
            path = list(self._path)
            if start is not None:
                relative_path = [
                    [
                        point[0] - start[0],
                        point[1] - start[1],
                        point[2] - start[2],
                    ]
                    for point in path
                ]
                relative_position = (
                    [
                        current[0] - start[0],
                        current[1] - start[1],
                        current[2] - start[2],
                    ]
                    if current is not None
                    else None
                )
                return_error_m = (
                    math.dist(current, start)
                    if current is not None
                    else None
                )
            else:
                relative_path = []
                relative_position = None
                return_error_m = None
            imu = diagnostics.get("imu", {})
            lidar = diagnostics.get("lidar", {})
            cube_attitude_age_ms = (
                (now_ns - self._cube_attitude_monotonic_ns) / 1.0e6
                if self._cube_attitude_monotonic_ns is not None
                else None
            )
            cube_attitude_fresh = bool(
                cube_attitude_age_ms is not None
                and cube_attitude_age_ms <= CUBE_ATTITUDE_STALE_S * 1000.0
            )
            cube_local_position_age_ms = (
                (now_ns - self._cube_local_position_monotonic_ns) / 1.0e6
                if self._cube_local_position_monotonic_ns is not None
                else None
            )
            cube_local_position_fresh = bool(
                cube_local_position_age_ms is not None
                and cube_local_position_age_ms
                <= CUBE_LOCAL_POSITION_STALE_S * 1000.0
            )
            cube_body_delta = _ned_delta_to_start_body_frd(
                self._current_cube_position_ned_m,
                self._start_cube_position_ned_m,
                self._start_cube_yaw_rad,
            )
            return {
                "schema_version": 1,
                "session": self.session_name,
                "guide_kind": self.guide_kind,
                "guide_phases": [dict(item) for item in self.guide_phases],
                "ready": (
                    self._stable_ready_locked(now_ns)
                    and self._failure is None
                ),
                "synchronized": synchronized,
                "publishing": publishing,
                "guide_started": self._guide_started,
                "guide_complete": elapsed_s >= self.guide_duration_s,
                "paused": paused,
                "stop_requested": self._stop_requested,
                "failed": self._failure is not None,
                "failure": self._failure,
                "elapsed_s": min(elapsed_s, self.guide_duration_s),
                "duration_s": self.guide_duration_s,
                "progress": min(1.0, elapsed_s / self.guide_duration_s),
                "phase": phase,
                "pose_output_to_cube": False,
                "odometry_rows": self._odometry_rows,
                "distance_m": self._distance_m,
                "return_error_m": return_error_m,
                "position_m": relative_position,
                "path": relative_path,
                "cube_messages": self._cube_messages,
                "cube_local_position_rows": self._cube_local_position_rows,
                "cube_attitude_fresh": cube_attitude_fresh,
                "cube_attitude_age_ms": cube_attitude_age_ms,
                "cube_local_position_fresh": cube_local_position_fresh,
                "cube_local_position_age_ms": cube_local_position_age_ms,
                "yaw": {
                    "lio_delta_deg": _angle_delta_deg(
                        self._current_lio_yaw_rad,
                        self._start_lio_yaw_rad,
                    ),
                    "cube_delta_deg": _angle_delta_deg(
                        self._current_cube_yaw_rad,
                        self._start_cube_yaw_rad,
                    ),
                    "cube_rate_dps": self._cube_yaw_rate_dps,
                    "cube_maximum_rate_dps": (
                        self._cube_maximum_yaw_rate_dps
                    ),
                    "rate_limit_dps": YAW_GUIDE_MAXIMUM_RATE_DPS,
                },
                "translation": {
                    "cube_body_delta_m": (
                        list(cube_body_delta)
                        if cube_body_delta is not None
                        else None
                    ),
                    "cube_horizontal_speed_mps": (
                        self._cube_horizontal_speed_mps
                    ),
                    "cube_maximum_horizontal_speed_mps": (
                        self._cube_maximum_horizontal_speed_mps
                    ),
                    "speed_limit_mps": (
                        TRANSLATION_GUIDE_MAXIMUM_SPEED_MPS
                    ),
                    "cube_maximum_yaw_deviation_deg": (
                        self._cube_maximum_yaw_deviation_deg
                    ),
                    "yaw_deviation_limit_deg": (
                        TRANSLATION_GUIDE_MAXIMUM_YAW_DEVIATION_DEG
                    ),
                },
                "imu": {
                    "connected": bool(imu.get("connected")),
                    "rate_hz": imu.get("rate_hz"),
                    "queue_drops": int(imu.get("queue_drops") or 0),
                    "clock_ready": bool(imu.get("clock", {}).get("ready")),
                    "clock_p95_ms": imu.get("clock", {}).get(
                        "residual_p95_ms"
                    ),
                },
                "lidar": {
                    "connected": bool(lidar.get("connected")),
                    "rate_hz": lidar.get("rate_hz"),
                    "queue_drops": int(lidar.get("queue_drops") or 0),
                    "clock_ready": bool(
                        lidar.get("clock", {}).get("ready")
                    ),
                    "clock_p95_ms": lidar.get("clock", {}).get(
                        "residual_p95_ms"
                    ),
                },
            }

    def _ready_locked(self, now_ns: int) -> bool:
        estimator_ready = bool(
            self._diagnostics.get("synchronized")
            and self._diagnostics.get("publishing")
            and self._current_position is not None
        )
        if not estimator_ready or self.guide_kind == "full":
            return estimator_ready
        attitude_fresh = bool(
            self._cube_attitude_monotonic_ns is not None
            and now_ns - self._cube_attitude_monotonic_ns
            <= int(CUBE_ATTITUDE_STALE_S * 1.0e9)
        )
        return attitude_fresh

    def _stable_ready_locked(self, now_ns: int) -> bool:
        return bool(
            self._raw_ready
            and self._ready_since_ns is not None
            and now_ns - self._ready_since_ns >= int(STABLE_LOCK_S * 1.0e9)
        )

    def _refresh_readiness_locked(self, now_ns: int) -> None:
        ready = self._ready_locked(now_ns)
        if ready and not self._raw_ready:
            self._ready_since_ns = now_ns
        elif not ready and self._raw_ready:
            self._ready_since_ns = None
            if self._guide_active_since_ns is not None:
                self._guide_accumulated_ns += (
                    now_ns - self._guide_active_since_ns
                )
                self._guide_active_since_ns = None
        self._raw_ready = ready

        stable = self._stable_ready_locked(now_ns)
        if (
            self._guide_started
            and self._failure is None
            and stable
            and self._guide_active_since_ns is None
        ):
            self._guide_active_since_ns = now_ns
        if (
            self.auto_start
            and not self._guide_started
            and stable
        ):
            self._start_guide_locked(now_ns)

    def _start_guide_locked(self, now_ns: int) -> None:
        self._guide_started = True
        self._guide_active_since_ns = now_ns
        self._guide_accumulated_ns = 0
        self._failure = None
        self._stop_requested = False
        self._start_position = self._current_position
        self._start_lio_yaw_rad = self._current_lio_yaw_rad
        self._start_cube_yaw_rad = self._current_cube_yaw_rad
        self._start_cube_position_ned_m = self._current_cube_position_ned_m
        self._cube_maximum_yaw_rate_dps = 0.0
        self._cube_maximum_yaw_deviation_deg = 0.0
        self._cube_maximum_horizontal_speed_mps = 0.0
        self._path.clear()
        self._translation_samples.clear()
        self._distance_m = 0.0
        self._last_path_time_ns = self._current_odometry_time_ns
        self._last_path_quaternion = self._current_quaternion
        if self._current_position is not None:
            self._path.append(self._current_position)
            self._record_translation_sample_locked(
                now_ns,
                self._current_position,
            )

    def _record_translation_sample_locked(
        self,
        now_ns: int,
        position: tuple[float, float, float],
    ) -> None:
        if (
            self.guide_kind != "translation"
            or not self._guide_started
            or self._start_position is None
        ):
            return
        self._translation_samples.append(
            (
                min(
                    self._elapsed_s_locked(now_ns),
                    self.guide_duration_s,
                ),
                tuple(
                    position[axis] - self._start_position[axis]
                    for axis in range(3)
                ),
            )
        )

    def _fail_locked(
        self,
        code: str,
        detail: str,
        measurements: dict[str, Any],
    ) -> None:
        now_ns = time.monotonic_ns()
        if self._guide_active_since_ns is not None:
            self._guide_accumulated_ns += (
                now_ns - self._guide_active_since_ns
            )
            self._guide_active_since_ns = None
        self._failure = {
            "code": code,
            "detail": detail,
            "measurements": measurements,
        }
        self._stop_requested = True

    def _elapsed_s_locked(self, now_ns: int) -> float:
        elapsed_ns = self._guide_accumulated_ns
        if self._guide_active_since_ns is not None:
            elapsed_ns += max(0, now_ns - self._guide_active_since_ns)
        return elapsed_ns / 1.0e9

    def _phase_locked(self, elapsed_s: float) -> dict[str, Any]:
        if not self._guide_started:
            return {
                "id": "sync",
                "label": "SYNCHRONIZING",
                "instruction": "Keep the aircraft still while both clocks lock.",
                "remaining_s": None,
            }
        for phase in self.guide_phases:
            if elapsed_s < float(phase["end_s"]):
                payload = dict(phase)
                payload["remaining_s"] = max(
                    0.0,
                    float(phase["end_s"]) - elapsed_s,
                )
                if phase["id"] == "axes":
                    axis_index = min(
                        2,
                        int((elapsed_s - float(phase["start_s"])) // 5.0),
                    )
                    axis = ("ROLL", "PITCH", "YAW")[axis_index]
                    payload["label"] = f"SLOW {axis}"
                return payload
        return {
            "id": "complete",
            "label": "SEQUENCE COMPLETE",
            "instruction": "Keep the aircraft still while the report is finalized.",
            "remaining_s": 0.0,
        }


def make_visual_handler(
    state: LioVisualState,
    static_dir: Path,
) -> type[SimpleHTTPRequestHandler]:
    class LioVisualHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(
                *args,
                directory=str(static_dir),
                **kwargs,
            )

        def log_message(self, _format: str, *args: Any) -> None:
            return

        def do_GET(self) -> None:
            if self.path == "/api/lio-assist":
                self._send_json(state.snapshot())
                return
            if self.path in ("/", "/lio-assist"):
                self.path = "/lio-assist.html"
            super().do_GET()

        def do_POST(self) -> None:
            if self.path == "/api/lio-assist/start":
                started, detail = state.start_guide()
                self._send_json(
                    {"started": started, "detail": detail},
                    (
                        HTTPStatus.OK
                        if started
                        else HTTPStatus.CONFLICT
                    ),
                )
                return
            if self.path == "/api/lio-assist/stop":
                state.request_stop()
                self._send_json({"stopping": True})
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def _send_json(
            self,
            payload: dict[str, Any],
            status: HTTPStatus = HTTPStatus.OK,
        ) -> None:
            encoded = json.dumps(payload, separators=(",", ":")).encode(
                "utf-8"
            )
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(encoded)

    return LioVisualHandler


class LioVisualServer:
    def __init__(
        self,
        state: LioVisualState,
        static_dir: Path,
        *,
        host: str,
        port: int,
        open_browser: bool,
    ) -> None:
        handler = make_visual_handler(state, static_dir)
        self._server = ThreadingHTTPServer((host, port), handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=partial(self._server.serve_forever, poll_interval=0.2),
            name="lio-visual-assist",
            daemon=True,
        )
        url_host = "127.0.0.1" if host == "0.0.0.0" else host
        self.url = f"http://{url_host}:{port}"
        self._open_browser = open_browser

    def start(self) -> None:
        self._thread.start()
        if self._open_browser:
            threading.Timer(0.5, partial(webbrowser.open, self.url)).start()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
