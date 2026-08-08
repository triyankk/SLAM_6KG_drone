"""Pilot-facing QGC guidance for the first airborne SLAM shadow flight."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math
import time
from typing import Any, Callable


@dataclass(frozen=True)
class FlightGuideSettings:
    minimum_altitude_m: float = 1.0
    maximum_altitude_m: float = 3.0
    initial_hold_s: float = 10.0
    outbound_distance_m: float = 0.5
    outbound_tolerance_m: float = 0.05
    outbound_hold_s: float = 5.0
    return_tolerance_m: float = 0.20
    final_hold_s: float = 10.0
    stable_speed_mps: float = 0.25
    telemetry_stale_s: float = 1.0
    prompt_repeat_s: float = 5.0
    hold_prompt_repeat_s: float = 2.0

    def __post_init__(self) -> None:
        positive = (
            self.minimum_altitude_m,
            self.maximum_altitude_m,
            self.initial_hold_s,
            self.outbound_distance_m,
            self.outbound_tolerance_m,
            self.outbound_hold_s,
            self.return_tolerance_m,
            self.final_hold_s,
            self.stable_speed_mps,
            self.telemetry_stale_s,
            self.prompt_repeat_s,
            self.hold_prompt_repeat_s,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("flight guide settings must be positive")
        if self.minimum_altitude_m >= self.maximum_altitude_m:
            raise ValueError("minimum altitude must be below maximum altitude")
        if self.outbound_tolerance_m >= self.outbound_distance_m:
            raise ValueError("outbound tolerance must be below target distance")


@dataclass(frozen=True)
class FlightGuideMessage:
    sequence: int
    host_monotonic_ns: int
    phase: str
    text: str
    severity: str
    beep: bool
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class FlightShadowGuide:
    """Advance a manual flight card from observed Cube telemetry only."""

    TERMINAL_PHASES = frozenset(("complete", "ended_early"))
    HOLD_PHASES = frozenset(("initial_hold", "hold_out", "final_hold"))

    PHASE_TEXT = {
        "sensor_startup": "SLAM TEST: WAIT FOR SENSOR READY",
        "ready": "SLAM TEST READY: ARM IN LOITER",
        "climb": "SLAM TEST: CLIMB TO 1-3M AND HOLD",
        "initial_hold": "SLAM TEST: HOLD AT 1-3M FOR 10S",
        "outbound": "SLAM TEST: MOVE FORWARD 0.5M SLOWLY",
        "hold_out": "SLAM TEST: HOLD POSITION FOR 5S",
        "return": "SLAM TEST: RETURN TO TAKEOFF MARK",
        "final_hold": "SLAM TEST: HOLD AT START FOR 10S",
        "land": "SLAM TEST COMPLETE: LAND IN LOITER",
        "complete": "SLAM TEST SAVED: DISARM CONFIRMED",
        "ended_early": "SLAM TEST ENDED EARLY: DATA SAVING",
    }

    def __init__(
        self,
        settings: FlightGuideSettings | None = None,
        *,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.settings = settings or FlightGuideSettings()
        self._clock_ns = clock_ns
        self.phase = "sensor_startup"
        self.pipeline_ready = False
        self.armed = False
        self.armed_once = False
        self.mode = "UNKNOWN"
        self._phase_started_ns = self._clock_ns()
        self._hold_started_ns: int | None = None
        self._last_prompt_ns: int | None = None
        self._last_warning_key: str | None = None
        self._last_warning_ns: int | None = None
        self._sequence = 0
        self._pending: deque[FlightGuideMessage] = deque()
        self._history: list[dict[str, Any]] = []

        self._range_altitude_m: float | None = None
        self._range_updated_ns: int | None = None
        self._relative_altitude_m: float | None = None
        self._relative_altitude_updated_ns: int | None = None
        self._local_position_ned_m: tuple[float, float, float] | None = None
        self._local_velocity_ned_mps: tuple[float, float, float] | None = None
        self._local_position_updated_ns: int | None = None
        self._yaw_rad: float | None = None
        self._attitude_updated_ns: int | None = None
        self._rc_updated_ns: int | None = None
        self._launch_position_ne_m: tuple[float, float] | None = None
        self._launch_yaw_rad: float | None = None
        self._hold_anchor_ne_m: tuple[float, float] | None = None

    @staticmethod
    def _fresh(updated_ns: int | None, now_ns: int, timeout_s: float) -> bool:
        if updated_ns is None:
            return False
        age_ns = now_ns - updated_ns
        return 0 <= age_ns <= round(timeout_s * 1.0e9)

    def set_pipeline_ready(
        self, ready: bool, *, now_ns: int | None = None
    ) -> None:
        self.pipeline_ready = bool(ready)
        self.tick(now_ns=now_ns)

    def observe_cube(
        self,
        message_type: str,
        data: dict[str, Any],
        *,
        mode_name: str | None = None,
        now_ns: int | None = None,
    ) -> None:
        now = self._clock_ns() if now_ns is None else int(now_ns)
        if message_type == "HEARTBEAT":
            if mode_name:
                self.mode = str(mode_name).upper()
            try:
                armed = bool(int(data["base_mode"]) & 128)
            except (KeyError, TypeError, ValueError):
                armed = self.armed
            was_armed = self.armed
            self.armed = armed
            if armed and not was_armed:
                self.armed_once = True
                if self.phase not in self.TERMINAL_PHASES:
                    self._transition("climb", now, beep=False)
            elif was_armed and not armed:
                terminal = "complete" if self.phase == "land" else "ended_early"
                self._transition(terminal, now, beep=False)
        elif message_type == "DISTANCE_SENSOR":
            try:
                if int(data.get("orientation", -1)) == 25:
                    distance_m = float(data["current_distance"]) / 100.0
                    minimum_m = float(data.get("min_distance", 0)) / 100.0
                    maximum_m = float(data.get("max_distance", 65535)) / 100.0
                    if minimum_m <= distance_m <= maximum_m:
                        self._range_altitude_m = distance_m
                        self._range_updated_ns = now
            except (KeyError, TypeError, ValueError):
                pass
        elif message_type == "GLOBAL_POSITION_INT":
            try:
                self._relative_altitude_m = float(data["relative_alt"]) / 1000.0
                self._relative_altitude_updated_ns = now
            except (KeyError, TypeError, ValueError):
                pass
        elif message_type == "LOCAL_POSITION_NED":
            try:
                self._local_position_ned_m = (
                    float(data["x"]),
                    float(data["y"]),
                    float(data["z"]),
                )
                self._local_velocity_ned_mps = (
                    float(data["vx"]),
                    float(data["vy"]),
                    float(data["vz"]),
                )
                self._local_position_updated_ns = now
            except (KeyError, TypeError, ValueError):
                pass
        elif message_type == "ATTITUDE":
            try:
                self._yaw_rad = float(data["yaw"])
                self._attitude_updated_ns = now
            except (KeyError, TypeError, ValueError):
                pass
        elif message_type == "RC_CHANNELS":
            valid = 0
            for index in range(1, 5):
                try:
                    pwm = int(data[f"chan{index}_raw"])
                except (KeyError, TypeError, ValueError):
                    continue
                if 800 <= pwm <= 2200:
                    valid += 1
            if valid == 4:
                self._rc_updated_ns = now
        self.tick(now_ns=now)

    def _altitude(self, now_ns: int) -> tuple[float | None, str | None]:
        timeout = self.settings.telemetry_stale_s
        if self._fresh(self._range_updated_ns, now_ns, timeout):
            return self._range_altitude_m, "downward_range"
        if self._fresh(self._relative_altitude_updated_ns, now_ns, timeout):
            return self._relative_altitude_m, "relative_altitude"
        if self._fresh(self._local_position_updated_ns, now_ns, timeout):
            if self._local_position_ned_m is not None:
                return -self._local_position_ned_m[2], "local_position"
        return None, None

    def _local_position(self, now_ns: int) -> tuple[float, float] | None:
        if not self._fresh(
            self._local_position_updated_ns,
            now_ns,
            self.settings.telemetry_stale_s,
        ):
            return None
        if self._local_position_ned_m is None:
            return None
        return self._local_position_ned_m[:2]

    def _horizontal_speed(self, now_ns: int) -> float | None:
        if self._local_position(now_ns) is None:
            return None
        if self._local_velocity_ned_mps is None:
            return None
        return math.hypot(
            self._local_velocity_ned_mps[0],
            self._local_velocity_ned_mps[1],
        )

    def _yaw(self, now_ns: int) -> float | None:
        if not self._fresh(
            self._attitude_updated_ns,
            now_ns,
            self.settings.telemetry_stale_s,
        ):
            return None
        return self._yaw_rad

    def _pilot_input_fresh(self, now_ns: int) -> bool:
        return self._fresh(
            self._rc_updated_ns,
            now_ns,
            self.settings.telemetry_stale_s,
        )

    def _altitude_valid(self, now_ns: int) -> bool:
        altitude, _ = self._altitude(now_ns)
        return bool(
            altitude is not None
            and self.settings.minimum_altitude_m
            <= altitude
            <= self.settings.maximum_altitude_m
        )

    def _stable(self, now_ns: int) -> bool:
        speed = self._horizontal_speed(now_ns)
        return bool(
            self._altitude_valid(now_ns)
            and speed is not None
            and speed <= self.settings.stable_speed_mps
        )

    def _distance_from_launch(self, now_ns: int) -> float | None:
        position = self._local_position(now_ns)
        if position is None or self._launch_position_ne_m is None:
            return None
        return math.dist(position, self._launch_position_ne_m)

    def _forward_distance(self, now_ns: int) -> float | None:
        position = self._local_position(now_ns)
        if (
            position is None
            or self._launch_position_ne_m is None
            or self._launch_yaw_rad is None
        ):
            return None
        north = position[0] - self._launch_position_ne_m[0]
        east = position[1] - self._launch_position_ne_m[1]
        return north * math.cos(self._launch_yaw_rad) + east * math.sin(
            self._launch_yaw_rad
        )

    def _hold_elapsed_s(self, now_ns: int) -> float:
        if self._hold_started_ns is None:
            return 0.0
        return max(0.0, (now_ns - self._hold_started_ns) / 1.0e9)

    def _reset_hold(self, now_ns: int) -> None:
        self._hold_started_ns = now_ns
        self._hold_anchor_ne_m = self._local_position(now_ns)

    def _hold_is_valid(
        self,
        now_ns: int,
        *,
        require_launch_tolerance: bool = False,
    ) -> bool:
        position = self._local_position(now_ns)
        if position is None or not self._stable(now_ns):
            self._reset_hold(now_ns)
            return False
        if require_launch_tolerance:
            distance = self._distance_from_launch(now_ns)
            if distance is None or distance > self.settings.return_tolerance_m:
                self._reset_hold(now_ns)
                return False
        if self._hold_anchor_ne_m is None:
            self._reset_hold(now_ns)
            return False
        if math.dist(position, self._hold_anchor_ne_m) > 0.15:
            self._reset_hold(now_ns)
            return False
        return True

    def _transition(
        self,
        phase: str,
        now_ns: int,
        *,
        beep: bool,
    ) -> None:
        if phase == self.phase:
            return
        self.phase = phase
        self._phase_started_ns = now_ns
        self._hold_started_ns = now_ns if phase in self.HOLD_PHASES else None
        self._hold_anchor_ne_m = (
            self._local_position(now_ns) if phase in self.HOLD_PHASES else None
        )
        self._last_warning_key = None
        if phase == "outbound":
            self._launch_position_ne_m = self._local_position(now_ns)
            self._launch_yaw_rad = self._yaw(now_ns)
        self._emit(
            self.PHASE_TEXT[phase],
            now_ns,
            severity="notice",
            beep=beep,
            reason="phase_transition",
        )

    def _emit(
        self,
        text: str,
        now_ns: int,
        *,
        severity: str,
        beep: bool,
        reason: str,
    ) -> None:
        if len(text.encode("ascii")) > 50:
            raise ValueError(f"QGC STATUSTEXT exceeds 50 bytes: {text}")
        self._sequence += 1
        message = FlightGuideMessage(
            sequence=self._sequence,
            host_monotonic_ns=now_ns,
            phase=self.phase,
            text=text,
            severity=severity,
            beep=beep,
            reason=reason,
        )
        self._pending.append(message)
        self._history.append(message.as_dict())
        self._last_prompt_ns = now_ns

    def _warn(self, key: str, text: str, now_ns: int) -> None:
        repeat_ns = round(self.settings.prompt_repeat_s * 1.0e9)
        if (
            key == self._last_warning_key
            and self._last_warning_ns is not None
            and now_ns - self._last_warning_ns < repeat_ns
        ):
            return
        self._last_warning_key = key
        self._last_warning_ns = now_ns
        self._emit(
            text,
            now_ns,
            severity="warning",
            beep=False,
            reason=key,
        )

    def _repeat_prompt(self, now_ns: int) -> None:
        repeat_s = (
            self.settings.hold_prompt_repeat_s
            if self.phase in self.HOLD_PHASES
            else self.settings.prompt_repeat_s
        )
        if (
            self._last_prompt_ns is not None
            and now_ns - self._last_prompt_ns < round(repeat_s * 1.0e9)
        ):
            return
        text = self.PHASE_TEXT[self.phase]
        if self.phase == "initial_hold":
            remaining = max(
                0,
                math.ceil(
                    self.settings.initial_hold_s
                    - self._hold_elapsed_s(now_ns)
                ),
            )
            text = f"SLAM TEST: HOLD {remaining}S MORE AT 1-3M"
        elif self.phase == "hold_out":
            remaining = max(
                0,
                math.ceil(
                    self.settings.outbound_hold_s
                    - self._hold_elapsed_s(now_ns)
                ),
            )
            text = f"SLAM TEST: HOLD POSITION {remaining}S MORE"
        elif self.phase == "final_hold":
            remaining = max(
                0,
                math.ceil(
                    self.settings.final_hold_s
                    - self._hold_elapsed_s(now_ns)
                ),
            )
            text = f"SLAM TEST: FINAL HOLD {remaining}S MORE"
        self._emit(
            text,
            now_ns,
            severity="notice",
            beep=False,
            reason="repeat",
        )

    def tick(self, *, now_ns: int | None = None) -> None:
        now = self._clock_ns() if now_ns is None else int(now_ns)
        if self.phase in self.TERMINAL_PHASES:
            return
        if not self.pipeline_ready:
            if self.armed:
                self._warn(
                    "pipeline_not_ready",
                    "SLAM SENSORS NOT READY: HOLD OR LAND",
                    now,
                )
            elif self.phase != "sensor_startup":
                self._transition("sensor_startup", now, beep=False)
            else:
                self._repeat_prompt(now)
            return
        if not self.armed:
            if self.phase != "ready":
                self._transition("ready", now, beep=False)
            else:
                self._repeat_prompt(now)
            return
        if self.phase in {"sensor_startup", "ready"}:
            self._transition("climb", now, beep=False)
        if self.mode != "LOITER":
            if self.phase in self.HOLD_PHASES:
                self._reset_hold(now)
            self._warn("mode", "SLAM TEST: SELECT LOITER AND HOLD", now)
            return
        if not self._pilot_input_fresh(now):
            if self.phase == "initial_hold":
                self._reset_hold(now)
            self._warn("rc_stale", "SLAM TEST: RC INPUT NOT SEEN - HOLD", now)
            return
        if not self._altitude_valid(now):
            if self.phase in self.HOLD_PHASES:
                self._reset_hold(now)
            self._warn("altitude", "SLAM TEST: RETURN TO 1-3M HEIGHT", now)
            return

        if self.phase == "climb":
            self._transition("initial_hold", now, beep=True)
        elif self.phase == "initial_hold":
            if not self._stable(now):
                self._reset_hold(now)
            elif self._hold_elapsed_s(now) >= self.settings.initial_hold_s:
                if self._local_position(now) is None or self._yaw(now) is None:
                    self._warn(
                        "pose_stale",
                        "SLAM TEST: POSITION OR YAW STALE - HOLD",
                        now,
                    )
                    return
                self._transition("outbound", now, beep=True)
        elif self.phase == "outbound":
            forward = self._forward_distance(now)
            if forward is None:
                self._warn(
                    "pose_stale",
                    "SLAM TEST: POSITION OR YAW STALE - HOLD",
                    now,
                )
                return
            if forward >= (
                self.settings.outbound_distance_m
                - self.settings.outbound_tolerance_m
            ):
                self._transition("hold_out", now, beep=True)
        elif self.phase == "hold_out":
            if self._hold_is_valid(now) and self._hold_elapsed_s(
                now
            ) >= self.settings.outbound_hold_s:
                self._transition("return", now, beep=True)
        elif self.phase == "return":
            distance = self._distance_from_launch(now)
            speed = self._horizontal_speed(now)
            if (
                distance is not None
                and distance <= self.settings.return_tolerance_m
                and speed is not None
                and speed <= self.settings.stable_speed_mps
            ):
                self._transition("final_hold", now, beep=True)
        elif self.phase == "final_hold":
            if self._hold_is_valid(
                now, require_launch_tolerance=True
            ) and self._hold_elapsed_s(now) >= self.settings.final_hold_s:
                self._transition("land", now, beep=True)
        self._repeat_prompt(now)

    def drain_messages(self) -> list[FlightGuideMessage]:
        messages = list(self._pending)
        self._pending.clear()
        return messages

    def status(self, *, now_ns: int | None = None) -> dict[str, Any]:
        now = self._clock_ns() if now_ns is None else int(now_ns)
        altitude_m, altitude_source = self._altitude(now)
        return {
            "phase": self.phase,
            "instruction": self.PHASE_TEXT[self.phase],
            "pipeline_ready": self.pipeline_ready,
            "armed": self.armed,
            "armed_once": self.armed_once,
            "mode": self.mode,
            "altitude_m": altitude_m,
            "altitude_source": altitude_source,
            "altitude_valid": self._altitude_valid(now),
            "pilot_input_fresh": self._pilot_input_fresh(now),
            "horizontal_speed_mps": self._horizontal_speed(now),
            "forward_distance_m": self._forward_distance(now),
            "distance_from_launch_m": self._distance_from_launch(now),
            "hold_elapsed_s": self._hold_elapsed_s(now),
            "messages_generated": self._sequence,
        }

    def report(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "mode": "qgc_advisory_only",
            "movement_commands_sent": False,
            "settings": asdict(self.settings),
            "final_status": self.status(),
            "history": list(self._history),
        }
