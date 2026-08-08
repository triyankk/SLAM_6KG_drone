"""Guided, props-off JT16 cardinal-direction calibration."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Callable, Sequence

from .config import ConfigError, FlightControllerConfig, load_config
from .paths import CALIBRATION_DIR, CONFIG_DIR, PROJECT_ROOT


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"
FLIGHT_SERVICE = "optflow-flight-logger.service"
CALIBRATION_STEP_TUNE = "MFT220L16G"


@dataclass(frozen=True)
class CalibrationDirection:
    key: str
    label: str
    bearing_deg: float


@dataclass(frozen=True)
class DirectionResult:
    direction: CalibrationDirection
    passed: bool
    measured_distance_m: float | None
    measured_angle_deg: float | None
    payload: dict[str, Any]


CARDINAL_DIRECTIONS = (
    CalibrationDirection("forward", "FORWARD", 0.0),
    CalibrationDirection("right", "RIGHT", 90.0),
    CalibrationDirection("rear", "REAR", 180.0),
    CalibrationDirection("left", "LEFT", -90.0),
)


def ordered_directions(start: str) -> tuple[CalibrationDirection, ...]:
    keys = tuple(direction.key for direction in CARDINAL_DIRECTIONS)
    try:
        start_index = keys.index(start)
    except ValueError as exc:
        raise ValueError(f"unknown start direction: {start}") from exc
    return (
        CARDINAL_DIRECTIONS[start_index:]
        + CARDINAL_DIRECTIONS[:start_index]
    )


def direction_result(
    direction: CalibrationDirection,
    payload: dict[str, Any],
) -> DirectionResult:
    target_check = payload.get("target_check")
    if not isinstance(target_check, dict):
        target_check = {}
    sources = target_check.get("sources")
    if not isinstance(sources, dict):
        sources = {}
    lidar = sources.get("lidar")
    if not isinstance(lidar, dict):
        lidar = target_check.get("fused")
    if not isinstance(lidar, dict):
        lidar = {}
    return DirectionResult(
        direction=direction,
        passed=bool(target_check.get("passed")),
        measured_distance_m=_optional_float(
            lidar.get("measured_distance_m")
        ),
        measured_angle_deg=_optional_float(
            lidar.get("measured_sector_angle_deg")
        ),
        payload=payload,
    )


def run_guided_sequence(
    directions: Sequence[CalibrationDirection],
    *,
    check_direction: Callable[[CalibrationDirection], DirectionResult],
    beep: Callable[[], None],
    show_direction: Callable[
        [CalibrationDirection, int, int, DirectionResult | None], None
    ],
    wait_for_positioning: Callable[[], None],
    show_failure: Callable[[DirectionResult], None],
    show_complete: Callable[[Sequence[DirectionResult]], None],
) -> tuple[bool, list[DirectionResult]]:
    if not directions:
        raise ValueError("at least one calibration direction is required")

    results: list[DirectionResult] = []
    previous: DirectionResult | None = None
    total = len(directions)
    for index, direction in enumerate(directions):
        show_direction(direction, index + 1, total, previous)
        wait_for_positioning()
        result = check_direction(direction)
        results.append(result)
        if not result.passed:
            show_failure(result)
            return False, results
        beep()
        previous = result

    show_complete(results)
    return True, results


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _write_screen(text: str) -> None:
    sys.stdout.write(text)
    sys.stdout.flush()


def _format_distance(value: float | None) -> str:
    return "unknown" if value is None else f"{value:.2f} m"


def _show_direction(
    direction: CalibrationDirection,
    index: int,
    total: int,
    previous: DirectionResult | None,
    *,
    target_distance_m: float,
) -> None:
    previous_line = ""
    if previous is not None:
        previous_line = (
            f"PASSED: {previous.direction.label} at "
            f"{_format_distance(previous.measured_distance_m)}\n\n"
        )
    rotation_line = (
        "Keep the current orientation for the first measurement."
        if previous is None
        else "Rotate the aircraft 90 deg counterclockwise now."
    )
    _write_screen(
        "\033[2J\033[H"
        "JT16 CARDINAL CALIBRATION\n"
        "==========================\n\n"
        f"{previous_line}"
        f"NEXT: {direction.label} "
        f"({direction.bearing_deg:+.0f} deg)\n\n"
        f"{rotation_line}\n"
        f"The wall must be on the aircraft's {direction.label} side.\n"
        f"Keep the JT16 center {target_distance_m:.2f} m from the wall.\n"
        "Keep the aircraft level and still during measurement.\n\n"
        f"Direction {index} of {total}\n"
    )


def _countdown(seconds: int) -> None:
    for remaining in range(seconds, 0, -1):
        _write_screen(
            f"\rMeasurement starts in {remaining:2d} seconds... "
        )
        time.sleep(1.0)
    _write_screen("\rStarting measurement now.               \n\n")


def _show_failure(result: DirectionResult) -> None:
    _write_screen(
        "\nCALIBRATION STOPPED\n"
        "===================\n"
        f"{result.direction.label} did not pass.\n"
        f"Measured distance: "
        f"{_format_distance(result.measured_distance_m)}\n"
        f"Measured bearing: "
        f"{result.measured_angle_deg if result.measured_angle_deg is not None else 'unknown'}\n"
        "No completion beep was sent and the next direction was not started.\n"
    )


def _show_complete(results: Sequence[DirectionResult]) -> None:
    lines = [
        "\033[2J\033[H",
        "JT16 CARDINAL CALIBRATION COMPLETE",
        "=================================",
        "",
    ]
    for result in results:
        lines.append(
            f"{result.direction.label:7s} "
            f"{result.direction.bearing_deg:+4.0f} deg  "
            f"{_format_distance(result.measured_distance_m)}"
        )
    lines.extend(
        (
            "",
            "All requested directions passed.",
            "The flight logger is being restored.",
            "",
        )
    )
    _write_screen("\n".join(lines))


class CubeCalibrationLink:
    def __init__(self, config: FlightControllerConfig) -> None:
        self.config = config
        self.master = None
        self.mavutil = None
        self.target_system = config.system_id
        self.target_component = 1

    def open(self) -> None:
        from pymavlink import mavutil

        self.mavutil = mavutil
        self.master = mavutil.mavlink_connection(
            self.config.endpoint,
            baud=self.config.baud,
            source_system=self.config.companion_system_id,
            source_component=self.config.companion_component_id,
        )
        heartbeat = self._wait_for_heartbeat()
        if heartbeat is None:
            raise RuntimeError("Cube heartbeat timed out")
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self._require_disarmed(heartbeat)

    def close(self) -> None:
        if self.master is not None:
            self.master.close()
            self.master = None

    def ensure_disarmed(self) -> None:
        heartbeat = self._wait_for_heartbeat()
        if heartbeat is None:
            raise RuntimeError("Cube heartbeat timed out")
        self._require_disarmed(heartbeat)

    def beep(self) -> None:
        self.ensure_disarmed()
        if self.master is None:
            raise RuntimeError("Cube MAVLink connection is closed")
        self.master.mav.play_tune_send(
            self.target_system,
            self.target_component,
            CALIBRATION_STEP_TUNE.encode("ascii"),
        )

    def _wait_for_heartbeat(self):
        if self.master is None or self.mavutil is None:
            return None
        deadline = time.monotonic() + self.config.heartbeat_timeout_s
        while time.monotonic() < deadline:
            try:
                message = self.master.recv_match(
                    type="HEARTBEAT",
                    blocking=True,
                    timeout=0.5,
                )
            except TypeError as exc:
                if (
                    "NoneType" not in str(exc)
                    or "item assignment" not in str(exc)
                    or not self._repair_pymavlink_instance_cache()
                ):
                    raise
                continue
            if message is None:
                continue
            if (
                message.autopilot
                == self.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                and message.get_srcComponent()
                == self.mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            ):
                return message
        return None

    def _repair_pymavlink_instance_cache(self) -> bool:
        """Repair mixed null/non-null MAVLink instance messages in pymavlink."""

        if self.master is None:
            return False
        repaired = False
        states = getattr(self.master, "sysid_state", {})
        for state in states.values():
            messages = getattr(state, "messages", {})
            for message in messages.values():
                if (
                    getattr(message, "_instance_field", None) is not None
                    and getattr(message, "_instances", None) is None
                ):
                    message._instances = {}
                    repaired = True
        return repaired

    def _require_disarmed(self, heartbeat) -> None:
        if self.mavutil is None:
            raise RuntimeError("MAVLink is unavailable")
        armed = bool(
            heartbeat.base_mode
            & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        if armed:
            raise RuntimeError(
                "guided JT16 calibration is forbidden while armed"
            )


class ObstacleCheckRunner:
    def __init__(
        self,
        *,
        config_path: Path,
        duration_s: float,
        target_distance_m: float,
        tolerance_m: float,
    ) -> None:
        self.config_path = config_path
        self.duration_s = duration_s
        self.target_distance_m = target_distance_m
        self.tolerance_m = tolerance_m

    def __call__(
        self, direction: CalibrationDirection
    ) -> DirectionResult:
        _write_screen(
            f"Measuring {direction.label} for "
            f"{self.duration_s:.0f} seconds...\n"
        )
        command = (
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "check_obstacles.py"),
            "--config",
            str(self.config_path),
            "--no-depth",
            "--duration",
            str(self.duration_s),
            "--target-distance",
            str(self.target_distance_m),
            "--target-angle",
            str(direction.bearing_deg),
            "--target-tolerance",
            str(self.tolerance_m),
        )
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=self.duration_s + 30.0,
            check=False,
        )
        try:
            payload = json.loads(completed.stdout)
        except json.JSONDecodeError as exc:
            detail = completed.stderr.strip() or completed.stdout.strip()
            raise RuntimeError(
                f"JT16 obstacle check produced invalid output: {detail}"
            ) from exc
        if completed.returncode not in (0, 2):
            detail = completed.stderr.strip() or "unknown error"
            raise RuntimeError(f"JT16 obstacle check failed: {detail}")
        return direction_result(direction, payload)


def _service_is_active() -> bool:
    completed = subprocess.run(
        ("systemctl", "--user", "is-active", FLIGHT_SERVICE),
        capture_output=True,
        text=True,
        check=False,
    )
    return completed.returncode == 0 and completed.stdout.strip() == "active"


def _service_action(action: str) -> None:
    completed = subprocess.run(
        ("systemctl", "--user", action, FLIGHT_SERVICE),
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(
            f"unable to {action} {FLIGHT_SERVICE}: {detail}"
        )


def _write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--start",
        choices=tuple(direction.key for direction in CARDINAL_DIRECTIONS),
        default="forward",
    )
    parser.add_argument("--target-distance", type=float, default=2.50)
    parser.add_argument("--target-tolerance", type=float, default=0.08)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--settle-seconds", type=int, default=10)
    parser.add_argument("--no-beep", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if (
            args.target_distance <= 0.0
            or args.target_tolerance <= 0.0
            or args.duration <= 0.0
            or args.settle_seconds <= 0
        ):
            raise ConfigError(
                "distance, tolerance, duration, and settle time "
                "must be positive"
            )
        config_path = args.config.resolve()
        config = load_config(config_path)
        if (
            config.obstacle_avoidance.stage != "shadow"
            or config.obstacle_avoidance.mavlink_output_enabled
        ):
            raise ConfigError(
                "guided calibration requires shadow mode with "
                "MAVLink obstacle output disabled"
            )
    except (ConfigError, OSError) as exc:
        _write_screen(f"JT16 calibration configuration error: {exc}\n")
        return 2

    started = datetime.now(timezone.utc)
    report_path = (
        CALIBRATION_DIR
        / "jt16"
        / "cardinal_runs"
        / f"{started.strftime('%Y%m%dT%H%M%SZ')}.json"
    )
    report: dict[str, Any] = {
        "started_utc": started.isoformat(),
        "start_direction": args.start,
        "target_distance_m": args.target_distance,
        "target_tolerance_m": args.target_tolerance,
        "settle_seconds": args.settle_seconds,
        "duration_s": args.duration,
        "shadow_only": True,
        "steps": [],
        "completed": False,
    }
    _write_report(report_path, report)

    service_was_active = _service_is_active()
    link = CubeCalibrationLink(config.flight_controller)
    exit_code = 2
    try:
        if service_was_active:
            _service_action("stop")
        link.open()
        checker = ObstacleCheckRunner(
            config_path=config_path,
            duration_s=args.duration,
            target_distance_m=args.target_distance,
            tolerance_m=args.target_tolerance,
        )

        def checked(
            direction: CalibrationDirection,
        ) -> DirectionResult:
            link.ensure_disarmed()
            result = checker(direction)
            report["steps"].append(
                {
                    "direction": direction.key,
                    "bearing_deg": direction.bearing_deg,
                    "passed": result.passed,
                    "measured_distance_m": result.measured_distance_m,
                    "measured_angle_deg": result.measured_angle_deg,
                    "payload": result.payload,
                }
            )
            _write_report(report_path, report)
            return result

        def completion_beep() -> None:
            if not args.no_beep:
                link.beep()

        directions = ordered_directions(args.start)
        success, _results = run_guided_sequence(
            directions,
            check_direction=checked,
            beep=completion_beep,
            show_direction=lambda direction, index, total, previous: (
                _show_direction(
                    direction,
                    index,
                    total,
                    previous,
                    target_distance_m=args.target_distance,
                )
            ),
            wait_for_positioning=lambda: _countdown(
                args.settle_seconds
            ),
            show_failure=_show_failure,
            show_complete=_show_complete,
        )
        report["completed"] = success
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        _write_report(report_path, report)
        exit_code = 0 if success else 2
    except (
        OSError,
        RuntimeError,
        subprocess.SubprocessError,
        ValueError,
    ) as exc:
        report["error"] = str(exc)
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        _write_report(report_path, report)
        _write_screen(
            "\nCALIBRATION STOPPED\n"
            "===================\n"
            f"{exc}\n"
        )
    finally:
        link.close()
        if service_was_active:
            try:
                _service_action("start")
            except RuntimeError as exc:
                _write_screen(f"\nFlight logger restore failed: {exc}\n")
                exit_code = 2

    _write_screen(f"\nCalibration report: {report_path}\n")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
