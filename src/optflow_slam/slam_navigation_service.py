"""Run the boot-owned SLAM mapper and guarded GPS-denied return supervisor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from .config import ConfigError, ProjectConfig, load_config
from .lio_shadow import run_shadow
from .mavlink_compat import install_pymavlink_instance_guard
from .paths import PROJECT_ROOT
from .runtime_lock import RuntimeLockError, cube_mavlink_lock
from .slam_navigation import audit_parameter_names
from .slam_return_setup import read_parameter


DEFAULT_CONFIG = PROJECT_ROOT / "config" / "system.yaml"
DEFAULT_OUTPUT_ROOT = (
    PROJECT_ROOT / "data" / "recordings" / "slam_navigation"
)
DEFAULT_UART_ENDPOINT = Path("/dev/ttyTHS1")
DEFAULT_UART_PREARM_DURATION_S = 35.0
UART_REOPEN_SETTLE_S = 1.0


def read_initial_cube_parameters(
    config: ProjectConfig,
) -> dict[str, float]:
    """Read the startup audit before high-rate sensor streams begin."""
    from pymavlink import mavutil

    install_pymavlink_instance_guard(mavutil)

    connection = mavutil.mavlink_connection(
        config.flight_controller.endpoint,
        baud=config.flight_controller.baud,
        source_system=config.flight_controller.companion_system_id,
        source_component=config.flight_controller.companion_component_id,
        autoreconnect=False,
        robust_parsing=True,
    )
    try:
        heartbeat = None
        deadline = (
            time.monotonic()
            + config.flight_controller.heartbeat_timeout_s
        )
        while time.monotonic() < deadline:
            candidate = connection.recv_match(
                type="HEARTBEAT", blocking=True, timeout=0.5
            )
            if candidate is None:
                continue
            if (
                int(candidate.autopilot)
                == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                and candidate.get_srcComponent()
                == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            ):
                heartbeat = candidate
                break
        if heartbeat is None:
            raise RuntimeError("Cube heartbeat missing during startup audit")
        names = audit_parameter_names(config.navigation.slam_return)
        parameters: dict[str, float] = {}
        for _pass in range(4):
            for name in names:
                if name in parameters:
                    continue
                value = read_parameter(
                    connection,
                    heartbeat.get_srcSystem(),
                    heartbeat.get_srcComponent(),
                    name,
                    timeout_s=0.35,
                    attempts=1,
                )
                if value is not None:
                    parameters[name] = value
            if len(parameters) == len(names):
                break
        return parameters
    finally:
        connection.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run; zero runs until the service is stopped",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
        return_config = config.navigation.slam_return
        print(
            "SLAM return runtime starting: "
            f"stage={return_config.stage}, "
            f"live_output={return_config.live_control_enabled}, "
            f"request=RC{return_config.rc_channel} high.",
            flush=True,
        )
        print(
            "The process never arms, takes off, lands, or selects a mode.",
            flush=True,
        )
        with cube_mavlink_lock("boot SLAM navigation service"):
            initial_parameters = read_initial_cube_parameters(config)
            print(
                "Cube startup audit received "
                f"{len(initial_parameters)}/"
                f"{len(audit_parameter_names(return_config))} parameters.",
                flush=True,
            )
            time.sleep(UART_REOPEN_SETTLE_S)
            report_path, report, digest = run_shadow(
                config,
                args.config,
                output_root=args.output_root,
                duration_s=args.duration,
                open_browser=False,
                slam_poc=True,
                slam_navigation=True,
                initial_cube_parameters=initial_parameters,
            )
        print(
            json.dumps(
                {
                    "result": report.get("result"),
                    "report": str(report_path),
                    "sha256": digest,
                    "control_permitted": report.get("control_permitted"),
                    "commands_sent": report.get("commands_sent", 0),
                },
                indent=2,
                sort_keys=True,
            ),
            flush=True,
        )
        return 0
    except (
        ConfigError,
        OSError,
        RuntimeError,
        RuntimeLockError,
        ValueError,
    ) as exc:
        print(f"SLAM return service error: {exc}", flush=True)
        return 2


def status_main() -> int:
    parser = argparse.ArgumentParser(
        description="Show the live guarded SLAM return state"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        status_path = Path(config.navigation.slam_return.status_file)
        if not status_path.is_absolute():
            status_path = PROJECT_ROOT / status_path
        payload = json.loads(status_path.read_text(encoding="utf-8"))
        updated_ns = int(payload.get("updated_unix_ns", 0))
        age_s = max(0.0, (time.time_ns() - updated_ns) / 1.0e9)
        summary = {
            "state": payload.get("state"),
            "stage": payload.get("stage"),
            "live": age_s <= 1.5,
            "age_s": round(age_s, 2),
            "control_permitted": payload.get("live_control_permitted"),
            "approval_reason": payload.get("approval_reason"),
            "velocity_output_started": payload.get("velocity_output_started"),
            "failed_health_gates": payload.get("failed_health_gates", []),
            "cube": payload.get("cube"),
            "command": payload.get("command"),
            "flight_test": payload.get("flight_test"),
            "odometry_guard": payload.get("odometry_guard"),
            "transport": payload.get("transport"),
            "status_file": str(status_path),
        }
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary["live"] else 1
    except (ConfigError, FileNotFoundError, OSError, ValueError) as exc:
        print(f"SLAM return status unavailable: {exc}")
        return 2


def _decode_message_text(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, (bytes, bytearray, memoryview)):
        text = bytes(value).decode("utf-8", errors="replace")
    else:
        text = str(value)
    return text.rstrip("\x00").strip()


def _read_live_uart_prearm(
    config: ProjectConfig,
    *,
    endpoint: Path,
    baud: int,
    duration_s: float,
) -> int:
    """Receive Cube arming text from a UART without transmitting MAVLink."""
    from pymavlink import mavutil

    if not 1.0 <= duration_s <= 120.0:
        raise ValueError("UART sample duration must be between 1 and 120 s")
    if baud <= 0:
        raise ValueError("UART baud must be positive")

    install_pymavlink_instance_guard(mavutil)
    print(
        f"Listening receive-only on {endpoint} at {baud} baud for "
        f"{duration_s:g}s...",
        flush=True,
    )
    with cube_mavlink_lock("receive-only Cube UART pre-arm monitor"):
        connection = mavutil.mavlink_connection(
            str(endpoint),
            baud=baud,
            source_system=config.flight_controller.companion_system_id,
            source_component=(
                config.flight_controller.companion_component_id
            ),
            autoreconnect=False,
            robust_parsing=True,
        )
        heartbeat_seen = False
        armed: bool | None = None
        status_texts: dict[str, tuple[int | None, str]] = {}
        warnings: dict[str, tuple[int | None, str]] = {}
        try:
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                message = connection.recv_match(
                    type=["HEARTBEAT", "STATUSTEXT"],
                    blocking=True,
                    timeout=min(0.5, max(0.0, deadline - time.monotonic())),
                )
                if message is None:
                    continue
                if (
                    message.get_srcSystem()
                    != config.flight_controller.system_id
                    or message.get_srcComponent()
                    != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                ):
                    continue
                message_type = message.get_type()
                if message_type == "HEARTBEAT":
                    if (
                        int(message.autopilot)
                        != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                    ):
                        continue
                    heartbeat_seen = True
                    armed = bool(
                        int(message.base_mode)
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                elif message_type == "STATUSTEXT":
                    data = message.to_dict()
                    text = _decode_message_text(data.get("text"))
                    if not text:
                        continue
                    try:
                        severity = int(data.get("severity"))
                    except (TypeError, ValueError):
                        severity = None
                    key = text.casefold()
                    status_texts[key] = (severity, text)
                    if (
                        key.startswith(("prearm:", "arm:"))
                        or (
                            severity is not None
                            and severity
                            <= mavutil.mavlink.MAV_SEVERITY_WARNING
                        )
                    ):
                        warnings[key] = (severity, text)
        finally:
            connection.close()

    print(f"CUBE_HEARTBEAT={'received' if heartbeat_seen else 'missing'}")
    print(
        "CUBE_ARMED="
        + ("unknown" if armed is None else str(armed).lower())
    )
    print(
        "STATUSTEXT_STREAM="
        + ("received" if status_texts else "missing")
    )
    print(f"STATUSTEXT_MESSAGES={len(status_texts)}")
    for severity, text in status_texts.values():
        print(f"- severity={severity} {text}")
    print(f"ARMING_RELEVANT_WARNINGS={len(warnings)}")
    for severity, text in warnings.values():
        print(f"- severity={severity} {text}")
    if not status_texts:
        print(
            "No STATUSTEXT was received, so current pre-arm errors cannot "
            "be determined from this UART sample."
        )
    elif not warnings:
        print("No arming-related warning was present in this sample.")
    return 0 if heartbeat_seen and status_texts else 1


def prearm_status_main() -> int:
    """Show cached or live-UART Cube arming failures."""
    parser = argparse.ArgumentParser(
        description="Show recent Cube PreArm/Arm telemetry errors"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--live-uart",
        action="store_true",
        help="Read receive-only telemetry directly from a stopped service's UART",
    )
    parser.add_argument(
        "--uart-endpoint", type=Path, default=DEFAULT_UART_ENDPOINT
    )
    parser.add_argument("--baud", type=int)
    parser.add_argument(
        "--duration", type=float, default=DEFAULT_UART_PREARM_DURATION_S
    )
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        if args.live_uart:
            return _read_live_uart_prearm(
                config,
                endpoint=args.uart_endpoint,
                baud=(
                    config.flight_controller.baud
                    if args.baud is None
                    else args.baud
                ),
                duration_s=args.duration,
            )
        status_path = Path(config.navigation.slam_return.status_file)
        if not status_path.is_absolute():
            status_path = PROJECT_ROOT / status_path
        payload = json.loads(status_path.read_text(encoding="utf-8"))
        updated_ns = int(payload.get("updated_unix_ns", 0))
        age_s = max(0.0, (time.time_ns() - updated_ns) / 1.0e9)
        service_live = age_s <= 1.5
        health_gates = payload.get("health_gates", {})
        telemetry_live = bool(
            service_live and health_gates.get("cube_heartbeat_fresh")
        )
        cube = payload.get("cube", {})

        print(
            f"SERVICE={'live' if service_live else 'stale'} "
            f"age={age_s:.2f}s"
        )
        print(
            f"CUBE_LINK={config.flight_controller.endpoint} "
            f"baud={config.flight_controller.baud}"
        )
        print(
            "CUBE_TELEMETRY="
            f"{'live' if telemetry_live else 'stale_or_missing'}"
        )
        if "prearm_errors" not in cube:
            print(
                "STATUSTEXT_CACHE=unavailable; restart "
                "optflow-flight-logger.service"
            )
            return 1

        status_texts = cube.get("recent_status_texts") or []
        warnings = [
            row
            for row in status_texts
            if str(row.get("text", ""))
            .casefold()
            .startswith(("prearm:", "arm:"))
            or (
                isinstance(row.get("severity"), int)
                and row["severity"] <= 4
            )
        ]
        window_s = float(cube.get("status_text_window_s", 0.0))
        print(
            "STATUSTEXT_STREAM="
            + ("recent" if status_texts else "missing")
        )
        print(f"STATUSTEXT_MESSAGES={len(status_texts)} window={window_s:g}s")
        for row in status_texts:
            severity = row.get("severity")
            age = float(row.get("age_s", 0.0))
            print(
                f"- severity={severity} age={age:.2f}s "
                f"{row.get('text', '')}"
            )
        print(f"ARMING_RELEVANT_WARNINGS={len(warnings)}")
        if not status_texts:
            print(
                "No recent STATUSTEXT is cached, so current pre-arm errors "
                "cannot be determined from this service status."
            )
        return 0 if service_live and telemetry_live and status_texts else 1
    except (
        ConfigError,
        FileNotFoundError,
        OSError,
        RuntimeError,
        RuntimeLockError,
        TypeError,
        ValueError,
    ) as exc:
        print(f"Cube pre-arm status unavailable: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
