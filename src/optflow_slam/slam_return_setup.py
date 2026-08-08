"""Audit the Cube contract and optionally shorten the Guided command timeout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

from .config import ConfigError, load_config
from .mavlink_compat import install_pymavlink_instance_guard
from .paths import PROJECT_ROOT
from .runtime_lock import cube_mavlink_lock
from .slam_navigation import (
    LAND_RC_OPTION,
    audit_cube_parameters,
    audit_parameter_names,
)


GUIDED_TIMEOUT_S = 0.5
MAV_PARAM_TYPE_REAL32 = 9
TELEM1_RATE_PROFILE = {
    "SR1_RAW_SENS": 1.0,
    "SR1_EXT_STAT": 2.0,
    "SR1_RC_CHAN": 2.0,
    "SR1_RAW_CTRL": 0.0,
    "SR1_POSITION": 3.0,
    "SR1_EXTRA1": 5.0,
    "SR1_EXTRA2": 2.0,
    "SR1_EXTRA3": 2.0,
    "SR1_PARAMS": 0.0,
    "SR1_ADSB": 0.0,
}


def _parameter_name(message: Any) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")


def read_parameter(
    connection: Any,
    target_system: int,
    target_component: int,
    name: str,
    *,
    timeout_s: float = 2.0,
    attempts: int = 3,
) -> float | None:
    for _attempt in range(attempts):
        connection.mav.param_request_read_send(
            target_system,
            target_component,
            name.encode("ascii"),
            -1,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = connection.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.2
            )
            if message is None or _parameter_name(message) != name:
                continue
            return float(message.param_value)
    return None


def write_parameter(
    connection: Any,
    target_system: int,
    target_component: int,
    name: str,
    value: float,
) -> float:
    connection.mav.param_set_send(
        target_system,
        target_component,
        name.encode("ascii"),
        float(value),
        MAV_PARAM_TYPE_REAL32,
    )
    confirmed = read_parameter(
        connection,
        target_system,
        target_component,
        name,
    )
    if confirmed is None or not abs(confirmed - value) < 0.01:
        raise RuntimeError(f"Cube did not confirm {name}={value:g}")
    return confirmed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "system.yaml",
    )
    parser.add_argument(
        "--apply-guided-timeout",
        action="store_true",
        help="set only GUID_TIMEOUT to 0.5 seconds while disarmed",
    )
    parser.add_argument(
        "--apply-channel-map",
        action="store_true",
        help="apply the configured raw return channel and Cube LAND channel",
    )
    parser.add_argument(
        "--apply-telem1-profile",
        action="store_true",
        help="apply conservative SR1 rates for the 57600-baud QGC link",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    connection = None
    hardware_lock = cube_mavlink_lock("SLAM return parameter tool")
    try:
        from pymavlink import mavutil

        install_pymavlink_instance_guard(mavutil)
        hardware_lock.acquire()
        config = load_config(args.config)
        controller_settings = config.navigation.slam_return
        connection = mavutil.mavlink_connection(
            config.flight_controller.endpoint,
            baud=config.flight_controller.baud,
            source_system=config.flight_controller.companion_system_id,
            source_component=config.flight_controller.companion_component_id,
            autoreconnect=False,
            robust_parsing=True,
        )
        heartbeat = None
        deadline = time.monotonic() + config.flight_controller.heartbeat_timeout_s
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
            raise RuntimeError("Cube autopilot heartbeat was not received")
        if int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError("Cube is armed; setup is read-only while flying")
        target_system = heartbeat.get_srcSystem()
        target_component = heartbeat.get_srcComponent()
        parameter_names = list(audit_parameter_names(controller_settings))
        if args.apply_telem1_profile:
            parameter_names.extend(TELEM1_RATE_PROFILE)
        parameter_names = list(dict.fromkeys(parameter_names))
        parameters: dict[str, float] = {}
        for name in parameter_names:
            value = read_parameter(
                connection,
                target_system,
                target_component,
                name,
            )
            if value is not None:
                parameters[name] = value

        writes = 0
        parameter_changes: dict[str, dict[str, float]] = {}
        if args.apply_channel_map:
            desired_channels = (
                (f"RC{controller_settings.rc_channel}_OPTION", 0.0),
                (
                    f"RC{controller_settings.land_rc_channel}_OPTION",
                    LAND_RC_OPTION,
                ),
            )
            missing = [name for name, _desired in desired_channels if name not in parameters]
            if missing:
                raise RuntimeError(
                    "Cube channel parameter is missing: " + ", ".join(missing)
                )
            for name, desired in desired_channels:
                before = parameters.get(name)
                assert before is not None
                if abs(before - desired) < 0.01:
                    continue
                confirmed = write_parameter(
                    connection,
                    target_system,
                    target_component,
                    name,
                    desired,
                )
                parameters[name] = confirmed
                parameter_changes[name] = {
                    "before": before,
                    "after": confirmed,
                }
                writes += 1
        if args.apply_telem1_profile:
            missing = sorted(set(TELEM1_RATE_PROFILE) - parameters.keys())
            if missing:
                raise RuntimeError(
                    "Cube TELEM1 parameter is missing: " + ", ".join(missing)
                )
            for name, desired in TELEM1_RATE_PROFILE.items():
                before = parameters[name]
                if abs(before - desired) < 0.01:
                    continue
                confirmed = write_parameter(
                    connection,
                    target_system,
                    target_component,
                    name,
                    desired,
                )
                parameters[name] = confirmed
                parameter_changes[name] = {
                    "before": before,
                    "after": confirmed,
                }
                writes += 1
        if args.apply_guided_timeout:
            before = parameters["GUID_TIMEOUT"]
            confirmed = write_parameter(
                connection,
                target_system,
                target_component,
                "GUID_TIMEOUT",
                GUIDED_TIMEOUT_S,
            )
            parameters["GUID_TIMEOUT"] = confirmed
            if abs(before - confirmed) >= 0.01:
                parameter_changes["GUID_TIMEOUT"] = {
                    "before": before,
                    "after": confirmed,
                }
                writes += 1

        gates, detail = audit_cube_parameters(
            parameters, controller_settings
        )
        result = {
            "schema_version": 1,
            "result": "pass" if gates and all(gates.values()) else "blocked",
            "detail": detail,
            "armed": False,
            "mode": mavutil.mode_string_v10(heartbeat),
            "parameter_writes": writes,
            "parameter_changes": parameter_changes,
            "parameters": dict(sorted(parameters.items())),
            "gates": gates,
            "failed_gates": [
                name for name, passed in gates.items() if not passed
            ],
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["result"] == "pass" else 1
    except (ConfigError, OSError, RuntimeError, ValueError) as exc:
        print(f"SLAM return setup error: {exc}")
        return 2
    finally:
        if connection is not None:
            connection.close()
        hardware_lock.release()


if __name__ == "__main__":
    raise SystemExit(main())
