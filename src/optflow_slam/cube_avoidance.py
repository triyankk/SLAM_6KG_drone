"""Inspect and apply Cube-native proximity avoidance parameters."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Any

from .config import ConfigError, ProjectConfig, load_config
from .cube_mount import (
    read_parameters,
    wait_for_cube_heartbeat,
    write_parameter,
)
from .paths import (
    CALIBRATION_DIR,
    CONFIG_DIR,
    PROJECT_ROOT,
    ensure_runtime_directories,
)
from .runtime_lock import cube_mavlink_lock


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"


def active_avoidance_requested(config: ProjectConfig) -> bool:
    obstacle = config.obstacle_avoidance
    return obstacle.stage == "active" and obstacle.mavlink_output_enabled


def desired_avoidance_parameters(
    config: ProjectConfig,
) -> dict[str, float]:
    obstacle = config.obstacle_avoidance
    native = obstacle.native
    active = active_avoidance_requested(config)
    parameters = {
        f"RC{obstacle.rc_toggle.channel}_OPTION": float(
            native.rc_option
        ),
        "PRX1_TYPE": float(native.proximity_type if active else 0),
        "AVOID_ENABLE": float(native.enable_mask if active else 0),
    }
    if active:
        parameters.update(
            {
                "AVOID_MARGIN": obstacle.hard_cg_clearance_m,
                "AVOID_DIST_MAX": obstacle.hard_cg_clearance_m,
                "AVOID_BEHAVE": float(native.behavior),
                "AVOID_BACKUP_SPD": native.backup_speed_mps,
                "AVOID_ACCEL_MAX": native.acceleration_max_mpss,
            }
        )
    return parameters


def _backup_path() -> Path:
    timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    return (
        CALIBRATION_DIR
        / f"cube-avoidance-parameters-before-{timestamp}.json"
    )


def _write_backup(
    config_path: Path,
    target_system: int,
    target_component: int,
    current: dict[str, float],
    desired: dict[str, float],
    active: bool,
) -> Path:
    ensure_runtime_directories()
    resolved_config = config_path.resolve()
    if not resolved_config.is_relative_to(PROJECT_ROOT):
        raise ValueError("Cube avoidance config must remain inside the project")
    payload = {
        "captured_at": datetime.now().astimezone().isoformat(),
        "config_path": str(resolved_config.relative_to(PROJECT_ROOT)),
        "cube": {
            "system_id": target_system,
            "component_id": target_component,
        },
        "active_avoidance_requested": active,
        "current": current,
        "desired": desired,
    }
    path = _backup_path()
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="ascii",
    )
    return path


def _apply_parameters(
    master: Any,
    desired: dict[str, float],
) -> dict[str, float]:
    written: dict[str, float] = {}
    written["AVOID_ENABLE"] = write_parameter(
        master, "AVOID_ENABLE", 0.0
    )
    for name, value in desired.items():
        if name == "AVOID_ENABLE":
            continue
        written[name] = write_parameter(master, name, value)
    written["AVOID_ENABLE"] = write_parameter(
        master,
        "AVOID_ENABLE",
        desired["AVOID_ENABLE"],
    )
    return written


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Back up and apply Cube-native proximity avoidance parameters"
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--apply",
        action="store_true",
        help="write configured values; the default is read-only",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
    except (ConfigError, OSError) as exc:
        print(f"Configuration error: {exc}")
        return 2

    from pymavlink import mavutil

    fc = config.flight_controller
    active = active_avoidance_requested(config)
    desired = desired_avoidance_parameters(config)
    master = None
    hardware_lock = cube_mavlink_lock("Cube avoidance parameter tool")
    try:
        hardware_lock.acquire()
        master = mavutil.mavlink_connection(
            fc.endpoint,
            baud=fc.baud,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        heartbeat = wait_for_cube_heartbeat(
            master, mavutil, fc.heartbeat_timeout_s
        )
        if heartbeat is None:
            raise RuntimeError("Cube heartbeat timed out")
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError("Cube is armed; refusing parameter access")

        master.target_system = heartbeat.get_srcSystem()
        master.target_component = heartbeat.get_srcComponent()
        current_with_types = read_parameters(master, set(desired))
        current = {
            name: value
            for name, (value, _param_type) in current_with_types.items()
        }

        state = "ACTIVE" if active else "SHADOW"
        print(f"Cube is disarmed. Avoidance profile: {state}")
        for name, wanted in desired.items():
            print(
                f"  {name:17} current={current[name]: .4f} "
                f"desired={wanted: .4f}"
            )

        if not args.apply:
            print("Read-only inspection complete. Add --apply to write values.")
            return 0

        backup_path = _write_backup(
            args.config,
            master.target_system,
            master.target_component,
            current,
            desired,
            active,
        )
        print(f"Backup: {backup_path}")
        written = _apply_parameters(master, desired)
        verified_with_types = read_parameters(master, set(desired))
        verified = {
            name: value
            for name, (value, _param_type) in verified_with_types.items()
        }
        for name, wanted in desired.items():
            if not math.isclose(
                verified[name], wanted, rel_tol=0.0, abs_tol=0.0005
            ):
                raise RuntimeError(
                    f"Readback mismatch: {name}={verified[name]}, "
                    f"expected {wanted}"
                )
        print(
            f"Applied and verified {len(written)} Cube avoidance parameters."
        )
        if active:
            print(
                "Active proximity requested; complete props-off sector and "
                "RC-toggle checks before flight."
            )
        else:
            print(
                "Shadow profile retained: RC switch is assigned, while PRX1 "
                "and avoidance remain disabled."
            )
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Cube avoidance configuration failed: {exc}")
        return 2
    finally:
        if master is not None:
            master.close()
        hardware_lock.release()


if __name__ == "__main__":
    raise SystemExit(main())
