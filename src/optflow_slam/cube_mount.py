"""Inspect and apply the measured Cube mounting transform."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time
from typing import Any

from .config import ConfigError, ProjectConfig, load_config
from .mavlink_compat import install_pymavlink_instance_guard
from .paths import (
    CALIBRATION_DIR,
    CONFIG_DIR,
    PROJECT_ROOT,
    ensure_runtime_directories,
)
from .runtime_lock import cube_mavlink_lock


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"


def desired_mount_parameters(config: ProjectConfig) -> dict[str, float]:
    mount = config.flight_controller.cube_mount
    values = {"AHRS_ORIENTATION": float(mount.ahrs_orientation)}
    for imu_index in (1, 2, 3):
        values.update(
            {
                f"INS_POS{imu_index}_X": mount.x_m,
                f"INS_POS{imu_index}_Y": mount.y_m,
                f"INS_POS{imu_index}_Z": mount.z_m,
            }
        )
    return values


def parameter_name(message: Any) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")


def send_gcs_heartbeat(master: Any) -> None:
    from pymavlink import mavutil

    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def read_parameters(
    master: Any,
    names: set[str],
    timeout_s: float = 45.0,
) -> dict[str, tuple[float, int]]:
    install_pymavlink_instance_guard()
    found: dict[str, tuple[float, int]] = {}
    deadline = time.monotonic() + timeout_s
    last_stream_packet = time.monotonic()
    next_heartbeat = time.monotonic() + 1.0
    next_request = time.monotonic()
    next_missing_request = time.monotonic() + 3.0
    stream_started = False

    while time.monotonic() < deadline:
        now = time.monotonic()
        if not stream_started and now >= next_request:
            send_gcs_heartbeat(master)
            master.mav.param_request_list_send(
                master.target_system,
                master.target_component,
            )
            next_request = now + 3.0
        if now >= next_missing_request:
            for name in sorted(names - found.keys()):
                master.mav.param_request_read_send(
                    master.target_system,
                    master.target_component,
                    name.encode("ascii"),
                    -1,
                )
            next_missing_request = now + 1.0
        if now >= next_heartbeat:
            send_gcs_heartbeat(master)
            next_heartbeat = now + 1.0
        message = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.25
        )
        if message is None:
            if (
                stream_started
                and found.keys() == names
                and now - last_stream_packet > 1.0
            ):
                return found
            continue
        if int(message.param_index) != 65535:
            stream_started = True
            last_stream_packet = time.monotonic()
        name = parameter_name(message)
        if name in names:
            found[name] = (float(message.param_value), int(message.param_type))
            if found.keys() == names:
                return found

    missing = ", ".join(sorted(names - found.keys()))
    if missing:
        raise RuntimeError(f"Cube parameter transfer missed: {missing}")
    return found


def write_parameter(
    master: Any,
    name: str,
    desired: float,
    timeout_s: float = 3.0,
) -> float:
    install_pymavlink_instance_guard()
    for _attempt in range(5):
        send_gcs_heartbeat(master)
        time.sleep(0.1)
        master.param_set_send(name, desired)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = master.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.25
            )
            if message is None or parameter_name(message) != name:
                continue
            actual = float(message.param_value)
            if math.isclose(actual, desired, rel_tol=0.0, abs_tol=0.0005):
                return actual
            break
    raise RuntimeError(f"Cube did not accept {name}={desired}")


def _backup_path() -> Path:
    timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    return CALIBRATION_DIR / f"cube-mount-parameters-before-{timestamp}.json"


def wait_for_cube_heartbeat(master: Any, mavutil: Any, timeout_s: float) -> Any:
    install_pymavlink_instance_guard(mavutil)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(
            type="HEARTBEAT", blocking=True, timeout=0.5
        )
        if (
            message is not None
            and message.autopilot
            == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            and message.get_srcComponent()
            == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        ):
            return message
    return None


def _write_backup(
    config_path: Path,
    target_system: int,
    target_component: int,
    current: dict[str, float],
    desired: dict[str, float],
) -> Path:
    ensure_runtime_directories()
    path = _backup_path()
    resolved_config = config_path.resolve()
    if not resolved_config.is_relative_to(PROJECT_ROOT):
        raise ValueError("Cube mount config must remain inside the project")
    payload = {
        "captured_at": datetime.now().astimezone().isoformat(),
        "config_path": str(resolved_config.relative_to(PROJECT_ROOT)),
        "cube": {
            "system_id": target_system,
            "component_id": target_component,
        },
        "current": current,
        "desired": desired,
    }
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="ascii",
    )
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Back up and apply the measured Cube mounting transform"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--apply",
        action="store_true",
        help="write the configured values; the default is read-only",
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
    desired = desired_mount_parameters(config)
    master = None
    hardware_lock = cube_mavlink_lock("Cube mount parameter tool")
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

        print("Cube is disarmed. Mount parameter comparison:")
        for name, wanted in desired.items():
            print(f"  {name:17} current={current[name]: .4f} desired={wanted: .4f}")

        if not args.apply:
            print("Read-only inspection complete. Add --apply to write values.")
            return 0

        backup_path = _write_backup(
            args.config,
            master.target_system,
            master.target_component,
            current,
            desired,
        )
        print(f"Backup: {backup_path}")

        written = {
            name: write_parameter(master, name, value)
            for name, value in desired.items()
        }
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
                    f"Readback mismatch: {name}={verified[name]}, expected {wanted}"
                )
        print(f"Applied and verified {len(written)} Cube mount parameters.")
        print("Re-run accelerometer calibration before arming or flight.")
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Cube mount configuration failed: {exc}")
        return 2
    finally:
        if master is not None:
            master.close()
        hardware_lock.release()


if __name__ == "__main__":
    raise SystemExit(main())
