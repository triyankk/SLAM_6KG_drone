#!/usr/bin/env python3

import argparse
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STATUS_PATH = REPO_ROOT / "logs" / "slam_calibration_status.json"
DEFAULT_LOG_PATH = REPO_ROOT / "logs" / "slam_calibration.log"


def parse_args():
    parser = argparse.ArgumentParser(description="Headless Brake-mode SLAM/VIO calibration service.")
    parser.add_argument("--config", default="config/autostart.yaml")
    parser.add_argument("--dry-run", action="store_true", help="Monitor and announce, but do not send odometry, movement, RTL, or fallback commands.")
    parser.add_argument("--status", action="store_true", help="Print the latest calibration state and health summary.")
    motion_group = parser.add_mutually_exclusive_group()
    motion_group.add_argument("--disable-motion", action="store_true", help="Force calibration movement commands off.")
    motion_group.add_argument("--enable-motion", action="store_true", help="Enable bounded pitch/roll/yaw nudges after safety checks.")
    return parser.parse_args()


def resolve_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    return path if path.is_absolute() else (REPO_ROOT / path).resolve()


def load_config(path: Path) -> dict:
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"Config must be a mapping: {path}")
    return payload


def status_path_from_config(config: dict) -> Path:
    calibration = config.get("calibration", {}) or {}
    return resolve_path(str(calibration.get("status_path", DEFAULT_STATUS_PATH)))


def log_path_from_config(config: dict) -> Path:
    calibration = config.get("calibration", {}) or {}
    return resolve_path(str(calibration.get("log_path", DEFAULT_LOG_PATH)))


def print_status(config: dict) -> int:
    status_path = status_path_from_config(config)
    if not status_path.exists():
        print(f"No status file yet: {status_path}")
        print("Start vio-flight.service or run scripts/brake_slam_calibration.py first.")
        return 1
    payload = json.loads(status_path.read_text(encoding="utf-8"))
    print("Latest SLAM calibration status")
    print(f"timestamp={payload.get('timestamp')}")
    print(f"state={payload.get('state')}")
    print(f"stage={payload.get('stage')}")
    print(f"mode={payload.get('mode')}")
    print(f"armed={payload.get('armed')}")
    print(f"landed_state={payload.get('landed_state')}")
    print(f"on_ground={payload.get('on_ground')}")
    print(f"rangefinder_height_m={payload.get('rangefinder_height_m')}")
    print(f"vio={payload.get('vio_health')} quality={payload.get('vio_quality')} tracking={payload.get('vio_tracking')}")
    print(f"imu={payload.get('imu_stability')}")
    print(f"ekf_external_nav={payload.get('ekf_external_nav_status')}")
    print(f"mavlink={payload.get('mavlink_status')}")
    print(f"rc_link={payload.get('rc_link')}")
    print(f"battery_remaining_pct={payload.get('battery_remaining_pct')}")
    print(f"action={payload.get('current_action')}")
    print(f"failure_reason={payload.get('failure_reason')}")
    print(f"dry_run={payload.get('dry_run')} movement_enabled={payload.get('movement_enabled')}")
    return 0


def apply_runtime_overrides(config: dict, args) -> None:
    calibration = config.setdefault("calibration", {})
    for key in ("profile_path", "status_path", "log_path"):
        if calibration.get(key):
            calibration[key] = str(resolve_path(str(calibration[key])))
    if args.dry_run:
        calibration["dry_run"] = True
        calibration["movement_commands_enabled"] = False
        calibration["auto_rtl_after_complete"] = False
        calibration["fallback_mode"] = ""
        config.setdefault("fc_setup", {})["select_source_set_on_stream"] = False
        config.setdefault("lidar_steering", {})["enabled"] = False
    if args.disable_motion:
        calibration["movement_commands_enabled"] = False
    if args.enable_motion:
        calibration["movement_commands_enabled"] = True


def print_summary(config: dict) -> None:
    calibration = config.get("calibration", {}) or {}
    fc_setup = config.get("fc_setup", {}) or {}
    print("Brake SLAM calibration monitor settings")
    print(f"source={config.get('source')}")
    print(f"ports={config.get('ports')}")
    print(f"calibration_mode={calibration.get('mode')}")
    print(f"target_height_m={calibration.get('target_height_m')}")
    print(f"dry_run={calibration.get('dry_run')}")
    print(f"movement_commands_enabled={calibration.get('movement_commands_enabled')}")
    print(f"kill_switch_confirmed={calibration.get('kill_switch_confirmed')}")
    print(f"fallback_mode={calibration.get('fallback_mode')}")
    print(f"auto_rtl_after_complete={calibration.get('auto_rtl_after_complete')}")
    print(f"slam_source_set={fc_setup.get('slam_source_set')}")
    print(f"status_path={calibration.get('status_path')}")
    print(f"log_path={calibration.get('log_path')}")
    print("No automatic takeoff is commanded by this monitor.")


def write_temp_config(config: dict) -> Path:
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False)
        return Path(handle.name)


def run_bridge_with_local_log(config: dict, temp_path: Path) -> int:
    log_path = log_path_from_config(config)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "run_slam_odometry_bridge.py"),
        "--config",
        str(temp_path),
    ]
    with log_path.open("a", encoding="utf-8") as log_handle:
        log_handle.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | state=IDLE mode=unknown armed=unknown landed=unknown on_ground=unknown rng=unknown vio=unknown imu=unknown ekf_extnav=unknown mavlink=unknown action=starting vio-flight monitor reason=\n")
        log_handle.flush()
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="")
            log_handle.write(line)
            log_handle.flush()
        return process.wait()


def main() -> int:
    args = parse_args()
    config_path = resolve_path(args.config)
    config = load_config(config_path)
    if args.status:
        return print_status(config)

    apply_runtime_overrides(config, args)
    print_summary(config)
    temp_path = write_temp_config(config)
    return run_bridge_with_local_log(config, temp_path)


if __name__ == "__main__":
    raise SystemExit(main())
