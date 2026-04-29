#!/usr/bin/env python3

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STATUS_PATH = REPO_ROOT / "logs" / "slam_calibration_status.json"
DEFAULT_LOG_PATH = REPO_ROOT / "logs" / "slam_calibration.log"
ACTIVE_BRIDGE_PROCESS: subprocess.Popen | None = None


def stop_active_bridge(signum=None, _frame=None) -> None:
    global ACTIVE_BRIDGE_PROCESS
    process = ACTIVE_BRIDGE_PROCESS
    if process is not None and process.poll() is None:
        print("Stopping child SLAM bridge process...")
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except Exception:  # noqa: BLE001
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except Exception:
                pass
    if signum is not None:
        raise SystemExit(128 + int(signum))


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
        print("Start intellisense_slam_bridge.service or run scripts/brake_slam_calibration.py first.")
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


def running_bridge_processes() -> list[str]:
    try:
        output = subprocess.check_output(
            ["ps", "-eo", "pid=,args="],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:  # noqa: BLE001
        return []

    current_pid = os.getpid()
    matches = []
    markers = (
        "scripts/runners/run_slam_odometry_bridge.py",
        "scripts/calibration/brake_slam_calibration.py",
    )
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, command = stripped.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == current_pid:
            continue
        if any(marker in command for marker in markers):
            matches.append(stripped)
    return matches


def refuse_parallel_bridge(config: dict) -> int:
    active = running_bridge_processes()
    if not active:
        return 0

    print("Another SLAM bridge/calibration monitor is already running.")
    print("Do not run two bridge processes at once: they fight over RealSense and MAVLink.")
    for process in active:
        print(f"- {process}")
    print("")
    print("Use this to inspect the running service instead:")
    print("  python3 scripts/calibration/brake_slam_calibration.py --status")
    print("  journalctl -u intellisense_slam_bridge.service -f")
    print("")
    print("If you intentionally want a manual dry-run, stop the service first:")
    print("  sudo systemctl stop intellisense_slam_bridge.service")
    print("")
    print_status(config)
    return 2


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
    print(f"movement_speed_m_s={calibration.get('movement_speed_m_s')}")
    print(f"vertical_speed_m_s={calibration.get('vertical_speed_m_s')}")
    print(f"altitude_hold_gain={calibration.get('altitude_hold_gain')}")
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
    global ACTIVE_BRIDGE_PROCESS
    log_path = log_path_from_config(config)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "runners" / "run_slam_odometry_bridge.py"),
        "--config",
        str(temp_path),
    ]
    with log_path.open("a", encoding="utf-8") as log_handle:
        log_handle.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | state=IDLE mode=unknown armed=unknown landed=unknown on_ground=unknown rng=unknown vio=unknown imu=unknown ekf_extnav=unknown mavlink=unknown action=starting SLAM bridge monitor reason=\n")
        log_handle.flush()
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        ACTIVE_BRIDGE_PROCESS = process
        assert process.stdout is not None
        try:
            for line in process.stdout:
                print(line, end="")
                log_handle.write(line)
                log_handle.flush()
            return process.wait()
        finally:
            stop_active_bridge()
            ACTIVE_BRIDGE_PROCESS = None


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGTERM, stop_active_bridge)
    signal.signal(signal.SIGINT, stop_active_bridge)
    config_path = resolve_path(args.config)
    config = load_config(config_path)
    if args.status:
        return print_status(config)

    apply_runtime_overrides(config, args)
    parallel_result = refuse_parallel_bridge(config)
    if parallel_result:
        return parallel_result
    print_summary(config)
    temp_path = write_temp_config(config)
    return run_bridge_with_local_log(config, temp_path)


if __name__ == "__main__":
    raise SystemExit(main())
