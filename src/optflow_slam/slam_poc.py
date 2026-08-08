"""Run the shortest safe, visible proof of the SLAM/VIO sensor pipeline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

from .config import ConfigError, load_config
from .lio_shadow import run_shadow
from .paths import PROJECT_ROOT


FLIGHT_SERVICE = "optflow-flight-logger.service"


def _service_active() -> bool:
    completed = subprocess.run(
        ("systemctl", "--user", "is-active", FLIGHT_SERVICE),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.returncode == 0


def _service_action(action: str) -> None:
    completed = subprocess.run(
        ("systemctl", "--user", action, FLIGHT_SERVICE),
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"flight logger {action} failed: {detail}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "system.yaml",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help=(
            "Seconds to record; zero uses dashboard stop for bench proof or "
            "post-disarm stop for flight shadow"
        ),
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "--flight-shadow",
        action="store_true",
        help=(
            "allow a pilot-flown arm cycle while all Cube control outputs "
            "remain disabled"
        ),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    service_was_active = False
    try:
        config = load_config(args.config)
        service_was_active = _service_active()
        if service_was_active:
            _service_action("stop")
        if args.flight_shadow:
            print(
                "ARMED FLIGHT SHADOW: pilot retains control in GPS Loiter. "
                "No pose, obstacle, mode, or velocity command is sent to Cube.",
                flush=True,
            )
        else:
            print(
                "Shadow proof only: keep the Cube disarmed and props removed. "
                "No pose or navigation command is sent to Cube.",
                flush=True,
            )
        output_root = args.output_root or (
            PROJECT_ROOT
            / "data"
            / "recordings"
            / ("slam_flights" if args.flight_shadow else "slam_poc")
        )
        report_path, report, digest = run_shadow(
            config,
            args.config,
            output_root=output_root,
            duration_s=args.duration,
            visual_host=args.host,
            visual_port=args.port,
            open_browser=not args.no_browser,
            slam_poc=True,
            flight_shadow=args.flight_shadow,
        )
        print(
            json.dumps(
                {
                    "result": report["result"],
                    "detail": report["detail"],
                    "report": str(report_path),
                    "sha256": digest,
                    "pose_sent_to_cube": False,
                    "obstacle_output_sent_to_cube": False,
                    "velocity_sent_to_cube": False,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0 if report["result"] in {"pass", "flight_shadow_pass"} else 1
    except (ConfigError, OSError, RuntimeError, ValueError) as exc:
        print(f"SLAM POC error: {exc}")
        return 2
    finally:
        if service_was_active and not _service_active():
            try:
                _service_action("start")
            except RuntimeError as exc:
                print(f"Warning: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
