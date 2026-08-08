"""Run one guided SLAM shadow flight per boot, then resume passive logging."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
from typing import Any

from .config import ConfigError, load_config
from .lio_shadow import run_shadow
from .paths import PROJECT_ROOT, RUNTIME_DIR


DEFAULT_CONFIG = PROJECT_ROOT / "config" / "system.yaml"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "data" / "recordings" / "slam_flights"
DEFAULT_STATUS_PATH = RUNTIME_DIR / "flight_supervisor_status.json"
BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _read_boot_id(path: Path = BOOT_ID_PATH) -> str:
    return path.read_text(encoding="ascii").strip()


def _read_status(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _write_status(path: Path, **values: Any) -> None:
    payload = {
        "schema_version": 1,
        "updated_utc": _utc_now(),
        **values,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _shadow_completed_this_boot(status: dict[str, Any], boot_id: str) -> bool:
    return bool(
        status.get("boot_id") == boot_id
        and status.get("state") == "shadow_flight_complete"
        and status.get("completed_arm_cycle") is True
    )


def _exec_passive_logger() -> None:
    logger = PROJECT_ROOT / "scripts" / "flight_logger_service.py"
    print(
        "SLAM shadow flight complete for this boot; "
        "continuing with the passive arm-triggered logger.",
        flush=True,
    )
    os.execv(sys.executable, (sys.executable, str(logger)))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS_PATH)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        boot_id = _read_boot_id()
        previous = _read_status(args.status_file)
        if _shadow_completed_this_boot(previous, boot_id):
            _exec_passive_logger()

        config = load_config(args.config)
        _write_status(
            args.status_file,
            state="shadow_starting",
            boot_id=boot_id,
            completed_arm_cycle=False,
            report=None,
            result=None,
            active_control_sent=False,
        )
        print(
            "Boot SLAM flight supervisor starting. It sends QGC instructions "
            "and tunes only; the pilot retains full control in Loiter.",
            flush=True,
        )
        _write_status(
            args.status_file,
            state="shadow_waiting_for_flight",
            boot_id=boot_id,
            completed_arm_cycle=False,
            report=None,
            result=None,
            active_control_sent=False,
        )
        report_path, report, digest = run_shadow(
            config,
            args.config,
            output_root=args.output_root,
            duration_s=0.0,
            visual_host="127.0.0.1",
            visual_port=8767,
            open_browser=False,
            slam_poc=True,
            flight_shadow=True,
        )
        lifecycle = report.get("flight_lifecycle", {})
        completed_arm_cycle = bool(lifecycle.get("completed_arm_cycle"))
        _write_status(
            args.status_file,
            state=(
                "shadow_flight_complete"
                if completed_arm_cycle
                else "shadow_stopped_without_flight"
            ),
            boot_id=boot_id,
            completed_arm_cycle=completed_arm_cycle,
            report=str(report_path),
            report_sha256=digest,
            result=report.get("result"),
            failed_gates=report.get("failed_gates", []),
            active_control_sent=False,
            qgc_guidance=report.get("qgc_guidance"),
        )
        print(
            json.dumps(
                {
                    "result": report.get("result"),
                    "report": str(report_path),
                    "sha256": digest,
                    "completed_arm_cycle": completed_arm_cycle,
                    "active_control_sent": False,
                },
                indent=2,
                sort_keys=True,
            ),
            flush=True,
        )
        if not completed_arm_cycle:
            return 1
        _exec_passive_logger()
        return 0
    except (ConfigError, OSError, RuntimeError, ValueError) as exc:
        try:
            _write_status(
                args.status_file,
                state="failed",
                boot_id=_read_boot_id(),
                completed_arm_cycle=False,
                error=str(exc),
                active_control_sent=False,
            )
        except OSError:
            pass
        print(f"Flight supervisor error: {exc}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
