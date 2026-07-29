"""Check the three Jetson SLAM sensors without opening the Cube UART."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

from .config import ConfigError, ProjectConfig, load_config
from .flight_service import DEFAULT_STATUS_PATH
from .models import ProbeResult
from .paths import CONFIG_DIR
from .readiness import probe_depth_camera, probe_lidar


def _read_logger_status(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        updated = datetime.fromisoformat(str(payload["updated_utc"]))
    except (
        FileNotFoundError,
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
    ) as exc:
        return None, f"automatic logger status unavailable: {exc}"
    age_s = (datetime.now(timezone.utc) - updated).total_seconds()
    if age_s > 3.0:
        return None, f"automatic logger status is stale by {age_s:.1f} s"
    return payload, None


def evaluate_live_imu(
    status: dict[str, Any], config: ProjectConfig
) -> ProbeResult:
    sensors = status.get("sensors", {})
    connected = bool(sensors.get("external_imu_connected"))
    age_ms = sensors.get("external_imu_age_ms")
    rate_hz = float(sensors.get("external_imu_rate_hz") or 0.0)
    maximum_age_ms = max(
        500.0, 3_000.0 / config.external_imu.expected_rate_hz
    )
    available = (
        connected
        and age_ms is not None
        and float(age_ms) <= maximum_age_ms
        and rate_hz >= 0.5 * config.external_imu.expected_rate_hz
    )
    return ProbeResult(
        "external_imu",
        available,
        (
            f"{config.external_imu.model} connected={connected}; "
            f"rate={rate_hz:.2f} Hz; age={age_ms} ms"
        ),
        {
            "connected": connected,
            "rate_hz": rate_hz,
            "age_ms": age_ms,
        },
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=CONFIG_DIR / "system.yaml"
    )
    parser.add_argument(
        "--status-file", type=Path, default=DEFAULT_STATUS_PATH
    )
    parser.add_argument("--json", action="store_true", dest="as_json")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
    except (ConfigError, OSError) as exc:
        print(f"Configuration error: {exc}")
        return 2

    status, status_error = _read_logger_status(args.status_file)
    if status is None:
        results = (
            ProbeResult("external_imu", False, status_error or "unknown"),
            ProbeResult(
                "depth_camera",
                False,
                "not probed without fresh disarmed logger status",
            ),
            ProbeResult(
                "lidar",
                False,
                "not probed without fresh disarmed logger status",
            ),
        )
    elif bool(status.get("vehicle", {}).get("armed")):
        results = (
            ProbeResult(
                "external_imu",
                False,
                "aircraft is armed; sensor bench check refused",
            ),
            ProbeResult(
                "depth_camera",
                False,
                "aircraft is armed; sensor bench check refused",
            ),
            ProbeResult(
                "lidar",
                False,
                "aircraft is armed; sensor bench check refused",
            ),
        )
    elif status.get("state") != "waiting_for_arm":
        state = status.get("state", "unknown")
        results = (
            ProbeResult(
                "external_imu", False, f"logger state is {state}"
            ),
            ProbeResult(
                "depth_camera", False, f"logger state is {state}"
            ),
            ProbeResult("lidar", False, f"logger state is {state}"),
        )
    else:
        results = (
            evaluate_live_imu(status, config),
            probe_depth_camera(config),
            probe_lidar(config),
        )

    ready = all(result.available for result in results)
    if args.as_json:
        print(
            json.dumps(
                {
                    "ready": ready,
                    "results": [asdict(result) for result in results],
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        for result in results:
            label = "PASS" if result.available else "BLOCK"
            print(f"[{label:5}] {result.name}: {result.detail}")
        print(f"READY={str(ready).lower()}")
    return 0 if ready else 2


if __name__ == "__main__":
    raise SystemExit(main())
