"""Command-line entry point for the read-only readiness probe."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from .config import ConfigError, load_config
from .models import Profile
from .paths import CONFIG_DIR
from .readiness import run_readiness


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only optFlow_slam hardware and safety readiness probe"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--profile",
        choices=[profile.value for profile in Profile],
        default=Profile.FC_BENCH.value,
    )
    parser.add_argument("--json", action="store_true", dest="as_json")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
    except (OSError, ConfigError) as exc:
        print(f"Configuration error: {exc}")
        return 2

    report = run_readiness(config, Profile(args.profile))
    if args.as_json:
        payload = {
            "profile": report.profile.value,
            "ready": report.ready,
            "required": sorted(report.required_names),
            "results": [asdict(result) for result in report.results],
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"optFlow_slam readiness profile: {report.profile.value}")
        print("This command does not arm the vehicle or send movement commands.")
        for result in report.results:
            required = result.name in report.required_names
            if result.available:
                label = "PASS"
            elif required:
                label = "BLOCK"
            else:
                label = "WAIT"
            print(f"[{label:5}] {result.name}: {result.detail}")
        print(f"READY={str(report.ready).lower()}")
    return 0 if report.ready else 2


if __name__ == "__main__":
    raise SystemExit(main())
