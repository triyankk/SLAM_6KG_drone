#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.bridge_config import load_bridge_config
from slam_core.slam_observer import SlamLoiterObserver


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inspect the integrated LOITER soft calibration / SLAM observation mode."
    )
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "autostart.yaml"))
    parser.add_argument("--dry-run", action="store_true", help="Validate observer config without touching MAVLink.")
    parser.add_argument("--status", action="store_true", help="Print the latest observer status JSON.")
    return parser.parse_args()


def print_status(config) -> int:
    status_path = Path(config.slam_observer.status_path).expanduser()
    if not status_path.exists():
        print(f"No observer status yet: {status_path}")
        print("Enter LOITER with the SLAM bridge running, then check again.")
        return 1
    payload = json.loads(status_path.read_text(encoding="utf-8"))
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


def main() -> None:
    args = parse_args()
    config = load_bridge_config(args.config)
    observer = SlamLoiterObserver(config.slam_observer, dry_run=True)

    if args.status:
        raise SystemExit(print_status(config))

    print("LOITER soft calibration / SLAM observation is embedded in the SLAM bridge service.")
    print(f"enabled={config.slam_observer.enable_loiter_observation}")
    print(f"interval_s={config.slam_observer.observation_message_interval_sec}")
    print(f"min_quality_for_poshold={config.slam_observer.min_quality_for_poshold}")
    print(f"live_soft_correction={config.slam_observer.enable_live_soft_correction}")
    print(f"auto_fallback_to_loiter={config.slam_observer.enable_auto_fallback_to_loiter}")
    print(f"log_path={config.slam_observer.log_path}")
    print(f"status_path={config.slam_observer.status_path}")
    print(json.dumps(observer.startup_summary(), indent=2, sort_keys=True))

    if args.dry_run:
        print("Dry-run complete: config parsed; no MAVLink connection opened.")
        return

    print("No parallel MAVLink reader was started. Use --status or the systemd service logs.")


if __name__ == "__main__":
    main()
