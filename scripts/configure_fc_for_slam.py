#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from intellisense_slam.bridge_config import SlamBridgeConfig, load_bridge_config
from intellisense_slam.fc_config import apply_fc_setup, request_active_source_set
from intellisense_slam.mavlink_bridge import connect_to_cube


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Configure the Cube for SLAM ExternalNav while preserving the existing"
            " GPS and optical-flow source sets."
        )
    )
    parser.add_argument("--config", default="")
    parser.add_argument("--ports", nargs="+")
    parser.add_argument("--baud", type=int)
    return parser.parse_args()


def resolve_config(args) -> SlamBridgeConfig:
    config = load_bridge_config(args.config) if args.config else SlamBridgeConfig()
    if args.ports is not None:
        config.ports = args.ports
    if args.baud is not None:
        config.baud = args.baud
    return config


def main():
    config = resolve_config(parse_args())
    connection = connect_to_cube(config.ports, config.baud, heartbeat_timeout_s=config.heartbeat_timeout_seconds)
    try:
        report = apply_fc_setup(connection.master, config.fc_setup)
        active_source = request_active_source_set(connection.master)
        print(
            "Connected to Cube for SLAM FC setup:"
            f" port={connection.port}"
            f" baud={connection.baud}"
            f" active_source={active_source if active_source is not None else 'unknown'}"
        )
        if report.changed:
            print("Changed parameters:")
            for item in report.changed:
                old_text = "unknown" if item.old_value is None else f"{item.old_value:g}"
                print(f"- {item.name}: {old_text} -> {item.new_value:g}")
        else:
            print("No FC parameter changes were needed.")

        if report.reboot_recommended:
            print("Reboot recommended: EKF/visual-odometry parameters changed on this boot.")
    finally:
        connection.master.close()


if __name__ == "__main__":
    main()
