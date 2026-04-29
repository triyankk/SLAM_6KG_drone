#!/usr/bin/env python3

import argparse
from pathlib import Path

from mavlink_reader import MavlinkReader, MonitorConfig, setup_logging


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Passive MAVLink health monitor for SLAM flight.")
    parser.add_argument("--config", default="config.yaml", help="Path to monitor config YAML.")
    parser.add_argument("--once", action="store_true", help="Connect, print one status line, then exit.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = MonitorConfig.from_yaml(Path(args.config))
    logger = setup_logging(config)
    logger.info("SLAM MAVLink monitor starting")
    logger.info("Passive read-only mode: no commands, params, arming, or mode changes will be sent")
    MavlinkReader(config, logger).run(once=args.once)


if __name__ == "__main__":
    main()
