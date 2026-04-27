#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.bridge_config import SlamBridgeConfig, load_bridge_config
from slam_core.fc_config import FlightControllerTelemetry, configure_telemetry_streams, drain_fc_telemetry
from slam_core.mavlink_bridge import connect_to_cube, expand_cube_ports


def parse_args():
    parser = argparse.ArgumentParser(description="Check MAVLink heartbeat and basic Cube telemetry.")
    parser.add_argument("--config", default="config/autostart.yaml")
    parser.add_argument("--ports", nargs="+")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--seconds", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_bridge_config(args.config) if Path(args.config).exists() else SlamBridgeConfig()
    ports = args.ports or config.ports
    baud = args.baud or config.baud
    print(f"Checking MAVLink heartbeat on ports: {expand_cube_ports(ports)} baud={baud}")
    connection = connect_to_cube(ports, baud, heartbeat_timeout_s=max(args.seconds, 1.0))
    configure_telemetry_streams(connection.master)
    state = FlightControllerTelemetry()
    deadline_s = time.time() + max(args.seconds, 1.0)
    while time.time() <= deadline_s:
        drain_fc_telemetry(connection.master, state)
        time.sleep(0.05)
    print(f"connected_port={connection.port}")
    print(f"mode={state.flight_mode}")
    print(f"armed={state.armed}")
    print(f"heartbeat_seen={state.last_heartbeat_s > 0.0}")
    print(f"rangefinder_m={state.rangefinder_distance_m}")
    print(f"ekf_flags={state.ekf_flags}")
    return 0 if state.last_heartbeat_s > 0.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
