#!/usr/bin/env python3

import argparse
import statistics
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.bridge_config import SlamBridgeConfig, load_bridge_config
from slam_core.fc_config import FlightControllerTelemetry, configure_telemetry_streams, drain_fc_telemetry, rangefinder_height_valid
from slam_core.mavlink_bridge import connect_to_cube


def parse_args():
    parser = argparse.ArgumentParser(description="Check Cube rangefinder/lidar DISTANCE_SENSOR data.")
    parser.add_argument("--config", default="config/autostart.yaml")
    parser.add_argument("--ports", nargs="+")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--seconds", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_bridge_config(args.config) if Path(args.config).exists() else SlamBridgeConfig()
    connection = connect_to_cube(args.ports or config.ports, args.baud or config.baud)
    configure_telemetry_streams(connection.master)
    state = FlightControllerTelemetry()
    samples: list[float] = []
    deadline_s = time.time() + max(args.seconds, 1.0)
    while time.time() <= deadline_s:
        drain_fc_telemetry(connection.master, state)
        if rangefinder_height_valid(state):
            samples.append(float(state.rangefinder_distance_m))
        time.sleep(0.05)

    if not samples:
        print("rangefinder_healthy=False")
        print("reason=no valid DISTANCE_SENSOR samples")
        return 1
    mean_m = statistics.fmean(samples)
    noise_m = statistics.pstdev(samples) if len(samples) > 1 else 0.0
    print(f"samples={len(samples)}")
    print(f"rangefinder_healthy=True")
    print(f"height_mean_m={mean_m:.3f}")
    print(f"height_noise_m={noise_m:.3f}")
    print(f"last_sensor_id={state.rangefinder_sensor_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
