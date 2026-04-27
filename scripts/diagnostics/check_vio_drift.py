#!/usr/bin/env python3

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.vio_backend import VioPoseSource


def parse_args():
    parser = argparse.ArgumentParser(description="Measure VIO XY drift while the drone/camera is stationary.")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--rate-hz", type=float, default=15.0)
    parser.add_argument("--max-drift-m", type=float, default=0.30)
    parser.add_argument("--max-noise-m", type=float, default=0.18)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    vio = VioPoseSource()
    samples = []
    period_s = 1.0 / max(args.rate_hz, 0.1)
    try:
        deadline_s = time.time() + max(args.seconds, 1.0)
        while time.time() <= deadline_s:
            loop_s = time.time()
            pose = vio.sample()
            if pose.tracking_state.startswith("ok"):
                samples.append(pose)
            remaining_s = period_s - (time.time() - loop_s)
            if remaining_s > 0:
                time.sleep(remaining_s)
    finally:
        vio.close()

    if len(samples) < 5:
        print("vio_drift_ok=False")
        print(f"reason=too few valid samples: {len(samples)}")
        return 1
    first = samples[0]
    last = samples[-1]
    drift_m = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m)
    x_noise_m = statistics.pstdev(sample.x_m for sample in samples)
    y_noise_m = statistics.pstdev(sample.y_m for sample in samples)
    max_noise_m = max(x_noise_m, y_noise_m)
    print(f"samples={len(samples)}")
    print(f"drift_m={drift_m:.3f}")
    print(f"xy_noise_m={max_noise_m:.3f}")
    print(f"last_quality={samples[-1].pose_quality}")
    print(f"last_tracking={samples[-1].tracking_state}")
    ok = drift_m <= args.max_drift_m and max_noise_m <= args.max_noise_m
    print(f"vio_drift_ok={ok}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
