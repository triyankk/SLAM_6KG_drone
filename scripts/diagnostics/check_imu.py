#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.external_imu import collect_imu_health


def parse_args():
    parser = argparse.ArgumentParser(description="Check IM10A IMU health.")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", default="auto")
    parser.add_argument("--scan-seconds", type=float, default=0.8)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = collect_imu_health(args.port, args.baud, args.scan_seconds)
    print(report.message)
    print(f"usb_present={report.usb_present}")
    print(f"port={report.port}")
    print(f"baud={report.baud}")
    print(f"stream_healthy={report.stream_healthy}")
    if report.sample is not None:
        print(
            "rpy_deg="
            f"({report.sample.roll_deg:+.2f},{report.sample.pitch_deg:+.2f},{report.sample.yaw_deg:+.2f})"
        )
        print(
            "gyro_deg_s="
            f"({report.sample.gx_deg_s:+.2f},{report.sample.gy_deg_s:+.2f},{report.sample.gz_deg_s:+.2f})"
        )
    return 0 if report.stream_healthy else 1


if __name__ == "__main__":
    raise SystemExit(main())
