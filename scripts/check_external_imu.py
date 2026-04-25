#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from intellisense_slam.external_imu import collect_imu_health


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check the external IM10A IMU stream that will be used by the Jetson-side SLAM bridge."
    )
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", default="auto")
    parser.add_argument("--scan-seconds", type=float, default=0.8)
    return parser.parse_args()


def main():
    args = parse_args()
    report = collect_imu_health(args.port, args.baud, args.scan_seconds)
    print("External IMU readiness")
    print("======================")
    print(report.message)
    print()
    print(f"usb_present={report.usb_present}")
    print(f"port={report.port}")
    print(f"baud={report.baud}")
    print(f"stream_healthy={report.stream_healthy}")
    if report.sample is not None:
        print(
            "orientation="
            f"({report.sample.roll_deg:+.2f}, {report.sample.pitch_deg:+.2f}, {report.sample.yaw_deg:+.2f})deg"
        )
        print(
            "gyro_deg_s="
            f"({report.sample.gx_deg_s:+.3f}, {report.sample.gy_deg_s:+.3f}, {report.sample.gz_deg_s:+.3f})"
        )
        print(
            "acc_g="
            f"({report.sample.ax_g:+.4f}, {report.sample.ay_g:+.4f}, {report.sample.az_g:+.4f})"
        )
        print(
            "mag_raw="
            f"({report.sample.mx_raw}, {report.sample.my_raw}, {report.sample.mz_raw})"
        )
        print(f"pressure_pa={report.sample.pressure_pa:.1f}")
        print(f"altitude_m={report.sample.altitude_m:.2f}")


if __name__ == "__main__":
    main()
