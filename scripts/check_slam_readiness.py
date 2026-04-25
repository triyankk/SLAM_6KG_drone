#!/usr/bin/env python3

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from intellisense_slam.readiness import build_readiness_report


def main():
    report = build_readiness_report()
    print("SLAM readiness report")
    print("====================")
    print(report.summary)
    print()
    for line in report.details:
        print(f"- {line}")
    print()
    if not report.realsense_found:
        print("Main blocker:")
        print("- No RealSense camera is currently detected on Jetson.")
        print("- Connect the camera first, then run this readiness check again.")
    elif not report.realsense_has_imu and not report.external_imu_stream_healthy:
        print("Main blocker:")
        print("- The current RealSense camera does not expose a motion sensor on Jetson.")
        print("- The external IMU path is not healthy yet, so we still do not have a trustworthy Jetson-side inertial source.")
        print("- Fix the IM10A serial stream or switch to an IMU-equipped depth camera.")
    elif report.external_imu_stream_healthy:
        print("Next milestone:")
        print("- The external IMU is healthy enough to bind into the Jetson-side SLAM bridge.")
        print("- The next step is backend fusion and frame calibration, not basic device bring-up anymore.")


if __name__ == "__main__":
    main()
