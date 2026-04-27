#!/usr/bin/env python3
"""
Collect paired VIO poses and IMU samples for offline calibration.

Usage:
  python3 scripts/collect_calibration_data.py --out recorded.csv --duration 60

The script samples the `vio` pose source (RealSense) and the external IM10A IMU if available,
and writes a simple CSV with timestamps, pose, and IMU fields for later hand-eye / extrinsic calibration.
"""

import argparse
import csv
import time
import sys
from pathlib import Path
from pathlib import Path as _P

# make `src` importable like other scripts in this repo
REPO_ROOT = _P(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.vio_backend import VioPoseSource
from slam_core.external_imu import Im10aReader


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--imu-port", default="auto")
    p.add_argument("--imu-baud", default="auto")
    return p.parse_args()


def main():
    args = parse_args()
    out_path = Path(args.out)

    vio = VioPoseSource()

    imu = None
    try:
        imu = Im10aReader.open(args.imu_port, args.imu_baud, scan_seconds=0.5)
    except Exception:
        imu = None

    deadline = time.time() + args.duration
    with out_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        header = [
            "t_s",
            # pose
            "x_m",
            "y_m",
            "z_m",
            "qw",
            "qx",
            "qy",
            "qz",
            # imu
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "gx_deg_s",
            "gy_deg_s",
            "gz_deg_s",
        ]
        writer.writerow(header)

        while time.time() < deadline:
            t = time.time()
            pose = vio.sample()
            imu_sample = None
            if imu is not None:
                imu_sample = imu.poll(duration_s=0.01)

            row = [
                t,
                pose.x_m,
                pose.y_m,
                pose.z_m,
                pose.qw,
                pose.qx,
                pose.qy,
                pose.qz,
                imu_sample.roll_deg if imu_sample is not None else "",
                imu_sample.pitch_deg if imu_sample is not None else "",
                imu_sample.yaw_deg if imu_sample is not None else "",
                imu_sample.gx_deg_s if imu_sample is not None else "",
                imu_sample.gy_deg_s if imu_sample is not None else "",
                imu_sample.gz_deg_s if imu_sample is not None else "",
            ]
            writer.writerow(row)
            time.sleep(0.05)


if __name__ == "__main__":
    main()
