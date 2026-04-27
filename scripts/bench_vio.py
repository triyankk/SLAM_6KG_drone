#!/usr/bin/env python3
"""
Run a short bench test that records VIO poses and IMU samples to a CSV.

Usage:
  python3 scripts/bench_vio.py --out /tmp/bench_$(date +%s).csv --duration 20

By default this does not send ODOMETRY to the flight controller; use `--send` to stream to the Cube.
"""

import argparse
import csv
import time
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.vio_backend import VioPoseSource
from slam_core.external_imu import Im10aReader
from slam_core.mavlink_bridge import connect_to_cube


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--imu-port", default="auto")
    p.add_argument("--imu-baud", default="auto")
    p.add_argument("--send", action="store_true", help="send odometry to Cube during bench run")
    p.add_argument("--ports", nargs="+", default=["/dev/ttyACM1", "/dev/ttyACM0"]) 
    p.add_argument("--baud", type=int, default=115200)
    return p.parse_args()


def main():
    args = parse_args()
    out = Path(args.out)
    vio = VioPoseSource()

    imu = None
    try:
        imu = Im10aReader.open(args.imu_port, args.imu_baud, scan_seconds=0.5)
    except Exception:
        imu = None

    connection = None
    if args.send:
        try:
            connection = connect_to_cube(args.ports, args.baud)
        except Exception as exc:
            print('Failed to connect to Cube, will not send odometry:', exc)
            connection = None

    deadline = time.time() + args.duration
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "t_s",
                "x_m",
                "y_m",
                "z_m",
                "qw",
                "qx",
                "qy",
                "qz",
                "tracking_state",
                "pose_quality",
                "feature_count",
                "tracked_feature_count",
                "inlier_count",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
            ]
        )
        while time.time() < deadline:
            pose = vio.sample()
            imu_s = None
            if imu is not None:
                imu_s = imu.poll(duration_s=0.01)
            row = [
                time.time(),
                pose.x_m,
                pose.y_m,
                pose.z_m,
                pose.qw,
                pose.qx,
                pose.qy,
                pose.qz,
                pose.tracking_state,
                pose.pose_quality,
                pose.feature_count,
                pose.tracked_feature_count,
                pose.inlier_count,
            ]
            if imu_s is not None:
                row += [imu_s.roll_deg, imu_s.pitch_deg, imu_s.yaw_deg]
            else:
                row += ["", "", ""]
            writer.writerow(row)
            if connection is not None:
                try:
                    from slam_core.mavlink_bridge import send_odometry
                    send_odometry(connection, pose)
                except Exception:
                    pass
            time.sleep(0.05)


if __name__ == '__main__':
    main()
