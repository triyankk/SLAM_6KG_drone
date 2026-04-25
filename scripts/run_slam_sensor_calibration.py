#!/usr/bin/env python3
"""Capture a short all-sensor calibration/health report for SLAM bring-up."""

import argparse
import csv
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from intellisense_slam.external_imu import Im10aReader
from intellisense_slam.lidar import LidarReader
from intellisense_slam.realsense_capture import open_depth_pipeline, wait_for_frame_bundle


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--out-dir", default=str(REPO_ROOT / "calibration_captures"))
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    run_id = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    out_path = out_dir / f"slam_sensor_calibration_{run_id}.csv"

    imu = None
    lidar = None
    pipeline_ctx = None
    pipeline = None
    try:
        imu = Im10aReader.open("auto", "auto", 1.0)
    except Exception as exc:  # noqa: BLE001
        print(f"IMU unavailable: {exc}")
    try:
        lidar = LidarReader.open("auto", 3000000, 72)
    except Exception as exc:  # noqa: BLE001
        print(f"JT lidar unavailable: {exc}")
    try:
        pipeline_ctx = open_depth_pipeline()
        pipeline = pipeline_ctx.__enter__()
    except Exception as exc:  # noqa: BLE001
        print(f"RealSense unavailable: {exc}")

    fields = [
        "unix_s",
        "imu_roll_deg",
        "imu_pitch_deg",
        "imu_yaw_deg",
        "lidar_min_m",
        "lidar_median_m",
        "lidar_packets",
        "depth_center_m",
    ]
    deadline_s = time.time() + max(args.duration, 1.0)
    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        while time.time() <= deadline_s:
            row = {field: "" for field in fields}
            row["unix_s"] = f"{time.time():.3f}"
            if imu is not None:
                sample = imu.poll(duration_s=0.02)
                if sample is not None:
                    row["imu_roll_deg"] = f"{sample.roll_deg:.3f}"
                    row["imu_pitch_deg"] = f"{sample.pitch_deg:.3f}"
                    row["imu_yaw_deg"] = f"{sample.yaw_deg:.3f}"
            if lidar is not None:
                snap = lidar.poll(duration_s=0.02)
                row["lidar_min_m"] = f"{snap.min_distance_m:.3f}"
                row["lidar_median_m"] = f"{snap.median_distance_m:.3f}"
                row["lidar_packets"] = str(snap.point_packets)
            if pipeline is not None:
                try:
                    bundle = wait_for_frame_bundle(pipeline, timeout_ms=200)
                    depth = bundle.depth_frame
                    if depth:
                        row["depth_center_m"] = f"{depth.get_distance(depth.get_width() // 2, depth.get_height() // 2):.3f}"
                except Exception:  # noqa: BLE001
                    pass
            writer.writerow(row)
            time.sleep(0.1)

    if imu is not None:
        imu.close()
    if lidar is not None:
        lidar.close()
    if pipeline_ctx is not None:
        pipeline_ctx.__exit__(None, None, None)
    print(f"Wrote SLAM sensor calibration capture: {out_path}")


if __name__ == "__main__":
    main()
