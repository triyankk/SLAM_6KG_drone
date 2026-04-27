#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.realsense_capture import list_devices, open_depth_pipeline, wait_for_frame_bundle


def parse_args():
    parser = argparse.ArgumentParser(description="Check RealSense device and frame health.")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    devices = list_devices()
    print("RealSense devices:")
    if not devices:
        print("- none detected")
        return 1
    for device in devices:
        print(
            f"- {device.name} serial={device.serial} product={device.product_line} "
            f"depth={device.has_depth_sensor} color={device.has_color_sensor} motion={device.has_motion_sensor}"
        )

    timestamps: list[float] = []
    depth_count = 0
    ir_count = 0
    with open_depth_pipeline(args.width, args.height, args.fps, infrared=True) as pipeline:
        deadline_s = time.time() + max(args.seconds, 1.0)
        while time.time() <= deadline_s:
            bundle = wait_for_frame_bundle(pipeline, timeout_ms=2000)
            timestamps.append(float(bundle.timestamp_ms))
            depth_count += 1 if bundle.depth_frame else 0
            ir_count += 1 if bundle.infrared_frame else 0

    frozen = len(set(round(ts, 3) for ts in timestamps)) <= 1
    print(f"frames={len(timestamps)} depth_frames={depth_count} infrared_frames={ir_count}")
    print(f"timestamps_frozen={frozen}")
    return 0 if timestamps and depth_count and ir_count and not frozen else 1


if __name__ == "__main__":
    raise SystemExit(main())
