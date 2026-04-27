#!/usr/bin/env python3
# Run:
#   python3 jt16_live_view.py --port /dev/ttyUSB0

import argparse
import math
import os
import select
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque

import cv2
import numpy as np

from jt16_serial_probe import (
    PacketStats,
    choose_jt16_port,
    consume_packets,
    extract_point_samples,
    open_raw_serial,
)


# JT16 Mini Vertical Angles: -15 to +15 degrees, 2 degree increments
APPROX_VERTICAL_ANGLES_DEG = np.array([
    -15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
    1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0
], dtype=np.float32)


@dataclass
class ViewerPoint:
    x_m: float
    y_m: float
    z_m: float
    reflectivity: int
    timestamp_s: float


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Open a live JT16 viewer window on the Jetson. The left pane shows a top-down "
            "XY map and the right pane shows a simple radius-vs-height view."
        )
    )
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=3000000)
    parser.add_argument(
        "--max-range-m",
        type=float,
        default=30.0,
        help="Maximum rendered range in meters.",
    )
    parser.add_argument(
        "--persistence-s",
        type=float,
        default=0.8,
        help="How long points stay visible in the viewer window.",
    )
    parser.add_argument(
        "--window-size",
        type=int,
        default=720,
        help="Main square render size in pixels.",
    )
    parser.add_argument(
        "--rotate-deg",
        type=float,
        default=0.0,
        help="Rotate the top-down view by this many degrees.",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=2.0,
        help="Console status rate in Hz.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run; 0 means until you close the window or press q.",
    )
    return parser.parse_args()


def reflectivity_color(reflectivity: int, age_ratio: float):
    intensity = max(0.2, 1.0 - age_ratio)
    green = int(min(255, reflectivity * 2 * intensity + 32))
    red = int(min(255, 255 * age_ratio))
    blue = int(min(255, 128 * (1.0 - age_ratio)))
    return blue, green, red


def draw_grid(canvas: np.ndarray, origin_x: int, origin_y: int, size_px: int, max_range_m: float):
    center = (origin_x + size_px // 2, origin_y + size_px // 2)
    cv2.rectangle(canvas, (origin_x, origin_y), (origin_x + size_px, origin_y + size_px), (60, 60, 60), 1)
    cv2.line(canvas, (center[0], origin_y), (center[0], origin_y + size_px), (50, 50, 50), 1)
    cv2.line(canvas, (origin_x, center[1]), (origin_x + size_px, center[1]), (50, 50, 50), 1)

    for ring_fraction in (0.25, 0.5, 0.75, 1.0):
        radius_px = max(1, int(size_px * ring_fraction * 0.5))
        cv2.circle(canvas, center, radius_px, (40, 40, 40), 1)
        label = f"{max_range_m * ring_fraction:.0f} m"
        cv2.putText(
            canvas,
            label,
            (center[0] + radius_px - 30, center[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (120, 120, 120),
            1,
            cv2.LINE_AA,
        )

    cv2.putText(canvas, "Top-down XY", (origin_x + 10, origin_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)


def draw_elevation_background(canvas: np.ndarray, origin_x: int, origin_y: int, width_px: int, height_px: int, max_range_m: float):
    cv2.rectangle(canvas, (origin_x, origin_y), (origin_x + width_px, origin_y + height_px), (60, 60, 60), 1)
    cv2.line(canvas, (origin_x, origin_y + height_px - 1), (origin_x + width_px, origin_y + height_px - 1), (60, 60, 60), 1)
    for fraction in (0.25, 0.5, 0.75, 1.0):
        x = origin_x + int(width_px * fraction)
        cv2.line(canvas, (x, origin_y), (x, origin_y + height_px), (40, 40, 40), 1)
        cv2.putText(
            canvas,
            f"{max_range_m * fraction:.0f} m",
            (x - 18, origin_y + height_px - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (120, 120, 120),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(canvas, "Radius vs height", (origin_x + 10, origin_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)


def render_frame(
    args,
    stats: PacketStats,
    points: Deque[ViewerPoint],
    start_s: float,
):
    now_s = time.time()
    size = max(360, args.window_size)
    side_width = max(320, size // 2)
    canvas = np.zeros((size, size + side_width, 3), dtype=np.uint8)

    draw_grid(canvas, 0, 0, size, args.max_range_m)
    draw_elevation_background(canvas, size, 0, side_width, size, args.max_range_m)

    scale_xy = (size * 0.5) / max(args.max_range_m, 1e-3)
    center_x = size // 2
    center_y = size // 2

    for point in points:
        age_ratio = min(1.0, max(0.0, (now_s - point.timestamp_s) / max(args.persistence_s, 1e-3)))
        color = reflectivity_color(point.reflectivity, age_ratio)

        px = int(center_x + point.x_m * scale_xy)
        py = int(center_y - point.y_m * scale_xy)
        if 0 <= px < size and 0 <= py < size:
            cv2.circle(canvas, (px, py), 2, color, -1, cv2.LINE_AA)

        radius_m = math.hypot(point.x_m, point.y_m)
        ex = int(size + (radius_m / max(args.max_range_m, 1e-3)) * (side_width - 20)) + 10
        ez = int((1.0 - ((point.z_m / max(args.max_range_m * 0.7, 1e-3)) + 0.1)) * (size - 20)) + 10
        if size <= ex < size + side_width and 0 <= ez < size:
            cv2.circle(canvas, (ex, ez), 2, color, -1, cv2.LINE_AA)

    status_lines = [
        f"Port: {args.port}",
        f"Baud: {args.baud}",
        f"Point packets: {stats.point_packets}",
        f"IMU packets: {stats.imu_packets}",
        f"Fault packets: {stats.fault_packets}",
        f"Last azimuth: {stats.last_azimuth_deg:.2f} deg",
        f"Last dist med: {stats.last_median_distance_m:.2f} m",
        f"Visible points: {len(points)}",
        f"Runtime: {now_s - start_s:.1f} s",
        "",
    ]
    if stats.point_packets <= 0:
        status_lines.extend(
            [
                "No JT16 packets yet.",
                "Check power, RS485 wiring,",
                "and whether DATA_A / DATA_B",
                "need to be swapped.",
            ]
        )
    else:
        status_lines.extend(
            [
                "Controls:",
                "q or Esc to quit",
            ]
        )

    text_y = 26
    for line in status_lines:
        cv2.putText(
            canvas,
            line,
            (size + 16, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        text_y += 24

    return canvas


def main():
    args = parse_args()
    port = choose_jt16_port(args.port)
    if not os.path.exists(port):
        raise SystemExit(f"{port} does not exist. Check serial ports with: ls -l /dev/jt16_usb /dev/ttyUSB*")

    fd = open_raw_serial(port, args.baud)
    buffer = bytearray()
    stats = PacketStats()
    history: Deque[ViewerPoint] = deque()
    start_s = time.time()
    next_status_s = start_s
    deadline_s = None if args.duration <= 0 else start_s + args.duration

    rotation_rad = math.radians(args.rotate_deg)
    cv2.namedWindow("JT16 Live View", cv2.WINDOW_NORMAL)

    def on_point_packet(packet: bytes):
        packet_time_s = time.time()
        for sample in extract_point_samples(packet):
            if sample.distance_m <= 0.0 or sample.distance_m > args.max_range_m:
                continue

            vertical_deg = float(APPROX_VERTICAL_ANGLES_DEG[sample.channel])
            vertical_rad = math.radians(vertical_deg)
            azimuth_rad = math.radians(sample.azimuth_deg) + rotation_rad
            horizontal_range_m = sample.distance_m * math.cos(vertical_rad)

            x_m = horizontal_range_m * math.sin(azimuth_rad)
            y_m = horizontal_range_m * math.cos(azimuth_rad)
            z_m = sample.distance_m * math.sin(vertical_rad)
            history.append(
                ViewerPoint(
                    x_m=x_m,
                    y_m=y_m,
                    z_m=z_m,
                    reflectivity=sample.reflectivity,
                    timestamp_s=packet_time_s,
                )
            )

    print(f"Opening JT16 live viewer on {port} at {args.baud} baud. Press q or Esc to exit.")
    try:
        while deadline_s is None or time.time() <= deadline_s:
            readable, _, _ = select.select([fd], [], [], 0.02)
            if readable:
                chunk = os.read(fd, 8192)
                if chunk:
                    buffer.extend(chunk)
                    consume_packets(buffer, stats, on_point_packet=on_point_packet)

            now_s = time.time()
            cutoff_s = now_s - max(args.persistence_s, 0.05)
            while history and history[0].timestamp_s < cutoff_s:
                history.popleft()

            frame = render_frame(args, stats, history, start_s)
            cv2.imshow("JT16 Live View", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

            if now_s >= next_status_s:
                elapsed_s = max(now_s - start_s, 1e-6)
                print(
                    "JT16 view:"
                    f" point={stats.point_packets} ({stats.point_packets / elapsed_s:.1f}/s)"
                    f" imu={stats.imu_packets} ({stats.imu_packets / elapsed_s:.1f}/s)"
                    f" fault={stats.fault_packets}"
                    f" visible={len(history)}"
                    f" dist_med={stats.last_median_distance_m:.2f}m"
                )
                next_status_s = now_s + 1.0 / max(args.status_rate, 0.1)
    finally:
        cv2.destroyAllWindows()
        os.close(fd)


if __name__ == "__main__":
    main()
