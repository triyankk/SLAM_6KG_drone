#!/usr/bin/env python3
# Run:
#   python3 imu_serial_preview.py --port /dev/ttyUSB0

import argparse
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
import serial

from imu_serial_common import (
    DEFAULT_BAUDS,
    Im10aTelemetry,
    OrientationEstimate,
    choose_port,
    detect_brltty_processes,
    infer_orientation,
    open_serial,
    parse_jy901_stream,
    probe_baud,
    safe_read,
    telemetry_orientation,
)


@dataclass
class PreviewState:
    port: str
    baud: int
    started_s: float = field(default_factory=time.time)
    total_bytes: int = 0
    total_lines: int = 0
    last_line: str = ""
    last_hex: str = ""
    text_buffer: bytearray = field(default_factory=bytearray)
    packet_buffer: bytearray = field(default_factory=bytearray)
    decoded_lines: deque = field(default_factory=lambda: deque(maxlen=8))
    orientation: Optional[OrientationEstimate] = None
    orientation_samples: int = 0
    telemetry: Im10aTelemetry = field(default_factory=Im10aTelemetry)
    acc_history: list[deque] = field(default_factory=lambda: [deque(maxlen=240) for _ in range(3)])
    gyro_history: list[deque] = field(default_factory=lambda: [deque(maxlen=240) for _ in range(3)])
    angle_history: list[deque] = field(default_factory=lambda: [deque(maxlen=240) for _ in range(3)])


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Preview the IM10A serial stream with a 3D orientation model and grouped acc/gyro/angle plots."
        )
    )
    parser.add_argument("--port", default="auto", help="Serial port path or 'auto'.")
    parser.add_argument("--baud", default="auto", help="Baud rate or 'auto'.")
    parser.add_argument(
        "--baud-candidates",
        type=int,
        nargs="*",
        default=DEFAULT_BAUDS,
        help="Baud candidates to test when --baud auto is used.",
    )
    parser.add_argument("--scan-seconds", type=float, default=1.0)
    parser.add_argument("--window-width", type=int, default=1500)
    parser.add_argument("--window-height", type=int, default=980)
    parser.add_argument("--cube-size", type=float, default=1.0)
    return parser.parse_args()


def consume_text_lines(state: PreviewState):
    while b"\n" in state.text_buffer:
        raw_line, _, remainder = state.text_buffer.partition(b"\n")
        state.text_buffer = bytearray(remainder)
        line = raw_line.decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        state.total_lines += 1
        state.last_line = line
        state.decoded_lines.appendleft(line)
        orientation = infer_orientation(line)
        if orientation is not None:
            state.orientation = orientation
            state.orientation_samples += 1


def append_triplet(history: list[deque], values: tuple[float, float, float]):
    for idx, value in enumerate(values):
        history[idx].append(value)


def consume_serial(state: PreviewState, ser: serial.Serial):
    chunk = safe_read(ser, 4096)
    if not chunk:
        return

    state.total_bytes += len(chunk)
    state.last_hex = chunk[:96].hex(" ")
    state.text_buffer.extend(chunk)
    state.packet_buffer.extend(chunk)

    consume_text_lines(state)

    packets = parse_jy901_stream(state.packet_buffer, state.telemetry)
    for packet in packets:
        packet_name = packet["packet_name"]
        if packet_name == "acc" and "acc_g" in packet:
            append_triplet(state.acc_history, packet["acc_g"])  # type: ignore[arg-type]
        elif packet_name == "gyro" and "gyro_deg_s" in packet:
            append_triplet(state.gyro_history, packet["gyro_deg_s"])  # type: ignore[arg-type]
        elif packet_name == "angle" and "angle_deg" in packet:
            append_triplet(state.angle_history, packet["angle_deg"])  # type: ignore[arg-type]

    orientation = telemetry_orientation(state.telemetry)
    if orientation is not None:
        state.orientation = orientation
        state.orientation_samples = state.telemetry.valid_frame_count


def draw_text(frame: np.ndarray, text: str, x: int, y: int, scale: float = 0.55, color=(220, 220, 220)):
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_series_plot(
    frame: np.ndarray,
    histories: list[deque],
    labels: tuple[str, str, str],
    colors: tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]],
    x: int,
    y: int,
    width: int,
    height: int,
    title: str,
):
    cv2.rectangle(frame, (x, y), (x + width, y + height), (60, 60, 60), 1)
    draw_text(frame, title, x + 8, y + 20, 0.55, (230, 230, 140))

    values = [list(history) for history in histories if history]
    if not values:
        draw_text(frame, "waiting for packet data", x + 8, y + 48, 0.45, (160, 160, 160))
        return

    flat = [value for series in values for value in series]
    min_v = min(flat)
    max_v = max(flat)
    if math.isclose(min_v, max_v):
        min_v -= 1.0
        max_v += 1.0

    plot_top = y + 30
    plot_height = height - 54
    plot_width = width - 16
    plot_left = x + 8
    draw_text(frame, f"min {min_v:.3f}", x + 8, y + height - 12, 0.42, (160, 160, 160))
    draw_text(frame, f"max {max_v:.3f}", x + width - 110, y + height - 12, 0.42, (160, 160, 160))

    for idx, history in enumerate(histories):
        draw_text(frame, labels[idx], x + 110 + idx * 80, y + 20, 0.45, colors[idx])
        if len(history) < 2:
            continue
        pts = []
        numeric = list(history)
        for sample_idx, value in enumerate(numeric):
            px = plot_left + int(sample_idx * (plot_width - 1) / max(len(numeric) - 1, 1))
            norm = (value - min_v) / (max_v - min_v)
            py = plot_top + plot_height - 1 - int(norm * (plot_height - 1))
            pts.append((px, py))
        cv2.polylines(frame, [np.array(pts, dtype=np.int32)], False, colors[idx], 1, cv2.LINE_AA)


def rotation_matrix_from_euler_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    return rz @ ry @ rx


def project_points(points: np.ndarray, width: int, height: int, scale: float = 180.0) -> np.ndarray:
    projected = []
    for x, y, z in points:
        distance = 4.0
        factor = distance / max(distance - z, 0.5)
        px = int(width / 2 + x * factor * scale)
        py = int(height / 2 - y * factor * scale)
        projected.append((px, py))
    return np.array(projected, dtype=np.int32)


def draw_3d_imu(frame: np.ndarray, state: PreviewState, x: int, y: int, width: int, height: int, cube_size: float):
    cv2.rectangle(frame, (x, y), (x + width, y + height), (60, 60, 60), 1)
    draw_text(frame, "3D IMU Orientation", x + 10, y + 24, 0.65, (230, 230, 140))
    if state.orientation is None:
        draw_text(frame, "Waiting for quaternion/angle packets", x + 10, y + 54, 0.55, (180, 180, 180))
        draw_text(frame, "This viewer expects the IM10A binary stream", x + 10, y + 82, 0.45, (150, 150, 150))
        draw_text(frame, "or falls back to labelled ASCII orientation lines.", x + 10, y + 104, 0.45, (150, 150, 150))
        return

    ori = state.orientation
    draw_text(
        frame,
        f"roll={ori.roll_deg:+.2f}  pitch={ori.pitch_deg:+.2f}  yaw={ori.yaw_deg:+.2f}",
        x + 10,
        y + 54,
        0.52,
        (180, 240, 180),
    )
    draw_text(frame, f"source={ori.source} samples={state.orientation_samples}", x + 10, y + 78, 0.45, (170, 220, 255))

    local = frame[y + 96 : y + height - 10, x + 10 : x + width - 10]
    local[:] = (8, 8, 8)
    center_h, center_w = local.shape[:2]

    cube = np.array(
        [
            [-cube_size, -cube_size, -cube_size],
            [cube_size, -cube_size, -cube_size],
            [cube_size, cube_size, -cube_size],
            [-cube_size, cube_size, -cube_size],
            [-cube_size, -cube_size, cube_size],
            [cube_size, -cube_size, cube_size],
            [cube_size, cube_size, cube_size],
            [-cube_size, cube_size, cube_size],
        ],
        dtype=np.float32,
    ) * 0.5
    rot = rotation_matrix_from_euler_deg(ori.roll_deg, ori.pitch_deg, ori.yaw_deg)
    rotated = (rot @ cube.T).T
    projected = project_points(rotated, center_w, center_h)

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    for start, end in edges:
        color = (100, 220, 255) if start < 4 and end < 4 else (130, 130, 255)
        cv2.line(local, tuple(projected[start]), tuple(projected[end]), color, 2, cv2.LINE_AA)

    origin = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    axes = np.array([[1.1, 0, 0], [0, 1.1, 0], [0, 0, 1.1]], dtype=np.float32)
    rotated_axes = (rot @ axes.T).T
    axis_points = project_points(np.vstack([origin, rotated_axes]), center_w, center_h)
    cv2.line(local, tuple(axis_points[0]), tuple(axis_points[1]), (0, 0, 255), 2, cv2.LINE_AA)
    cv2.line(local, tuple(axis_points[0]), tuple(axis_points[2]), (0, 255, 0), 2, cv2.LINE_AA)
    cv2.line(local, tuple(axis_points[0]), tuple(axis_points[3]), (255, 150, 0), 2, cv2.LINE_AA)
    draw_text(local, "X", axis_points[1][0] + 4, axis_points[1][1], 0.45, (0, 0, 255))
    draw_text(local, "Y", axis_points[2][0] + 4, axis_points[2][1], 0.45, (0, 255, 0))
    draw_text(local, "Z", axis_points[3][0] + 4, axis_points[3][1], 0.45, (255, 150, 0))


def render(state: PreviewState, width: int, height: int, cube_size: float) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    elapsed_s = max(time.time() - state.started_s, 1e-6)
    bytes_per_s = state.total_bytes / elapsed_s

    left_width = max(460, int(width * 0.42))
    draw_text(frame, "IM10A 3D Preview", 24, 32, 0.9, (240, 240, 240))
    draw_text(frame, f"port={state.port} baud={state.baud}", 24, 64)
    draw_text(frame, f"bytes={state.total_bytes}  bytes_per_s={bytes_per_s:.1f}", 24, 88)
    draw_text(frame, f"valid_frames={state.telemetry.valid_frame_count}", 24, 112)
    draw_text(frame, f"frame_counts={state.telemetry.frame_counts}", 24, 136, 0.47)
    draw_text(frame, f"last_frame={state.telemetry.last_frame_name or 'none'}", 24, 160, 0.5)
    brltty = detect_brltty_processes()
    draw_text(frame, f"brltty_processes={len(brltty)}", 24, 184, 0.5, (255, 180, 120) if brltty else (140, 210, 140))

    draw_3d_imu(frame, state, 24, 214, left_width - 36, height - 238, cube_size=cube_size)

    panel_x = left_width + 12
    draw_text(frame, "Telemetry", panel_x, 32, 0.65, (230, 230, 140))
    y = 62
    if state.telemetry.acc_g is not None:
        draw_text(frame, f"acc_g={tuple(round(v, 4) for v in state.telemetry.acc_g)}", panel_x, y, 0.47)
        y += 22
    if state.telemetry.gyro_deg_s is not None:
        draw_text(frame, f"gyro_dps={tuple(round(v, 3) for v in state.telemetry.gyro_deg_s)}", panel_x, y, 0.47)
        y += 22
    if state.telemetry.angle_deg is not None:
        draw_text(frame, f"angle_deg={tuple(round(v, 3) for v in state.telemetry.angle_deg)}", panel_x, y, 0.47)
        y += 22
    if state.telemetry.quaternion is not None:
        draw_text(frame, f"quat={tuple(round(v, 4) for v in state.telemetry.quaternion)}", panel_x, y, 0.47)
        y += 22
    if state.telemetry.mag_raw is not None:
        draw_text(frame, f"mag_raw={state.telemetry.mag_raw}", panel_x, y, 0.47)
        y += 22
    if state.telemetry.pressure_pa is not None:
        draw_text(frame, f"pressure_pa={state.telemetry.pressure_pa}", panel_x, y, 0.47)
        y += 22
    if state.telemetry.altitude_m is not None:
        draw_text(frame, f"altitude_m={state.telemetry.altitude_m:.2f}", panel_x, y, 0.47)
        y += 22

    draw_text(frame, "Last decoded lines", panel_x, y + 28, 0.6, (230, 230, 140))
    y += 56
    for line in state.decoded_lines:
        draw_text(frame, line[:96], panel_x, y, 0.45, (200, 200, 200))
        y += 20
    draw_text(frame, f"last_hex={state.last_hex[:110]}", panel_x, y + 10, 0.4, (130, 180, 255))

    plot_top = max(y + 42, 320)
    plot_width = width - panel_x - 24
    plot_height = max(120, (height - plot_top - 36) // 3)
    draw_series_plot(
        frame,
        state.acc_history,
        ("ax", "ay", "az"),
        ((255, 120, 120), (120, 255, 120), (120, 180, 255)),
        panel_x,
        plot_top,
        plot_width,
        plot_height - 8,
        "Acceleration (g)",
    )
    draw_series_plot(
        frame,
        state.gyro_history,
        ("gx", "gy", "gz"),
        ((255, 120, 120), (120, 255, 120), (120, 180, 255)),
        panel_x,
        plot_top + plot_height,
        plot_width,
        plot_height - 8,
        "Angular Velocity (deg/s)",
    )
    draw_series_plot(
        frame,
        state.angle_history,
        ("roll", "pitch", "yaw"),
        ((255, 120, 120), (120, 255, 120), (120, 180, 255)),
        panel_x,
        plot_top + plot_height * 2,
        plot_width,
        plot_height - 8,
        "Euler Angle (deg)",
    )
    return frame


def main():
    args = parse_args()
    port = choose_port(args.port)

    if args.baud == "auto":
        probe = probe_baud(port, args.baud_candidates, args.scan_seconds)
        baud = probe.baud
        print(
            "Auto baud result:"
            f" port={port}"
            f" baud={probe.baud}"
            f" frames={probe.valid_frame_count}"
            f" bytes={probe.byte_count}"
            f" nonzero_ratio={probe.nonzero_ratio:.2f}"
            f" sample_hex={probe.sample_hex[:80]}"
        )
    else:
        baud = int(args.baud)

    ser = open_serial(port, baud)
    state = PreviewState(port=port, baud=baud)
    window_name = f"IM10A 3D Preview {port}"

    try:
        while True:
            consume_serial(state, ser)
            frame = render(state, args.window_width, args.window_height, args.cube_size)
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
