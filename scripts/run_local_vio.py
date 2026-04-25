#!/usr/bin/env python3

import argparse
import csv
import math
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from intellisense_slam.external_imu import Im10aReader, apply_imu_sample_to_pose
from intellisense_slam.fc_config import (
    FlightControllerTelemetry,
    configure_telemetry_streams,
    drain_fc_telemetry,
    rangefinder_height_valid,
    request_active_source_set,
)
from intellisense_slam.mavlink_bridge import connect_to_cube
from intellisense_slam.types import PoseSample
from intellisense_slam.vio_backend import VioPoseSource


def quaternion_to_yaw_deg(sample: PoseSample) -> float:
    qw, qx, qy, qz = sample.qw, sample.qx, sample.qy, sample.qz
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run the local VIO stack on Jetson without touching Cube or GCS. "
            "This is the safe development path while FC VisOdom and shared telemetry are still constrained."
        )
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Run time in seconds. 0 means run until stopped.")
    parser.add_argument("--csv-out", default="", help="Optional CSV path for logging the local VIO session.")
    parser.add_argument("--imu", choices=["on", "off"], default="on")
    parser.add_argument("--imu-port", default="auto")
    parser.add_argument("--imu-baud", default="auto")
    parser.add_argument("--imu-scan-seconds", type=float, default=0.8)
    parser.add_argument("--use-imu-orientation", choices=["on", "off"], default="on")
    parser.add_argument(
        "--cube-height",
        choices=["on", "off"],
        default="on",
        help="Use Cube DISTANCE_SENSOR height when available.",
    )
    parser.add_argument("--ports", nargs="+", default=["/dev/ttyACM0", "/dev/ttyACM1"])
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--heartbeat-timeout-seconds", type=float, default=8.0)
    parser.add_argument("--preview", choices=["on", "off"], default="on")
    parser.add_argument("--window-width", type=int, default=1280)
    parser.add_argument("--window-height", type=int, default=860)
    parser.add_argument("--history-points", type=int, default=1200)
    parser.add_argument("--status-seconds", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=15.0)
    return parser.parse_args()


def open_csv_writer(path_text: str):
    if not path_text:
        return None, None
    path = Path(path_text).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    writer.writerow(
        [
            "t_s",
            "pose_x_m",
            "pose_y_m",
            "pose_z_m",
            "pose_qw",
            "pose_qx",
            "pose_qy",
            "pose_qz",
            "pose_vx_m_s",
            "pose_vy_m_s",
            "pose_vz_m_s",
            "pose_quality",
            "tracking_state",
            "feature_count",
            "tracked_feature_count",
            "inlier_count",
            "cube_mode",
            "cube_source_set",
            "cube_rangefinder_m",
            "cube_rangefinder_ok",
            "yaw_deg",
            "imu_roll_deg",
            "imu_pitch_deg",
            "imu_yaw_deg",
            "imu_gx_deg_s",
            "imu_gy_deg_s",
            "imu_gz_deg_s",
            "imu_ax_g",
            "imu_ay_g",
            "imu_az_g",
        ]
    )
    return handle, writer


def world_to_canvas(points: list[tuple[float, float]], width: int, height: int) -> np.ndarray:
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    if not points:
        return canvas

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min((width - 80) / span_x, (height - 80) / span_y)

    def project(point_x: float, point_y: float) -> tuple[int, int]:
        px = int((point_x - min_x) * scale + 40)
        py = int(height - ((point_y - min_y) * scale + 40))
        return px, py

    projected = [project(point_x, point_y) for point_x, point_y in points]
    if len(projected) >= 2:
        cv2.polylines(canvas, [np.array(projected, dtype=np.int32)], False, (70, 210, 255), 2, cv2.LINE_AA)
    cv2.circle(canvas, projected[-1], 6, (255, 240, 120), -1, cv2.LINE_AA)
    cv2.putText(canvas, "top-down XY trajectory", (24, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (230, 230, 230), 1, cv2.LINE_AA)
    return canvas


def draw_heading_arrow(frame: np.ndarray, center: tuple[int, int], yaw_deg: float):
    yaw_rad = math.radians(yaw_deg)
    end_x = int(center[0] + math.cos(yaw_rad) * 40.0)
    end_y = int(center[1] - math.sin(yaw_rad) * 40.0)
    cv2.arrowedLine(frame, center, (end_x, end_y), (120, 255, 120), 2, cv2.LINE_AA, tipLength=0.2)


def open_cube_height_feed(args):
    if args.cube_height != "on":
        return None, None
    connection = connect_to_cube(args.ports, args.baud, heartbeat_timeout_s=args.heartbeat_timeout_seconds)
    configure_telemetry_streams(connection.master)
    state = FlightControllerTelemetry(active_source_set=request_active_source_set(connection.master))
    return connection, state


def close_cube_height_feed(connection) -> None:
    if connection is None:
        return
    master = getattr(connection, "master", None)
    if master is not None and hasattr(master, "close"):
        try:
            master.close()
        except Exception:  # noqa: BLE001
            pass


def cube_status_summary(cube_state) -> str:
    if cube_state is None:
        return "off"
    range_text = "-1.00"
    if cube_state.rangefinder_distance_m is not None:
        range_text = f"{cube_state.rangefinder_distance_m:.2f}"
    return (
        f"mode={cube_state.flight_mode}"
        f" rng={range_text}"
        f" ok={'yes' if rangefinder_height_valid(cube_state) else 'no'}"
    )


def render_preview(
    width: int,
    height: int,
    path_points: list[tuple[float, float]],
    pose: PoseSample,
    imu_sample,
    cube_state,
    elapsed_s: float,
) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    left_width = int(width * 0.58)
    traj = world_to_canvas(path_points, left_width - 24, height - 24)
    frame[12 : 12 + traj.shape[0], 12 : 12 + traj.shape[1]] = traj

    yaw_deg = quaternion_to_yaw_deg(pose)
    draw_heading_arrow(frame, (left_width // 2, height - 80), yaw_deg)

    panel_x = left_width + 16
    cv2.putText(frame, "Local VIO Session", (panel_x, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (240, 240, 240), 1, cv2.LINE_AA)
    lines = [
        f"elapsed_s={elapsed_s:.1f}",
        f"tracking={pose.tracking_state}",
        f"quality={pose.pose_quality}",
        f"xyz=({pose.x_m:+.3f}, {pose.y_m:+.3f}, {pose.z_m:+.3f})",
        f"vel=({pose.vx_m_s:+.3f}, {pose.vy_m_s:+.3f}, {pose.vz_m_s:+.3f})",
        f"yaw_deg={yaw_deg:+.2f}",
        f"features={pose.feature_count} tracked={pose.tracked_feature_count} inliers={pose.inlier_count}",
    ]
    if imu_sample is not None:
        lines.extend(
            [
                f"imu_rpy=({imu_sample.roll_deg:+.2f}, {imu_sample.pitch_deg:+.2f}, {imu_sample.yaw_deg:+.2f})",
                f"imu_gyro=({imu_sample.gx_deg_s:+.2f}, {imu_sample.gy_deg_s:+.2f}, {imu_sample.gz_deg_s:+.2f})",
                f"imu_acc=({imu_sample.ax_g:+.3f}, {imu_sample.ay_g:+.3f}, {imu_sample.az_g:+.3f})",
            ]
        )
    if cube_state is not None:
        cube_rng_ok = rangefinder_height_valid(cube_state)
        cube_rng_text = "waiting"
        if cube_state.rangefinder_distance_m is not None:
            cube_rng_text = f"{cube_state.rangefinder_distance_m:.3f}m"
        lines.extend(
            [
                f"cube_mode={cube_state.flight_mode}",
                f"cube_source_set={cube_state.active_source_set if cube_state.active_source_set is not None else 'unknown'}",
                f"cube_rng={cube_rng_text} ok={'yes' if cube_rng_ok else 'no'}",
            ]
        )
    y = 72
    for line in lines:
        cv2.putText(frame, line, (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (210, 210, 210), 1, cv2.LINE_AA)
        y += 28
    return frame


def main():
    args = parse_args()
    vio = VioPoseSource()
    imu_reader = None
    cube_connection = None
    cube_state = None
    if args.imu == "on":
        imu_reader = Im10aReader.open(args.imu_port, args.imu_baud, args.imu_scan_seconds)
    if args.cube_height == "on":
        cube_connection, cube_state = open_cube_height_feed(args)

    csv_handle, csv_writer = open_csv_writer(args.csv_out)
    history = deque(maxlen=max(args.history_points, 10))
    started_s = time.time()
    next_status_s = started_s
    period_s = 1.0 / max(args.rate_hz, 0.1)
    deadline_s = started_s + args.duration if args.duration > 0 else None
    window_name = "Local VIO"

    print(
        "Running local VIO session:"
        f" imu={'off' if imu_reader is None else f'{imu_reader.port}@{imu_reader.baud}'}"
        f" cube_height={'off' if cube_connection is None else f'{cube_connection.port}@{cube_connection.baud}'}"
        f" preview={args.preview}"
        f" csv={'off' if csv_writer is None else args.csv_out}"
        f" rate={args.rate_hz:.1f}Hz"
    )

    try:
        while True:
            loop_started_s = time.time()
            if deadline_s is not None and loop_started_s >= deadline_s:
                break

            if cube_connection is not None and cube_state is not None:
                drain_fc_telemetry(cube_connection.master, cube_state)
                vio.set_external_height_m(
                    cube_state.rangefinder_distance_m if rangefinder_height_valid(cube_state) else None
                )

            pose = vio.sample()
            imu_sample = None
            if imu_reader is not None:
                imu_sample = imu_reader.poll(duration_s=min(0.02, period_s))
                if imu_sample is not None and args.use_imu_orientation == "on":
                    pose = apply_imu_sample_to_pose(pose, imu_sample)
                    pose.tracking_state = f"{pose.tracking_state}+imu"
                    pose.source_name = "vio+imu"
            if cube_connection is not None and cube_state is not None:
                if rangefinder_height_valid(cube_state):
                    pose.z_m = -float(cube_state.rangefinder_distance_m)
                    pose.tracking_state = f"{pose.tracking_state}+rng"
                    pose.source_name = f"{pose.source_name}+rng" if pose.source_name else "rng"

            history.append((pose.x_m, pose.y_m))
            elapsed_s = time.time() - started_s

            if csv_writer is not None:
                csv_writer.writerow(
                    [
                        elapsed_s,
                        pose.x_m,
                        pose.y_m,
                        pose.z_m,
                        pose.qw,
                        pose.qx,
                        pose.qy,
                        pose.qz,
                        pose.vx_m_s,
                        pose.vy_m_s,
                        pose.vz_m_s,
                        pose.pose_quality,
                        pose.tracking_state,
                        pose.feature_count,
                        pose.tracked_feature_count,
                        pose.inlier_count,
                        "" if cube_state is None else cube_state.flight_mode,
                        "" if cube_state is None or cube_state.active_source_set is None else cube_state.active_source_set,
                        "" if cube_state is None or cube_state.rangefinder_distance_m is None else cube_state.rangefinder_distance_m,
                        "" if cube_state is None else int(rangefinder_height_valid(cube_state)),
                        quaternion_to_yaw_deg(pose),
                        "" if imu_sample is None else imu_sample.roll_deg,
                        "" if imu_sample is None else imu_sample.pitch_deg,
                        "" if imu_sample is None else imu_sample.yaw_deg,
                        "" if imu_sample is None else imu_sample.gx_deg_s,
                        "" if imu_sample is None else imu_sample.gy_deg_s,
                        "" if imu_sample is None else imu_sample.gz_deg_s,
                        "" if imu_sample is None else imu_sample.ax_g,
                        "" if imu_sample is None else imu_sample.ay_g,
                        "" if imu_sample is None else imu_sample.az_g,
                    ]
                )

            now_s = time.time()
            if now_s >= next_status_s:
                print(
                    "Local VIO:"
                    f" state={pose.tracking_state}"
                    f" q={pose.pose_quality}"
                    f" xyz=({pose.x_m:+.3f},{pose.y_m:+.3f},{pose.z_m:+.3f})"
                    f" vel=({pose.vx_m_s:+.3f},{pose.vy_m_s:+.3f},{pose.vz_m_s:+.3f})"
                    f" feats={pose.feature_count}/{pose.tracked_feature_count}/{pose.inlier_count}"
                    f" cube={cube_status_summary(cube_state)}"
                    f" imu={'off' if imu_sample is None else f'rpy=({imu_sample.roll_deg:+.1f},{imu_sample.pitch_deg:+.1f},{imu_sample.yaw_deg:+.1f})'}"
                )
                next_status_s = now_s + max(args.status_seconds, 0.2)

            if args.preview == "on":
                frame = render_preview(
                    args.window_width,
                    args.window_height,
                    list(history),
                    pose,
                    imu_sample,
                    cube_state,
                    elapsed_s,
                )
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break

            remaining_s = period_s - (time.time() - loop_started_s)
            if remaining_s > 0:
                time.sleep(remaining_s)
    finally:
        if csv_handle is not None:
            csv_handle.close()
        if imu_reader is not None:
            imu_reader.close()
        close_cube_height_feed(cube_connection)
        vio.close()
        if args.preview == "on":
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
