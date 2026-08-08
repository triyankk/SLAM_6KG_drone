"""Timestamped JT16 + IM10A ROS 2 bridge for shadow-mode FAST-LIO2."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import threading
import time
from typing import Any

import numpy as np

from .clock_sync import AffineClockMapper
from .config import ConfigError, ProjectConfig, RotationConfig, load_config
from .im10a import Im10aDecoder, Im10aSample, Im10aSampleAssembler
from .paths import PROJECT_ROOT


JT16_FRAME_HEADER = struct.Struct("<8sIIQQ")
JT16_FRAME_MAGIC = b"OFJT16P1"
JT16_FRAME_VERSION = 2
JT16_MAXIMUM_POINTS = 1_000_000
JT16_MINIMUM_FRAME_SPAN_S = 0.10
JT16_MAXIMUM_FRAME_SPAN_S = 0.30
JT16_MAXIMUM_POINT_GAP_S = 0.05
JT16_SCAN_LINES = 16
JT16_MINIMUM_LIO_RANGE_M = 0.80
JT16_MAXIMUM_LIO_RANGE_M = 30.0
JT16_MINIMUM_FILTERED_POINTS = JT16_SCAN_LINES
JT16_INPUT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("timestamp", "<f8"),
        ("ring", "<u2"),
        ("intensity", "u1"),
        ("confidence", "u1"),
    ],
    align=False,
)
# Match the aligned PCL point layout used by Hesai's FAST-LIO2 Point type.
FAST_LIO_POINT_DTYPE = np.dtype(
    {
        "names": ("x", "y", "z", "intensity", "ring", "timestamp"),
        "formats": ("<f4", "<f4", "<f4", "<f4", "<u2", "<f8"),
        "offsets": (0, 4, 8, 16, 20, 24),
        "itemsize": 32,
    }
)


def _project_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def rotation_matrix(rotation: RotationConfig) -> np.ndarray:
    """Return the sensor-to-body Rz(yaw) Ry(pitch) Rx(roll) rotation."""

    roll, pitch, yaw = np.radians(
        (rotation.roll_deg, rotation.pitch_deg, rotation.yaw_deg)
    )
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=np.float64,
    )


def pack_fast_lio_points(
    records: np.ndarray,
    sensor_to_body: np.ndarray,
) -> np.ndarray:
    if records.dtype != JT16_INPUT_DTYPE:
        records = np.asarray(records, dtype=JT16_INPUT_DTYPE)
    if sensor_to_body.shape != (3, 3):
        raise ValueError("sensor_to_body must be a 3x3 matrix")
    # JT16 SDK XYZ is +Y forward, +X right, +Z up. Convert it to the
    # forward/right/down sensor frame before applying measured mount rotation.
    forward_frd = np.column_stack(
        (records["y"], records["x"], -records["z"])
    ).astype(np.float64, copy=False)
    transformed = forward_frd @ sensor_to_body.T
    packed = np.zeros(len(records), dtype=FAST_LIO_POINT_DTYPE)
    packed["x"] = transformed[:, 0]
    packed["y"] = transformed[:, 1]
    packed["z"] = transformed[:, 2]
    packed["intensity"] = records["intensity"].astype(np.float32)
    packed["ring"] = records["ring"]
    packed["timestamp"] = records["timestamp"]
    return packed


def filter_fast_lio_points(
    points: np.ndarray,
    *,
    minimum_range_m: float = JT16_MINIMUM_LIO_RANGE_M,
    maximum_range_m: float = JT16_MAXIMUM_LIO_RANGE_M,
    scan_lines: int = JT16_SCAN_LINES,
) -> tuple[np.ndarray, dict[str, int]]:
    """Remove malformed JT16 points before PCL can index the cloud."""

    values = np.asarray(points)
    if values.dtype != FAST_LIO_POINT_DTYPE:
        values = np.asarray(values, dtype=FAST_LIO_POINT_DTYPE)
    if minimum_range_m < 0.0 or maximum_range_m <= minimum_range_m:
        raise ValueError("JT16 LIO range limits are invalid")
    if scan_lines <= 0:
        raise ValueError("JT16 scan line count must be positive")

    coordinates = np.column_stack(
        (values["x"], values["y"], values["z"])
    ).astype(np.float64, copy=False)
    finite = np.all(np.isfinite(coordinates), axis=1)
    valid_ring = values["ring"] < scan_lines
    range_ok = np.zeros(len(values), dtype=bool)
    candidates = finite & valid_ring
    bounded = candidates & np.all(
        np.abs(coordinates) <= maximum_range_m,
        axis=1,
    )
    if np.any(bounded):
        bounded_coordinates = coordinates[bounded]
        squared_range = np.einsum(
            "ij,ij->i", bounded_coordinates, bounded_coordinates
        )
        range_ok[bounded] = (
            squared_range >= minimum_range_m * minimum_range_m
        ) & (squared_range <= maximum_range_m * maximum_range_m)

    valid = candidates & range_ok
    stats = {
        "input_points": int(len(values)),
        "accepted_points": int(np.count_nonzero(valid)),
        "non_finite_points": int(np.count_nonzero(~finite)),
        "invalid_ring_points": int(np.count_nonzero(finite & ~valid_ring)),
        "out_of_range_points": int(
            np.count_nonzero(candidates & ~range_ok)
        ),
    }
    return np.ascontiguousarray(values[valid]), stats


def jt16_frame_time_metrics(timestamps: np.ndarray) -> tuple[float, float]:
    values = np.asarray(timestamps, dtype=np.float64)
    if values.ndim != 1 or len(values) < 2:
        raise ValueError("JT16 frame needs at least two point timestamps")
    if not np.all(np.isfinite(values)):
        raise ValueError("JT16 frame contains non-finite timestamps")
    differences = np.diff(values)
    if np.any(differences < 0.0):
        raise ValueError("JT16 frame timestamps must be monotonic")
    return float(values[-1] - values[0]), float(np.max(differences))


def _read_exact(
    process: subprocess.Popen[bytes],
    size: int,
    stop_event: threading.Event,
) -> bytes | None:
    output = process.stdout
    if output is None:
        return None
    descriptor = output.fileno()
    collected = bytearray()
    while len(collected) < size and not stop_event.is_set():
        if process.poll() is not None:
            return None
        ready, _, _ = select.select((descriptor,), (), (), 0.2)
        if not ready:
            continue
        chunk = os.read(descriptor, size - len(collected))
        if not chunk:
            return None
        collected.extend(chunk)
    return bytes(collected) if len(collected) == size else None


def _rate_hz(count: int, first_ns: int | None, last_ns: int | None) -> float | None:
    if count < 2 or first_ns is None or last_ns is None or last_ns <= first_ns:
        return None
    return (count - 1) / ((last_ns - first_ns) / 1.0e9)


class BridgeState:
    def __init__(self, config: ProjectConfig, *, proof_mode: bool = False) -> None:
        clock = config.lidar_inertial_odometry.clock_sync
        imu_residual_limit_ms = (
            max(10.0, clock.maximum_imu_residual_p95_ms)
            if proof_mode
            else clock.maximum_imu_residual_p95_ms
        )
        self.imu_clock = AffineClockMapper(
            window_samples=clock.window_samples,
            minimum_samples=clock.minimum_imu_samples,
            minimum_span_s=clock.minimum_span_s,
            maximum_drift_ppm=clock.maximum_drift_ppm,
            maximum_residual_p95_ms=imu_residual_limit_ms,
            maximum_window_span_s=clock.maximum_imu_window_span_s,
        )
        self.lidar_clock = AffineClockMapper(
            window_samples=clock.window_samples,
            minimum_samples=clock.minimum_lidar_samples,
            minimum_span_s=clock.minimum_span_s,
            maximum_drift_ppm=clock.maximum_drift_ppm,
            maximum_residual_p95_ms=clock.maximum_lidar_residual_p95_ms,
            maximum_window_span_s=clock.maximum_lidar_window_span_s,
        )
        self.time_offset_lidar_to_imu_s = (
            clock.time_offset_lidar_to_imu_s
        )
        self.proof_mode = proof_mode
        self.imu_residual_limit_ms = imu_residual_limit_ms
        self.imu_queue: deque[Im10aSample] = deque(maxlen=4000)
        self.lidar_queue: deque[tuple[float, np.ndarray, int]] = deque(maxlen=40)
        self.lock = threading.Lock()
        self.imu_connected = False
        self.lidar_connected = False
        self.imu_error: str | None = None
        self.lidar_error: str | None = None
        self.imu_samples = 0
        self.lidar_frames = 0
        self.imu_published = 0
        self.lidar_published = 0
        self.imu_first_ns: int | None = None
        self.imu_last_ns: int | None = None
        self.lidar_first_ns: int | None = None
        self.lidar_last_ns: int | None = None
        self.imu_checksum_errors = 0
        self.imu_payload_errors = 0
        self.imu_incomplete_samples = 0
        self.imu_queue_drops = 0
        self.imu_sync_discards = 0
        self.lidar_non_monotonic_frames = 0
        self.lidar_queue_drops = 0
        self.lidar_sync_discards = 0
        self.lidar_received_frames = 0
        self.lidar_time_rejected_frames = 0
        self.lidar_runtime_time_rejected_frames = 0
        self.lidar_input_points = 0
        self.lidar_published_points = 0
        self.lidar_non_finite_points = 0
        self.lidar_invalid_ring_points = 0
        self.lidar_out_of_range_points = 0
        self.lidar_sparse_filtered_frames = 0
        self.lidar_frame_span_s: float | None = None
        self.lidar_maximum_point_gap_s: float | None = None

    def diagnostics(self) -> dict[str, Any]:
        with self.lock:
            clocks_ready = self.imu_clock.ready and self.lidar_clock.ready
            return {
                "schema_version": 1,
                "mode": "shadow",
                "proof_mode": self.proof_mode,
                "pose_output_to_cube": False,
                "synchronized": clocks_ready,
                "publishing": clocks_ready,
                "time_offset_lidar_to_imu_s": (
                    self.time_offset_lidar_to_imu_s
                ),
                "imu_residual_limit_ms": self.imu_residual_limit_ms,
                "imu": {
                    "connected": self.imu_connected,
                    "error": self.imu_error,
                    "samples": self.imu_samples,
                    "published": self.imu_published,
                    "queued": len(self.imu_queue),
                    "rate_hz": _rate_hz(
                        self.imu_samples,
                        self.imu_first_ns,
                        self.imu_last_ns,
                    ),
                    "checksum_errors": self.imu_checksum_errors,
                    "payload_errors": self.imu_payload_errors,
                    "incomplete_samples": self.imu_incomplete_samples,
                    "queue_drops": self.imu_queue_drops,
                    "sync_discards": self.imu_sync_discards,
                    "clock": self.imu_clock.fit.as_dict(),
                    "clock_window_limit_s": (
                        self.imu_clock.maximum_window_span_s
                    ),
                },
                "lidar": {
                    "connected": self.lidar_connected,
                    "error": self.lidar_error,
                    "received_frames": self.lidar_received_frames,
                    "frames": self.lidar_frames,
                    "published": self.lidar_published,
                    "queued": len(self.lidar_queue),
                    "rate_hz": _rate_hz(
                        self.lidar_frames,
                        self.lidar_first_ns,
                        self.lidar_last_ns,
                    ),
                    "non_monotonic_frames": (
                        self.lidar_non_monotonic_frames
                    ),
                    "time_rejected_frames": (
                        self.lidar_time_rejected_frames
                    ),
                    "runtime_time_rejected_frames": (
                        self.lidar_runtime_time_rejected_frames
                    ),
                    "input_points": self.lidar_input_points,
                    "published_points": self.lidar_published_points,
                    "rejected_points": {
                        "non_finite": self.lidar_non_finite_points,
                        "invalid_ring": self.lidar_invalid_ring_points,
                        "out_of_range": self.lidar_out_of_range_points,
                    },
                    "sparse_filtered_frames": (
                        self.lidar_sparse_filtered_frames
                    ),
                    "frame_span_s": self.lidar_frame_span_s,
                    "maximum_point_gap_s": (
                        self.lidar_maximum_point_gap_s
                    ),
                    "queue_drops": self.lidar_queue_drops,
                    "sync_discards": self.lidar_sync_discards,
                    "clock": self.lidar_clock.fit.as_dict(),
                    "clock_window_limit_s": (
                        self.lidar_clock.maximum_window_span_s
                    ),
                },
            }


def _imu_worker(
    config: ProjectConfig,
    state: BridgeState,
    stop_event: threading.Event,
) -> None:
    try:
        import serial

        decoder = Im10aDecoder()
        assembler = Im10aSampleAssembler()
        with serial.Serial(
            config.external_imu.symlink,
            config.external_imu.baud,
            timeout=0.02,
            exclusive=True,
        ) as port:
            port.reset_input_buffer()
            with state.lock:
                state.imu_connected = True
            while not stop_event.is_set():
                data = port.read(max(1, port.in_waiting))
                received_ns = time.monotonic_ns()
                if not data:
                    continue
                samples: list[Im10aSample] = []
                for measurement in decoder.feed(data):
                    sample = assembler.push(measurement, received_ns)
                    if sample is not None:
                        samples.append(sample)
                if not samples:
                    continue
                with state.lock:
                    # The last sample is the closest observation of when this
                    # USB batch reached the host.
                    anchor = samples[-1]
                    state.imu_clock.add(
                        anchor.sensor_time_s,
                        anchor.host_monotonic_ns / 1.0e9,
                    )
                    for sample in samples:
                        if len(state.imu_queue) == state.imu_queue.maxlen:
                            state.imu_queue_drops += 1
                        state.imu_queue.append(sample)
                        state.imu_samples += 1
                        state.imu_first_ns = (
                            state.imu_first_ns or sample.host_monotonic_ns
                        )
                        state.imu_last_ns = sample.host_monotonic_ns
                        state.imu_checksum_errors = decoder.checksum_errors
                        state.imu_payload_errors = decoder.payload_errors
                        state.imu_incomplete_samples = (
                            assembler.incomplete_samples
                        )
    except Exception as exc:
        with state.lock:
            state.imu_connected = False
            state.imu_error = str(exc)
        stop_event.set()


def _lidar_worker(
    config: ProjectConfig,
    state: BridgeState,
    stop_event: threading.Event,
    bridge_log: Path,
) -> None:
    lidar = config.lidar
    bridge = _project_path(lidar.bridge_binary)
    correction = _project_path(lidar.correction_file)
    process: subprocess.Popen[bytes] | None = None
    log_output = None
    try:
        if not bridge.is_file() or not os.access(bridge, os.X_OK):
            raise OSError(f"JT16 bridge is not executable: {bridge}")
        if not correction.is_file():
            raise OSError(f"JT16 correction is missing: {correction}")
        bridge_log.parent.mkdir(parents=True, exist_ok=True)
        log_output = bridge_log.open("wb")
        process = subprocess.Popen(
            (
                str(bridge),
                "--device",
                lidar.symlink,
                "--baud",
                str(lidar.baud),
                "--correction",
                str(correction),
                "--startup-timeout",
                "5",
            ),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=log_output,
            bufsize=0,
        )
        sensor_to_body = rotation_matrix(lidar.rotation_to_body_frd)
        with state.lock:
            state.lidar_connected = True
        while not stop_event.is_set():
            header = _read_exact(process, JT16_FRAME_HEADER.size, stop_event)
            if header is None:
                break
            magic, version, count, callback_ns, frame_index = (
                JT16_FRAME_HEADER.unpack(header)
            )
            if magic != JT16_FRAME_MAGIC or version != JT16_FRAME_VERSION:
                raise ValueError("JT16 bridge frame header is invalid")
            if count <= 0 or count > JT16_MAXIMUM_POINTS:
                raise ValueError("JT16 point count is outside limits")
            payload = _read_exact(
                process,
                count * JT16_INPUT_DTYPE.itemsize,
                stop_event,
            )
            if payload is None:
                break
            records = np.frombuffer(payload, dtype=JT16_INPUT_DTYPE).copy()
            timestamps = records["timestamp"]
            with state.lock:
                state.lidar_received_frames += 1
            non_monotonic = bool(np.any(np.diff(timestamps) < 0.0))
            if non_monotonic:
                records = records[np.argsort(timestamps, kind="stable")]
                timestamps = records["timestamp"]
            frame_span_s, maximum_gap_s = jt16_frame_time_metrics(timestamps)
            valid_timing = (
                JT16_MINIMUM_FRAME_SPAN_S
                <= frame_span_s
                <= JT16_MAXIMUM_FRAME_SPAN_S
                and maximum_gap_s <= JT16_MAXIMUM_POINT_GAP_S
            )
            if not valid_timing:
                with state.lock:
                    state.lidar_time_rejected_frames += 1
                    if state.imu_clock.ready and state.lidar_clock.ready:
                        state.lidar_runtime_time_rejected_frames += 1
                continue
            first_sensor_s = float(timestamps[0])
            last_sensor_s = float(timestamps[-1])
            points = pack_fast_lio_points(records, sensor_to_body)
            points, point_stats = filter_fast_lio_points(points)
            with state.lock:
                state.lidar_input_points += point_stats["input_points"]
                state.lidar_non_finite_points += point_stats[
                    "non_finite_points"
                ]
                state.lidar_invalid_ring_points += point_stats[
                    "invalid_ring_points"
                ]
                state.lidar_out_of_range_points += point_stats[
                    "out_of_range_points"
                ]
                if len(points) < JT16_MINIMUM_FILTERED_POINTS:
                    state.lidar_sparse_filtered_frames += 1
                    continue
                state.lidar_clock.add(
                    last_sensor_s,
                    callback_ns / 1.0e9,
                )
                if len(state.lidar_queue) == state.lidar_queue.maxlen:
                    state.lidar_queue_drops += 1
                state.lidar_queue.append(
                    (first_sensor_s, points, int(frame_index))
                )
                state.lidar_frames += 1
                state.lidar_published_points += len(points)
                state.lidar_first_ns = state.lidar_first_ns or callback_ns
                state.lidar_last_ns = callback_ns
                state.lidar_frame_span_s = frame_span_s
                state.lidar_maximum_point_gap_s = maximum_gap_s
                if non_monotonic:
                    state.lidar_non_monotonic_frames += 1
        if process.poll() is not None and not stop_event.is_set():
            raise RuntimeError(f"JT16 bridge exited with {process.returncode}")
    except Exception as exc:
        with state.lock:
            state.lidar_connected = False
            state.lidar_error = str(exc)
        stop_event.set()
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if process is not None and process.stdout is not None:
            process.stdout.close()
        if log_output is not None:
            log_output.close()


def _validate_shadow_configuration(config: ProjectConfig) -> None:
    lio = config.lidar_inertial_odometry
    if lio.stage != "shadow":
        raise ConfigError("this bridge is deliberately restricted to shadow stage")
    if lio.pose_output_to_cube_enabled:
        raise ConfigError("shadow bridge cannot send pose to Cube")
    if not config.external_imu.sensor_time_enabled:
        raise ConfigError(
            "IM10A sensor time is disabled; apply and verify the LIO profile first"
        )
    if config.external_imu.expected_rate_hz < lio.required_imu_rate_hz:
        raise ConfigError(
            "configured IM10A rate is below the LIO requirement"
        )
    if not config.external_imu.axis_map_verified:
        raise ConfigError("IM10A body-axis map is not verified")


def run_ros_bridge(
    config: ProjectConfig,
    *,
    bridge_log: Path,
    proof_mode: bool = False,
) -> int:
    try:
        import rclpy
        from builtin_interfaces.msg import Time as RosTime
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import Imu, PointCloud2, PointField
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(
            f"ROS 2 Python runtime is unavailable: {exc}; "
            "run ./optflow build-lio"
        ) from exc

    state = BridgeState(config, proof_mode=proof_mode)
    stop_event = threading.Event()
    realtime_offset_ns = time.time_ns() - time.monotonic_ns()
    imu_signs = (
        config.external_imu.body_axis_signs.x,
        config.external_imu.body_axis_signs.y,
        config.external_imu.body_axis_signs.z,
    )
    lio = config.lidar_inertial_odometry
    imu_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1000,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    def ros_stamp(monotonic_s: float) -> Any:
        unix_ns = int(round(monotonic_s * 1.0e9)) + realtime_offset_ns
        stamp = RosTime()
        stamp.sec = unix_ns // 1_000_000_000
        stamp.nanosec = unix_ns % 1_000_000_000
        return stamp

    class LioBridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("optflow_lio_sensor_bridge")
            self.imu_publisher = self.create_publisher(
                Imu,
                lio.imu_topic,
                imu_qos,
            )
            self.lidar_publisher = self.create_publisher(
                PointCloud2,
                lio.pointcloud_topic,
                qos_profile_sensor_data,
            )
            self.diagnostic_publisher = self.create_publisher(
                String,
                lio.diagnostics_topic,
                10,
            )
            self.create_timer(0.002, self.drain)
            self.create_timer(0.5, self.publish_diagnostics)
            self.last_imu_stamp_s = -math.inf
            self.last_lidar_stamp_s = -math.inf
            self.clocks_were_ready = False

        def drain(self) -> None:
            with state.lock:
                clocks_ready = state.imu_clock.ready and state.lidar_clock.ready
                if not clocks_ready:
                    self.clocks_were_ready = False
                    return
                if not self.clocks_were_ready:
                    state.imu_sync_discards += len(state.imu_queue)
                    state.lidar_sync_discards += len(state.lidar_queue)
                    state.imu_queue.clear()
                    state.lidar_queue.clear()
                    self.clocks_were_ready = True
                    return
                imu_samples = []
                for _ in range(min(40, len(state.imu_queue))):
                    sample = state.imu_queue.popleft()
                    imu_samples.append(
                        (sample, state.imu_clock.map(sample.sensor_time_s))
                    )
                lidar_frames = []
                for _ in range(min(2, len(state.lidar_queue))):
                    first_sensor_s, points, frame_index = (
                        state.lidar_queue.popleft()
                    )
                    lidar_frames.append(
                        (
                            state.lidar_clock.map(first_sensor_s),
                            points,
                            frame_index,
                        )
                    )

            for sample, mapped_s in imu_samples:
                if mapped_s <= self.last_imu_stamp_s:
                    continue
                message = Imu()
                message.header.stamp = ros_stamp(mapped_s)
                message.header.frame_id = "body_frd"
                message.orientation_covariance[0] = -1.0
                message.linear_acceleration.x = sample.accel_mss[0] * imu_signs[0]
                message.linear_acceleration.y = sample.accel_mss[1] * imu_signs[1]
                message.linear_acceleration.z = sample.accel_mss[2] * imu_signs[2]
                message.angular_velocity.x = sample.gyro_rads[0] * imu_signs[0]
                message.angular_velocity.y = sample.gyro_rads[1] * imu_signs[1]
                message.angular_velocity.z = sample.gyro_rads[2] * imu_signs[2]
                self.imu_publisher.publish(message)
                self.last_imu_stamp_s = mapped_s
                with state.lock:
                    state.imu_published += 1

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="intensity",
                    offset=16,
                    datatype=PointField.FLOAT32,
                    count=1,
                ),
                PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
                PointField(
                    name="timestamp",
                    offset=24,
                    datatype=PointField.FLOAT64,
                    count=1,
                ),
            ]
            for mapped_s, points, _frame_index in lidar_frames:
                if mapped_s <= self.last_lidar_stamp_s:
                    continue
                message = PointCloud2()
                message.header.stamp = ros_stamp(mapped_s)
                message.header.frame_id = "jt16_body_aligned"
                message.height = 1
                message.width = len(points)
                message.fields = fields
                message.is_bigendian = False
                message.point_step = FAST_LIO_POINT_DTYPE.itemsize
                message.row_step = message.point_step * message.width
                message.is_dense = bool(
                    np.all(np.isfinite(points["x"]))
                    and np.all(np.isfinite(points["y"]))
                    and np.all(np.isfinite(points["z"]))
                )
                message.data = points.tobytes()
                self.lidar_publisher.publish(message)
                self.last_lidar_stamp_s = mapped_s
                with state.lock:
                    state.lidar_published += 1

        def publish_diagnostics(self) -> None:
            message = String()
            message.data = json.dumps(
                {
                    **state.diagnostics(),
                    "host_monotonic_ns": time.monotonic_ns(),
                    "host_unix_ns": time.time_ns(),
                },
                sort_keys=True,
            )
            self.diagnostic_publisher.publish(message)

    rclpy.init()
    node = LioBridgeNode()
    workers = (
        threading.Thread(
            target=_imu_worker,
            args=(config, state, stop_event),
            name="lio-im10a",
            daemon=True,
        ),
        threading.Thread(
            target=_lidar_worker,
            args=(config, state, stop_event, bridge_log),
            name="lio-jt16",
            daemon=True,
        ),
    )
    for worker in workers:
        worker.start()
    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok() and not stop_event.is_set():
            raise
    finally:
        stop_event.set()
        for worker in workers:
            worker.join(timeout=7.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    diagnostics = state.diagnostics()
    if diagnostics["imu"]["error"] or diagnostics["lidar"]["error"]:
        print(json.dumps(diagnostics, indent=2, sort_keys=True))
        return 2
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish synchronized JT16 and IM10A data for shadow LIO",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "system.yaml",
    )
    parser.add_argument(
        "--bridge-log",
        type=Path,
        default=PROJECT_ROOT / "runtime" / "lio" / "jt16-bridge.log",
    )
    parser.add_argument(
        "--proof-mode",
        action="store_true",
        help="Use a shadow-only 10 ms IMU clock residual limit",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
        _validate_shadow_configuration(config)
        return run_ros_bridge(
            config,
            bridge_log=args.bridge_log,
            proof_mode=args.proof_mode,
        )
    except (ConfigError, OSError, RuntimeError, ValueError) as exc:
        print(f"LIO bridge error: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
