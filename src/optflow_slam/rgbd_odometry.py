"""Metric RGB-D odometry with an IMU rotation prior for shadow SLAM tests."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time
import traceback
from typing import Any, Callable, Protocol

import numpy as np

from .config import DepthCameraConfig, RotationConfig
from .obstacles import DepthObstacleExtractor, ObstacleScan
from .pointcloud import VoxelMap


RGBD_WIDTH = 320
RGBD_HEIGHT = 240
MINIMUM_DEPTH_M = 0.30
MAXIMUM_DEPTH_M = 6.0


class RowWriter(Protocol):
    def write(self, row: dict[str, Any]) -> None: ...


class RgbdStateSink(Protocol):
    def update_rgbd(self, row: dict[str, Any]) -> None: ...

    def update_rgbd_map(
        self, points_m: np.ndarray, colors_rgb: np.ndarray
    ) -> None: ...

    def set_rgbd_error(self, detail: str) -> None: ...


def load_cv2() -> Any:
    """Load the JetPack OpenCV build without changing the venv globally."""

    try:
        import cv2

        return cv2
    except ImportError:
        system_packages = "/usr/lib/python3/dist-packages"
        if system_packages not in sys.path:
            sys.path.append(system_packages)
        import cv2

        return cv2


def _rotation_matrix(rotation: RotationConfig) -> np.ndarray:
    roll = math.radians(rotation.roll_deg)
    pitch = math.radians(rotation.pitch_deg)
    yaw = math.radians(rotation.yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotate_x = np.array(
        ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)),
        dtype=np.float64,
    )
    rotate_y = np.array(
        ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)),
        dtype=np.float64,
    )
    rotate_z = np.array(
        ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )
    return rotate_z @ rotate_y @ rotate_x


def camera_to_body_transform(camera: DepthCameraConfig) -> np.ndarray:
    """Return T_body_camera for RealSense optical coordinates."""

    nominal_body_from_optical = np.array(
        ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = (
        _rotation_matrix(camera.rotation_from_forward_frd)
        @ nominal_body_from_optical
    )
    position = camera.position_from_cg_frd_m
    transform[:3, 3] = (position.x, position.y, position.z)
    return transform


def _rodrigues(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1.0e-12:
        return np.eye(3, dtype=np.float64)
    axis = rotation_vector / angle
    x_value, y_value, z_value = axis
    skew = np.array(
        (
            (0.0, -z_value, y_value),
            (z_value, 0.0, -x_value),
            (-y_value, x_value, 0.0),
        ),
        dtype=np.float64,
    )
    return (
        np.eye(3, dtype=np.float64)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def rotation_angle_rad(rotation: np.ndarray) -> float:
    cosine = float((np.trace(rotation) - 1.0) * 0.5)
    return math.acos(max(-1.0, min(1.0, cosine)))


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    """Convert a proper 3x3 rotation matrix to a normalized quaternion."""

    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ),
            dtype=np.float64,
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(
                max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            ) * 2.0
            quaternion = np.array(
                (
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                )
            )
        elif index == 1:
            scale = math.sqrt(
                max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            ) * 2.0
            quaternion = np.array(
                (
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                )
            )
        else:
            scale = math.sqrt(
                max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            ) * 2.0
            quaternion = np.array(
                (
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                )
            )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [float(value) for value in quaternion / norm]


@dataclass(frozen=True)
class GyroPrior:
    transform_current_from_previous: np.ndarray
    sample_count: int
    covered_duration_s: float


class GyroPriorBuffer:
    """Integrate timestamped body-frame gyro samples between camera frames."""

    def __init__(self, maximum_age_s: float = 3.0) -> None:
        self.maximum_age_ns = int(maximum_age_s * 1.0e9)
        self._samples: deque[tuple[int, np.ndarray]] = deque()
        self._condition = threading.Condition()

    def add(self, timestamp_ns: int, angular_velocity_rads: Any) -> None:
        values = np.asarray(angular_velocity_rads, dtype=np.float64)
        if values.shape != (3,) or not np.isfinite(values).all():
            return
        timestamp = int(timestamp_ns)
        with self._condition:
            if self._samples and timestamp <= self._samples[-1][0]:
                return
            self._samples.append((timestamp, values.copy()))
            cutoff = timestamp - self.maximum_age_ns
            while len(self._samples) > 2 and self._samples[1][0] < cutoff:
                self._samples.popleft()
            self._condition.notify_all()

    def wait_rotation_prior(
        self,
        start_ns: int,
        end_ns: int,
        body_from_camera_rotation: np.ndarray,
        *,
        timeout_s: float = 0.40,
    ) -> GyroPrior | None:
        deadline_s = time.monotonic() + timeout_s
        with self._condition:
            while True:
                prior = self.rotation_prior(
                    start_ns,
                    end_ns,
                    body_from_camera_rotation,
                )
                if prior is not None:
                    return prior
                remaining_s = deadline_s - time.monotonic()
                if remaining_s <= 0.0:
                    return None
                self._condition.wait(min(0.03, remaining_s))

    def rotation_prior(
        self,
        start_ns: int,
        end_ns: int,
        body_from_camera_rotation: np.ndarray,
        *,
        maximum_edge_gap_s: float = 0.025,
    ) -> GyroPrior | None:
        if end_ns <= start_ns:
            return None
        with self._condition:
            samples = [
                (timestamp, values.copy())
                for timestamp, values in self._samples
                if start_ns - 30_000_000 <= timestamp <= end_ns + 30_000_000
            ]
        if len(samples) < 2:
            return None

        before_start = [item for item in samples if item[0] <= start_ns]
        before_end = [item for item in samples if item[0] <= end_ns]
        if not before_start or not before_end:
            return None
        start_sample = before_start[-1]
        end_sample = before_end[-1]
        maximum_edge_gap_ns = int(maximum_edge_gap_s * 1.0e9)
        if (
            start_ns - start_sample[0] > maximum_edge_gap_ns
            or end_ns - end_sample[0] > maximum_edge_gap_ns
        ):
            return None

        selected = [start_sample]
        selected.extend(
            item for item in samples if start_ns < item[0] < end_ns
        )
        selected.append((end_ns, end_sample[1]))
        selected.sort(key=lambda item: item[0])
        body_from_camera = np.asarray(
            body_from_camera_rotation, dtype=np.float64
        )
        camera_from_body = body_from_camera.T
        active_previous_from_current = np.eye(3, dtype=np.float64)
        covered_s = 0.0
        for first, second in zip(selected, selected[1:]):
            interval_start = max(start_ns, first[0])
            interval_end = min(end_ns, second[0])
            if interval_end <= interval_start:
                continue
            duration_s = (interval_end - interval_start) / 1.0e9
            omega_body = 0.5 * (first[1] + second[1])
            omega_camera = camera_from_body @ omega_body
            active_previous_from_current = (
                active_previous_from_current
                @ _rodrigues(omega_camera * duration_s)
            )
            covered_s += duration_s
        expected_s = (end_ns - start_ns) / 1.0e9
        if covered_s < expected_s * 0.80:
            return None
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = active_previous_from_current.T
        return GyroPrior(transform, len(selected), covered_s)


@dataclass(frozen=True)
class RgbdEstimate:
    initialized: bool
    tracking: bool
    transform_world_camera: np.ndarray
    transform_current_previous: np.ndarray | None
    compute_ms: float
    valid_depth_fraction: float
    step_translation_m: float | None
    step_rotation_deg: float | None
    gyro_prior_used: bool
    gyro_prior_samples: int


class RgbdOdometryEngine:
    """Stateful wrapper around OpenCV's dense metric RGB-D odometry."""

    def __init__(
        self,
        camera_matrix: np.ndarray,
        *,
        cv2_module: Any | None = None,
        minimum_depth_m: float = MINIMUM_DEPTH_M,
        maximum_depth_m: float = MAXIMUM_DEPTH_M,
    ) -> None:
        self.cv2 = cv2_module or load_cv2()
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        if self.camera_matrix.shape != (3, 3):
            raise ValueError("camera_matrix must have shape (3, 3)")
        self.minimum_depth_m = float(minimum_depth_m)
        self.maximum_depth_m = float(maximum_depth_m)
        self.odometry = self.cv2.rgbd.RgbdOdometry_create(
            self.camera_matrix,
            self.minimum_depth_m,
            self.maximum_depth_m,
            0.10,
            np.array((7, 7, 7, 10), dtype=np.int32),
            np.array((10, 5, 3, 1), dtype=np.float32),
            0.10,
            self.cv2.rgbd.Odometry_RIGID_BODY_MOTION,
        )
        self.odometry.setMaxTranslation(0.50)
        self.odometry.setMaxRotation(0.50)
        self._previous_gray: np.ndarray | None = None
        self._previous_depth: np.ndarray | None = None
        self._previous_mask: np.ndarray | None = None
        self.transform_world_camera = np.eye(4, dtype=np.float64)

    def process(
        self,
        gray: np.ndarray,
        depth_m: np.ndarray,
        *,
        initial_transform: np.ndarray | None = None,
        gyro_prior_samples: int = 0,
    ) -> RgbdEstimate:
        image = np.ascontiguousarray(gray, dtype=np.uint8)
        depth = np.ascontiguousarray(depth_m, dtype=np.float32)
        if image.ndim != 2 or depth.shape != image.shape:
            raise ValueError("gray and depth_m must be matching 2D arrays")
        mask = np.where(
            np.isfinite(depth)
            & (depth >= self.minimum_depth_m)
            & (depth <= self.maximum_depth_m),
            255,
            0,
        ).astype(np.uint8)
        valid_fraction = float(np.count_nonzero(mask) / mask.size)
        if self._previous_gray is None:
            self._set_reference(image, depth, mask)
            return RgbdEstimate(
                initialized=True,
                tracking=False,
                transform_world_camera=self.transform_world_camera.copy(),
                transform_current_previous=None,
                compute_ms=0.0,
                valid_depth_fraction=valid_fraction,
                step_translation_m=None,
                step_rotation_deg=None,
                gyro_prior_used=False,
                gyro_prior_samples=0,
            )

        started_s = time.perf_counter()
        prior = (
            np.asarray(initial_transform, dtype=np.float64)
            if initial_transform is not None
            else np.eye(4, dtype=np.float64)
        )
        if prior.shape != (4, 4) or not np.isfinite(prior).all():
            raise ValueError("initial_transform must be a finite 4x4 matrix")
        success, transform = self.odometry.compute(
            self._previous_gray,
            self._previous_depth,
            self._previous_mask,
            image,
            depth,
            mask,
            np.eye(4, dtype=np.float64),
            prior,
        )
        compute_ms = (time.perf_counter() - started_s) * 1000.0
        transform = np.asarray(transform, dtype=np.float64)
        tracking = bool(
            success
            and transform.shape == (4, 4)
            and np.isfinite(transform).all()
        )
        step_translation_m = None
        step_rotation_deg = None
        if tracking:
            step_translation_m = float(np.linalg.norm(transform[:3, 3]))
            step_rotation_deg = math.degrees(
                rotation_angle_rad(transform[:3, :3])
            )
            if step_translation_m > 0.50 or step_rotation_deg > 30.0:
                tracking = False
            else:
                self.transform_world_camera = (
                    self.transform_world_camera @ np.linalg.inv(transform)
                )
        self._set_reference(image, depth, mask)
        return RgbdEstimate(
            initialized=True,
            tracking=tracking,
            transform_world_camera=self.transform_world_camera.copy(),
            transform_current_previous=(transform.copy() if tracking else None),
            compute_ms=compute_ms,
            valid_depth_fraction=valid_fraction,
            step_translation_m=step_translation_m if tracking else None,
            step_rotation_deg=step_rotation_deg if tracking else None,
            gyro_prior_used=initial_transform is not None,
            gyro_prior_samples=(gyro_prior_samples if initial_transform is not None else 0),
        )

    def _set_reference(
        self, gray: np.ndarray, depth: np.ndarray, mask: np.ndarray
    ) -> None:
        self._previous_gray = gray.copy()
        self._previous_depth = depth.copy()
        self._previous_mask = mask.copy()


def _local_flu_transform(transform_body0_body: np.ndarray) -> np.ndarray:
    frd_to_flu = np.diag((1.0, -1.0, -1.0, 1.0))
    return frd_to_flu @ transform_body0_body @ frd_to_flu


def backproject_depth(
    depth_m: np.ndarray,
    color_bgr: np.ndarray,
    camera_matrix: np.ndarray,
    *,
    stride: int = 4,
    minimum_depth_m: float = MINIMUM_DEPTH_M,
    maximum_depth_m: float = MAXIMUM_DEPTH_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Backproject a sampled registered RGB-D frame in optical coordinates."""

    if stride < 1:
        raise ValueError("stride must be positive")
    depth = np.asarray(depth_m, dtype=np.float32)
    color = np.asarray(color_bgr, dtype=np.uint8)
    if depth.ndim != 2 or color.shape != (*depth.shape, 3):
        raise ValueError("depth and color dimensions do not match")
    rows = np.arange(0, depth.shape[0], stride)
    columns = np.arange(0, depth.shape[1], stride)
    grid_x, grid_y = np.meshgrid(columns, rows)
    sampled_depth = depth[grid_y, grid_x]
    valid = (
        np.isfinite(sampled_depth)
        & (sampled_depth >= minimum_depth_m)
        & (sampled_depth <= maximum_depth_m)
    )
    z_value = sampled_depth[valid]
    x_value = (
        (grid_x[valid] - camera_matrix[0, 2])
        * z_value
        / camera_matrix[0, 0]
    )
    y_value = (
        (grid_y[valid] - camera_matrix[1, 2])
        * z_value
        / camera_matrix[1, 1]
    )
    points = np.column_stack((x_value, y_value, z_value)).astype(np.float32)
    colors = color[grid_y[valid], grid_x[valid]][:, ::-1].copy()
    return points, colors


class RgbdOdometryWorker(threading.Thread):
    """Own the D415, run odometry, and incrementally save a colored map."""

    def __init__(
        self,
        camera: DepthCameraConfig,
        output: RowWriter,
        map_path: Path,
        stop_event: threading.Event,
        *,
        gyro_buffer: GyroPriorBuffer | None = None,
        state_sink: RgbdStateSink | None = None,
        obstacle_extractor: DepthObstacleExtractor | None = None,
        obstacle_sink: Callable[[ObstacleScan], None] | None = None,
        obstacle_rate_hz: float = 10.0,
        cloud_sink: Callable[
            [np.ndarray, np.ndarray, int, float, int], None
        ]
        | None = None,
    ) -> None:
        super().__init__(name="slam-poc-rgbd", daemon=True)
        self.camera = camera
        self.output = output
        self.map_path = map_path
        self.stop_event = stop_event
        self.gyro_buffer = gyro_buffer
        self.state_sink = state_sink
        if (obstacle_extractor is None) != (obstacle_sink is None):
            raise ValueError(
                "obstacle extractor and sink must be configured together"
            )
        if obstacle_rate_hz <= 0.0:
            raise ValueError("obstacle rate must be positive")
        self.obstacle_extractor = obstacle_extractor
        self.obstacle_sink = obstacle_sink
        self.cloud_sink = cloud_sink
        self.obstacle_period_s = 1.0 / obstacle_rate_hz
        self.error: str | None = None
        self.frames = 0
        self.tracked_frames = 0
        self.gyro_prior_frames = 0
        self.map_keyframes = 0
        self.map_points = 0
        self.path_length_m = 0.0
        self.obstacle_frames = 0
        self.obstacle_errors = 0
        self.spatial_frames = 0
        self.spatial_errors = 0
        self._voxel_map = VoxelMap(voxel_size_m=0.06, max_voxels=350_000)

    def run(self) -> None:
        pipeline = None
        try:
            cv2 = load_cv2()
            import pyrealsense2 as rs

            pipeline = rs.pipeline()
            stream_config = rs.config()
            if self.camera.serial:
                stream_config.enable_device(self.camera.serial)
            stream_config.enable_stream(
                rs.stream.depth,
                self.camera.width,
                self.camera.height,
                rs.format.z16,
                self.camera.fps,
            )
            stream_config.enable_stream(
                rs.stream.color,
                self.camera.width,
                self.camera.height,
                rs.format.bgr8,
                self.camera.fps,
            )
            profile = pipeline.start(stream_config)
            align = rs.align(rs.stream.color)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())
            for _ in range(20):
                if self.stop_event.is_set():
                    return
                align.process(pipeline.wait_for_frames(timeout_ms=2000))

            frames = align.process(pipeline.wait_for_frames(timeout_ms=2000))
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                raise RuntimeError("D415 did not produce aligned RGB-D frames")
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            scale_x = RGBD_WIDTH / float(intrinsics.width)
            scale_y = RGBD_HEIGHT / float(intrinsics.height)
            camera_matrix = np.array(
                (
                    (intrinsics.fx * scale_x, 0.0, intrinsics.ppx * scale_x),
                    (0.0, intrinsics.fy * scale_y, intrinsics.ppy * scale_y),
                    (0.0, 0.0, 1.0),
                ),
                dtype=np.float64,
            )
            engine = RgbdOdometryEngine(camera_matrix, cv2_module=cv2)
            body_from_camera = camera_to_body_transform(self.camera)
            camera_from_body = np.linalg.inv(body_from_camera)
            last_frame_ns: int | None = None
            last_body_transform: np.ndarray | None = None
            last_keyframe_transform: np.ndarray | None = None
            last_map_publish_s = float("-inf")
            last_obstacle_publish_s = float("-inf")
            frame_times: deque[float] = deque(maxlen=90)

            while not self.stop_event.is_set():
                frames = align.process(
                    pipeline.wait_for_frames(timeout_ms=2000)
                )
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                host_monotonic_ns = time.monotonic_ns()
                host_unix_ns = time.time_ns()
                color_full = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_full = depth_raw.astype(np.float32) * depth_scale
                now_s = time.monotonic()
                if (
                    self.obstacle_extractor is not None
                    and self.obstacle_sink is not None
                    and now_s - last_obstacle_publish_s
                    >= self.obstacle_period_s
                ):
                    last_obstacle_publish_s = now_s
                    try:
                        self.obstacle_sink(
                            self.obstacle_extractor.extract(
                                depth_raw,
                                depth_scale_m=depth_scale,
                                fx=intrinsics.fx,
                                fy=intrinsics.fy,
                                ppx=intrinsics.ppx,
                                ppy=intrinsics.ppy,
                                monotonic_ns=host_monotonic_ns,
                            )
                        )
                        self.obstacle_frames += 1
                    except (TypeError, ValueError):
                        self.obstacle_errors += 1
                color = cv2.resize(
                    color_full,
                    (RGBD_WIDTH, RGBD_HEIGHT),
                    interpolation=cv2.INTER_AREA,
                )
                depth = cv2.resize(
                    depth_full,
                    (RGBD_WIDTH, RGBD_HEIGHT),
                    interpolation=cv2.INTER_NEAREST,
                )
                gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
                prior = None
                if self.gyro_buffer is not None and last_frame_ns is not None:
                    prior = self.gyro_buffer.wait_rotation_prior(
                        last_frame_ns,
                        host_monotonic_ns,
                        body_from_camera[:3, :3],
                    )
                estimate = engine.process(
                    gray,
                    depth,
                    initial_transform=(
                        prior.transform_current_from_previous
                        if prior is not None
                        else None
                    ),
                    gyro_prior_samples=(prior.sample_count if prior else 0),
                )
                last_frame_ns = host_monotonic_ns
                self.frames += 1
                if estimate.tracking:
                    self.tracked_frames += 1
                if estimate.gyro_prior_used:
                    self.gyro_prior_frames += 1

                body_transform = (
                    body_from_camera
                    @ estimate.transform_world_camera
                    @ camera_from_body
                )
                local_transform = _local_flu_transform(body_transform)
                if estimate.tracking and last_body_transform is not None:
                    self.path_length_m += float(
                        np.linalg.norm(
                            body_transform[:3, 3]
                            - last_body_transform[:3, 3]
                        )
                    )
                if estimate.tracking or last_body_transform is None:
                    last_body_transform = body_transform.copy()

                frame_times.append(time.monotonic())
                measured_fps = (
                    0.0
                    if len(frame_times) < 2
                    else (len(frame_times) - 1)
                    / (frame_times[-1] - frame_times[0])
                )
                if self.cloud_sink is not None:
                    try:
                        cloud_camera, cloud_colors = backproject_depth(
                            depth,
                            color,
                            camera_matrix,
                            stride=4,
                        )
                        homogeneous = np.column_stack(
                            (
                                cloud_camera,
                                np.ones(len(cloud_camera), dtype=np.float32),
                            )
                        )
                        cloud_body_frd = (
                            body_from_camera @ homogeneous.T
                        ).T[:, :3]
                        self.cloud_sink(
                            cloud_body_frd.astype(np.float32, copy=False),
                            cloud_colors,
                            len(cloud_camera),
                            measured_fps,
                            host_monotonic_ns,
                        )
                        self.spatial_frames += 1
                    except (OSError, TypeError, ValueError):
                        self.spatial_errors += 1
                row = {
                    "schema_version": 1,
                    "host_monotonic_ns": host_monotonic_ns,
                    "host_unix_ns": host_unix_ns,
                    "camera_timestamp_ms": float(color_frame.get_timestamp()),
                    "initialized": estimate.initialized,
                    "tracking": estimate.tracking,
                    "frames": self.frames,
                    "tracked_frames": self.tracked_frames,
                    "tracking_success_ratio": (
                        self.tracked_frames / max(1, self.frames - 1)
                    ),
                    "measured_fps": measured_fps,
                    "compute_ms": estimate.compute_ms,
                    "valid_depth_fraction": estimate.valid_depth_fraction,
                    "step_translation_m": estimate.step_translation_m,
                    "step_rotation_deg": estimate.step_rotation_deg,
                    "gyro_prior_used": estimate.gyro_prior_used,
                    "gyro_prior_samples": estimate.gyro_prior_samples,
                    "gyro_prior_coverage_ratio": (
                        self.gyro_prior_frames / max(1, self.frames - 1)
                    ),
                    "position_body_frd_m": [
                        float(value) for value in body_transform[:3, 3]
                    ],
                    "position_local_flu_m": [
                        float(value) for value in local_transform[:3, 3]
                    ],
                    "quaternion_local_flu_xyzw": matrix_to_quaternion_xyzw(
                        local_transform[:3, :3]
                    ),
                    "path_length_m": self.path_length_m,
                    "map_keyframes": self.map_keyframes,
                    "map_points": len(self._voxel_map),
                }
                self.output.write(row)
                if self.state_sink is not None:
                    self.state_sink.update_rgbd(row)

                if self._is_keyframe(body_transform, last_keyframe_transform):
                    points_camera, colors_rgb = backproject_depth(
                        depth, color, camera_matrix, stride=3
                    )
                    homogeneous = np.column_stack(
                        (points_camera, np.ones(len(points_camera)))
                    )
                    points_body0_frd = (
                        body_from_camera
                        @ estimate.transform_world_camera
                        @ homogeneous.T
                    ).T[:, :3]
                    points_local_flu = points_body0_frd.copy()
                    points_local_flu[:, 1:] *= -1.0
                    self._voxel_map.add(points_local_flu, colors_rgb)
                    self.map_keyframes += 1
                    self.map_points = len(self._voxel_map)
                    last_keyframe_transform = body_transform.copy()

                now_s = time.monotonic()
                if now_s - last_map_publish_s >= 0.75:
                    self._publish_map(maximum_points=45_000)
                    last_map_publish_s = now_s
                if self.frames % 150 == 0:
                    self._voxel_map.write(self.map_path)
        except Exception as exc:
            self.error = f"{exc}\n{traceback.format_exc(limit=8)}"
            if self.state_sink is not None:
                self.state_sink.set_rgbd_error(str(exc))
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
            try:
                self._publish_map(maximum_points=45_000)
                self._voxel_map.write(self.map_path)
                self.map_points = len(self._voxel_map)
            except OSError as exc:
                if self.error is None:
                    self.error = f"unable to save RGB-D map: {exc}"

    @staticmethod
    def _is_keyframe(
        current: np.ndarray,
        previous: np.ndarray | None,
    ) -> bool:
        if previous is None:
            return True
        relative = np.linalg.inv(previous) @ current
        translation = float(np.linalg.norm(relative[:3, 3]))
        rotation_deg = math.degrees(rotation_angle_rad(relative[:3, :3]))
        return translation >= 0.04 or rotation_deg >= 3.0

    def _publish_map(self, *, maximum_points: int) -> None:
        if self.state_sink is None:
            return
        points, colors = self._voxel_map.cloud()
        if len(points) > maximum_points:
            selected = np.linspace(
                0, len(points) - 1, maximum_points, dtype=np.int32
            )
            points = points[selected]
            colors = colors[selected]
        self.state_sink.update_rgbd_map(points, colors)
