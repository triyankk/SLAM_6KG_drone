"""Bounded live point-cloud streams for the browser visualizer."""

from __future__ import annotations

import base64
from collections import deque
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import threading
import time
from typing import Any, Callable

import numpy as np

from .config import ProjectConfig, RotationConfig
from .paths import PROJECT_ROOT, RUNTIME_DIR
from .pointcloud import body_frd_to_local_up_rotation
from .obstacles import ObstacleScan, PointObstacleExtractor


SPATIAL_QUANTIZATION_M = 0.01
SPATIAL_STREAM_HZ = 8.0
DEFAULT_SPATIAL_FRAME_DIR = RUNTIME_DIR / "spatial_frames"
SPATIAL_FILE_POLL_S = 0.05
SPATIAL_FILE_STALE_S = 3.0
SPATIAL_SOURCES = ("depth_camera", "lidar")


def _rotation_matrix(rotation: RotationConfig) -> np.ndarray:
    roll = math.radians(rotation.roll_deg)
    pitch = math.radians(rotation.pitch_deg)
    yaw = math.radians(rotation.yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotate_x = np.array(
        ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    )
    rotate_y = np.array(
        ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    )
    rotate_z = np.array(
        ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    )
    return rotate_z @ rotate_y @ rotate_x


def camera_points_to_body_frd(
    camera_points_xyz_m: np.ndarray,
    config: ProjectConfig,
) -> np.ndarray:
    """Transform RealSense optical XYZ points into body FRD."""

    points = np.asarray(camera_points_xyz_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("camera points must have shape (N, 3)")
    camera = config.depth_camera
    forward_frd = points[:, (2, 0, 1)]
    rotation = _rotation_matrix(camera.rotation_from_forward_frd)
    position = camera.position_from_cg_frd_m
    translation = np.array((position.x, position.y, position.z))
    return (forward_frd @ rotation.T + translation).astype(
        np.float32, copy=False
    )


def lidar_points_to_body_frd(
    lidar_points_xyz_m: np.ndarray,
    config: ProjectConfig,
) -> np.ndarray:
    """Transform Hesai XYZ points into body FRD."""

    points = np.asarray(lidar_points_xyz_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("lidar points must have shape (N, 3)")
    lidar = config.lidar
    forward_frd = np.column_stack(
        (points[:, 1], points[:, 0], -points[:, 2])
    )
    rotation = _rotation_matrix(lidar.rotation_to_body_frd)
    position = lidar.position_from_cg_frd_m
    translation = np.array((position.x, position.y, position.z))
    return (forward_frd @ rotation.T + translation).astype(
        np.float32, copy=False
    )


def voxel_sample(
    points_m: np.ndarray,
    colors_rgb: np.ndarray,
    *,
    voxel_size_m: float,
    max_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Keep one deterministic point per voxel and enforce a hard frame cap."""

    points = np.asarray(points_m, dtype=np.float32)
    colors = np.asarray(colors_rgb, dtype=np.uint8)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if colors.shape != (len(points), 3):
        raise ValueError("colors must have shape (N, 3)")
    if voxel_size_m <= 0.0 or max_points <= 0:
        raise ValueError("voxel size and max points must be positive")

    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    colors = colors[finite]
    if not len(points):
        return points, colors

    keys = np.floor(points / voxel_size_m).astype(np.int32)
    _, selected = np.unique(keys, axis=0, return_index=True)
    selected.sort()
    if len(selected) > max_points:
        keep = np.linspace(
            0, len(selected) - 1, max_points, dtype=np.int32
        )
        selected = selected[keep]
    return points[selected], colors[selected]


def lidar_point_colors(
    points_body_frd_m: np.ndarray, intensity: np.ndarray
) -> np.ndarray:
    """Color a lidar frame by height and return strength."""
    points = np.asarray(points_body_frd_m, dtype=np.float32)
    energy = np.clip(
        np.asarray(intensity, dtype=np.float32) / 255.0, 0.0, 1.0
    )
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("lidar points must have shape (N, 3)")
    if energy.shape != (len(points),):
        raise ValueError("lidar intensity must have shape (N,)")
    height = np.clip((-points[:, 2] + 1.5) / 4.0, 0.0, 1.0)
    colors = np.empty((len(points), 3), dtype=np.uint8)
    colors[:, 0] = np.clip(
        35.0 + 125.0 * height + 45.0 * energy, 0, 255
    )
    colors[:, 1] = np.clip(135.0 + 95.0 * energy, 0, 255)
    colors[:, 2] = np.clip(
        235.0 - 75.0 * height + 20.0 * energy, 0, 255
    )
    return colors


def encode_spatial_frame(
    source: str,
    points_body_frd_m: np.ndarray,
    colors_rgb: np.ndarray,
    *,
    input_points: int,
    frame_rate_hz: float,
    frame_monotonic_ns: int | None = None,
    detail: str = "Live body-frame point cloud",
) -> dict[str, Any]:
    """Encode one bounded browser frame without assigning a queue sequence."""
    if source not in SPATIAL_SOURCES:
        raise ValueError(f"unsupported spatial source: {source}")
    points = np.asarray(points_body_frd_m, dtype=np.float32)
    colors = np.asarray(colors_rgb, dtype=np.uint8)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("spatial points must have shape (N, 3)")
    if colors.shape != (len(points), 3):
        raise ValueError("spatial colors must have shape (N, 3)")

    quantized = np.clip(
        np.rint(points / SPATIAL_QUANTIZATION_M),
        np.iinfo(np.int16).min,
        np.iinfo(np.int16).max,
    ).astype("<i2")
    event: dict[str, Any] = {
        "schema_version": 1,
        "kind": "frame",
        "source": source,
        "frame_id": "body_frd",
        "encoding": "int16_le_base64",
        "scale_m": SPATIAL_QUANTIZATION_M,
        "frame_monotonic_ns": (
            time.monotonic_ns()
            if frame_monotonic_ns is None
            else int(frame_monotonic_ns)
        ),
        "input_points": int(input_points),
        "point_count": len(points),
        "frame_rate_hz": round(float(frame_rate_hz), 3),
        "detail": detail,
        "points_b64": base64.b64encode(quantized.tobytes()).decode(
            "ascii"
        ),
        "colors_b64": base64.b64encode(colors.tobytes()).decode("ascii"),
    }
    if len(points):
        event["bounds_m"] = {
            "min": [round(float(value), 3) for value in points.min(axis=0)],
            "max": [round(float(value), 3) for value in points.max(axis=0)],
        }
    return event


class SpatialFrameStore:
    """Latest source status plus a short, loss-detectable frame queue."""

    def __init__(self, max_events: int = 12) -> None:
        self._condition = threading.Condition()
        self._events: deque[dict[str, Any]] = deque(maxlen=max_events)
        self._sequence = 0
        self._status: dict[str, dict[str, Any]] = {
            source: {
                "connected": False,
                "detail": "Waiting",
                "frame_rate_hz": 0.0,
                "input_points": 0,
                "display_points": 0,
                "updated_monotonic": None,
            }
            for source in SPATIAL_SOURCES
        }

    def latest_sequence(self) -> int:
        with self._condition:
            return self._sequence

    def publish_status(
        self,
        source: str,
        *,
        connected: bool,
        detail: str,
        **metrics: Any,
    ) -> None:
        now = time.monotonic()
        with self._condition:
            status = self._status.setdefault(source, {})
            status.update(
                connected=connected,
                detail=detail,
                updated_monotonic=now,
                **metrics,
            )
            self._sequence += 1
            self._events.append(
                {
                    "kind": "status",
                    "sequence": self._sequence,
                    "source": source,
                    **self._public_status(status, now),
                }
            )
            self._condition.notify_all()

    def publish_frame(
        self,
        source: str,
        points_body_frd_m: np.ndarray,
        colors_rgb: np.ndarray,
        *,
        input_points: int,
        frame_rate_hz: float,
        frame_monotonic_ns: int | None = None,
        detail: str = "Live body-frame point cloud",
    ) -> None:
        event = encode_spatial_frame(
            source,
            points_body_frd_m,
            colors_rgb,
            input_points=input_points,
            frame_rate_hz=frame_rate_hz,
            frame_monotonic_ns=frame_monotonic_ns,
            detail=detail,
        )
        self.publish_encoded_frame(event)

    def publish_encoded_frame(self, encoded: dict[str, Any]) -> None:
        """Queue a validated frame produced by the navigation runtime."""
        event = deepcopy(encoded)
        source = str(event.get("source", ""))
        if source not in SPATIAL_SOURCES or event.get("kind") != "frame":
            raise ValueError("invalid shared spatial frame")
        if (
            event.get("frame_id") != "body_frd"
            or event.get("encoding") != "int16_le_base64"
        ):
            raise ValueError("unsupported shared spatial encoding")
        point_count = int(event.get("point_count", -1))
        if not 0 <= point_count <= 20_000:
            raise ValueError("shared spatial point count is invalid")
        point_bytes = base64.b64decode(
            str(event.get("points_b64", "")), validate=True
        )
        color_bytes = base64.b64decode(
            str(event.get("colors_b64", "")), validate=True
        )
        if len(point_bytes) != point_count * 6:
            raise ValueError("shared spatial point payload is invalid")
        if len(color_bytes) != point_count * 3:
            raise ValueError("shared spatial color payload is invalid")

        now = time.monotonic()
        with self._condition:
            status = self._status.setdefault(source, {})
            status.update(
                connected=True,
                detail=str(
                    event.get("detail", "Live body-frame point cloud")
                ),
                frame_rate_hz=round(
                    float(event.get("frame_rate_hz", 0.0)), 3
                ),
                input_points=int(event.get("input_points", point_count)),
                display_points=point_count,
                updated_monotonic=now,
            )
            self._sequence += 1
            event["sequence"] = self._sequence
            self._events.append(event)
            self._condition.notify_all()

    @staticmethod
    def _public_status(
        status: dict[str, Any], now: float
    ) -> dict[str, Any]:
        public = {
            key: deepcopy(value)
            for key, value in status.items()
            if key != "updated_monotonic"
        }
        updated = status.get("updated_monotonic")
        public["age_ms"] = (
            None
            if updated is None
            else max(0, round((now - float(updated)) * 1000))
        )
        return public

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._condition:
            return {
                "kind": "snapshot",
                "sequence": self._sequence,
                "sources": {
                    source: self._public_status(status, now)
                    for source, status in self._status.items()
                },
            }

    def wait_after(
        self, sequence: int, timeout: float = 1.0
    ) -> tuple[list[dict[str, Any]], int]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence > sequence,
                timeout=timeout,
            )
            if not self._events:
                return [], 0
            oldest = int(self._events[0]["sequence"])
            dropped = max(0, oldest - sequence - 1)
            events = [
                event
                for event in self._events
                if int(event["sequence"]) > sequence
            ]
            return events, dropped


class SpatialFrameFilePublisher:
    """Atomically share service-owned point clouds with a monitor process."""

    def __init__(self, directory: Path = DEFAULT_SPATIAL_FRAME_DIR) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            for source in SPATIAL_SOURCES:
                try:
                    (self.directory / f"{source}.json").unlink()
                except FileNotFoundError:
                    pass

    def publish_frame(
        self,
        source: str,
        points_body_frd_m: np.ndarray,
        colors_rgb: np.ndarray,
        *,
        input_points: int,
        frame_rate_hz: float,
        frame_monotonic_ns: int,
        detail: str,
    ) -> None:
        event = encode_spatial_frame(
            source,
            points_body_frd_m,
            colors_rgb,
            input_points=input_points,
            frame_rate_hz=frame_rate_hz,
            frame_monotonic_ns=frame_monotonic_ns,
            detail=detail,
        )
        payload = json.dumps(event, separators=(",", ":")) + "\n"
        target = self.directory / f"{source}.json"
        temporary = self.directory / f".{source}.tmp"
        with self._lock:
            temporary.write_text(payload, encoding="ascii")
            temporary.replace(target)


class SpatialFrameFileSource(threading.Thread):
    """Mirror atomic runtime cloud frames into the visualizer SSE store."""

    def __init__(
        self,
        store: SpatialFrameStore,
        stop_event: threading.Event,
        directory: Path = DEFAULT_SPATIAL_FRAME_DIR,
        *,
        stale_timeout_s: float = SPATIAL_FILE_STALE_S,
    ) -> None:
        super().__init__(name="runtime-spatial-source", daemon=True)
        self.store = store
        self.stop_event = stop_event
        self.directory = Path(directory)
        self.stale_timeout_s = float(stale_timeout_s)
        self._last_frame_ns = {source: -1 for source in SPATIAL_SOURCES}
        self._last_detail: dict[str, str] = {}

    def _publish_status_once(self, source: str, detail: str) -> None:
        if self._last_detail.get(source) == detail:
            return
        self._last_detail[source] = detail
        self.store.publish_status(
            source,
            connected=False,
            detail=detail,
        )

    def run(self) -> None:
        while not self.stop_event.is_set():
            now_wall_s = time.time()
            for source in SPATIAL_SOURCES:
                path = self.directory / f"{source}.json"
                try:
                    stat = path.stat()
                    age_s = max(0.0, now_wall_s - stat.st_mtime)
                    if age_s > self.stale_timeout_s:
                        self._publish_status_once(
                            source,
                            f"SLAM runtime {source} frame is stale",
                        )
                        continue
                    payload = json.loads(path.read_text(encoding="ascii"))
                    frame_ns = int(payload.get("frame_monotonic_ns", -1))
                    if frame_ns <= self._last_frame_ns[source]:
                        continue
                    self.store.publish_encoded_frame(payload)
                    self._last_frame_ns[source] = frame_ns
                    self._last_detail.pop(source, None)
                except FileNotFoundError:
                    self._publish_status_once(
                        source,
                        f"Waiting for SLAM runtime {source} frames",
                    )
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    self._publish_status_once(
                        source,
                        f"Invalid SLAM runtime {source} frame: {exc}",
                    )
            self.stop_event.wait(SPATIAL_FILE_POLL_S)


class RealSenseSpatialSource(threading.Thread):
    """Publish sampled D415 RGB-D points in body FRD."""

    def __init__(
        self,
        store: SpatialFrameStore,
        stop_event: threading.Event,
        config: ProjectConfig,
        *,
        target_rate_hz: float = SPATIAL_STREAM_HZ,
        max_points: int = 6_000,
        obstacle_sink: Callable[[ObstacleScan], None] | None = None,
    ) -> None:
        super().__init__(name="visualizer-d415-cloud", daemon=True)
        self.store = store
        self.stop_event = stop_event
        self.config = config
        self.camera = config.depth_camera
        self.period_s = 1.0 / target_rate_hz
        self.max_points = max_points
        self.obstacle_sink = obstacle_sink
        self.obstacle_extractor = (
            PointObstacleExtractor(
                config.obstacle_avoidance,
                source="depth_camera",
            )
            if obstacle_sink is not None
            and config.obstacle_avoidance.depth_camera_enabled
            else None
        )

    def run(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            self.store.publish_status(
                "depth_camera",
                connected=False,
                detail=f"pyrealsense2 unavailable: {exc}",
            )
            return

        while not self.stop_event.is_set():
            pipeline = None
            try:
                self.store.publish_status(
                    "depth_camera",
                    connected=False,
                    detail="Opening D415 RGB-D streams",
                )
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
                    rs.format.rgb8,
                    self.camera.fps,
                )
                profile = pipeline.start(stream_config)
                align = rs.align(rs.stream.color)
                depth_scale_m = float(
                    profile.get_device()
                    .first_depth_sensor()
                    .get_depth_scale()
                )
                started = time.monotonic()
                last_frame = -math.inf
                published = 0
                while not self.stop_event.is_set():
                    frames = pipeline.wait_for_frames(timeout_ms=2000)
                    now = time.monotonic()
                    if now - last_frame < self.period_s:
                        continue
                    aligned = align.process(frames)
                    depth_frame = aligned.get_depth_frame()
                    color_frame = aligned.get_color_frame()
                    if not depth_frame or not color_frame:
                        continue
                    depth = np.asanyarray(depth_frame.get_data())
                    color = np.asanyarray(color_frame.get_data())
                    intrinsics = (
                        depth_frame.profile.as_video_stream_profile().intrinsics
                    )
                    stride = 6
                    rows = np.arange(
                        0, depth.shape[0], stride, dtype=np.int32
                    )
                    columns = np.arange(
                        0, depth.shape[1], stride, dtype=np.int32
                    )
                    uu, vv = np.meshgrid(columns, rows)
                    depth_m = (
                        depth[vv, uu].astype(np.float32) * depth_scale_m
                    )
                    valid = (
                        np.isfinite(depth_m)
                        & (depth_m >= 0.20)
                        & (depth_m <= 8.0)
                    )
                    forward = depth_m[valid]
                    right = (
                        (uu[valid].astype(np.float32) - intrinsics.ppx)
                        / intrinsics.fx
                        * forward
                    )
                    down = (
                        (vv[valid].astype(np.float32) - intrinsics.ppy)
                        / intrinsics.fy
                        * forward
                    )
                    camera_points = np.column_stack(
                        (right, down, forward)
                    )
                    body_points = camera_points_to_body_frd(
                        camera_points, self.config
                    )
                    colors = color[vv[valid], uu[valid], :3]
                    input_points = len(body_points)
                    frame_monotonic_ns = time.monotonic_ns()
                    if (
                        self.obstacle_extractor is not None
                        and self.obstacle_sink is not None
                    ):
                        self.obstacle_sink(
                            self.obstacle_extractor.extract(
                                body_points,
                                monotonic_ns=frame_monotonic_ns,
                            )
                        )
                    body_points, colors = voxel_sample(
                        body_points,
                        colors,
                        voxel_size_m=0.04,
                        max_points=self.max_points,
                    )
                    published += 1
                    rate_hz = published / max(0.001, now - started)
                    self.store.publish_frame(
                        "depth_camera",
                        body_points,
                        colors,
                        input_points=input_points,
                        frame_rate_hz=rate_hz,
                        frame_monotonic_ns=frame_monotonic_ns,
                        detail="D415 aligned RGB-D point cloud",
                    )
                    last_frame = now
            except (OSError, RuntimeError, TypeError, ValueError) as exc:
                self.store.publish_status(
                    "depth_camera",
                    connected=False,
                    detail=str(exc),
                )
                self.stop_event.wait(2.0)
            finally:
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except RuntimeError:
                        pass


class HesaiSpatialSource(threading.Thread):
    """Publish official-SDK JT16 frames in body FRD."""

    FRAME_HEADER = struct.Struct("<8sIIQQ")
    FRAME_MAGIC = b"OFJT16P1"
    FRAME_VERSION = 2
    MAXIMUM_POINTS = 1_000_000
    POINT_DTYPE = np.dtype(
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

    def __init__(
        self,
        store: SpatialFrameStore,
        stop_event: threading.Event,
        config: ProjectConfig,
        *,
        max_points: int = 8_000,
        obstacle_sink: Callable[[ObstacleScan], None] | None = None,
    ) -> None:
        super().__init__(name="visualizer-jt16-cloud", daemon=True)
        self.store = store
        self.stop_event = stop_event
        self.config = config
        self.lidar = config.lidar
        self.max_points = max_points
        self.obstacle_sink = obstacle_sink
        self.obstacle_extractor = (
            PointObstacleExtractor(
                config.obstacle_avoidance,
                source="lidar",
            )
            if obstacle_sink is not None
            and config.obstacle_avoidance.lidar_enabled
            else None
        )

    def _project_path(self, value: str) -> Path:
        path = Path(value)
        return path if path.is_absolute() else PROJECT_ROOT / path

    def _read_exact(
        self, process: subprocess.Popen, size: int
    ) -> bytes | None:
        if process.stdout is None:
            return None
        descriptor = process.stdout.fileno()
        collected = bytearray()
        while len(collected) < size:
            if self.stop_event.is_set() or process.poll() is not None:
                return None
            ready, _, _ = select.select((descriptor,), (), (), 0.2)
            if not ready:
                continue
            chunk = os.read(descriptor, size - len(collected))
            if not chunk:
                return None
            collected.extend(chunk)
        return bytes(collected)

    def run(self) -> None:
        while not self.stop_event.is_set():
            process: subprocess.Popen | None = None
            try:
                bridge = self._project_path(self.lidar.bridge_binary)
                correction = self._project_path(
                    self.lidar.correction_file
                )
                if not bridge.is_file() or not os.access(bridge, os.X_OK):
                    raise OSError(
                        "JT16 bridge missing; run ./optflow build-jt16"
                    )
                if not correction.is_file():
                    raise OSError(
                        f"JT16 correction file missing: {correction}"
                    )
                if not Path(self.lidar.symlink).exists():
                    raise OSError(
                        f"JT16 serial device missing: {self.lidar.symlink}"
                    )
                self.store.publish_status(
                    "lidar",
                    connected=False,
                    detail="Starting official Hesai SDK bridge",
                )
                process = subprocess.Popen(
                    [
                        str(bridge),
                        "--device",
                        self.lidar.symlink,
                        "--baud",
                        str(self.lidar.baud),
                        "--correction",
                        str(correction),
                        "--startup-timeout",
                        "5",
                    ],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=0,
                )
                started = time.monotonic()
                frames = 0
                while not self.stop_event.is_set():
                    header_bytes = self._read_exact(
                        process, self.FRAME_HEADER.size
                    )
                    if header_bytes is None:
                        break
                    (
                        magic,
                        version,
                        points_in_frame,
                        frame_monotonic_ns,
                        _frame_index,
                    ) = self.FRAME_HEADER.unpack(header_bytes)
                    if (
                        magic != self.FRAME_MAGIC
                        or version != self.FRAME_VERSION
                    ):
                        raise ValueError("JT16 bridge frame is invalid")
                    if not 0 < points_in_frame <= self.MAXIMUM_POINTS:
                        raise ValueError(
                            "JT16 point count is outside limits"
                        )
                    payload = self._read_exact(
                        process,
                        points_in_frame * self.POINT_DTYPE.itemsize,
                    )
                    if payload is None:
                        break
                    records = np.frombuffer(payload, dtype=self.POINT_DTYPE)
                    lidar_points = np.column_stack(
                        (records["x"], records["y"], records["z"])
                    )
                    body_points = lidar_points_to_body_frd(
                        lidar_points, self.config
                    )
                    distance = np.linalg.norm(body_points, axis=1)
                    valid = (
                        np.isfinite(body_points).all(axis=1)
                        & (distance >= 0.25)
                        & (distance <= 20.0)
                    )
                    body_points = body_points[valid]
                    intensity = records["intensity"][valid]
                    if (
                        self.obstacle_extractor is not None
                        and self.obstacle_sink is not None
                    ):
                        self.obstacle_sink(
                            self.obstacle_extractor.extract(
                                body_points,
                                monotonic_ns=frame_monotonic_ns,
                            )
                        )
                    colors = lidar_point_colors(body_points, intensity)
                    input_points = len(body_points)
                    body_points, colors = voxel_sample(
                        body_points,
                        colors,
                        voxel_size_m=0.06,
                        max_points=self.max_points,
                    )
                    frames += 1
                    rate_hz = frames / max(
                        0.001, time.monotonic() - started
                    )
                    self.store.publish_frame(
                        "lidar",
                        body_points,
                        colors,
                        input_points=input_points,
                        frame_rate_hz=rate_hz,
                        frame_monotonic_ns=frame_monotonic_ns,
                        detail="JT16 official-SDK 3D point cloud",
                    )
                if (
                    process.poll() is not None
                    and not self.stop_event.is_set()
                ):
                    raise RuntimeError(
                        f"JT16 bridge exited with {process.returncode}"
                    )
            except (OSError, RuntimeError, ValueError) as exc:
                self.store.publish_status(
                    "lidar",
                    connected=False,
                    detail=str(exc),
                )
                self.stop_event.wait(2.0)
            finally:
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.0)
                if process is not None and process.stdout is not None:
                    process.stdout.close()


class DemoSpatialSource(threading.Thread):
    """Generate a fixed room observed from the animated demo aircraft."""

    def __init__(
        self,
        store: SpatialFrameStore,
        telemetry_store: Any,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name="demo-spatial-cloud", daemon=True)
        self.store = store
        self.telemetry_store = telemetry_store
        self.stop_event = stop_event
        self.world_points, self.world_colors = self._build_room()

    @staticmethod
    def _build_room() -> tuple[np.ndarray, np.ndarray]:
        rng = np.random.default_rng(42)
        count = 14_000
        surfaces = rng.integers(0, 5, size=count)
        points = np.empty((count, 3), dtype=np.float32)
        points[:, 0] = rng.uniform(-5.0, 5.0, count)
        points[:, 1] = rng.uniform(-4.0, 4.0, count)
        points[:, 2] = rng.uniform(0.0, 3.2, count)
        points[surfaces == 0, 2] = 0.0
        points[surfaces == 1, 0] = -5.0
        points[surfaces == 2, 0] = 5.0
        points[surfaces == 3, 1] = -4.0
        points[surfaces == 4, 1] = 4.0

        pillar_count = 2_000
        angle = rng.uniform(0.0, 2.0 * math.pi, pillar_count)
        height = rng.uniform(0.0, 2.6, pillar_count)
        pillar = np.column_stack(
            (
                1.8 + 0.42 * np.cos(angle),
                -0.6 + 0.42 * np.sin(angle),
                height,
            )
        ).astype(np.float32)
        points = np.vstack((points, pillar))

        colors = np.empty((len(points), 3), dtype=np.uint8)
        colors[:, 0] = np.clip(75 + points[:, 2] * 42, 0, 255)
        colors[:, 1] = np.clip(150 + points[:, 0] * 9, 0, 255)
        colors[:, 2] = np.clip(190 + points[:, 1] * 7, 0, 255)
        colors[-pillar_count:] = np.array((235, 172, 76), dtype=np.uint8)
        return points, colors

    def run(self) -> None:
        started = time.monotonic()
        frames = 0
        while not self.stop_event.wait(0.16):
            telemetry = self.telemetry_store.snapshot()
            local = telemetry["local_position"]
            attitude = telemetry["attitude"]
            position = np.array(
                (
                    float(local["x_m"]),
                    float(local["y_m"]),
                    -float(local["z_down_m"]),
                )
            )
            rotation = body_frd_to_local_up_rotation(
                float(attitude["roll_rad"]),
                float(attitude["pitch_rad"]),
                float(attitude["yaw_rad"]),
            )
            body_points = (self.world_points - position) @ rotation
            distance = np.linalg.norm(body_points, axis=1)
            lidar_valid = distance <= 12.0
            lidar_points, lidar_colors = voxel_sample(
                body_points[lidar_valid],
                self.world_colors[lidar_valid],
                voxel_size_m=0.08,
                max_points=8_000,
            )

            forward = body_points[:, 0]
            camera_valid = (
                (forward > 0.25)
                & (distance <= 8.0)
                & (np.abs(body_points[:, 1]) < forward * 0.72)
                & (np.abs(body_points[:, 2]) < forward * 0.48)
            )
            camera_points, camera_colors = voxel_sample(
                body_points[camera_valid],
                self.world_colors[camera_valid],
                voxel_size_m=0.05,
                max_points=6_000,
            )
            frames += 1
            rate_hz = frames / max(0.001, time.monotonic() - started)
            frame_time = time.monotonic_ns()
            self.store.publish_frame(
                "lidar",
                lidar_points,
                lidar_colors,
                input_points=int(lidar_valid.sum()),
                frame_rate_hz=rate_hz,
                frame_monotonic_ns=frame_time,
                detail="Demo 360-degree lidar cloud",
            )
            self.store.publish_frame(
                "depth_camera",
                camera_points,
                camera_colors,
                input_points=int(camera_valid.sum()),
                frame_rate_hz=rate_hz,
                frame_monotonic_ns=frame_time,
                detail="Demo forward RGB-D cloud",
            )
