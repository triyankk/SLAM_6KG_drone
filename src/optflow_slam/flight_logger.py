"""Record a passive, synchronized flight dataset for SLAM development."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import select
import shutil
import signal
import socket
import struct
import subprocess
import threading
import time
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np

from .config import ConfigError, ProjectConfig, load_config
from .obstacles import (
    DepthObstacleExtractor,
    LidarObstacleExtractor,
    ObstacleScan,
)
from .paths import CONFIG_DIR, PROJECT_ROOT, RECORDING_DIR
from .pointcloud import (
    MapPose,
    VoxelMap,
    camera_optical_to_local,
    write_binary_ply,
)


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"
DEFAULT_FLIGHT_ROOT = RECORDING_DIR / "flights"
SCHEMA_VERSION = 1
HOLD_MODES = frozenset(("POSHOLD", "LOITER", "FLOWHOLD", "BRAKE"))
POSITION_TARGET_X_IGNORE = 1 << 0
POSITION_TARGET_Y_IGNORE = 1 << 1
POSITION_TARGET_VX_IGNORE = 1 << 3
POSITION_TARGET_VY_IGNORE = 1 << 4
TIMING_MAVLINK_TYPES = frozenset(
    ("ATTITUDE", "HIGHRES_IMU", "SCALED_IMU", "SYSTEM_TIME")
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def _wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _safe_name(value: str | None) -> str:
    if not value:
        return "flight"
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "-", value).strip(".-")
    return cleaned[:48] or "flight"


class NdjsonWriter:
    """Thread-safe, periodically flushed newline-delimited JSON writer."""

    def __init__(self, path: Path, flush_interval_s: float = 1.0) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._output = path.open("w", encoding="utf-8")
        self._lock = threading.Lock()
        self._last_flush = time.monotonic()
        self._flush_interval_s = flush_interval_s
        self.rows = 0

    def write(self, payload: dict[str, Any]) -> None:
        encoded = json.dumps(
            _json_safe(payload), separators=(",", ":"), sort_keys=False
        )
        with self._lock:
            self._output.write(encoded)
            self._output.write("\n")
            self.rows += 1
            now = time.monotonic()
            if now - self._last_flush >= self._flush_interval_s:
                self._durable_flush()
                self._last_flush = now

    def _durable_flush(self) -> None:
        self._output.flush()
        os.fsync(self._output.fileno())

    def close(self) -> None:
        with self._lock:
            if self._output.closed:
                return
            self._durable_flush()
            self._output.close()


class IdealHoldShadow:
    """Non-commanding perfect-pose reference for a local target or hold."""

    def __init__(
        self,
        *,
        min_flow_quality: int,
        position_gain: float = 0.8,
        velocity_gain: float = 1.2,
        max_tilt_deg: float = 10.0,
    ) -> None:
        self.min_flow_quality = min_flow_quality
        self.position_gain = position_gain
        self.velocity_gain = velocity_gain
        self.max_tilt_rad = math.radians(max_tilt_deg)
        self._last_time_s: float | None = None
        self._origin_yaw_rad: float | None = None
        self._cube_origin: tuple[float, float, float] | None = None
        self._flow_position_m = np.zeros(2, dtype=np.float64)
        self._latest_pose: MapPose | None = None

    @property
    def latest_pose(self) -> MapPose | None:
        return self._latest_pose

    def update(
        self, snapshot: dict[str, Any], host_monotonic_ns: int
    ) -> dict[str, Any]:
        now_s = host_monotonic_ns / 1.0e9
        dt_s = (
            None
            if self._last_time_s is None
            else now_s - self._last_time_s
        )
        self._last_time_s = now_s

        attitude = snapshot.get("attitude", {})
        roll_rad = float(attitude.get("roll_rad") or 0.0)
        pitch_rad = float(attitude.get("pitch_rad") or 0.0)
        yaw_rad = float(attitude.get("yaw_rad") or 0.0)
        if self._origin_yaw_rad is None:
            self._origin_yaw_rad = yaw_rad
        yaw_relative_rad = _wrap_pi(yaw_rad - self._origin_yaw_rad)

        flow = snapshot.get("flow", {})
        flow_quality = int(flow.get("quality") or 0)
        flow_age_ms = flow.get("age_ms")
        flow_fresh = (
            flow_age_ms is not None
            and float(flow_age_ms) <= 250.0
            and flow_quality >= self.min_flow_quality
        )
        velocity_body = np.array(
            (
                float(flow.get("comp_x_mps") or 0.0),
                float(flow.get("comp_y_mps") or 0.0),
            ),
            dtype=np.float64,
        )
        cosine = math.cos(yaw_relative_rad)
        sine = math.sin(yaw_relative_rad)
        body_to_map_xy = np.array(
            ((cosine, -sine), (sine, cosine)), dtype=np.float64
        )
        flow_velocity_map = body_to_map_xy @ velocity_body
        if (
            flow_fresh
            and dt_s is not None
            and 0.0 < dt_s <= 0.25
        ):
            self._flow_position_m += flow_velocity_map * dt_s

        local = snapshot.get("local_position", {})
        local_age_ms = local.get("age_ms")
        local_fresh = (
            local_age_ms is not None and float(local_age_ms) <= 250.0
        )
        if local_fresh:
            local_values = (
                float(local.get("x_m") or 0.0),
                float(local.get("y_m") or 0.0),
                float(local.get("z_down_m") or 0.0),
            )
            if self._cube_origin is None:
                self._cube_origin = local_values
            position_xy = np.array(
                (
                    local_values[0] - self._cube_origin[0],
                    local_values[1] - self._cube_origin[1],
                )
            )
            velocity_map = np.array(
                (
                    float(local.get("vx_mps") or 0.0),
                    float(local.get("vy_mps") or 0.0),
                )
            )
            relative_z_up_m = -(local_values[2] - self._cube_origin[2])
            pose_source = "cube_local_position"
        else:
            position_xy = self._flow_position_m.copy()
            velocity_map = flow_velocity_map
            relative_z_up_m = 0.0
            pose_source = "hflow_dead_reckoning"

        distance = snapshot.get("range", {})
        range_age_ms = distance.get("age_ms")
        range_fresh = (
            range_age_ms is not None
            and float(range_age_ms) <= 250.0
            and float(distance.get("distance_m") or 0.0) > 0.0
        )
        map_z_m = (
            float(distance["distance_m"])
            if range_fresh
            else relative_z_up_m
        )
        self._latest_pose = MapPose(
            x_m=float(position_xy[0]),
            y_m=float(position_xy[1]),
            z_m=map_z_m,
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            yaw_rad=yaw_relative_rad,
            source=pose_source,
        )

        desired_position_xy = np.zeros(2, dtype=np.float64)
        desired_velocity_xy = np.zeros(2, dtype=np.float64)
        target = snapshot.get("position_target", {})
        target_age_ms = target.get("age_ms")
        target_mask = int(target.get("type_mask") or 0)
        target_fresh = (
            local_fresh
            and target_age_ms is not None
            and float(target_age_ms) <= 250.0
        )
        position_target_valid = (
            target_fresh
            and (target_mask & POSITION_TARGET_X_IGNORE) == 0
            and (target_mask & POSITION_TARGET_Y_IGNORE) == 0
            and target.get("x_m") is not None
            and target.get("y_m") is not None
            and self._cube_origin is not None
        )
        velocity_target_valid = (
            target_fresh
            and (target_mask & POSITION_TARGET_VX_IGNORE) == 0
            and (target_mask & POSITION_TARGET_VY_IGNORE) == 0
            and target.get("vx_mps") is not None
            and target.get("vy_mps") is not None
        )
        if position_target_valid:
            desired_position_xy = np.array(
                (
                    float(target["x_m"]) - self._cube_origin[0],
                    float(target["y_m"]) - self._cube_origin[1],
                )
            )
        if velocity_target_valid:
            desired_velocity_xy = np.array(
                (
                    float(target["vx_mps"]),
                    float(target["vy_mps"]),
                )
            )
        reference_source = (
            "cube_position_target"
            if position_target_valid or velocity_target_valid
            else "session_origin_stationary"
        )

        acceleration_map = (
            self.position_gain * (desired_position_xy - position_xy)
            + self.velocity_gain * (desired_velocity_xy - velocity_map)
        )
        map_to_body_xy = body_to_map_xy.T
        acceleration_body = map_to_body_xy @ acceleration_map
        predicted_pitch_rad = _clamp(
            -math.atan2(float(acceleration_body[0]), 9.80665),
            -self.max_tilt_rad,
            self.max_tilt_rad,
        )
        predicted_roll_rad = _clamp(
            math.atan2(float(acceleration_body[1]), 9.80665),
            -self.max_tilt_rad,
            self.max_tilt_rad,
        )

        vehicle = snapshot.get("vehicle", {})
        mode = str(vehicle.get("mode") or "UNKNOWN").upper()
        armed = bool(vehicle.get("armed"))
        supported_mode = mode in HOLD_MODES or (
            mode == "GUIDED"
            and (position_target_valid or velocity_target_valid)
        )
        prediction_applicable = armed and supported_mode
        if not armed:
            applicability = "vehicle_disarmed"
        elif not supported_mode:
            applicability = "mode_or_pilot_target_not_stationary_hold"
        elif not (flow_fresh or local_fresh):
            applicability = "horizontal_pose_stale"
            prediction_applicable = False
        else:
            applicability = (
                "cube_target_shadow_only"
                if reference_source == "cube_position_target"
                else "stationary_hold_shadow_only"
            )

        imu = snapshot.get("imu", {})
        external = snapshot.get("ros_imu", {}).get("body_preview", {})
        cube_gyro = np.array(
            (
                float(imu.get("gyro_x_rads") or 0.0),
                float(imu.get("gyro_y_rads") or 0.0),
                float(imu.get("gyro_z_rads") or 0.0),
            )
        )
        external_gyro = np.array(
            (
                float(external.get("gyro_x_rads") or 0.0),
                float(external.get("gyro_y_rads") or 0.0),
                float(external.get("gyro_z_rads") or 0.0),
            )
        )
        cube_accel = np.array(
            (
                float(imu.get("accel_x_mss") or 0.0),
                float(imu.get("accel_y_mss") or 0.0),
                float(imu.get("accel_z_mss") or 0.0),
            )
        )
        external_accel = np.array(
            (
                float(external.get("accel_x_mss") or 0.0),
                float(external.get("accel_y_mss") or 0.0),
                float(external.get("accel_z_mss") or 0.0),
            )
        )

        return {
            "schema_version": SCHEMA_VERSION,
            "host_monotonic_ns": host_monotonic_ns,
            "cube_time_boot_ms": attitude.get("time_boot_ms"),
            "vehicle": {
                "armed": armed,
                "mode": mode,
            },
            "pose_observation": {
                "source": pose_source,
                "x_m": self._latest_pose.x_m,
                "y_m": self._latest_pose.y_m,
                "z_up_m": self._latest_pose.z_m,
                "roll_rad": roll_rad,
                "pitch_rad": pitch_rad,
                "yaw_relative_rad": yaw_relative_rad,
                "flow_quality": flow_quality,
                "flow_fresh": flow_fresh,
                "local_position_fresh": local_fresh,
                "range_fresh": range_fresh,
            },
            "perfect_slam_stationary_hold": {
                "assumption": (
                    "observed pose is exact; desired local XY comes from a "
                    "fresh Cube target or otherwise the session origin"
                ),
                "prediction_applicable": prediction_applicable,
                "applicability": applicability,
                "reference_source": reference_source,
                "desired_x_m": float(desired_position_xy[0]),
                "desired_y_m": float(desired_position_xy[1]),
                "desired_vx_mps": float(desired_velocity_xy[0]),
                "desired_vy_mps": float(desired_velocity_xy[1]),
                "position_gain_s2": self.position_gain,
                "velocity_gain_s": self.velocity_gain,
                "max_tilt_deg": math.degrees(self.max_tilt_rad),
                "accel_body_x_mss": float(acceleration_body[0]),
                "accel_body_y_mss": float(acceleration_body[1]),
                "predicted_roll_rad": predicted_roll_rad,
                "predicted_pitch_rad": predicted_pitch_rad,
                "actual_roll_rad": roll_rad,
                "actual_pitch_rad": pitch_rad,
                "roll_residual_rad": _wrap_pi(
                    roll_rad - predicted_roll_rad
                ),
                "pitch_residual_rad": _wrap_pi(
                    pitch_rad - predicted_pitch_rad
                ),
                "not_for_flight_control": True,
            },
            "sensor_crosscheck": {
                "cube_minus_external_gyro_rads": (
                    cube_gyro - external_gyro
                ).tolist(),
                "cube_minus_external_accel_mss": (
                    cube_accel - external_accel
                ).tolist(),
                "external_imu_age_ms": snapshot.get("ros_imu", {}).get(
                    "age_ms"
                ),
                "cube_imu_age_ms": imu.get("age_ms"),
            },
        }


class FlightSession:
    """Own all files and mutable state for one recording session."""

    def __init__(
        self,
        root: Path,
        name: str | None,
        config: ProjectConfig,
        config_path: Path,
        telemetry_url: str,
        raw_events_url: str,
    ) -> None:
        started = datetime.now(timezone.utc)
        stamp = started.strftime("%Y%m%dT%H%M%SZ")
        stem = f"{stamp}_{_safe_name(name)}"
        session_dir = root / stem
        suffix = 1
        while session_dir.exists():
            suffix += 1
            session_dir = root / f"{stem}_{suffix}"
        session_dir.mkdir(parents=True)

        self.path = session_dir
        self.pointcloud_dir = session_dir / "pointcloud"
        self.pointcloud_frames_dir = self.pointcloud_dir / "frames"
        self.realsense_dir = session_dir / "realsense"
        self.lidar_dir = session_dir / "lidar"
        self.analysis_dir = session_dir / "analysis"
        self.cube_dir = session_dir / "cube"
        for directory in (
            self.pointcloud_frames_dir,
            self.realsense_dir,
            self.lidar_dir,
            self.analysis_dir,
            self.cube_dir,
        ):
            directory.mkdir(parents=True)

        self.telemetry = NdjsonWriter(session_dir / "telemetry.ndjson")
        self.sensor_events = NdjsonWriter(
            session_dir / "sensor_events.ndjson"
        )
        self.sensor_timing = NdjsonWriter(
            session_dir / "sensor_timing.ndjson"
        )
        self.shadow = NdjsonWriter(session_dir / "shadow_predictions.ndjson")
        self.events = NdjsonWriter(
            session_dir / "events.ndjson", flush_interval_s=0.0
        )
        self._lock = threading.Lock()
        self._closed = False
        self._latest_pose: MapPose | None = None
        self._latest_snapshot: dict[str, Any] | None = None
        self._source_stats: dict[str, Any] = {}
        self._shadow_model = IdealHoldShadow(
            min_flow_quality=max(
                1, config.flight_controller.hflow_min_bench_quality
            )
        )
        self._manifest: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "status": "recording",
            "session_id": session_dir.name,
            "started_utc": started.isoformat(timespec="milliseconds"),
            "ended_utc": None,
            "passive_only": True,
            "flight_commands_sent": False,
            "telemetry_url": telemetry_url,
            "raw_events_url": raw_events_url,
            "config_path": str(config_path.resolve()),
            "config_sha256": hashlib.sha256(
                config_path.read_bytes()
            ).hexdigest(),
            "hardware": asdict(config),
            "calibration_warnings": {
                "camera_intrinsics_verified": (
                    config.calibration.camera_intrinsics_verified
                ),
                "camera_to_body_extrinsics_verified": (
                    config.calibration.camera_to_body_extrinsics_verified
                ),
                "imu_to_body_extrinsics_verified": (
                    config.calibration.imu_to_body_extrinsics_verified
                ),
                "lidar_to_body_extrinsics_verified": (
                    config.calibration.lidar_to_body_extrinsics_verified
                ),
                "sensor_time_sync_verified": (
                    config.calibration.sensor_time_sync_verified
                ),
            },
            "shadow_model": {
                "name": "perfect_pose_local_target_reference",
                "purpose": "offline comparison only",
                "flight_control_eligible": False,
            },
            "pointcloud": {
                "frame": "local_x_y_z_up",
                "camera_mount": (
                    "assumed forward optical to body FRD; measured "
                    "extrinsics still required"
                ),
                "pose": (
                    "Cube LOCAL_POSITION_NED when fresh, otherwise HFlow "
                    "dead reckoning"
                ),
                "slam_optimized": False,
            },
            "files": {
                "telemetry": "telemetry.ndjson",
                "sensor_events": "sensor_events.ndjson",
                "sensor_timing": "sensor_timing.ndjson",
                "shadow_predictions": "shadow_predictions.ndjson",
                "events": "events.ndjson",
                "environment_cloud": "pointcloud/flight_environment.ply",
                "lidar_packets": "lidar/jt16_serial.bin",
            },
            "source_stats": self._source_stats,
        }
        self._write_manifest()
        self.event("session", "recording_started", {"path": str(session_dir)})

    def _write_manifest(self) -> None:
        temporary = self.path / "manifest.json.tmp"
        temporary.write_text(
            json.dumps(
                _json_safe(self._manifest),
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path / "manifest.json")

    def event(
        self, source: str, event: str, detail: dict[str, Any] | str
    ) -> None:
        self.events.write(
            {
                "schema_version": SCHEMA_VERSION,
                "host_time_utc": _utc_now(),
                "host_monotonic_ns": time.monotonic_ns(),
                "source": source,
                "event": event,
                "detail": detail,
            }
        )

    def record_snapshot(
        self,
        snapshot: dict[str, Any],
        received_monotonic_ns: int,
        host_time_utc: str | None = None,
    ) -> None:
        row = {
            "schema_version": SCHEMA_VERSION,
            "host_time_utc": host_time_utc or _utc_now(),
            "host_monotonic_ns": received_monotonic_ns,
            "snapshot": snapshot,
        }
        self.telemetry.write(row)
        shadow = self._shadow_model.update(snapshot, received_monotonic_ns)
        shadow["host_time_utc"] = row["host_time_utc"]
        self.shadow.write(shadow)
        with self._lock:
            self._latest_snapshot = snapshot
            self._latest_pose = self._shadow_model.latest_pose

    def record_sensor_event(self, event: dict[str, Any]) -> None:
        self.sensor_events.write(event)
        source = str(event.get("source", ""))
        sample_type = str(event.get("type", ""))
        if source == "external_imu" or (
            source == "cube_mavlink"
            and sample_type in TIMING_MAVLINK_TYPES
        ):
            data = event.get("data", {})
            if not isinstance(data, dict):
                data = {}
            self.record_sensor_timing(
                {
                    "source": source,
                    "sample_type": sample_type,
                    "host_monotonic_ns": event.get("host_monotonic_ns"),
                    "host_unix_ns": event.get("host_unix_ns"),
                    "source_sequence": event.get("sequence"),
                    "sensor_time_boot_ms": data.get("time_boot_ms"),
                    "sensor_time_usec": data.get("time_usec"),
                    "sensor_time_s": data.get("sensor_time_s"),
                    "sensor_timestamp_available": (
                        (
                            source == "external_imu"
                            and data.get("sensor_time_s") is not None
                        )
                        or (
                            source == "cube_mavlink"
                            and (
                                data.get("time_boot_ms") is not None
                                or data.get("time_usec") is not None
                            )
                        )
                    ),
                }
            )

    def record_sensor_timing(self, row: dict[str, Any]) -> None:
        self.sensor_timing.write(
            {
                "schema_version": SCHEMA_VERSION,
                **row,
            }
        )

    def latest_pose(self) -> MapPose | None:
        with self._lock:
            return self._latest_pose

    def latest_snapshot(self) -> dict[str, Any] | None:
        with self._lock:
            return self._latest_snapshot

    def set_source_stats(self, source: str, **values: Any) -> None:
        with self._lock:
            current = self._source_stats.setdefault(source, {})
            current.update(values)

    def close(
        self, *, status: str = "complete", reason: str | None = None
    ) -> None:
        if status not in ("complete", "interrupted"):
            raise ValueError("session status must be complete or interrupted")
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self.event(
            "session",
            "recording_stopped",
            {"status": status, "reason": reason},
        )
        self.telemetry.close()
        self.sensor_events.close()
        self.sensor_timing.close()
        self.shadow.close()
        self.events.close()
        self._manifest.update(
            status=status,
            stop_reason=reason,
            ended_utc=_utc_now(),
            rows={
                "telemetry": self.telemetry.rows,
                "sensor_events": self.sensor_events.rows,
                "sensor_timing": self.sensor_timing.rows,
                "shadow_predictions": self.shadow.rows,
                "events": self.events.rows,
            },
        )
        self._write_manifest()


class TelemetryStreamClient(threading.Thread):
    """Consume the visualizer SSE feed without opening either serial device."""

    def __init__(
        self,
        session: FlightSession,
        stop_event: threading.Event,
        url: str,
        sample_rate_hz: float,
    ) -> None:
        super().__init__(name="flight-telemetry-client", daemon=True)
        self.session = session
        self.stop_event = stop_event
        self.url = url
        self.sample_period_s = 1.0 / sample_rate_hz

    def run(self) -> None:
        last_sample_s = -math.inf
        while not self.stop_event.is_set():
            try:
                request = Request(
                    self.url,
                    headers={
                        "Accept": "text/event-stream",
                        "Cache-Control": "no-cache",
                    },
                )
                with urlopen(request, timeout=3.0) as response:
                    self.session.event(
                        "telemetry", "connected", {"url": self.url}
                    )
                    while not self.stop_event.is_set():
                        line = response.readline()
                        if not line:
                            raise RuntimeError("telemetry stream closed")
                        if not line.startswith(b"data:"):
                            continue
                        now_s = time.monotonic()
                        if now_s - last_sample_s < self.sample_period_s:
                            continue
                        snapshot = json.loads(line[5:])
                        self.session.record_snapshot(
                            snapshot, time.monotonic_ns()
                        )
                        last_sample_s = now_s
            except (
                HTTPError,
                URLError,
                OSError,
                RuntimeError,
                ValueError,
                json.JSONDecodeError,
            ) as exc:
                self.session.event(
                    "telemetry", "disconnected", {"error": str(exc)}
                )
                self.stop_event.wait(1.0)


class RawEventStreamClient(threading.Thread):
    """Preserve every decoded source event and report sequence loss."""

    def __init__(
        self,
        session: FlightSession,
        stop_event: threading.Event,
        url: str,
    ) -> None:
        super().__init__(name="raw-sensor-event-client", daemon=True)
        self.session = session
        self.stop_event = stop_event
        self.url = url

    def run(self) -> None:
        last_sequence: int | None = None
        received = 0
        gaps = 0
        while not self.stop_event.is_set():
            try:
                request = Request(
                    self.url,
                    headers={
                        "Accept": "text/event-stream",
                        "Cache-Control": "no-cache",
                    },
                )
                with urlopen(request, timeout=3.0) as response:
                    self.session.event(
                        "sensor_events", "connected", {"url": self.url}
                    )
                    while not self.stop_event.is_set():
                        line = response.readline()
                        if not line:
                            raise RuntimeError("raw sensor stream closed")
                        if not line.startswith(b"data:"):
                            continue
                        event = json.loads(line[5:])
                        sequence = int(event["sequence"])
                        dropped = int(event.get("dropped_before", 0) or 0)
                        if (
                            last_sequence is not None
                            and sequence > last_sequence + 1
                        ):
                            dropped = max(
                                dropped, sequence - last_sequence - 1
                            )
                        if dropped:
                            gaps += dropped
                            event["observed_gap_before"] = dropped
                            self.session.event(
                                "sensor_events",
                                "sequence_gap",
                                {
                                    "previous_sequence": last_sequence,
                                    "current_sequence": sequence,
                                    "dropped": dropped,
                                },
                            )
                        self.session.record_sensor_event(event)
                        received += 1
                        last_sequence = sequence
                        if received % 100 == 0:
                            self.session.set_source_stats(
                                "sensor_events",
                                received=received,
                                sequence_gaps=gaps,
                                last_sequence=last_sequence,
                            )
            except (
                HTTPError,
                URLError,
                OSError,
                RuntimeError,
                ValueError,
                json.JSONDecodeError,
            ) as exc:
                self.session.event(
                    "sensor_events",
                    "disconnected",
                    {"error": str(exc)},
                )
                self.stop_event.wait(1.0)
        self.session.set_source_stats(
            "sensor_events",
            received=received,
            sequence_gaps=gaps,
            last_sequence=last_sequence,
        )


class RawIpPcapWriter:
    """Write received UDP payloads into a valid DLT_RAW PCAP."""

    def __init__(self, path: Path, destination_ip: str, port: int) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._output = path.open("wb")
        self._destination_ip = str(ipaddress.ip_address(destination_ip))
        self._port = port
        self._output.write(
            struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 101)
        )

    @staticmethod
    def _ip_checksum(header: bytes) -> int:
        if len(header) % 2:
            header += b"\0"
        total = sum(struct.unpack(f"!{len(header) // 2}H", header))
        total = (total & 0xFFFF) + (total >> 16)
        total += total >> 16
        return (~total) & 0xFFFF

    def write(
        self, payload: bytes, source_ip: str, source_port: int, time_ns: int
    ) -> None:
        source = socket.inet_aton(source_ip)
        destination = socket.inet_aton(self._destination_ip)
        udp_length = 8 + len(payload)
        ip_length = 20 + udp_length
        ip_header = struct.pack(
            "!BBHHHBBH4s4s",
            0x45,
            0,
            ip_length,
            0,
            0,
            64,
            socket.IPPROTO_UDP,
            0,
            source,
            destination,
        )
        checksum = self._ip_checksum(ip_header)
        ip_header = struct.pack(
            "!BBHHHBBH4s4s",
            0x45,
            0,
            ip_length,
            0,
            0,
            64,
            socket.IPPROTO_UDP,
            checksum,
            source,
            destination,
        )
        udp_header = struct.pack(
            "!HHHH", source_port, self._port, udp_length, 0
        )
        packet = ip_header + udp_header + payload
        seconds, nanoseconds = divmod(time_ns, 1_000_000_000)
        self._output.write(
            struct.pack(
                "<IIII",
                seconds,
                nanoseconds // 1000,
                len(packet),
                len(packet),
            )
        )
        self._output.write(packet)

    def flush(self) -> None:
        self._output.flush()

    def close(self) -> None:
        if self._output.closed:
            return
        self._output.flush()
        self._output.close()


class LidarPacketRecorder(threading.Thread):
    """Legacy reader retained only for replaying old Ethernet-era tests."""

    def __init__(
        self,
        session: FlightSession,
        stop_event: threading.Event,
        *,
        port: int,
        configured_lidar_ip: str,
        destination_ip: str,
    ) -> None:
        super().__init__(name="jt16-pcap-recorder", daemon=True)
        self.session = session
        self.stop_event = stop_event
        self.port = port
        self.configured_lidar_ip = configured_lidar_ip
        self.destination_ip = destination_ip

    def run(self) -> None:
        packet_count = 0
        byte_count = 0
        first_source: str | None = None
        sock: socket.socket | None = None
        writer: RawIpPcapWriter | None = None
        try:
            writer = RawIpPcapWriter(
                self.session.lidar_dir / "jt16_packets.pcap",
                self.destination_ip,
                self.port,
            )
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(0.5)
            sock.bind(("", self.port))
            self.session.event(
                "lidar",
                "udp_capture_ready",
                {
                    "port": self.port,
                    "expected_source": self.configured_lidar_ip,
                },
            )
            while not self.stop_event.is_set():
                try:
                    payload, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                timestamp_ns = time.time_ns()
                source_ip, source_port = address
                writer.write(
                    payload, source_ip, source_port, timestamp_ns
                )
                packet_count += 1
                byte_count += len(payload)
                if first_source is None:
                    first_source = source_ip
                    self.session.event(
                        "lidar",
                        "first_packet",
                        {
                            "source_ip": source_ip,
                            "bytes": len(payload),
                        },
                    )
                    if source_ip != self.configured_lidar_ip:
                        self.session.event(
                            "lidar",
                            "source_ip_mismatch",
                            {
                                "configured": self.configured_lidar_ip,
                                "observed": source_ip,
                            },
                        )
                if packet_count % 100 == 0:
                    writer.flush()
                    self.session.set_source_stats(
                        "lidar",
                        packets=packet_count,
                        payload_bytes=byte_count,
                        source_ip=first_source,
                    )
        except (OSError, ValueError) as exc:
            self.session.event(
                "lidar", "capture_error", {"error": str(exc)}
            )
        finally:
            if sock is not None:
                sock.close()
            if writer is not None:
                writer.close()
            self.session.set_source_stats(
                "lidar",
                packets=packet_count,
                payload_bytes=byte_count,
                source_ip=first_source,
            )


class LidarSerialRecorder(threading.Thread):
    """Capture the JT16 RS485 byte stream without decoding point geometry."""

    def __init__(
        self,
        session: FlightSession,
        stop_event: threading.Event,
        *,
        endpoint: str,
        baud: int,
    ) -> None:
        super().__init__(name="jt16-serial-recorder", daemon=True)
        self.session = session
        self.stop_event = stop_event
        self.endpoint = endpoint
        self.baud = baud

    def run(self) -> None:
        byte_count = 0
        header_candidates = 0
        previous_tail = b""
        output = None
        connection = None
        last_flush_s = time.monotonic()
        try:
            import serial

            connection = serial.Serial(
                self.endpoint,
                baudrate=self.baud,
                timeout=0.1,
                exclusive=True,
            )
            output = (
                self.session.lidar_dir / "jt16_serial.bin"
            ).open("wb")
            self.session.event(
                "lidar",
                "serial_capture_ready",
                {"endpoint": self.endpoint, "baud": self.baud},
            )
            while not self.stop_event.is_set():
                chunk = connection.read(65536)
                if not chunk:
                    continue
                output.write(chunk)
                byte_count += len(chunk)
                framed = previous_tail + chunk
                header_candidates += framed.count(b"\xee\xff")
                previous_tail = framed[-1:]
                now_s = time.monotonic()
                if now_s - last_flush_s >= 1.0:
                    output.flush()
                    os.fsync(output.fileno())
                    last_flush_s = now_s
                    self.session.set_source_stats(
                        "lidar",
                        transport="serial_rs485",
                        endpoint=self.endpoint,
                        baud=self.baud,
                        bytes=byte_count,
                        header_candidates=header_candidates,
                        framing_only=True,
                    )
        except (ImportError, OSError, ValueError) as exc:
            self.session.event(
                "lidar", "serial_capture_error", {"error": str(exc)}
            )
        finally:
            if connection is not None:
                try:
                    connection.close()
                except OSError:
                    pass
            if output is not None:
                output.flush()
                os.fsync(output.fileno())
                output.close()
            self.session.set_source_stats(
                "lidar",
                transport="serial_rs485",
                endpoint=self.endpoint,
                baud=self.baud,
                bytes=byte_count,
                header_candidates=header_candidates,
                framing_only=True,
            )


class HesaiLidarRecorder(threading.Thread):
    """Decode JT16 frames with Hesai's SDK and preserve the serial packets."""

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
        session: FlightSession,
        stop_event: threading.Event,
        config: ProjectConfig,
        *,
        obstacle_sink: Callable[[ObstacleScan], None] | None = None,
    ) -> None:
        super().__init__(name="hesai-jt16-recorder", daemon=True)
        self.session = session
        self.stop_event = stop_event
        self.lidar = config.lidar
        self.obstacle_sink = obstacle_sink
        self.extractor = (
            LidarObstacleExtractor(config.obstacle_avoidance, self.lidar)
            if (
                obstacle_sink is not None
                and config.obstacle_avoidance.lidar_enabled
            )
            else None
        )

    @staticmethod
    def _project_path(value: str) -> Path:
        path = Path(value)
        return path if path.is_absolute() else PROJECT_ROOT / path

    def _read_exact(
        self,
        process: subprocess.Popen,
        size: int,
    ) -> bytes | None:
        output = process.stdout
        if output is None:
            return None
        descriptor = output.fileno()
        collected = bytearray()
        while len(collected) < size:
            if self.stop_event.is_set():
                return None
            if process.poll() is not None:
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
        bridge = self._project_path(self.lidar.bridge_binary)
        correction = self._project_path(self.lidar.correction_file)
        raw_path = self.session.lidar_dir / "jt16_serial.bin"
        stderr_path = self.session.lidar_dir / "jt16_bridge.log"
        process: subprocess.Popen | None = None
        stderr_output = None
        frame_count = 0
        point_count = 0
        extraction_errors = 0
        first_frame_monotonic_ns: int | None = None
        last_frame_monotonic_ns: int | None = None
        bridge_forced_stop = False
        started_s = time.monotonic()
        try:
            if not bridge.is_file() or not os.access(bridge, os.X_OK):
                raise OSError(
                    f"JT16 bridge is missing; run ./optflow build-jt16: "
                    f"{bridge}"
                )
            if not correction.is_file():
                raise OSError(
                    f"JT16 correction file is missing: {correction}"
                )
            if not Path(self.lidar.symlink).exists():
                raise OSError(
                    f"JT16 serial device is missing: {self.lidar.symlink}"
                )
            stderr_output = stderr_path.open("wb")
            command = [
                str(bridge),
                "--device",
                self.lidar.symlink,
                "--baud",
                str(self.lidar.baud),
                "--correction",
                str(correction),
                "--raw-output",
                str(raw_path),
                "--startup-timeout",
                "5",
            ]
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=stderr_output,
                bufsize=0,
            )
            self.session.event(
                "lidar",
                "sdk_bridge_started",
                {
                    "endpoint": self.lidar.symlink,
                    "baud": self.lidar.baud,
                    "sdk_revision": self.lidar.sdk_revision,
                    "correction_file": str(correction),
                    "correction_verified": (
                        self.lidar.correction_verified
                    ),
                },
            )

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
                    frame_index,
                ) = self.FRAME_HEADER.unpack(header_bytes)
                if magic != self.FRAME_MAGIC or version != self.FRAME_VERSION:
                    raise ValueError("JT16 bridge frame header is invalid")
                if (
                    points_in_frame <= 0
                    or points_in_frame > self.MAXIMUM_POINTS
                ):
                    raise ValueError(
                        "JT16 bridge point count is outside limits"
                    )
                payload = self._read_exact(
                    process, points_in_frame * self.POINT_DTYPE.itemsize
                )
                if payload is None:
                    break
                host_receive_monotonic_ns = time.monotonic_ns()
                host_receive_unix_ns = time.time_ns()
                records = np.frombuffer(payload, dtype=self.POINT_DTYPE)
                points = np.column_stack(
                    (records["x"], records["y"], records["z"])
                )
                point_timestamps = records["timestamp"]
                finite_point_timestamps = point_timestamps[
                    np.isfinite(point_timestamps)
                ]
                point_timestamp_min_s = (
                    float(finite_point_timestamps.min())
                    if len(finite_point_timestamps)
                    else None
                )
                point_timestamp_max_s = (
                    float(finite_point_timestamps.max())
                    if len(finite_point_timestamps)
                    else None
                )
                point_timestamp_span_s = (
                    point_timestamp_max_s - point_timestamp_min_s
                    if point_timestamp_min_s is not None
                    and point_timestamp_max_s is not None
                    else None
                )
                self.session.record_sensor_timing(
                    {
                        "source": "jt16_frame",
                        "host_receive_monotonic_ns": (
                            host_receive_monotonic_ns
                        ),
                        "host_receive_unix_ns": host_receive_unix_ns,
                        "bridge_callback_monotonic_ns": (
                            frame_monotonic_ns
                        ),
                        "frame_index": frame_index,
                        "point_count": points_in_frame,
                        "point_timestamp_min_s": point_timestamp_min_s,
                        "point_timestamp_max_s": point_timestamp_max_s,
                        "point_timestamp_span_s": point_timestamp_span_s,
                        "ring_min": (
                            int(records["ring"].min())
                            if len(records)
                            else None
                        ),
                        "ring_max": (
                            int(records["ring"].max())
                            if len(records)
                            else None
                        ),
                    }
                )
                if first_frame_monotonic_ns is None:
                    first_frame_monotonic_ns = host_receive_monotonic_ns
                last_frame_monotonic_ns = host_receive_monotonic_ns
                frame_count += 1
                point_count += points_in_frame
                if self.extractor is not None and self.obstacle_sink is not None:
                    try:
                        self.obstacle_sink(
                            self.extractor.extract(
                                points,
                                monotonic_ns=frame_monotonic_ns,
                            )
                        )
                    except (TypeError, ValueError) as exc:
                        extraction_errors += 1
                        if extraction_errors == 1:
                            self.session.event(
                                "obstacles",
                                "lidar_extraction_error",
                                {"error": str(exc)},
                            )

                elapsed_s = max(0.001, time.monotonic() - started_s)
                self.session.set_source_stats(
                    "lidar",
                    transport="serial_rs485_hesai_sdk",
                    endpoint=self.lidar.symlink,
                    baud=self.lidar.baud,
                    sdk_revision=self.lidar.sdk_revision,
                    frames=frame_count,
                    frame_rate_hz=round(frame_count / elapsed_s, 3),
                    points=point_count,
                    latest_points=points_in_frame,
                    ring_min=(
                        int(records["ring"].min())
                        if len(records)
                        else None
                    ),
                    ring_max=(
                        int(records["ring"].max())
                        if len(records)
                        else None
                    ),
                    point_timestamp_span_s=(
                        point_timestamp_span_s
                    ),
                    extraction_errors=extraction_errors,
                    raw_bytes=(
                        raw_path.stat().st_size
                        if raw_path.exists()
                        else 0
                    ),
                )

            if (
                process.poll() is not None
                and not self.stop_event.is_set()
            ):
                raise RuntimeError(
                    f"JT16 bridge exited with code {process.returncode}"
                )
        except (OSError, RuntimeError, ValueError) as exc:
            self.session.event(
                "lidar", "sdk_bridge_error", {"error": str(exc)}
            )
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    bridge_forced_stop = True
                    process.kill()
                    process.wait(timeout=2.0)
                    self.session.event(
                        "lidar",
                        "sdk_bridge_forced_stop",
                        {
                            "scope": "capture_finalization_only",
                            "capture_frames_preserved": frame_count,
                        },
                    )
            if process is not None and process.stdout is not None:
                process.stdout.close()
            if stderr_output is not None:
                stderr_output.flush()
                os.fsync(stderr_output.fileno())
                stderr_output.close()
            elapsed_s = max(0.001, time.monotonic() - started_s)
            active_duration_s = (
                (
                    last_frame_monotonic_ns - first_frame_monotonic_ns
                )
                / 1.0e9
                if first_frame_monotonic_ns is not None
                and last_frame_monotonic_ns is not None
                and last_frame_monotonic_ns > first_frame_monotonic_ns
                else None
            )
            active_frame_rate_hz = (
                (frame_count - 1) / active_duration_s
                if active_duration_s is not None and frame_count >= 2
                else frame_count / elapsed_s
            )
            self.session.set_source_stats(
                "lidar",
                transport="serial_rs485_hesai_sdk",
                endpoint=self.lidar.symlink,
                baud=self.lidar.baud,
                sdk_revision=self.lidar.sdk_revision,
                frames=frame_count,
                frame_rate_hz=round(active_frame_rate_hz, 3),
                active_duration_s=active_duration_s,
                points=point_count,
                extraction_errors=extraction_errors,
                raw_bytes=(
                    raw_path.stat().st_size if raw_path.exists() else 0
                ),
                bridge_exit_code=(
                    None if process is None else process.returncode
                ),
                bridge_forced_stop=bridge_forced_stop,
                bridge_forced_stop_scope=(
                    "capture_finalization_only"
                    if bridge_forced_stop
                    else None
                ),
            )


class RealSensePointCloudRecorder(threading.Thread):
    """Record a RealSense bag and build a sampled local-frame PLY map."""

    def __init__(
        self,
        session: FlightSession,
        stop_event: threading.Event,
        config: ProjectConfig,
        *,
        pointcloud_rate_hz: float,
        point_stride: int,
        voxel_size_m: float,
        record_bag: bool,
        obstacle_sink: Callable[[ObstacleScan], None] | None = None,
    ) -> None:
        super().__init__(name="realsense-flight-recorder", daemon=True)
        self.session = session
        self.stop_event = stop_event
        self.camera = config.depth_camera
        self.camera_intrinsics_verified = (
            config.calibration.camera_intrinsics_verified
        )
        self.period_s = 1.0 / pointcloud_rate_hz
        self.point_stride = point_stride
        self.voxel_map = VoxelMap(voxel_size_m=voxel_size_m)
        self.record_bag = record_bag
        self.obstacle_sink = obstacle_sink
        self.obstacle_extractor = (
            DepthObstacleExtractor(config.obstacle_avoidance, self.camera)
            if (
                obstacle_sink is not None
                and config.obstacle_avoidance.depth_camera_enabled
            )
            else None
        )
        self.obstacle_period_s = (
            1.0 / config.obstacle_avoidance.target_rate_hz
        )

    @staticmethod
    def _frame_timing(frame: Any, prefix: str) -> dict[str, Any]:
        if not frame:
            return {
                f"{prefix}_frame_number": None,
                f"{prefix}_sensor_timestamp_ms": None,
                f"{prefix}_timestamp_domain": None,
            }
        try:
            return {
                f"{prefix}_frame_number": int(frame.get_frame_number()),
                f"{prefix}_sensor_timestamp_ms": float(
                    frame.get_timestamp()
                ),
                f"{prefix}_timestamp_domain": str(
                    frame.get_frame_timestamp_domain()
                ),
            }
        except RuntimeError:
            return {
                f"{prefix}_frame_number": None,
                f"{prefix}_sensor_timestamp_ms": None,
                f"{prefix}_timestamp_domain": None,
            }

    def run(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            self.session.event(
                "realsense",
                "dependency_error",
                {"error": str(exc)},
            )
            return

        pipeline = None
        frame_count = 0
        frame_timeouts = 0
        mapped_frames = 0
        obstacle_frames = 0
        last_obstacle_nearest_m: float | None = None
        last_map_time = -math.inf
        last_obstacle_time = -math.inf
        last_flush_time = time.monotonic()
        try:
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
            if self.record_bag:
                stream_config.enable_record_to_file(
                    str(self.session.realsense_dir / "flight.bag")
                )
            profile = pipeline.start(stream_config)
            align = rs.align(rs.stream.color)
            device = profile.get_device()
            depth_sensor = device.first_depth_sensor()
            depth_scale_m = float(depth_sensor.get_depth_scale())
            self.session.event(
                "realsense",
                "recording_started",
                {
                    "serial": device.get_info(
                        rs.camera_info.serial_number
                    ),
                    "depth_scale_m": depth_scale_m,
                    "bag_enabled": self.record_bag,
                },
            )
            intrinsics_written = False

            while not self.stop_event.is_set():
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=2000)
                except RuntimeError as exc:
                    frame_timeouts += 1
                    if frame_timeouts == 1 or frame_timeouts % 10 == 0:
                        self.session.event(
                            "realsense",
                            "frame_timeout",
                            {
                                "count": frame_timeouts,
                                "error": str(exc),
                            },
                        )
                    self.session.set_source_stats(
                        "realsense",
                        frames_received=frame_count,
                        frame_timeouts=frame_timeouts,
                        pointcloud_frames=mapped_frames,
                        map_voxels=len(self.voxel_map),
                    )
                    continue
                frame_count += 1
                host_monotonic_ns = time.monotonic_ns()
                host_unix_ns = time.time_ns()
                now = host_monotonic_ns / 1.0e9
                original_depth_frame = frames.get_depth_frame()
                original_color_frame = frames.get_color_frame()
                self.session.record_sensor_timing(
                    {
                        "source": "realsense_frameset",
                        "host_monotonic_ns": host_monotonic_ns,
                        "host_unix_ns": host_unix_ns,
                        **self._frame_timing(
                            original_depth_frame, "depth"
                        ),
                        **self._frame_timing(
                            original_color_frame, "color"
                        ),
                    }
                )
                aligned = align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                depth = np.asanyarray(depth_frame.get_data())
                intrinsics = (
                    depth_frame.profile.as_video_stream_profile().intrinsics
                )
                if not intrinsics_written:
                    intrinsics_payload = {
                        "schema_version": SCHEMA_VERSION,
                        "width": intrinsics.width,
                        "height": intrinsics.height,
                        "fx": intrinsics.fx,
                        "fy": intrinsics.fy,
                        "ppx": intrinsics.ppx,
                        "ppy": intrinsics.ppy,
                        "model": str(intrinsics.model),
                        "coefficients": list(intrinsics.coeffs),
                        "depth_scale_m": depth_scale_m,
                        "verified": self.camera_intrinsics_verified,
                        "note": (
                            "captured device values; verified by the "
                            "project calibration gate"
                            if self.camera_intrinsics_verified
                            else (
                                "captured device values; calibration gate "
                                "remains open"
                            )
                        ),
                    }
                    (
                        self.session.realsense_dir / "intrinsics.json"
                    ).write_text(
                        json.dumps(intrinsics_payload, indent=2) + "\n",
                        encoding="utf-8",
                    )
                    intrinsics_written = True

                if (
                    self.obstacle_extractor is not None
                    and self.obstacle_sink is not None
                    and now - last_obstacle_time
                    >= self.obstacle_period_s
                ):
                    last_obstacle_time = now
                    try:
                        scan = self.obstacle_extractor.extract(
                            depth,
                            depth_scale_m=depth_scale_m,
                            fx=intrinsics.fx,
                            fy=intrinsics.fy,
                            ppx=intrinsics.ppx,
                            ppy=intrinsics.ppy,
                            monotonic_ns=time.monotonic_ns(),
                        )
                        self.obstacle_sink(scan)
                        obstacle_frames += 1
                        last_obstacle_nearest_m = scan.nearest_distance_m
                    except (TypeError, ValueError) as exc:
                        self.session.event(
                            "obstacles",
                            "depth_extraction_error",
                            {"error": str(exc)},
                        )

                if now - last_map_time < self.period_s:
                    continue
                pose = self.session.latest_pose()
                if pose is None:
                    continue
                color = np.asanyarray(color_frame.get_data())
                rows = np.arange(
                    0, depth.shape[0], self.point_stride, dtype=np.int32
                )
                columns = np.arange(
                    0, depth.shape[1], self.point_stride, dtype=np.int32
                )
                uu, vv = np.meshgrid(columns, rows)
                depth_m = depth[vv, uu].astype(np.float32) * depth_scale_m
                valid = (
                    np.isfinite(depth_m)
                    & (depth_m >= 0.20)
                    & (depth_m <= 12.0)
                )
                z_camera = depth_m[valid]
                x_camera = (
                    (uu[valid].astype(np.float32) - intrinsics.ppx)
                    / intrinsics.fx
                    * z_camera
                )
                y_camera = (
                    (vv[valid].astype(np.float32) - intrinsics.ppy)
                    / intrinsics.fy
                    * z_camera
                )
                camera_points = np.column_stack(
                    (x_camera, y_camera, z_camera)
                )
                colors = color[vv[valid], uu[valid], :3]
                local_points = camera_optical_to_local(
                    camera_points, pose
                )
                mapped_frames += 1
                frame_path = (
                    self.session.pointcloud_frames_dir
                    / f"frame_{mapped_frames:06d}.ply"
                )
                write_binary_ply(frame_path, local_points, colors)
                self.voxel_map.add(local_points, colors)
                last_map_time = now

                if now - last_flush_time >= 10.0:
                    self.voxel_map.write(
                        self.session.pointcloud_dir
                        / "flight_environment.partial.ply"
                    )
                    last_flush_time = now
                self.session.set_source_stats(
                    "realsense",
                    frames_received=frame_count,
                    frame_timeouts=frame_timeouts,
                    pointcloud_frames=mapped_frames,
                    map_voxels=len(self.voxel_map),
                    obstacle_frames=obstacle_frames,
                    obstacle_nearest_m=last_obstacle_nearest_m,
                    rejected_new_voxels=(
                        self.voxel_map.rejected_new_voxels
                    ),
                )
        except (OSError, RuntimeError, ValueError) as exc:
            self.session.event(
                "realsense", "recording_error", {"error": str(exc)}
            )
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
            self.voxel_map.write(
                self.session.pointcloud_dir / "flight_environment.ply"
            )
            partial = (
                self.session.pointcloud_dir
                / "flight_environment.partial.ply"
            )
            if partial.exists():
                partial.unlink()
            self.session.set_source_stats(
                "realsense",
                frames_received=frame_count,
                frame_timeouts=frame_timeouts,
                pointcloud_frames=mapped_frames,
                map_voxels=len(self.voxel_map),
                obstacle_frames=obstacle_frames,
                obstacle_nearest_m=last_obstacle_nearest_m,
                rejected_new_voxels=self.voxel_map.rejected_new_voxels,
            )


def _check_telemetry(url: str) -> str | None:
    try:
        request = Request(url, headers={"Accept": "text/event-stream"})
        with urlopen(request, timeout=3.0) as response:
            line = response.readline()
            if line.startswith(b"data:"):
                json.loads(line[5:])
                return None
            return "visualizer stream returned no telemetry event"
    except (
        HTTPError,
        URLError,
        OSError,
        ValueError,
        json.JSONDecodeError,
    ) as exc:
        return str(exc)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--name", help="Short field-test label")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_FLIGHT_ROOT,
        help="Session parent directory",
    )
    parser.add_argument(
        "--telemetry-url",
        default="http://127.0.0.1:8765/api/stream",
        help="Visualizer snapshot SSE endpoint",
    )
    parser.add_argument(
        "--raw-events-url",
        help="Visualizer loss-detectable raw-event SSE endpoint",
    )
    parser.add_argument(
        "--telemetry-rate",
        type=float,
        default=30.0,
        help="Compact timeline sampling rate in Hz",
    )
    parser.add_argument(
        "--pointcloud-rate",
        type=float,
        default=2.0,
        help="D415 PLY keyframe rate in Hz",
    )
    parser.add_argument(
        "--point-stride",
        type=int,
        default=8,
        help="D415 pixel stride used for PLY keyframes",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.08,
        help="Merged environment-cloud voxel size in metres",
    )
    parser.add_argument(
        "--no-depth",
        action="store_true",
        help="Disable all D415 recording",
    )
    parser.add_argument(
        "--no-realsense-bag",
        action="store_true",
        help="Build PLY files without the full-rate RealSense bag",
    )
    parser.add_argument(
        "--no-lidar",
        action="store_true",
        help="Disable JT16 SDK decoding and raw serial capture",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Stop automatically after this many seconds",
    )
    parser.add_argument(
        "--min-free-gb",
        type=float,
        default=5.0,
        help="Stop recording below this free-space threshold",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    positive = (
        ("telemetry-rate", args.telemetry_rate),
        ("pointcloud-rate", args.pointcloud_rate),
        ("point-stride", args.point_stride),
        ("voxel-size", args.voxel_size),
        ("min-free-gb", args.min_free_gb),
    )
    for name, value in positive:
        if float(value) <= 0:
            raise ConfigError(f"{name} must be positive")
    if args.duration is not None and args.duration <= 0:
        raise ConfigError("duration must be positive")


def main() -> int:
    args = build_parser().parse_args()
    try:
        _validate_args(args)
        config = load_config(args.config)
    except (ConfigError, OSError) as exc:
        print(f"Flight logger configuration error: {exc}")
        return 2

    telemetry_error = _check_telemetry(args.telemetry_url)
    if telemetry_error is not None:
        print(
            "Flight logger needs the shared visualizer telemetry stream: "
            f"{telemetry_error}"
        )
        print(
            "Start it first: "
            "./optflow visualizer --host 0.0.0.0 --no-browser"
        )
        return 2

    raw_events_url = args.raw_events_url
    if raw_events_url is None:
        if args.telemetry_url.endswith("/api/stream"):
            raw_events_url = (
                args.telemetry_url[: -len("/api/stream")] + "/api/events"
            )
        else:
            raw_events_url = args.telemetry_url.rstrip("/") + "/api/events"

    args.output_root.mkdir(parents=True, exist_ok=True)
    session = FlightSession(
        args.output_root,
        args.name,
        config,
        args.config,
        args.telemetry_url,
        raw_events_url,
    )
    stop_event = threading.Event()
    sources: list[threading.Thread] = [
        TelemetryStreamClient(
            session,
            stop_event,
            args.telemetry_url,
            args.telemetry_rate,
        ),
        RawEventStreamClient(
            session,
            stop_event,
            raw_events_url,
        ),
    ]
    if not args.no_depth:
        sources.append(
            RealSensePointCloudRecorder(
                session,
                stop_event,
                config,
                pointcloud_rate_hz=args.pointcloud_rate,
                point_stride=args.point_stride,
                voxel_size_m=args.voxel_size,
                record_bag=not args.no_realsense_bag,
            )
        )
    if not args.no_lidar:
        sources.append(
            HesaiLidarRecorder(
                session,
                stop_event,
                config,
            )
        )

    def request_stop(_signum=None, _frame=None) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    for source in sources:
        source.start()

    print(f"Passive flight recording: {session.path}")
    print(
        "No arming, mode changes, movement commands, or ExternalNav are sent."
    )
    print("Press Ctrl+C after landing to finalize the map and report.")
    started_s = time.monotonic()
    low_disk = False
    try:
        while not stop_event.wait(1.0):
            if (
                args.duration is not None
                and time.monotonic() - started_s >= args.duration
            ):
                stop_event.set()
                break
            free_gb = shutil.disk_usage(args.output_root).free / 1.0e9
            session.set_source_stats("storage", free_gb=round(free_gb, 3))
            if free_gb < args.min_free_gb:
                low_disk = True
                session.event(
                    "storage",
                    "minimum_free_space_reached",
                    {
                        "free_gb": free_gb,
                        "minimum_gb": args.min_free_gb,
                    },
                )
                print(
                    "Recording stopped before filling the disk: "
                    f"{free_gb:.2f} GB free"
                )
                stop_event.set()
                break
    finally:
        stop_event.set()
        for source in sources:
            source.join()
        session.close()

    from .flight_analysis import analyze_session

    report = analyze_session(session.path)
    print(f"Flight report: {report}")
    return 3 if low_disk else 0


if __name__ == "__main__":
    raise SystemExit(main())
