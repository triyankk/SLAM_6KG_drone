"""External SLAM pose ingestion helpers.

The bridge's native VIO source is useful, but GPS-denied work often starts with
a sidecar process such as Cartographer, OpenVINS, RTAB-Map, or a ROS/MAVROS
node. This module gives those sidecars a small, dependency-light contract:
send local NED/FRD pose samples as UDP JSON and let the existing MAVLink gates
decide whether the flight controller may see them.
"""

import json
import math
import socket
import time
from dataclasses import dataclass
from typing import Any

from .types import PoseSample


def _as_float(mapping: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = mapping.get(key, default)
    if value is None:
        return default
    return float(value)


def _as_int(mapping: dict[str, Any], key: str, default: int = 0) -> int:
    value = mapping.get(key, default)
    if value is None:
        return default
    return int(value)


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    half = yaw_rad * 0.5
    return math.cos(half), 0.0, 0.0, math.sin(half)


def _extract_xyz(payload: dict[str, Any]) -> tuple[float, float, float]:
    if "position_m" in payload and isinstance(payload["position_m"], list):
        values = payload["position_m"]
        return float(values[0]), float(values[1]), float(values[2])
    if "position" in payload and isinstance(payload["position"], dict):
        position = payload["position"]
        return float(position["x"]), float(position["y"]), float(position["z"])
    return _as_float(payload, "x_m"), _as_float(payload, "y_m"), _as_float(payload, "z_m")


def _extract_velocity(payload: dict[str, Any]) -> tuple[float, float, float]:
    if "velocity_m_s" in payload and isinstance(payload["velocity_m_s"], list):
        values = payload["velocity_m_s"]
        return float(values[0]), float(values[1]), float(values[2])
    if "velocity" in payload and isinstance(payload["velocity"], dict):
        velocity = payload["velocity"]
        return (
            float(velocity.get("x", 0.0)),
            float(velocity.get("y", 0.0)),
            float(velocity.get("z", 0.0)),
        )
    return (
        _as_float(payload, "vx_m_s"),
        _as_float(payload, "vy_m_s"),
        _as_float(payload, "vz_m_s"),
    )


def _extract_quaternion(payload: dict[str, Any]) -> tuple[float, float, float, float]:
    if "quaternion" in payload:
        quaternion = payload["quaternion"]
        if isinstance(quaternion, list):
            return float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3])
        if isinstance(quaternion, dict):
            return (
                float(quaternion.get("w", 1.0)),
                float(quaternion.get("x", 0.0)),
                float(quaternion.get("y", 0.0)),
                float(quaternion.get("z", 0.0)),
            )
    if "q" in payload and isinstance(payload["q"], list):
        q = payload["q"]
        return float(q[0]), float(q[1]), float(q[2]), float(q[3])
    if all(key in payload for key in ("qw", "qx", "qy", "qz")):
        return (
            _as_float(payload, "qw", 1.0),
            _as_float(payload, "qx"),
            _as_float(payload, "qy"),
            _as_float(payload, "qz"),
        )
    if "yaw_rad" in payload:
        return _quaternion_from_yaw(float(payload["yaw_rad"]))
    if "yaw_deg" in payload:
        return _quaternion_from_yaw(math.radians(float(payload["yaw_deg"])))
    return 1.0, 0.0, 0.0, 0.0


def _timestamp_us(payload: dict[str, Any]) -> int:
    for key in ("timestamp_us", "time_usec", "t_usec"):
        if key in payload and payload[key] is not None:
            return int(payload[key])
    for key in ("timestamp_s", "time_s", "t"):
        if key in payload and payload[key] is not None:
            return int(float(payload[key]) * 1_000_000)
    return int(time.time() * 1_000_000)


def pose_sample_from_json(payload: dict[str, Any], default_source_name: str = "external_udp") -> PoseSample:
    """Build a PoseSample from a JSON-compatible mapping.

    The expected flight-frame contract is local NED position with body FRD
    attitude, matching the bridge's existing MAVLink ODOMETRY/GPS_INPUT path.
    ROS sidecars should do ENU-to-NED conversion before sending this packet.
    """

    x_m, y_m, z_m = _extract_xyz(payload)
    vx_m_s, vy_m_s, vz_m_s = _extract_velocity(payload)
    qw, qx, qy, qz = _extract_quaternion(payload)
    return PoseSample(
        timestamp_us=_timestamp_us(payload),
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        qw=qw,
        qx=qx,
        qy=qy,
        qz=qz,
        vx_m_s=vx_m_s,
        vy_m_s=vy_m_s,
        vz_m_s=vz_m_s,
        roll_rate_rad_s=_as_float(payload, "roll_rate_rad_s"),
        pitch_rate_rad_s=_as_float(payload, "pitch_rate_rad_s"),
        yaw_rate_rad_s=_as_float(payload, "yaw_rate_rad_s"),
        pose_quality=max(0, min(100, _as_int(payload, "pose_quality", _as_int(payload, "quality", 100)))),
        tracking_state=str(payload.get("tracking_state", payload.get("tracking", "ok"))),
        feature_count=_as_int(payload, "feature_count"),
        tracked_feature_count=_as_int(payload, "tracked_feature_count"),
        inlier_count=_as_int(payload, "inlier_count"),
        source_name=str(payload.get("source_name", payload.get("source", default_source_name))),
    )


@dataclass
class ExternalPoseUdpSource:
    """Receive local SLAM pose samples over UDP JSON."""

    bind_host: str = "127.0.0.1"
    bind_port: int = 15560
    max_age_s: float = 0.35
    first_sample_timeout_s: float = 3.0
    max_packet_bytes: int = 8192

    def __post_init__(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.bind_host, int(self.bind_port)))
        self.socket.setblocking(False)
        self.bind_port = int(self.socket.getsockname()[1])
        self._latest: PoseSample | None = None
        self._latest_received_s = 0.0

    def close(self) -> None:
        self.socket.close()

    def sample(self) -> PoseSample:
        deadline_s = time.time() + self.first_sample_timeout_s if self._latest is None else time.time()
        while True:
            self._drain_packets()
            if self._latest is not None:
                age_s = time.time() - self._latest_received_s
                if age_s <= self.max_age_s:
                    return self._latest
                stale = self._latest
                return PoseSample(
                    timestamp_us=stale.timestamp_us,
                    x_m=stale.x_m,
                    y_m=stale.y_m,
                    z_m=stale.z_m,
                    qw=stale.qw,
                    qx=stale.qx,
                    qy=stale.qy,
                    qz=stale.qz,
                    vx_m_s=0.0,
                    vy_m_s=0.0,
                    vz_m_s=0.0,
                    pose_quality=0,
                    tracking_state=f"stale_{age_s:.1f}s",
                    feature_count=stale.feature_count,
                    tracked_feature_count=stale.tracked_feature_count,
                    inlier_count=stale.inlier_count,
                    source_name=stale.source_name or "external_udp",
                )
            if time.time() >= deadline_s:
                raise RuntimeError(
                    f"No external SLAM pose received on udp://{self.bind_host}:{self.bind_port}"
                )
            time.sleep(0.02)

    def _drain_packets(self) -> None:
        while True:
            try:
                data, _ = self.socket.recvfrom(self.max_packet_bytes)
            except BlockingIOError:
                return
            payload = json.loads(data.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("external SLAM pose UDP packet must be a JSON object")
            self._latest = pose_sample_from_json(payload)
            self._latest_received_s = time.time()
