import csv
import math
import time
from dataclasses import dataclass, field
from typing import Any, Protocol

from .types import PoseSample


def quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    half = yaw_rad * 0.5
    return math.cos(half), 0.0, 0.0, math.sin(half)


class PoseSource(Protocol):
    def sample(self) -> PoseSample:
        ...


@dataclass
class HoverPoseSource:
    altitude_m: float = 1.5

    def sample(self) -> PoseSample:
        now_us = int(time.time() * 1e6)
        qw, qx, qy, qz = quaternion_from_yaw(0.0)
        return PoseSample(
            timestamp_us=now_us,
            x_m=0.0,
            y_m=0.0,
            z_m=-self.altitude_m,
            qw=qw,
            qx=qx,
            qy=qy,
            qz=qz,
        )


@dataclass
class CirclePoseSource:
    radius_m: float = 1.0
    period_s: float = 20.0
    altitude_m: float = 1.5
    start_s: float = field(default_factory=time.time)

    def sample(self) -> PoseSample:
        now_s = time.time()
        phase = ((now_s - self.start_s) / max(self.period_s, 1e-6)) * 2.0 * math.pi
        x_m = self.radius_m * math.cos(phase)
        y_m = self.radius_m * math.sin(phase)
        yaw = phase + math.pi / 2.0
        vx = -(2.0 * math.pi / self.period_s) * self.radius_m * math.sin(phase)
        vy = (2.0 * math.pi / self.period_s) * self.radius_m * math.cos(phase)
        qw, qx, qy, qz = quaternion_from_yaw(yaw)
        return PoseSample(
            timestamp_us=int(now_s * 1e6),
            x_m=x_m,
            y_m=y_m,
            z_m=-self.altitude_m,
            qw=qw,
            qx=qx,
            qy=qy,
            qz=qz,
            vx_m_s=vx,
            vy_m_s=vy,
        )


@dataclass
class CsvReplayPoseSource:
    rows: list[PoseSample]
    start_s: float = field(default_factory=time.time)

    @classmethod
    def from_path(cls, path: str) -> "CsvReplayPoseSource":
        rows: list[PoseSample] = []
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                t_s = float(row["t_s"])
                yaw_deg = float(row.get("yaw_deg", 0.0))
                qw, qx, qy, qz = quaternion_from_yaw(math.radians(yaw_deg))
                rows.append(
                    PoseSample(
                        timestamp_us=int(t_s * 1e6),
                        x_m=float(row["x_m"]),
                        y_m=float(row["y_m"]),
                        z_m=float(row["z_m"]),
                        qw=qw,
                        qx=qx,
                        qy=qy,
                        qz=qz,
                        vx_m_s=float(row.get("vx_m_s", 0.0)),
                        vy_m_s=float(row.get("vy_m_s", 0.0)),
                        vz_m_s=float(row.get("vz_m_s", 0.0)),
                    )
                )
        if not rows:
            raise ValueError(f"No replay rows found in {path}")
        base_us = rows[0].timestamp_us
        for row in rows:
            row.timestamp_us -= base_us
        return cls(rows=rows)

    def sample(self) -> PoseSample:
        elapsed_us = int((time.time() - self.start_s) * 1e6)
        for row in self.rows:
            if row.timestamp_us >= elapsed_us:
                return PoseSample(
                    timestamp_us=int(time.time() * 1e6),
                    x_m=row.x_m,
                    y_m=row.y_m,
                    z_m=row.z_m,
                    qw=row.qw,
                    qx=row.qx,
                    qy=row.qy,
                    qz=row.qz,
                    vx_m_s=row.vx_m_s,
                    vy_m_s=row.vy_m_s,
                    vz_m_s=row.vz_m_s,
                )
        last = self.rows[-1]
        return PoseSample(
            timestamp_us=int(time.time() * 1e6),
            x_m=last.x_m,
            y_m=last.y_m,
            z_m=last.z_m,
            qw=last.qw,
            qx=last.qx,
            qy=last.qy,
            qz=last.qz,
            vx_m_s=last.vx_m_s,
            vy_m_s=last.vy_m_s,
            vz_m_s=last.vz_m_s,
        )


def make_pose_source(source: str, csv_path: str = "", external_pose_config: Any | None = None) -> PoseSource:
    if source == "hover":
        return HoverPoseSource()
    if source == "circle":
        return CirclePoseSource(start_s=time.time())
    if source == "csv":
        if not csv_path:
            raise ValueError("--csv-path is required when --source csv")
        return CsvReplayPoseSource.from_path(csv_path)
    if source == "vio":
        from .vio_backend import VioPoseSource

        return VioPoseSource()
    if source in {"external_udp", "slam_udp"}:
        from .external_pose import ExternalPoseUdpSource

        if external_pose_config is None:
            return ExternalPoseUdpSource()
        return ExternalPoseUdpSource(
            bind_host=str(getattr(external_pose_config, "bind_host", "127.0.0.1")),
            bind_port=int(getattr(external_pose_config, "bind_port", 15560)),
            max_age_s=float(getattr(external_pose_config, "max_age_s", 0.35)),
            first_sample_timeout_s=float(getattr(external_pose_config, "first_sample_timeout_s", 3.0)),
        )
    raise ValueError(f"Unsupported source: {source}")
