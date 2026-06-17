"""GPS-denied flight readiness evaluation.

This module is intentionally policy-heavy and side-effect-light. It turns the
state scattered across pose, IMU, EKF, rangefinder, GPS2, and observer logic
into one report that can be printed, sent to GCS, and written as JSON.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .fc_config import (
    FlightControllerTelemetry,
    gps_reference_valid,
    mavlink_heartbeat_valid,
    rangefinder_height_valid,
    rc_link_valid,
    recent_status_blocks_slam,
)


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:  # noqa: BLE001
        return False


def _fmt_fix_sats(fix_type: int | None, satellites: int | None) -> str:
    fix_text = "unknown" if fix_type is None else str(fix_type)
    sat_text = "unknown" if satellites is None else str(satellites)
    return f"fix={fix_text} sats={sat_text}"


def _compact(items: list[str], limit: int = 3) -> str:
    if not items:
        return ""
    if len(items) <= limit:
        return "; ".join(items)
    return "; ".join(items[:limit]) + f"; +{len(items) - limit} more"


@dataclass
class GpsDeniedReadinessConfig:
    enabled: bool = True
    require_imu: bool = True
    require_rc_link: bool = True
    require_rangefinder: bool = True
    require_ekf_status: bool = True
    require_attitude: bool = True
    require_local_position: bool = True
    require_origin_for_gps_input: bool = True
    require_calibration_or_observer: bool = True
    stable_seconds: float = 3.0
    max_pose_dt_s: float = 0.35
    max_pose_jump_m: float = 1.25
    max_velocity_m_s: float = 5.0
    max_rangefinder_disagreement_m: float = 1.0
    min_observer_score: float = 7.0
    announce_interval_s: float = 10.0
    status_write_interval_s: float = 1.0
    status_path: str = "logs/gps_denied_readiness.json"


@dataclass
class GpsDeniedReadinessReport:
    enabled: bool
    stage: str
    ready: bool
    active: bool
    stable_for_s: float
    score: int
    blockers: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    details: dict[str, Any] = field(default_factory=dict)

    @property
    def ok_without_stability(self) -> bool:
        return not self.blockers

    def compact_message(self) -> str:
        if not self.enabled:
            return "GPS-DENIED GATE DISABLED"
        if self.active:
            return f"GPS-DENIED ACTIVE: score={self.score}/100 stable={self.stable_for_s:.1f}s"
        if self.ready:
            return f"GPS-DENIED READY: score={self.score}/100 stable={self.stable_for_s:.1f}s"
        if self.blockers:
            return f"GPS-DENIED WAIT: {_compact(self.blockers)}"
        return f"GPS-DENIED STABILIZING: score={self.score}/100 stable={self.stable_for_s:.1f}s"

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "enabled": self.enabled,
            "stage": self.stage,
            "ready": self.ready,
            "active": self.active,
            "stable_for_s": round(float(self.stable_for_s), 3),
            "score": int(self.score),
            "blockers": list(self.blockers),
            "warnings": list(self.warnings),
            "details": self.details,
        }


class GpsDeniedReadinessTracker:
    """Stateful evaluator for SLAM flight readiness.

    The tracker remembers the previous pose timestamp and position so it can
    catch stale streams and jumps, two of the most common GPS-denied failures.
    """

    def __init__(self, config: GpsDeniedReadinessConfig | None = None):
        self.config = config or GpsDeniedReadinessConfig()
        self._stable_since_s: float | None = None
        self._previous_timestamp_us: int | None = None
        self._previous_xyz_m: tuple[float, float, float] | None = None
        self._last_status_write_s = 0.0

    def update(
        self,
        pose,
        imu_sample,
        fc_state: FlightControllerTelemetry,
        *,
        gps_input_enabled: bool,
        gps_input_fixed_fix: bool,
        gps_input_origin_valid: bool,
        target_mode: str,
        calibration_profile_valid: bool,
        observer_summary: dict | None,
        using_gps_input_bridge: bool,
        slam_pose_gps2_recent: bool = False,
        now_s: float | None = None,
    ) -> GpsDeniedReadinessReport:
        now = time.time() if now_s is None else now_s
        config = self.config
        if not config.enabled:
            return GpsDeniedReadinessReport(
                enabled=False,
                stage="disabled",
                ready=False,
                active=False,
                stable_for_s=0.0,
                score=0,
                warnings=["GPS-denied readiness gate disabled by config"],
            )

        blockers: list[str] = []
        warnings: list[str] = []
        score = 100

        if not mavlink_heartbeat_valid(fc_state):
            blockers.append("MAVLink heartbeat missing")
            score -= 20
        if config.require_rc_link and not rc_link_valid(fc_state):
            blockers.append("RC link missing")
            score -= 12
        if config.require_local_position and fc_state.local_position is None:
            blockers.append("EKF local position missing")
            score -= 10
        if config.require_attitude and fc_state.attitude is None:
            blockers.append("attitude telemetry missing")
            score -= 8
        if config.require_ekf_status and fc_state.ekf_flags is None:
            blockers.append("EKF status missing")
            score -= 8
        if config.require_imu and imu_sample is None:
            blockers.append("external IMU missing")
            score -= 12
        if config.require_rangefinder and not rangefinder_height_valid(fc_state):
            blockers.append("rangefinder height missing")
            score -= 14

        pose_blockers, pose_warnings, pose_score_penalty = self._pose_health(
            pose,
            fc_state,
            now,
        )
        blockers.extend(pose_blockers)
        warnings.extend(pose_warnings)
        score -= pose_score_penalty

        observer_ready = self._observer_ready(observer_summary)
        observer_score = 0.0 if observer_summary is None else float(observer_summary.get("score", 0.0) or 0.0)
        if config.require_calibration_or_observer and not calibration_profile_valid and not observer_ready:
            blockers.append(
                "needs Brake calibration or LOITER observer"
                f" score>={config.min_observer_score:.1f}"
            )
            score -= 15

        if using_gps_input_bridge:
            if not gps_input_enabled:
                blockers.append("GPS2 GPS_INPUT disabled")
                score -= 18
            elif not gps_input_fixed_fix and config.require_origin_for_gps_input and not gps_input_origin_valid:
                if gps_reference_valid(fc_state, 3, 8):
                    blockers.append("GPS2 origin not locked yet")
                else:
                    blockers.append(f"GPS2 origin needs GPS/manual origin {_fmt_fix_sats(fc_state.gps_fix_type, fc_state.gps_satellites)}")
                score -= 14
        else:
            warnings.append("using ExternalNav ODOMETRY path instead of GPS2 bridge")

        if recent_status_blocks_slam(fc_state):
            blockers.append(f"FC warning active: {fc_state.status_text or 'status text'}")
            score -= 15

        mode = str(fc_state.flight_mode or "UNKNOWN").upper()
        target_mode = target_mode.strip().upper()
        if target_mode and target_mode != "ANY" and mode != target_mode:
            warnings.append(f"waiting for {target_mode}; current mode={mode}")
        if not fc_state.armed:
            warnings.append("vehicle disarmed")

        no_blockers = not blockers
        if no_blockers:
            if self._stable_since_s is None:
                self._stable_since_s = now
        else:
            self._stable_since_s = None
        stable_for_s = 0.0 if self._stable_since_s is None else max(0.0, now - self._stable_since_s)
        ready = no_blockers and stable_for_s >= max(config.stable_seconds, 0.0)
        active = bool(
            ready
            and fc_state.armed
            and (target_mode in {"", "ANY"} or mode == target_mode)
            and (not using_gps_input_bridge or slam_pose_gps2_recent)
        )
        if active:
            stage = "active"
        elif ready:
            stage = "ready"
        elif no_blockers:
            stage = "stabilizing"
        else:
            stage = "blocked"

        return GpsDeniedReadinessReport(
            enabled=True,
            stage=stage,
            ready=ready,
            active=active,
            stable_for_s=stable_for_s,
            score=max(0, min(100, int(round(score)))),
            blockers=blockers,
            warnings=warnings,
            details={
                "mode": mode,
                "armed": bool(fc_state.armed),
                "gps1": {
                    "fix_type": fc_state.gps_fix_type,
                    "satellites": fc_state.gps_satellites,
                },
                "gps2": {
                    "fix_type": fc_state.gps2_fix_type,
                    "satellites": fc_state.gps2_satellites,
                    "origin_valid": gps_input_origin_valid,
                    "gps_input_enabled": gps_input_enabled,
                },
                "observer": {
                    "score": round(observer_score, 2),
                    "recommendation": None if observer_summary is None else observer_summary.get("recommendation"),
                    "ready": observer_ready,
                },
                "calibration_profile_valid": bool(calibration_profile_valid),
                "pose": self._pose_details(pose),
                "rangefinder_m": fc_state.rangefinder_distance_m,
                "rc_channel_count": fc_state.rc_channel_count,
            },
        )

    def maybe_write_status(self, report: GpsDeniedReadinessReport) -> None:
        path_text = self.config.status_path
        if not path_text:
            return
        now_s = time.time()
        if now_s - self._last_status_write_s < max(self.config.status_write_interval_s, 0.1):
            return
        path = Path(path_text).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report.to_dict(), indent=2, sort_keys=True), encoding="utf-8")
        self._last_status_write_s = now_s

    def _observer_ready(self, observer_summary: dict | None) -> bool:
        if not observer_summary:
            return False
        if str(observer_summary.get("recommendation", "")) != "ready_for_no_gps_poshold":
            return False
        return float(observer_summary.get("score", 0.0) or 0.0) >= self.config.min_observer_score

    def _pose_health(
        self,
        pose,
        fc_state: FlightControllerTelemetry,
        now_s: float,
    ) -> tuple[list[str], list[str], int]:
        blockers: list[str] = []
        warnings: list[str] = []
        penalty = 0
        if pose is None:
            return ["SLAM pose missing"], warnings, 30

        tracking = str(getattr(pose, "tracking_state", "unknown"))
        quality = int(getattr(pose, "pose_quality", 0) or 0)
        if not tracking.startswith("ok"):
            blockers.append(f"SLAM tracking={tracking}")
            penalty += 18
        if quality < 60:
            blockers.append(f"SLAM quality {quality}<60")
            penalty += min(18, max(4, int((60 - quality) * 0.25)))

        values = (
            getattr(pose, "x_m", None),
            getattr(pose, "y_m", None),
            getattr(pose, "z_m", None),
            getattr(pose, "vx_m_s", None),
            getattr(pose, "vy_m_s", None),
            getattr(pose, "vz_m_s", None),
            getattr(pose, "qw", None),
            getattr(pose, "qx", None),
            getattr(pose, "qy", None),
            getattr(pose, "qz", None),
        )
        if not all(_finite(value) for value in values):
            blockers.append("SLAM pose has non-finite values")
            penalty += 25

        qw = float(getattr(pose, "qw", 1.0) or 1.0)
        qx = float(getattr(pose, "qx", 0.0) or 0.0)
        qy = float(getattr(pose, "qy", 0.0) or 0.0)
        qz = float(getattr(pose, "qz", 0.0) or 0.0)
        quat_norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if not 0.7 <= quat_norm <= 1.3:
            blockers.append(f"bad quaternion norm {quat_norm:.2f}")
            penalty += 12

        vx = float(getattr(pose, "vx_m_s", 0.0) or 0.0)
        vy = float(getattr(pose, "vy_m_s", 0.0) or 0.0)
        vz = float(getattr(pose, "vz_m_s", 0.0) or 0.0)
        speed_m_s = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed_m_s > self.config.max_velocity_m_s:
            blockers.append(f"SLAM speed too high {speed_m_s:.1f}m/s")
            penalty += 12

        timestamp_us = int(getattr(pose, "timestamp_us", 0) or 0)
        if self._previous_timestamp_us is not None and timestamp_us > 0:
            dt_s = (timestamp_us - self._previous_timestamp_us) / 1_000_000.0
            if dt_s > self.config.max_pose_dt_s:
                blockers.append(f"SLAM timestamp gap {dt_s:.2f}s")
                penalty += 12
            elif dt_s <= 0.0:
                warnings.append("SLAM timestamp repeated")
        self._previous_timestamp_us = timestamp_us or self._previous_timestamp_us

        xyz = (
            float(getattr(pose, "x_m", 0.0) or 0.0),
            float(getattr(pose, "y_m", 0.0) or 0.0),
            float(getattr(pose, "z_m", 0.0) or 0.0),
        )
        if self._previous_xyz_m is not None:
            jump_m = math.sqrt(sum((a - b) ** 2 for a, b in zip(xyz, self._previous_xyz_m)))
            if jump_m > self.config.max_pose_jump_m:
                blockers.append(f"SLAM pose jump {jump_m:.2f}m")
                penalty += 14
        self._previous_xyz_m = xyz

        if rangefinder_height_valid(fc_state):
            rangefinder_m = float(fc_state.rangefinder_distance_m or 0.0)
            disagreement_m = abs(abs(xyz[2]) - rangefinder_m)
            if disagreement_m > self.config.max_rangefinder_disagreement_m:
                blockers.append(f"range/SLAM height mismatch {disagreement_m:.1f}m")
                penalty += 15
        elif self.config.require_rangefinder:
            warnings.append("rangefinder unavailable, so height cannot be cross-checked")

        return blockers, warnings, penalty

    def _pose_details(self, pose) -> dict[str, Any] | None:
        if pose is None:
            return None
        return {
            "source": getattr(pose, "source_name", ""),
            "tracking": getattr(pose, "tracking_state", ""),
            "quality": getattr(pose, "pose_quality", None),
            "timestamp_us": getattr(pose, "timestamp_us", None),
            "x_m": getattr(pose, "x_m", None),
            "y_m": getattr(pose, "y_m", None),
            "z_m": getattr(pose, "z_m", None),
            "vx_m_s": getattr(pose, "vx_m_s", None),
            "vy_m_s": getattr(pose, "vy_m_s", None),
            "vz_m_s": getattr(pose, "vz_m_s", None),
        }
