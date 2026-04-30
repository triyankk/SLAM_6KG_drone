"""LOITER soft-calibration observer for SLAM/VIO health.

The observer is intentionally boring in LOITER: it watches the normal GPS/EKF
flight reference, scores SLAM quality, and learns bounded internal correction
estimates. It does not steer the vehicle in LOITER. In SLAM/POSHOLD mode it can
warn or request LOITER fallback if the quality score collapses.
"""

import json
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from pymavlink import mavutil

from .bridge_config import SlamObserverConfig
from .calibration import quaternion_from_yaw_deg, quaternion_multiply, rotate_xy
from .fc_config import (
    FlightControllerTelemetry,
    mavlink_heartbeat_valid,
    rangefinder_height_valid,
    send_gcs_event,
    set_vehicle_mode,
)


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _wrap_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def _pose_yaw_deg(pose) -> float:
    qw = float(getattr(pose, "qw", 1.0))
    qx = float(getattr(pose, "qx", 0.0))
    qy = float(getattr(pose, "qy", 0.0))
    qz = float(getattr(pose, "qz", 0.0))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _attitude_deg(fc_state: FlightControllerTelemetry) -> dict[str, float] | None:
    attitude = fc_state.attitude
    if attitude is None:
        return None
    return {
        "roll_deg": math.degrees(float(getattr(attitude, "roll", 0.0))),
        "pitch_deg": math.degrees(float(getattr(attitude, "pitch", 0.0))),
        "yaw_deg": math.degrees(float(getattr(attitude, "yaw", 0.0))),
    }


def _local_position_dict(fc_state: FlightControllerTelemetry) -> dict[str, float] | None:
    local = fc_state.local_position
    if local is None:
        return None
    return {
        "x_m": float(getattr(local, "x", 0.0)),
        "y_m": float(getattr(local, "y", 0.0)),
        "z_m": float(getattr(local, "z", 0.0)),
        "vx_m_s": float(getattr(local, "vx", 0.0)),
        "vy_m_s": float(getattr(local, "vy", 0.0)),
        "vz_m_s": float(getattr(local, "vz", 0.0)),
    }


def _pose_dict(pose) -> dict[str, float | int | str]:
    return {
        "x_m": float(getattr(pose, "x_m", 0.0)),
        "y_m": float(getattr(pose, "y_m", 0.0)),
        "z_m": float(getattr(pose, "z_m", 0.0)),
        "vx_m_s": float(getattr(pose, "vx_m_s", 0.0)),
        "vy_m_s": float(getattr(pose, "vy_m_s", 0.0)),
        "vz_m_s": float(getattr(pose, "vz_m_s", 0.0)),
        "yaw_deg": _pose_yaw_deg(pose),
        "quality": int(getattr(pose, "pose_quality", 0)),
        "tracking": str(getattr(pose, "tracking_state", "unknown")),
        "features": int(getattr(pose, "feature_count", 0)),
        "tracked_features": int(getattr(pose, "tracked_feature_count", 0)),
        "inliers": int(getattr(pose, "inlier_count", 0)),
    }


def _gps1_healthy(fc_state: FlightControllerTelemetry) -> bool:
    return (fc_state.gps_fix_type or 0) >= 3 and (fc_state.gps_satellites or 0) >= 8


def _any_gps_healthy(fc_state: FlightControllerTelemetry) -> bool:
    return _gps1_healthy(fc_state) or (
        (fc_state.gps2_fix_type or 0) >= 3 and (fc_state.gps2_satellites or 0) >= 8
    )


@dataclass
class ObserverBaseline:
    """Reference snapshot taken when LOITER observation starts."""

    local_x_m: float
    local_y_m: float
    local_z_m: float
    vio_x_m: float
    vio_y_m: float
    vio_z_m: float
    yaw_offset_deg: float


@dataclass
class ObserverState:
    """Mutable observer memory.

    This survives across mode changes so a good LOITER observation can inform a
    later GPS2 SLAM feed, but the actual LOITER flight control remains untouched.
    """

    active: bool = False
    started_s: float = 0.0
    sample_count: int = 0
    baseline: ObserverBaseline | None = None
    quality_score: float = 0.0
    best_score: float = 0.0
    drift_xy_m: float = 0.0
    velocity_drift_m_s: float = 0.0
    yaw_error_deg: float = 0.0
    altitude_error_m: float = 0.0
    scale_ratio: float | None = None
    scale_confidence: float = 0.0
    soft_calibration_confidence: float = 0.0
    current_recommendation: str = "waiting"
    last_pose_timestamp_us: int | None = None
    last_pose_dt_s: float | None = None
    last_message_s: float = 0.0
    last_quality_message_s: float = 0.0
    last_logged_s: float = 0.0
    last_status_write_s: float = 0.0
    last_reported_score: float | None = None
    last_quality_bucket: str = ""
    fallback_warned_s: float = 0.0
    correction_valid: bool = False
    correction_samples: int = 0
    correction_yaw_offset_deg: float = 0.0
    correction_scale_xy: float = 1.0
    correction_x_offset_m: float = 0.0
    correction_y_offset_m: float = 0.0
    last_correction_update_s: float = 0.0
    last_ready_s: float = 0.0


class SlamLoiterObserver:
    """Score SLAM health and learn safe correction estimates during LOITER."""

    def __init__(self, config: SlamObserverConfig, dry_run: bool = False):
        self.config = config
        self.dry_run = dry_run
        self.state = ObserverState()

    def startup_summary(self) -> dict:
        return {
            "enabled": self.config.enable_loiter_observation,
            "score": round(self.state.quality_score, 2),
            "best_score": round(self.state.best_score, 2),
            "active": self.state.active,
            "recommendation": self.state.current_recommendation,
            "live_soft_correction": self.config.enable_live_soft_correction,
            "auto_fallback_to_loiter": self.config.enable_auto_fallback_to_loiter,
            "correction_valid": self.state.correction_valid,
        }

    def announce_ready(self, master) -> None:
        if self.config.enable_loiter_observation and not self.dry_run:
            self._send(master, "SLAM observer ready. LOITER soft calibration available.")
            if self.config.enable_live_soft_correction:
                self._send(master, "SLAM live soft correction enabled: bounded GPS2 pose correction only.")
            if self.config.enable_auto_fallback_to_loiter:
                self._send(master, "SLAM auto fallback enabled: critical SLAM plus healthy GPS switches to LOITER.")

    def update(
        self,
        master,
        fc_state: FlightControllerTelemetry,
        pose,
        imu_sample,
        imu_expected: bool = True,
    ) -> dict:
        now_s = time.time()
        mode = str(fc_state.flight_mode or "").upper()

        if not self.config.enable_loiter_observation:
            return self._summary(fc_state, pose, imu_sample, "disabled")

        if mode == "LOITER":
            # LOITER is the safest learning mode because ArduPilot is still
            # flying from its normal GPS/EKF stack. We compare SLAM against that
            # reference and never send motion commands here.
            if not self.state.active:
                self._start_observation(master, fc_state, pose, now_s)
            self.state.sample_count += 1
            metrics = self._compute_metrics(fc_state, pose, imu_sample, imu_expected, now_s)
            self._maybe_log(fc_state, pose, imu_sample, metrics, now_s)
            self._maybe_write_status(fc_state, pose, imu_sample, metrics, now_s)
            self._maybe_send_loiter_messages(master, metrics, now_s)
            return metrics

        if mode == "POSHOLD":
            # POSHOLD is where SLAM quality matters. If the score falls below
            # the critical threshold, the observer can warn or request LOITER
            # fallback, but only when that behavior is enabled in config.
            metrics = self._compute_poshold_metrics(fc_state, pose, imu_sample, imu_expected, now_s)
            self._maybe_write_status(fc_state, pose, imu_sample, metrics, now_s)
            self._maybe_warn_or_fallback(master, fc_state, metrics, now_s)
            return metrics

        was_active = self.state.active
        self.state.active = False
        self.state.baseline = None
        if was_active:
            self._write_status(self._summary(fc_state, pose, imu_sample, self._stored_recommendation()))

        metrics = self._summary(fc_state, pose, imu_sample, self._stored_recommendation())
        self._maybe_warn_or_fallback(master, fc_state, metrics, now_s)
        return metrics

    def _start_observation(self, master, fc_state: FlightControllerTelemetry, pose, now_s: float) -> None:
        self.state.active = True
        self.state.started_s = now_s
        self.state.sample_count = 0
        self.state.baseline = None
        self.state.last_message_s = 0.0
        self.state.last_quality_message_s = 0.0
        self.state.last_reported_score = None
        self.state.last_quality_bucket = ""
        self._try_set_baseline(fc_state, pose)
        if not self.dry_run:
            self._send(master, "LOITER active: SLAM observation mode started.")

    def _try_set_baseline(self, fc_state: FlightControllerTelemetry, pose) -> None:
        if self.state.baseline is not None:
            return
        local = _local_position_dict(fc_state)
        attitude = _attitude_deg(fc_state)
        if local is None or attitude is None or pose is None:
            return
        self.state.baseline = ObserverBaseline(
            local_x_m=local["x_m"],
            local_y_m=local["y_m"],
            local_z_m=local["z_m"],
            vio_x_m=float(getattr(pose, "x_m", 0.0)),
            vio_y_m=float(getattr(pose, "y_m", 0.0)),
            vio_z_m=float(getattr(pose, "z_m", 0.0)),
            yaw_offset_deg=_wrap_deg(attitude["yaw_deg"] - _pose_yaw_deg(pose)),
        )

    def _compute_metrics(
        self,
        fc_state: FlightControllerTelemetry,
        pose,
        imu_sample,
        imu_expected: bool,
        now_s: float,
    ) -> dict:
        self._try_set_baseline(fc_state, pose)

        local = _local_position_dict(fc_state)
        attitude = _attitude_deg(fc_state)
        pose_info = _pose_dict(pose)
        baseline = self.state.baseline

        pose_ok = str(pose_info["tracking"]).startswith("ok") and all(
            _finite(pose_info[name])
            for name in ("x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s")
        )
        mavlink_ok = mavlink_heartbeat_valid(fc_state)
        gps_ok = _any_gps_healthy(fc_state)
        imu_ok = (not imu_expected) or imu_sample is not None
        range_ok = rangefinder_height_valid(fc_state)

        drift_xy_m = 0.0
        altitude_error_m = 0.0
        scale_ratio = None
        scale_confidence = 0.0
        if baseline is not None and local is not None:
            ref_dx = local["x_m"] - baseline.local_x_m
            ref_dy = local["y_m"] - baseline.local_y_m
            ref_dz = local["z_m"] - baseline.local_z_m
            vio_dx = float(pose_info["x_m"]) - baseline.vio_x_m
            vio_dy = float(pose_info["y_m"]) - baseline.vio_y_m
            vio_dz = float(pose_info["z_m"]) - baseline.vio_z_m
            drift_xy_m = math.hypot(vio_dx - ref_dx, vio_dy - ref_dy)
            altitude_error_m = abs(vio_dz - ref_dz)
            ref_dist = math.hypot(ref_dx, ref_dy)
            vio_dist = math.hypot(vio_dx, vio_dy)
            if ref_dist > 0.5 and vio_dist > 0.05:
                scale_ratio = vio_dist / ref_dist
                scale_confidence = max(0.0, min(1.0, 1.0 - abs(scale_ratio - 1.0)))

        if range_ok:
            altitude_error_m = max(
                altitude_error_m,
                abs(abs(float(pose_info["z_m"])) - float(fc_state.rangefinder_distance_m or 0.0)),
            )

        velocity_drift_m_s = 0.0
        if local is not None:
            velocity_drift_m_s = math.sqrt(
                (float(pose_info["vx_m_s"]) - local["vx_m_s"]) ** 2
                + (float(pose_info["vy_m_s"]) - local["vy_m_s"]) ** 2
                + (float(pose_info["vz_m_s"]) - local["vz_m_s"]) ** 2
            )

        yaw_error_deg = 0.0
        if attitude is not None and baseline is not None:
            yaw_error_deg = abs(
                _wrap_deg(attitude["yaw_deg"] - float(pose_info["yaw_deg"]) - baseline.yaw_offset_deg)
            )

        timestamp_stable = True
        timestamp_us = int(getattr(pose, "timestamp_us", 0) or 0)
        if self.state.last_pose_timestamp_us is not None and timestamp_us > 0:
            dt_s = (timestamp_us - self.state.last_pose_timestamp_us) / 1_000_000.0
            self.state.last_pose_dt_s = dt_s
            timestamp_stable = 0.0 < dt_s <= 0.5
        self.state.last_pose_timestamp_us = timestamp_us or self.state.last_pose_timestamp_us

        quality = float(pose_info["quality"])
        duration_s = max(0.0, now_s - self.state.started_s)
        confidence = min(1.0, duration_s / 60.0)

        # Start optimistic, then subtract for each missing or inconsistent
        # signal. This makes the score easy to audit in logs: low score means
        # something concrete was missing, drifting, stale, or unhealthy.
        score = 10.0
        if not pose_ok:
            score -= 5.0
        if local is None:
            score -= 2.5
        if attitude is None:
            score -= 1.0
        if not mavlink_ok:
            score -= 2.0
        if not gps_ok:
            score -= 1.0
        if not imu_ok:
            score -= 0.8
        if fc_state.ekf_flags is None:
            score -= 0.5
        if not timestamp_stable:
            score -= 0.8
        score -= min(3.0, drift_xy_m * 2.0)
        score -= min(1.5, velocity_drift_m_s * 0.75)
        score -= min(1.5, yaw_error_deg / 20.0 * 1.5)
        score -= min(1.0, altitude_error_m)
        if scale_ratio is not None:
            score -= min(1.5, abs(scale_ratio - 1.0) * 3.0)
        score -= min(1.5, max(0.0, 60.0 - quality) / 60.0 * 1.5)
        score = max(0.0, min(10.0, score))
        score *= 0.4 + 0.6 * confidence
        score = max(0.0, min(10.0, score))

        self.state.quality_score = score
        self.state.best_score = max(self.state.best_score, score)
        self.state.drift_xy_m = drift_xy_m
        self.state.velocity_drift_m_s = velocity_drift_m_s
        self.state.yaw_error_deg = yaw_error_deg
        self.state.altitude_error_m = altitude_error_m
        self.state.scale_ratio = scale_ratio
        self.state.scale_confidence = scale_confidence
        self.state.soft_calibration_confidence = confidence
        self.state.current_recommendation = self._recommendation(score)
        if self.state.current_recommendation == "ready_for_no_gps_poshold":
            self.state.last_ready_s = now_s
        self._update_soft_correction(fc_state, pose_info, local, attitude, score, scale_ratio, pose_ok, now_s)

        return self._summary(fc_state, pose, imu_sample, self.state.current_recommendation)

    def _compute_poshold_metrics(
        self,
        fc_state: FlightControllerTelemetry,
        pose,
        imu_sample,
        imu_expected: bool,
        now_s: float,
    ) -> dict:
        self.state.active = False
        self.state.baseline = None
        pose_info = _pose_dict(pose)
        pose_ok = str(pose_info["tracking"]).startswith("ok") and all(
            _finite(pose_info[name])
            for name in ("x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s")
        )
        mavlink_ok = mavlink_heartbeat_valid(fc_state)
        imu_ok = (not imu_expected) or imu_sample is not None
        timestamp_stable = True
        timestamp_us = int(getattr(pose, "timestamp_us", 0) or 0)
        if self.state.last_pose_timestamp_us is not None and timestamp_us > 0:
            dt_s = (timestamp_us - self.state.last_pose_timestamp_us) / 1_000_000.0
            self.state.last_pose_dt_s = dt_s
            timestamp_stable = 0.0 < dt_s <= 0.5
        self.state.last_pose_timestamp_us = timestamp_us or self.state.last_pose_timestamp_us

        runtime_score = max(0.0, float(self.state.quality_score))
        if runtime_score <= 0.0 and pose_ok and mavlink_ok:
            runtime_score = 3.0 + min(3.0, float(pose_info["quality"]) / 100.0 * 3.0)
            if timestamp_stable:
                runtime_score += 0.5
            if imu_ok:
                runtime_score += 0.3
            runtime_score = min(runtime_score, self.config.min_quality_for_poshold - 0.1)
        if not pose_ok:
            runtime_score = min(runtime_score, 1.5)
        if float(pose_info["quality"]) < 35.0:
            runtime_score = min(runtime_score, 2.5)
        if not timestamp_stable:
            runtime_score = min(runtime_score, 2.8)
        if not mavlink_ok:
            runtime_score = min(runtime_score, 2.0)
        if not imu_ok:
            runtime_score = min(runtime_score, 3.0)
        if rangefinder_height_valid(fc_state) and abs(abs(float(pose_info["z_m"])) - float(fc_state.rangefinder_distance_m or 0.0)) > 1.5:
            runtime_score = min(runtime_score, 2.9)

        self.state.quality_score = max(0.0, min(10.0, runtime_score))
        recommendation = self._recommendation(self.state.quality_score)
        if recommendation != "critical" and self.state.last_ready_s > 0.0:
            recommendation = self._stored_recommendation()
        self.state.current_recommendation = recommendation
        return self._summary(fc_state, pose, imu_sample, recommendation)

    def _summary(self, fc_state: FlightControllerTelemetry, pose, imu_sample, recommendation: str) -> dict:
        return {
            "enabled": self.config.enable_loiter_observation,
            "active": self.state.active,
            "mode": fc_state.flight_mode,
            "score": round(float(self.state.quality_score), 2),
            "best_score": round(float(self.state.best_score), 2),
            "samples": self.state.sample_count,
            "drift_xy_m": round(float(self.state.drift_xy_m), 3),
            "velocity_drift_m_s": round(float(self.state.velocity_drift_m_s), 3),
            "yaw_error_deg": round(float(self.state.yaw_error_deg), 2),
            "altitude_error_m": round(float(self.state.altitude_error_m), 3),
            "scale_ratio": None if self.state.scale_ratio is None else round(float(self.state.scale_ratio), 3),
            "scale_confidence": round(float(self.state.scale_confidence), 2),
            "soft_calibration_confidence": round(float(self.state.soft_calibration_confidence), 2),
            "recommendation": recommendation,
            "live_soft_correction": self.config.enable_live_soft_correction,
            "auto_fallback_to_loiter": self.config.enable_auto_fallback_to_loiter,
            "correction": {
                "valid": self.state.correction_valid,
                "samples": self.state.correction_samples,
                "yaw_offset_deg": round(float(self.state.correction_yaw_offset_deg), 2),
                "scale_xy": round(float(self.state.correction_scale_xy), 4),
                "x_offset_m": round(float(self.state.correction_x_offset_m), 3),
                "y_offset_m": round(float(self.state.correction_y_offset_m), 3),
                "last_update_s": round(float(self.state.last_correction_update_s), 3),
            },
            "gps1_fix_type": fc_state.gps_fix_type,
            "gps1_satellites": fc_state.gps_satellites,
            "gps2_fix_type": fc_state.gps2_fix_type,
            "gps2_satellites": fc_state.gps2_satellites,
            "vio_tracking": None if pose is None else getattr(pose, "tracking_state", None),
            "vio_quality": None if pose is None else getattr(pose, "pose_quality", None),
            "imu": "present" if imu_sample is not None else "missing",
        }

    def _stored_recommendation(self) -> str:
        if self.state.last_ready_s > 0.0 and self.state.quality_score >= self.config.min_quality_for_poshold:
            return "ready_for_no_gps_poshold"
        if self.state.quality_score > 0.0:
            return self._recommendation(self.state.quality_score)
        return "inactive"

    def _recommendation(self, score: float) -> str:
        if score < self.config.critical_quality_threshold:
            return "critical"
        if score < self.config.weak_quality_threshold:
            return "weak"
        if score < self.config.min_quality_for_poshold:
            return "observe_longer"
        return "ready_for_no_gps_poshold"

    def _quality_bucket(self, score: float) -> str:
        if score < self.config.critical_quality_threshold:
            return "critical"
        if score < self.config.weak_quality_threshold:
            return "weak"
        if score < self.config.min_quality_for_poshold:
            return "low"
        if score < 9.0:
            return "good"
        return "excellent"

    def _quality_message(self, score: float) -> str:
        if score < self.config.critical_quality_threshold:
            return f"SLAM quality critical: {score:.1f}/10. Fallback to LOITER recommended."
        if score < self.config.weak_quality_threshold:
            return f"SLAM quality weak: {score:.1f}/10. Use LOITER longer or run calibration."
        if score < self.config.min_quality_for_poshold:
            return (
                f"SLAM quality low: {score:.1f}/10. Use LOITER observation or run calibration before No-GPS PosHold."
            )
        if score < 9.0:
            return f"SLAM quality ready for No-GPS PosHold: {score:.1f}/10"
        return f"SLAM quality ready for No-GPS PosHold: {score:.1f}/10"

    def soft_correction_ready(self) -> bool:
        return (
            self.config.enable_live_soft_correction
            and self.state.correction_valid
            and self.state.quality_score >= self.config.min_quality_for_poshold
        )

    def apply_soft_correction(self, pose):
        """Return a pose adjusted by the learned LOITER correction.

        The correction is intentionally bounded and only used when the observer
        says quality is high enough. This prevents one bad LOITER sample from
        becoming a large GPS2 jump in POSHOLD.
        """

        if not self.soft_correction_ready():
            return pose
        scale = max(0.75, min(1.25, float(self.state.correction_scale_xy)))
        yaw_offset = max(-35.0, min(35.0, float(self.state.correction_yaw_offset_deg)))
        x_scaled = float(pose.x_m) * scale
        y_scaled = float(pose.y_m) * scale
        vx_scaled = float(pose.vx_m_s) * scale
        vy_scaled = float(pose.vy_m_s) * scale
        x_rot, y_rot = rotate_xy(x_scaled, y_scaled, yaw_offset)
        vx_rot, vy_rot = rotate_xy(vx_scaled, vy_scaled, yaw_offset)
        corrected_quaternion = quaternion_multiply(
            quaternion_from_yaw_deg(yaw_offset),
            (pose.qw, pose.qx, pose.qy, pose.qz),
        )
        source_name = f"{pose.source_name}+soft" if getattr(pose, "source_name", "") else "soft"
        try:
            return replace(
                pose,
                x_m=x_rot + self.state.correction_x_offset_m,
                y_m=y_rot + self.state.correction_y_offset_m,
                qw=corrected_quaternion[0],
                qx=corrected_quaternion[1],
                qy=corrected_quaternion[2],
                qz=corrected_quaternion[3],
                vx_m_s=vx_rot,
                vy_m_s=vy_rot,
                source_name=source_name,
            )
        except TypeError:
            pose.x_m = x_rot + self.state.correction_x_offset_m
            pose.y_m = y_rot + self.state.correction_y_offset_m
            pose.qw, pose.qx, pose.qy, pose.qz = corrected_quaternion
            pose.vx_m_s = vx_rot
            pose.vy_m_s = vy_rot
            pose.source_name = source_name
            return pose

    def _update_soft_correction(
        self,
        fc_state: FlightControllerTelemetry,
        pose_info: dict[str, Any],
        local: dict[str, float] | None,
        attitude: dict[str, float] | None,
        score: float,
        scale_ratio: float | None,
        pose_ok: bool,
        now_s: float,
    ) -> None:
        if not self.config.enable_live_soft_correction:
            return
        # Live correction learns only from stable GPS-assisted LOITER data. If
        # the estimate is weak or geometrically implausible, keep the previous
        # correction instead of chasing noisy measurements.
        if not pose_ok or local is None or attitude is None:
            return
        if score < self.config.weak_quality_threshold:
            return
        yaw_offset = _wrap_deg(float(attitude["yaw_deg"]) - float(pose_info["yaw_deg"]))
        if abs(yaw_offset) > 35.0:
            return
        scale_xy = self.state.correction_scale_xy
        if scale_ratio is not None and scale_ratio > 0.01:
            scale_xy = 1.0 / float(scale_ratio)
        if not 0.75 <= scale_xy <= 1.25:
            return
        scaled_x = float(pose_info["x_m"]) * scale_xy
        scaled_y = float(pose_info["y_m"]) * scale_xy
        rotated_x, rotated_y = rotate_xy(scaled_x, scaled_y, yaw_offset)
        x_offset = float(local["x_m"]) - rotated_x
        y_offset = float(local["y_m"]) - rotated_y

        if not self.state.correction_valid:
            self.state.correction_yaw_offset_deg = yaw_offset
            self.state.correction_scale_xy = scale_xy
            self.state.correction_x_offset_m = x_offset
            self.state.correction_y_offset_m = y_offset
            self.state.correction_valid = True
        else:
            alpha = 0.06
            yaw_delta = _wrap_deg(yaw_offset - self.state.correction_yaw_offset_deg)
            self.state.correction_yaw_offset_deg = _wrap_deg(
                self.state.correction_yaw_offset_deg + yaw_delta * alpha
            )
            self.state.correction_scale_xy += (scale_xy - self.state.correction_scale_xy) * alpha
            self.state.correction_x_offset_m += (x_offset - self.state.correction_x_offset_m) * alpha
            self.state.correction_y_offset_m += (y_offset - self.state.correction_y_offset_m) * alpha
        self.state.correction_samples += 1
        self.state.last_correction_update_s = now_s

    def _maybe_send_loiter_messages(self, master, metrics: dict, now_s: float) -> None:
        interval_s = max(self.config.observation_message_interval_sec, 1.0)
        score = float(metrics.get("score", 0.0))
        bucket = self._quality_bucket(score)

        if now_s - self.state.last_message_s >= interval_s:
            self._send(master, "SLAM observing LOITER data for soft calibration.")
            self._send(master, self._quality_message(score), severity=self._severity_for_score(score))
            self.state.last_message_s = now_s
            self.state.last_quality_message_s = now_s
            self.state.last_reported_score = score
            self.state.last_quality_bucket = bucket
            return

        crossed_threshold = bool(self.state.last_quality_bucket and bucket != self.state.last_quality_bucket)
        changed_enough = (
            self.state.last_reported_score is None
            or abs(score - self.state.last_reported_score) >= self.config.quality_update_delta
        )
        if (crossed_threshold or changed_enough) and now_s - self.state.last_quality_message_s >= 5.0:
            if bucket == "good":
                self._send(master, f"SLAM quality good: {score:.1f}/10")
            elif bucket == "excellent":
                self._send(master, f"SLAM quality improving: {score:.1f}/10")
            else:
                self._send(master, self._quality_message(score), severity=self._severity_for_score(score))
            self.state.last_quality_message_s = now_s
            self.state.last_reported_score = score
            self.state.last_quality_bucket = bucket

    def _maybe_warn_or_fallback(
        self,
        master,
        fc_state: FlightControllerTelemetry,
        metrics: dict,
        now_s: float,
    ) -> None:
        mode = str(fc_state.flight_mode or "").upper()
        if mode != "POSHOLD" or not fc_state.armed:
            return
        score = float(metrics.get("score", self.state.quality_score))
        if score >= self.config.critical_quality_threshold:
            return
        if now_s - self.state.fallback_warned_s < 5.0:
            return
        if self.config.enable_auto_fallback_to_loiter and _gps1_healthy(fc_state):
            self._send(master, "SLAM quality critical: switching to LOITER.", severity=mavutil.mavlink.MAV_SEVERITY_WARNING)
            if not self.dry_run:
                set_vehicle_mode(master, "LOITER")
        else:
            self._send(master, "SLAM quality critical: fallback to LOITER recommended.", severity=mavutil.mavlink.MAV_SEVERITY_WARNING)
        self.state.fallback_warned_s = now_s

    def _severity_for_score(self, score: float) -> int:
        if score < self.config.critical_quality_threshold:
            return mavutil.mavlink.MAV_SEVERITY_ERROR
        if score < self.config.weak_quality_threshold:
            return mavutil.mavlink.MAV_SEVERITY_WARNING
        return mavutil.mavlink.MAV_SEVERITY_INFO

    def _send(self, master, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_INFO) -> None:
        if self.dry_run or master is None:
            return
        try:
            send_gcs_event(master, text, severity=severity)
        except Exception:
            pass

    def _log_payload(self, fc_state: FlightControllerTelemetry, pose, imu_sample, metrics: dict) -> dict:
        local = _local_position_dict(fc_state)
        attitude = _attitude_deg(fc_state)
        return {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "flight_mode": fc_state.flight_mode,
            "armed": fc_state.armed,
            "gps_position": {
                "gps1_lat": fc_state.gps_lat,
                "gps1_lon": fc_state.gps_lon,
                "gps1_alt_mm": fc_state.gps_alt_mm,
                "gps2_lat": fc_state.gps2_lat,
                "gps2_lon": fc_state.gps2_lon,
                "gps2_alt_mm": fc_state.gps2_alt_mm,
                "global_lat": fc_state.global_lat,
                "global_lon": fc_state.global_lon,
                "global_alt_mm": fc_state.global_alt_mm,
                "global_relative_alt_mm": fc_state.global_relative_alt_mm,
            },
            "gps_velocity": {
                "gps1_vel_cm_s": fc_state.gps_vel_cm_s,
                "gps1_cog_cd": fc_state.gps_cog_cd,
                "global_vx_cm_s": fc_state.global_vx_cm_s,
                "global_vy_cm_s": fc_state.global_vy_cm_s,
                "global_vz_cm_s": fc_state.global_vz_cm_s,
                "local_vx_m_s": None if local is None else local["vx_m_s"],
                "local_vy_m_s": None if local is None else local["vy_m_s"],
                "local_vz_m_s": None if local is None else local["vz_m_s"],
            },
            "ekf_local_position": local,
            "vio_position": _pose_dict(pose),
            "attitude": attitude,
            "throttle": {
                "vfr_throttle_pct": fc_state.vfr_throttle_pct,
                "rc_throttle_raw": fc_state.rc_channels.get(3),
            },
            "rangefinder_altitude_m": fc_state.rangefinder_distance_m,
            "barometer_altitude_m": fc_state.baro_alt_m,
            "vfr_altitude_m": fc_state.vfr_alt_m,
            "ekf_status": {
                "flags": fc_state.ekf_flags,
                "status_text": fc_state.status_text,
                "status_severity": fc_state.status_severity,
            },
            "imu": None
            if imu_sample is None
            else {
                "roll_deg": getattr(imu_sample, "roll_deg", None),
                "pitch_deg": getattr(imu_sample, "pitch_deg", None),
                "yaw_deg": getattr(imu_sample, "yaw_deg", None),
                "gx_deg_s": getattr(imu_sample, "gx_deg_s", None),
                "gy_deg_s": getattr(imu_sample, "gy_deg_s", None),
                "gz_deg_s": getattr(imu_sample, "gz_deg_s", None),
                "altitude_m": getattr(imu_sample, "altitude_m", None),
            },
            "mavlink_health": "ok" if mavlink_heartbeat_valid(fc_state) else "timeout",
            "sensor_health": {
                "gps_reference": "ok" if _any_gps_healthy(fc_state) else "bad",
                "rangefinder": "ok" if rangefinder_height_valid(fc_state) else "bad",
                "imu": "ok" if imu_sample is not None else "missing",
                "vio": "ok" if str(getattr(pose, "tracking_state", "")).startswith("ok") else "bad",
            },
            "drift_estimate": {
                "xy_m": metrics.get("drift_xy_m"),
                "velocity_m_s": metrics.get("velocity_drift_m_s"),
                "yaw_deg": metrics.get("yaw_error_deg"),
                "altitude_m": metrics.get("altitude_error_m"),
            },
            "slam_quality_score": metrics.get("score"),
            "current_recommendation": metrics.get("recommendation"),
        }

    def _maybe_log(self, fc_state: FlightControllerTelemetry, pose, imu_sample, metrics: dict, now_s: float) -> None:
        if not self.config.log_observation_data or not self.config.log_path:
            return
        if now_s - self.state.last_logged_s < 1.0:
            return
        path = Path(self.config.log_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(self._log_payload(fc_state, pose, imu_sample, metrics), sort_keys=True) + "\n")
        self.state.last_logged_s = now_s

    def _maybe_write_status(self, fc_state, pose, imu_sample, metrics, now_s: float) -> None:
        if now_s - self.state.last_status_write_s < 1.0:
            return
        self._write_status(metrics)
        self.state.last_status_write_s = now_s

    def _write_status(self, payload: dict) -> None:
        if not self.config.status_path:
            return
        path = Path(self.config.status_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
