"""Support layer for the legacy RealSense optical-flow PosHold bridge.

This module deliberately stays small and boring. The legacy bridge already had
the PosHold behavior that worked in the field, so the additions here only add
observation, logging, external-IMU attitude adaptation, bounded soft correction,
and Brake-mode calibration bookkeeping around that older control path.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from pymavlink import mavutil


GPS_EPOCH_UNIX_S = 315964800
GPS_UTC_LEAP_SECONDS = 18
REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class SimpleAttitude:
    roll: float
    pitch: float
    yaw: float


@dataclass
class LegacySoftCorrection:
    enabled: bool = True
    velocity_scale: float = 1.0
    confidence: float = 0.0
    samples: int = 0

    def apply_velocity_ned(self, vn_m_s: float, ve_m_s: float) -> tuple[float, float]:
        if not self.enabled or self.confidence < 0.35:
            return vn_m_s, ve_m_s
        scale = max(0.70, min(1.30, self.velocity_scale))
        return vn_m_s * scale, ve_m_s * scale

    def learn_scale(self, gps_speed_m_s: float, flow_speed_m_s: float) -> None:
        if gps_speed_m_s < 0.15 or flow_speed_m_s < 0.15:
            return
        ratio = max(0.70, min(1.30, gps_speed_m_s / max(flow_speed_m_s, 1e-3)))
        alpha = 0.04 if self.samples > 20 else 0.12
        self.velocity_scale = (1.0 - alpha) * self.velocity_scale + alpha * ratio
        self.confidence = min(1.0, self.confidence + 0.01)
        self.samples += 1

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "velocity_scale": self.velocity_scale,
            "confidence": self.confidence,
            "samples": self.samples,
        }


def gps_week_time(now_s: float | None = None) -> tuple[int, int]:
    unix_s = time.time() if now_s is None else float(now_s)
    gps_s = unix_s - GPS_EPOCH_UNIX_S + GPS_UTC_LEAP_SECONDS
    if gps_s <= 0:
        return 0, 0
    week = int(gps_s // 604800)
    week_ms = int((gps_s - week * 604800) * 1000)
    return week, week_ms


def gps_input_time_fields(now_s: float | None = None) -> tuple[int, int, int]:
    unix_s = time.time() if now_s is None else float(now_s)
    week, week_ms = gps_week_time(unix_s)
    return int(unix_s * 1e6), week, week_ms


def resolve_repo_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    return path if path.is_absolute() else (REPO_ROOT / path).resolve()


def external_imu_attitude(fc_attitude, imu_sample, yaw_mode: str = "fc") -> SimpleAttitude:
    if imu_sample is None:
        return fc_attitude
    fc_yaw = float(getattr(fc_attitude, "yaw", math.radians(imu_sample.yaw_deg))) if fc_attitude is not None else math.radians(imu_sample.yaw_deg)
    yaw = math.radians(imu_sample.yaw_deg) if yaw_mode == "external" else fc_yaw
    return SimpleAttitude(
        roll=math.radians(float(imu_sample.roll_deg)),
        pitch=math.radians(float(imu_sample.pitch_deg)),
        yaw=yaw,
    )


def gps_fix_ok(monitor, min_fix: int = 3, min_sats: int = 6) -> bool:
    gps = getattr(monitor, "gps_raw_int", None)
    if gps is None:
        return False
    return int(getattr(gps, "fix_type", 0) or 0) >= min_fix and int(getattr(gps, "satellites_visible", 0) or 0) >= min_sats


def gps_reference_velocity_m_s(monitor) -> tuple[Optional[float], Optional[float]]:
    gpos = getattr(monitor, "global_position_int", None)
    if gpos is not None:
        return float(getattr(gpos, "vx", 0) or 0) / 100.0, float(getattr(gpos, "vy", 0) or 0) / 100.0
    local = getattr(monitor, "local_position", None)
    if local is not None:
        return float(getattr(local, "vx", 0.0) or 0.0), float(getattr(local, "vy", 0.0) or 0.0)
    return None, None


def gps_reference_offset_m(
    monitor,
    origin_lat_deg: Optional[float],
    origin_lon_deg: Optional[float],
) -> tuple[Optional[float], Optional[float]]:
    if origin_lat_deg is None or origin_lon_deg is None:
        return None, None
    gpos = getattr(monitor, "global_position_int", None)
    gps = getattr(monitor, "gps_raw_int", None)
    lat_raw = getattr(gpos, "lat", None) if gpos is not None else getattr(gps, "lat", None)
    lon_raw = getattr(gpos, "lon", None) if gpos is not None else getattr(gps, "lon", None)
    if lat_raw in (None, 0) or lon_raw in (None, 0):
        return None, None

    lat_deg = float(lat_raw) / 1e7
    lon_deg = float(lon_raw) / 1e7
    earth_radius_m = 6378137.0
    origin_lat_rad = math.radians(origin_lat_deg)
    north_m = math.radians(lat_deg - origin_lat_deg) * earth_radius_m
    east_m = math.radians(lon_deg - origin_lon_deg) * earth_radius_m * max(math.cos(origin_lat_rad), 0.01)
    return north_m, east_m


class LegacyLoiterObserver:
    def __init__(
        self,
        enabled: bool,
        log_path: str,
        message_interval_s: float,
        min_quality_for_poshold: float,
        weak_quality_threshold: float,
        critical_quality_threshold: float,
        live_correction_enabled: bool,
        profile_path: str,
    ):
        self.enabled = enabled
        self.log_path = resolve_repo_path(log_path)
        self.message_interval_s = max(message_interval_s, 5.0)
        self.min_quality_for_poshold = min_quality_for_poshold
        self.weak_quality_threshold = weak_quality_threshold
        self.critical_quality_threshold = critical_quality_threshold
        self.last_message_s = 0.0
        self.last_score_message_s = 0.0
        self.last_score = 0.0
        self.active = False
        self.best_score = 0.0
        self.correction = LegacySoftCorrection(enabled=live_correction_enabled)
        self.profile_path = resolve_repo_path(profile_path)
        self._load_profile()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)

    def _load_profile(self) -> None:
        if not self.profile_path.exists():
            return
        try:
            payload = json.loads(self.profile_path.read_text(encoding="utf-8"))
        except Exception:
            return
        scale = float(payload.get("velocity_scale", self.correction.velocity_scale))
        confidence = float(payload.get("confidence", self.correction.confidence))
        samples = int(payload.get("samples", self.correction.samples))
        self.correction.velocity_scale = max(0.70, min(1.30, scale))
        self.correction.confidence = max(0.0, min(1.0, confidence))
        self.correction.samples = max(0, samples)

    def save_profile(self) -> None:
        self.profile_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            **self.correction.to_dict(),
            "best_score": self.best_score,
        }
        self.profile_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def apply_velocity_ned(self, vn_m_s: float, ve_m_s: float) -> tuple[float, float]:
        return self.correction.apply_velocity_ned(vn_m_s, ve_m_s)

    def update(
        self,
        master,
        monitor,
        flow_state,
        flow,
        distance_m: float,
        flow_health_ok: bool,
        flow_health_reason: str,
        imu_sample,
        origin_lat_deg: Optional[float],
        origin_lon_deg: Optional[float],
        now_s: float,
    ) -> dict:
        if not self.enabled:
            return {"score": 0.0, "recommendation": "observer disabled", "best_score": self.best_score}

        in_loiter = str(getattr(monitor, "flight_mode", "")).upper() == "LOITER"
        if in_loiter and not self.active:
            self.active = True
            self.last_message_s = 0.0
            self._announce(master, "LOITER active: legacy SLAM observation mode started.")
        elif not in_loiter and self.active:
            self.active = False
            self._announce(master, "LOITER observation paused; legacy bridge normal mode monitoring.")

        metrics = self._score(
            monitor,
            flow_state,
            flow,
            distance_m,
            flow_health_ok,
            imu_sample,
            origin_lat_deg,
            origin_lon_deg,
        )
        score = metrics["score"]
        self.best_score = max(self.best_score, score)

        if in_loiter:
            self._learn_from_loiter(monitor, flow_state, flow_health_ok)
            if now_s - self.last_message_s >= self.message_interval_s:
                self._announce(master, "SLAM observing LOITER data for soft calibration.")
                self._announce(master, self._score_message(score))
                self.last_message_s = now_s
            if abs(score - self.last_score) >= 0.5 and now_s - self.last_score_message_s >= 5.0:
                self._announce(master, self._score_message(score))
                self.last_score_message_s = now_s
                self.last_score = score
            self._log(metrics, monitor, flow_state, flow_health_reason, imu_sample, now_s)

        recommendation = self._recommendation(score)
        return {"score": score, "recommendation": recommendation, "best_score": self.best_score}

    def _learn_from_loiter(self, monitor, flow_state, flow_health_ok: bool) -> None:
        if not flow_health_ok:
            return
        gps_vn, gps_ve = gps_reference_velocity_m_s(monitor)
        if gps_vn is None or gps_ve is None:
            return
        gps_speed = math.hypot(gps_vn, gps_ve)
        flow_speed = math.hypot(float(flow_state.last_vn_m_s), float(flow_state.last_ve_m_s))
        self.correction.learn_scale(gps_speed, flow_speed)
        if self.correction.samples % 25 == 0:
            self.save_profile()

    def _score(
        self,
        monitor,
        flow_state,
        flow,
        distance_m: float,
        flow_health_ok: bool,
        imu_sample,
        origin_lat_deg: Optional[float],
        origin_lon_deg: Optional[float],
    ) -> dict:
        score = 0.0
        score += min(2.0, max(0.0, float(getattr(flow, "quality", 0) or 0) / 255.0 * 2.0))
        score += min(1.5, max(0.0, float(getattr(flow, "tracks", 0) or 0) / 120.0 * 1.5))
        score += 1.0 if distance_m > 0.0 else 0.0
        score += 1.0 if imu_sample is not None else 0.0
        score += 1.0 if flow_health_ok else 0.0
        score += 1.0 if gps_fix_ok(monitor) else 0.0

        gps_n, gps_e = gps_reference_offset_m(monitor, origin_lat_deg, origin_lon_deg)
        drift_m = None
        if gps_n is not None and gps_e is not None:
            drift_m = math.hypot(gps_n - float(flow_state.north_m), gps_e - float(flow_state.east_m))
            if drift_m <= 0.5:
                score += 1.2
            elif drift_m <= 1.5:
                score += 0.7
            elif drift_m <= 3.0:
                score += 0.3

        gps_vn, gps_ve = gps_reference_velocity_m_s(monitor)
        velocity_drift_m_s = None
        if gps_vn is not None and gps_ve is not None:
            velocity_drift_m_s = math.hypot(gps_vn - float(flow_state.last_vn_m_s), gps_ve - float(flow_state.last_ve_m_s))
            if velocity_drift_m_s <= 0.25:
                score += 1.3
            elif velocity_drift_m_s <= 0.6:
                score += 0.8
            elif velocity_drift_m_s <= 1.0:
                score += 0.4

        ekf_status = getattr(monitor, "ekf_status", None)
        if ekf_status is not None:
            variances = [
                float(getattr(ekf_status, "velocity_variance", 99.0) or 99.0),
                float(getattr(ekf_status, "pos_horiz_variance", 99.0) or 99.0),
            ]
            if max(variances) < 0.4:
                score += 1.0
            elif max(variances) < 0.8:
                score += 0.5

        score = max(0.0, min(10.0, score))
        return {
            "score": score,
            "drift_m": drift_m,
            "velocity_drift_m_s": velocity_drift_m_s,
            "velocity_scale": self.correction.velocity_scale,
            "correction_confidence": self.correction.confidence,
        }

    def _recommendation(self, score: float) -> str:
        if score >= self.min_quality_for_poshold:
            return f"ready for No-GPS PosHold: {score:.1f}/10"
        if score < self.critical_quality_threshold:
            return f"critical: fallback to LOITER recommended: {score:.1f}/10"
        if score < self.weak_quality_threshold:
            return f"weak: use LOITER longer or Brake calibration: {score:.1f}/10"
        return f"usable but needs more observation: {score:.1f}/10"

    def _score_message(self, score: float) -> str:
        if score >= self.min_quality_for_poshold:
            return f"SLAM quality ready for No-GPS PosHold: {score:.1f}/10"
        if score < self.critical_quality_threshold:
            return f"SLAM quality critical: {score:.1f}/10. Fallback to LOITER recommended."
        if score < self.weak_quality_threshold:
            return f"SLAM quality weak: {score:.1f}/10. Use LOITER longer or run calibration."
        return f"SLAM quality improving: {score:.1f}/10"

    def _announce(self, master, text: str, severity=mavutil.mavlink.MAV_SEVERITY_NOTICE) -> None:
        try:
            master.mav.statustext_send(severity, f"LEGACY SLAM: {text}"[:50].encode("utf-8"))
        except Exception:
            pass
        print(f"LEGACY SLAM: {text}")

    def _log(self, metrics, monitor, flow_state, flow_health_reason: str, imu_sample, now_s: float) -> None:
        payload = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now_s)),
            "mode": getattr(monitor, "flight_mode", "UNKNOWN"),
            "armed": bool(getattr(monitor, "armed", False)),
            "gps_fix": int(getattr(getattr(monitor, "gps_raw_int", None), "fix_type", 0) or 0),
            "gps_sats": int(getattr(getattr(monitor, "gps_raw_int", None), "satellites_visible", 0) or 0),
            "flow_n_m": flow_state.north_m,
            "flow_e_m": flow_state.east_m,
            "flow_vn_m_s": flow_state.last_vn_m_s,
            "flow_ve_m_s": flow_state.last_ve_m_s,
            "flow_reason": flow_health_reason,
            "imu": "present" if imu_sample is not None else "missing",
            **metrics,
            "recommendation": self._recommendation(metrics["score"]),
        }
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")


class LegacyBrakeCalibrator:
    def __init__(
        self,
        enabled: bool,
        duration_s: float,
        min_samples: int,
        profile_path: str,
    ):
        self.enabled = enabled
        self.duration_s = max(duration_s, 3.0)
        self.min_samples = max(min_samples, 10)
        self.profile_path = resolve_repo_path(profile_path)
        self.active = False
        self.start_s = 0.0
        self.samples = 0
        self.completed = False
        self.last_collect_message_s = 0.0

    def update(
        self,
        master,
        monitor,
        flow_health_ok: bool,
        observer: LegacyLoiterObserver,
        now_s: float,
    ) -> None:
        if not self.enabled:
            return
        in_brake = str(getattr(monitor, "flight_mode", "")).upper() == "BRAKE"
        if in_brake and not self.active and not self.completed:
            self.active = True
            self.start_s = now_s
            self.samples = 0
            self.last_collect_message_s = 0.0
            self._announce(master, "BRAKE selected: legacy flow calibration started.")
        if not in_brake:
            if self.active:
                self._announce(master, "BRAKE calibration paused; mode changed.")
            self.active = False
            self.completed = False
            return
        if not self.active:
            return
        if flow_health_ok:
            self.samples += 1
        elapsed_s = now_s - self.start_s
        if elapsed_s >= self.duration_s and self.samples >= self.min_samples:
            observer.save_profile()
            self._write_profile(observer)
            self.completed = True
            self.active = False
            self._announce(
                master,
                f"BRAKE calibration complete: scale={observer.correction.velocity_scale:.2f} confidence={observer.correction.confidence:.2f}.",
            )
        elif self.samples > 0 and now_s - self.last_collect_message_s >= 10.0:
            self._announce(master, f"BRAKE calibration collecting: {self.samples} samples.")
            self.last_collect_message_s = now_s

    def _write_profile(self, observer: LegacyLoiterObserver) -> None:
        self.profile_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "samples": self.samples,
            "velocity_scale": observer.correction.velocity_scale,
            "confidence": observer.correction.confidence,
            "best_observer_score": observer.best_score,
        }
        self.profile_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _announce(self, master, text: str) -> None:
        try:
            master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, f"LEGACY SLAM: {text}"[:50].encode("utf-8"))
        except Exception:
            pass
        print(f"LEGACY SLAM: {text}")
