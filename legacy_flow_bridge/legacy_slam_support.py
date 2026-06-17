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
STATUSTEXT_LIMIT = 50


def send_gcs(master, text: str, severity=mavutil.mavlink.MAV_SEVERITY_NOTICE, direct_sender=None) -> None:
    """Send compact GCS text that survives MAVLink STATUSTEXT truncation."""
    payload = f"LGC {text}"[:STATUSTEXT_LIMIT]
    try:
        master.mav.statustext_send(severity, payload.encode("utf-8"))
    except Exception:
        pass
    if direct_sender is not None:
        try:
            direct_sender(payload, severity)
        except Exception:
            pass
    print(f"LEGACY SLAM: {text}")


def wrap_angle_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


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
        min_ready_health_s: float,
        live_correction_enabled: bool,
        profile_path: str,
        direct_status_sender=None,
    ):
        self.enabled = enabled
        self.log_path = resolve_repo_path(log_path)
        self.message_interval_s = max(message_interval_s, 5.0)
        self.min_quality_for_poshold = min_quality_for_poshold
        self.weak_quality_threshold = weak_quality_threshold
        self.critical_quality_threshold = critical_quality_threshold
        self.min_ready_health_s = max(0.0, min_ready_health_s)
        self.last_message_s = 0.0
        self.last_score_message_s = 0.0
        self.last_score = 0.0
        self.active = False
        self.best_score = 0.0
        self.health_started_s: Optional[float] = None
        self.health_ok_duration_s = 0.0
        self.correction = LegacySoftCorrection(enabled=live_correction_enabled)
        self.direct_status_sender = direct_status_sender
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
            self._announce(master, "OBS ON: LOITER learning")
        elif not in_loiter and self.active:
            self.active = False
            self._announce(master, "OBS PAUSE: mode changed")

        self._update_health_streak(in_loiter, flow_health_ok, now_s)
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
        ready_for_poshold = (
            score >= self.min_quality_for_poshold
            and flow_health_ok
            and self.health_ok_duration_s >= self.min_ready_health_s
        )

        if in_loiter:
            self._learn_from_loiter(monitor, flow_state, flow_health_ok)
            if now_s - self.last_message_s >= self.message_interval_s:
                self._announce(master, self._score_message(score, ready_for_poshold))
                self.last_message_s = now_s
            if abs(score - self.last_score) >= 0.5 and now_s - self.last_score_message_s >= 5.0:
                self._announce(master, self._score_message(score, ready_for_poshold))
                self.last_score_message_s = now_s
                self.last_score = score
            self._log(
                metrics,
                monitor,
                flow_state,
                flow_health_ok,
                flow_health_reason,
                imu_sample,
                now_s,
            )

        recommendation = self._recommendation(score, ready_for_poshold)
        return {
            "score": score,
            "recommendation": recommendation,
            "best_score": self.best_score,
            "ready_for_poshold": ready_for_poshold,
            "health_ok_duration_s": self.health_ok_duration_s,
        }

    def _update_health_streak(self, in_loiter: bool, flow_health_ok: bool, now_s: float) -> None:
        if not in_loiter or not flow_health_ok:
            self.health_started_s = None
            self.health_ok_duration_s = 0.0
            return
        if self.health_started_s is None:
            self.health_started_s = now_s
        self.health_ok_duration_s = max(0.0, now_s - self.health_started_s)

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

        if not flow_health_ok:
            score = min(score, self.weak_quality_threshold - 0.1)

        score = max(0.0, min(10.0, score))
        return {
            "score": score,
            "drift_m": drift_m,
            "velocity_drift_m_s": velocity_drift_m_s,
            "velocity_scale": self.correction.velocity_scale,
            "correction_confidence": self.correction.confidence,
            "health_ok_duration_s": self.health_ok_duration_s,
        }

    def _recommendation(self, score: float, ready_for_poshold: bool) -> str:
        if ready_for_poshold:
            return f"ready for No-GPS PosHold: {score:.1f}/10"
        if score >= self.min_quality_for_poshold:
            return (
                "not ready: waiting for stable flow/range "
                f"{self.health_ok_duration_s:.1f}/{self.min_ready_health_s:.1f}s"
            )
        if score < self.critical_quality_threshold:
            return f"critical: fallback to LOITER recommended: {score:.1f}/10"
        if score < self.weak_quality_threshold:
            return f"weak: use LOITER longer or Brake calibration: {score:.1f}/10"
        return f"usable but needs more observation: {score:.1f}/10"

    def _score_message(self, score: float, ready_for_poshold: bool) -> str:
        if ready_for_poshold:
            return f"OBS READY q={score:.1f} stable={self.health_ok_duration_s:.0f}s"
        if score >= self.min_quality_for_poshold:
            return (
                f"OBS WAIT q={score:.1f} stable "
                f"{self.health_ok_duration_s:.0f}/{self.min_ready_health_s:.0f}s"
            )
        if score < self.critical_quality_threshold:
            return f"OBS CRIT q={score:.1f}; use LOITER"
        if score < self.weak_quality_threshold:
            return f"OBS WEAK q={score:.1f}; run BRAKE cal"
        return f"OBS q={score:.1f}; keep LOITER"

    def _announce(self, master, text: str, severity=mavutil.mavlink.MAV_SEVERITY_NOTICE) -> None:
        send_gcs(master, text, severity, self.direct_status_sender)

    def _log(
        self,
        metrics,
        monitor,
        flow_state,
        flow_health_ok: bool,
        flow_health_reason: str,
        imu_sample,
        now_s: float,
    ) -> None:
        ready_for_poshold = (
            metrics["score"] >= self.min_quality_for_poshold
            and flow_health_ok
            and self.health_ok_duration_s >= self.min_ready_health_s
        )
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
            "flow_health_ok": bool(flow_health_ok),
            "flow_reason": flow_health_reason,
            "imu": "present" if imu_sample is not None else "missing",
            **metrics,
            "recommendation": self._recommendation(metrics["score"], ready_for_poshold),
        }
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")


class LegacyBrakeCalibrator:
    PHASES = (
        ("HOLD", "hold steady"),
        ("ROLL", "pilot roll L/R"),
        ("PITCH", "pilot pitch F/B"),
        ("YAW", "pilot yaw L/R"),
        ("THR", "pilot climb/desc"),
        ("SAVE", "saving profile"),
    )

    def __init__(
        self,
        enabled: bool,
        duration_s: float,
        min_samples: int,
        profile_path: str,
        direct_status_sender=None,
    ):
        self.enabled = enabled
        self.duration_s = max(duration_s, 12.0)
        self.min_samples = max(min_samples, 10)
        self.profile_path = resolve_repo_path(profile_path)
        self.active = False
        self.start_s = 0.0
        self.phase_started_s = 0.0
        self.phase_index = 0
        self.samples = 0
        self.completed = False
        self.last_phase_message_s = 0.0
        self.last_progress_bucket = -1
        self.phase_goal_s = max(3.0, self.duration_s / max(len(self.PHASES) - 1, 1))
        self.phase_timeout_s = max(8.0, self.phase_goal_s * 2.0)
        self.roll_pos = False
        self.roll_neg = False
        self.pitch_pos = False
        self.pitch_neg = False
        self.yaw_base_deg: Optional[float] = None
        self.yaw_min_delta_deg = 0.0
        self.yaw_max_delta_deg = 0.0
        self.throttle_min: Optional[float] = None
        self.throttle_max: Optional[float] = None
        self.alt_min_m: Optional[float] = None
        self.alt_max_m: Optional[float] = None
        self.flow_bad_samples = 0
        self.direct_status_sender = direct_status_sender
        self.last_wait_message_s = 0.0

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
        armed = bool(getattr(monitor, "armed", False))
        if in_brake and not armed:
            if self.active:
                self._announce(master, "CAL PAUSED: vehicle disarmed")
            self.active = False
            self.completed = False
            if now_s - self.last_wait_message_s >= 5.0:
                self._announce(master, "CAL WAIT ARM: fly, then select BRAKE")
                self.last_wait_message_s = now_s
            return
        if in_brake and not self.active and not self.completed:
            self._start(master, monitor, now_s)
        if not in_brake:
            if self.active:
                self._announce(master, "CAL PAUSED: mode changed")
            self.active = False
            self.completed = False
            return
        if not self.active:
            return

        self._observe(monitor, flow_health_ok)
        if flow_health_ok:
            self.samples += 1
        else:
            self.flow_bad_samples += 1

        elapsed_s = now_s - self.start_s
        phase_elapsed_s = now_s - self.phase_started_s
        progress_pct = self._progress_pct(phase_elapsed_s)
        score = self._calibration_score()
        self._maybe_announce_progress(master, progress_pct, score, phase_elapsed_s, now_s)

        if self._current_phase_complete(phase_elapsed_s):
            self._advance_phase(master, now_s)

        if self.phase_index >= len(self.PHASES) - 1 and self.samples >= self.min_samples:
            observer.save_profile()
            self._write_profile(observer)
            self.completed = True
            self.active = False
            self._announce(
                master,
                (
                    f"CAL DONE 100% score={score:.1f}"
                    f" scale={observer.correction.velocity_scale:.2f}"
                ),
            )
        elif self.phase_index >= len(self.PHASES) - 1 and now_s - self.last_phase_message_s >= 3.0:
            self._announce(master, f"CAL 95% need samples {self.samples}/{self.min_samples}")
            self.last_phase_message_s = now_s

    def _start(self, master, monitor, now_s: float) -> None:
        self.active = True
        self.start_s = now_s
        self.phase_started_s = now_s
        self.phase_index = 0
        self.samples = 0
        self.flow_bad_samples = 0
        self.completed = False
        self.last_phase_message_s = 0.0
        self.last_progress_bucket = -1
        self.last_wait_message_s = 0.0
        self.roll_pos = False
        self.roll_neg = False
        self.pitch_pos = False
        self.pitch_neg = False
        self.yaw_base_deg = self._yaw_deg(monitor)
        self.yaw_min_delta_deg = 0.0
        self.yaw_max_delta_deg = 0.0
        self.throttle_min = None
        self.throttle_max = None
        self.alt_min_m = None
        self.alt_max_m = None
        self._announce(master, "CAL START: BRAKE manual")
        self._announce(master, "CAL 0% HOLD: keep steady")

    def _observe(self, monitor, flow_health_ok: bool) -> None:
        attitude = getattr(monitor, "attitude", None)
        if attitude is not None:
            roll_deg = math.degrees(float(getattr(attitude, "roll", 0.0) or 0.0))
            pitch_deg = math.degrees(float(getattr(attitude, "pitch", 0.0) or 0.0))
            if roll_deg >= 4.0:
                self.roll_pos = True
            if roll_deg <= -4.0:
                self.roll_neg = True
            if pitch_deg >= 4.0:
                self.pitch_pos = True
            if pitch_deg <= -4.0:
                self.pitch_neg = True
            yaw_deg = math.degrees(float(getattr(attitude, "yaw", 0.0) or 0.0))
            if self.yaw_base_deg is None:
                self.yaw_base_deg = yaw_deg
            yaw_delta = wrap_angle_deg(yaw_deg - self.yaw_base_deg)
            self.yaw_min_delta_deg = min(self.yaw_min_delta_deg, yaw_delta)
            self.yaw_max_delta_deg = max(self.yaw_max_delta_deg, yaw_delta)

        throttle = self._throttle_value(monitor)
        if throttle is not None:
            self.throttle_min = throttle if self.throttle_min is None else min(self.throttle_min, throttle)
            self.throttle_max = throttle if self.throttle_max is None else max(self.throttle_max, throttle)

        altitude_m = self._altitude_m(monitor)
        if altitude_m is not None:
            self.alt_min_m = altitude_m if self.alt_min_m is None else min(self.alt_min_m, altitude_m)
            self.alt_max_m = altitude_m if self.alt_max_m is None else max(self.alt_max_m, altitude_m)

    def _current_phase_complete(self, phase_elapsed_s: float) -> bool:
        phase, _hint = self.PHASES[self.phase_index]
        if phase == "HOLD":
            return phase_elapsed_s >= min(4.0, self.phase_goal_s) and self.samples >= 5
        if phase == "ROLL":
            return (self.roll_pos and self.roll_neg) or phase_elapsed_s >= self.phase_timeout_s
        if phase == "PITCH":
            return (self.pitch_pos and self.pitch_neg) or phase_elapsed_s >= self.phase_timeout_s
        if phase == "YAW":
            return self._yaw_span_deg() >= 12.0 or phase_elapsed_s >= self.phase_timeout_s
        if phase == "THR":
            return self._throttle_span() >= 80.0 or self._alt_span_m() >= 0.25 or phase_elapsed_s >= self.phase_timeout_s
        return False

    def _advance_phase(self, master, now_s: float) -> None:
        phase, _hint = self.PHASES[self.phase_index]
        if phase in ("ROLL", "PITCH", "YAW", "THR") and not self._phase_observed(phase):
            self._announce(master, f"CAL {self._progress_pct(0):.0f}% {phase} weak; continuing")
        self.phase_index = min(self.phase_index + 1, len(self.PHASES) - 1)
        self.phase_started_s = now_s
        self.last_phase_message_s = 0.0
        self.last_progress_bucket = -1
        next_phase, hint = self.PHASES[self.phase_index]
        if next_phase != "SAVE":
            self._announce(master, f"CAL {self._progress_pct(0):.0f}% {next_phase}: {hint}")

    def _maybe_announce_progress(
        self,
        master,
        progress_pct: int,
        score: float,
        phase_elapsed_s: float,
        now_s: float,
    ) -> None:
        bucket = int(progress_pct // 10) * 10
        if bucket <= self.last_progress_bucket and now_s - self.last_phase_message_s < 3.0:
            return
        phase, hint = self.PHASES[self.phase_index]
        status = self._phase_status(phase)
        if phase_elapsed_s > self.phase_goal_s and not self._phase_observed(phase):
            status = hint
        self._announce(master, f"CAL {progress_pct}% {phase}: {status} q={score:.1f}")
        self.last_progress_bucket = max(self.last_progress_bucket, bucket)
        self.last_phase_message_s = now_s

    def _progress_pct(self, phase_elapsed_s: float) -> int:
        if self.phase_index >= len(self.PHASES) - 1:
            return 95
        phase_fraction = min(1.0, phase_elapsed_s / max(self.phase_goal_s, 1e-3))
        raw = (self.phase_index + phase_fraction) / max(len(self.PHASES) - 1, 1)
        return max(0, min(95, int(round(raw * 95.0))))

    def _phase_observed(self, phase: str) -> bool:
        if phase == "HOLD":
            return self.samples >= 5
        if phase == "ROLL":
            return self.roll_pos and self.roll_neg
        if phase == "PITCH":
            return self.pitch_pos and self.pitch_neg
        if phase == "YAW":
            return self._yaw_span_deg() >= 12.0
        if phase == "THR":
            return self._throttle_span() >= 80.0 or self._alt_span_m() >= 0.25
        return True

    def _phase_status(self, phase: str) -> str:
        if phase == "HOLD":
            return f"samples {self.samples}/{self.min_samples}"
        if phase == "ROLL":
            return f"L={'Y' if self.roll_neg else 'n'} R={'Y' if self.roll_pos else 'n'}"
        if phase == "PITCH":
            return f"F={'Y' if self.pitch_neg else 'n'} B={'Y' if self.pitch_pos else 'n'}"
        if phase == "YAW":
            return f"span {self._yaw_span_deg():.0f}deg"
        if phase == "THR":
            return f"thr {self._throttle_span():.0f} alt {self._alt_span_m():.1f}m"
        return "saving"

    def _calibration_score(self) -> float:
        phase_points = sum(1 for phase, _hint in self.PHASES[:-1] if self._phase_observed(phase))
        phase_score = phase_points / max(len(self.PHASES) - 1, 1)
        sample_score = min(1.0, self.samples / max(float(self.min_samples), 1.0))
        flow_penalty = min(0.3, self.flow_bad_samples / max(float(self.samples + self.flow_bad_samples), 1.0))
        return max(0.0, min(10.0, 10.0 * (0.65 * phase_score + 0.35 * sample_score - flow_penalty)))

    def _yaw_deg(self, monitor) -> Optional[float]:
        attitude = getattr(monitor, "attitude", None)
        if attitude is None:
            return None
        return math.degrees(float(getattr(attitude, "yaw", 0.0) or 0.0))

    def _yaw_span_deg(self) -> float:
        return max(0.0, self.yaw_max_delta_deg - self.yaw_min_delta_deg)

    def _throttle_value(self, monitor) -> Optional[float]:
        rc = getattr(monitor, "rc_channels", None)
        if rc is not None:
            value = getattr(rc, "chan3_raw", None)
            if value not in (None, 0):
                return float(value)
        vfr = getattr(monitor, "vfr_hud", None)
        if vfr is not None:
            value = getattr(vfr, "throttle", None)
            if value is not None:
                return float(value)
        return None

    def _throttle_span(self) -> float:
        if self.throttle_min is None or self.throttle_max is None:
            return 0.0
        return max(0.0, self.throttle_max - self.throttle_min)

    def _altitude_m(self, monitor) -> Optional[float]:
        local = getattr(monitor, "local_position", None)
        if local is not None:
            try:
                return -float(getattr(local, "z"))
            except (TypeError, ValueError):
                pass
        gpos = getattr(monitor, "global_position_int", None)
        if gpos is not None:
            rel_alt_mm = getattr(gpos, "relative_alt", None)
            if rel_alt_mm is not None:
                return float(rel_alt_mm) / 1000.0
        return None

    def _alt_span_m(self) -> float:
        if self.alt_min_m is None or self.alt_max_m is None:
            return 0.0
        return max(0.0, self.alt_max_m - self.alt_min_m)

    def _write_profile(self, observer: LegacyLoiterObserver) -> None:
        self.profile_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "samples": self.samples,
            "calibration_score": self._calibration_score(),
            "roll_left_seen": self.roll_neg,
            "roll_right_seen": self.roll_pos,
            "pitch_forward_seen": self.pitch_neg,
            "pitch_back_seen": self.pitch_pos,
            "yaw_span_deg": self._yaw_span_deg(),
            "throttle_span": self._throttle_span(),
            "altitude_span_m": self._alt_span_m(),
            "flow_bad_samples": self.flow_bad_samples,
            "velocity_scale": observer.correction.velocity_scale,
            "confidence": observer.correction.confidence,
            "best_observer_score": observer.best_score,
        }
        self.profile_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _announce(self, master, text: str) -> None:
        send_gcs(master, text, direct_sender=self.direct_status_sender)
