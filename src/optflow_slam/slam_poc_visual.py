"""Live dashboard state and evidence report for the shadow SLAM proof."""

from __future__ import annotations

import base64
from collections import deque
from copy import deepcopy
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable
import webbrowser

import numpy as np

from .rtl_shadow import LocalReturnShadow, ReturnSettings


GUIDE_SETTLE_S = 3.0
GUIDE_HOLD_S = 3.0
GUIDE_TARGET_M = 0.30
GUIDE_RETURN_TOLERANCE_M = 0.12
GUIDE_HOLD_MOTION_TOLERANCE_M = 0.05
GUIDE_VERTICAL_LIMIT_M = 0.15

GUIDE_COPY = {
    "synchronizing": {
        "label": "PREPARING",
        "instruction": "SENSORS SYNCHRONIZING",
        "detail": "Keep the aircraft disarmed and still.",
    },
    "settle": {
        "label": "STEP 1 OF 5",
        "instruction": "HOLD STILL",
        "detail": "The visualizer is recording the start position.",
    },
    "outbound": {
        "label": "STEP 2 OF 5",
        "instruction": "MOVE HORIZONTALLY",
        "detail": "Move slowly in one clear direction; keep height and yaw steady.",
    },
    "hold_out": {
        "label": "STEP 3 OF 5",
        "instruction": "HOLD POSITION",
        "detail": "Keep the aircraft motionless while both trajectories settle.",
    },
    "return": {
        "label": "STEP 4 OF 5",
        "instruction": "RETURN TO START",
        "detail": "Move slowly back along the same path; keep height steady.",
    },
    "final_hold": {
        "label": "STEP 5 OF 5",
        "instruction": "FINAL HOLD",
        "detail": "Keep the aircraft motionless at the start position.",
    },
    "complete": {
        "label": "SEQUENCE COMPLETE",
        "instruction": "PROOF MOTION COMPLETE",
        "detail": "Stop and save the proof when you are ready.",
    },
}


class SlamPocState:
    """Thread-safe fused view of RGB-D VO, IMU, and FAST-LIO outputs."""

    def __init__(
        self,
        session_name: str,
        maximum_path_points: int = 2400,
        *,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        return_settings: ReturnSettings | None = None,
        allow_armed: bool = False,
        guide_enabled: bool = True,
    ) -> None:
        self.session_name = session_name
        self._clock_ns = clock_ns
        self._allow_armed = bool(allow_armed)
        self._guide_enabled = bool(guide_enabled)
        self.started_ns = self._clock_ns()
        self._lock = threading.Lock()
        self._stop_requested = False
        self._diagnostics: dict[str, Any] = {}
        self._lio_origin: np.ndarray | None = None
        self._lio_position: list[float] | None = None
        self._lio_path: deque[list[float]] = deque(maxlen=maximum_path_points)
        self._lio_rows = 0
        self._lio_path_length_m = 0.0
        self._lio_ever_publishing = False
        self._lio_ever_synchronized = False
        self._rgbd: dict[str, Any] = {
            "connected": False,
            "tracking": False,
            "error": None,
            "frames": 0,
            "tracked_frames": 0,
            "tracking_success_ratio": 0.0,
            "measured_fps": 0.0,
            "compute_ms": None,
            "valid_depth_fraction": 0.0,
            "gyro_prior_coverage_ratio": 0.0,
            "position_local_flu_m": None,
            "quaternion_local_flu_xyzw": None,
            "path_length_m": 0.0,
            "map_keyframes": 0,
            "map_points": 0,
            "updated_monotonic_ns": None,
        }
        self._rgbd_path: deque[list[float]] = deque(
            maxlen=maximum_path_points
        )
        self._map_payload: dict[str, Any] = {
            "sequence": 0,
            "encoding": "int16_le_base64",
            "scale_m": 0.01,
            "point_count": 0,
            "points_b64": "",
            "colors_b64": "",
        }
        self._cube_messages = 0
        self._cube_armed = False
        self._cube_armed_once = False
        self._cube_arm_ns: int | None = None
        self._cube_disarm_ns: int | None = None
        self._ready_tune_sent = False
        self._guide_phase = "synchronizing"
        self._guide_sequence = 0
        self._guide_phase_started_ns = self.started_ns
        self._guide_rgbd_origin: np.ndarray | None = None
        self._guide_lio_origin: np.ndarray | None = None
        self._guide_rgbd_hold_anchor: np.ndarray | None = None
        self._guide_lio_hold_anchor: np.ndarray | None = None
        self._rtl_shadow = LocalReturnShadow(return_settings)

    def update_diagnostics(self, diagnostics: dict[str, Any]) -> None:
        with self._lock:
            self._diagnostics = deepcopy(diagnostics)
            self._lio_ever_publishing = bool(
                self._lio_ever_publishing or diagnostics.get("publishing")
            )
            self._lio_ever_synchronized = bool(
                self._lio_ever_synchronized
                or diagnostics.get("synchronized")
            )

    def update_odometry(
        self,
        position_m: list[float] | tuple[float, ...],
        *,
        timestamp_ns: int | None = None,
        quaternion_xyzw: list[float] | tuple[float, ...] | None = None,
    ) -> None:
        del timestamp_ns, quaternion_xyzw
        now_ns = self._clock_ns()
        position = np.asarray(position_m, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            return
        with self._lock:
            if self._lio_origin is None:
                self._lio_origin = position.copy()
            relative = position - self._lio_origin
            point = [
                float(relative[0]),
                float(-relative[1]),
                float(-relative[2]),
            ]
            if self._lio_position is not None:
                self._lio_path_length_m += math.dist(
                    self._lio_position, point
                )
            self._lio_position = point
            self._lio_path.append(point)
            self._lio_rows += 1
            if self._guide_phase in {"outbound", "hold_out"}:
                self._rtl_shadow.observe_outbound(now_ns, point)
            elif self._guide_phase in {"return", "final_hold"}:
                self._rtl_shadow.observe_return(now_ns, point)

    def update_cube(
        self, message_type: str, data: dict[str, Any] | None = None
    ) -> None:
        with self._lock:
            self._cube_messages += 1
            if message_type == "HEARTBEAT" and isinstance(data, dict):
                try:
                    armed = bool(int(data["base_mode"]) & 128)
                except (KeyError, TypeError, ValueError):
                    return
                now_ns = self._clock_ns()
                if armed and not self._cube_armed:
                    self._cube_armed_once = True
                    self._cube_arm_ns = now_ns
                    self._cube_disarm_ns = None
                elif not armed and self._cube_armed and self._cube_armed_once:
                    self._cube_disarm_ns = now_ns
                self._cube_armed = armed
                if armed and not self._allow_armed:
                    self._stop_requested = True

    def ready_for_motion(self) -> bool:
        snapshot = self.snapshot()
        rgbd = snapshot["rgbd"]
        lio = snapshot["lio"]
        return bool(
            (self._allow_armed or not self._cube_armed)
            and rgbd["connected"]
            and int(rgbd["frames"]) >= 30
            and float(rgbd["tracking_success_ratio"]) >= 0.70
            and float(rgbd["gyro_prior_coverage_ratio"]) >= 0.50
            and lio["ever_publishing"]
            and int(lio["rows"]) >= 5
        )

    def mark_ready_tune_sent(self) -> None:
        with self._lock:
            self._ready_tune_sent = True

    def update_rgbd(self, row: dict[str, Any]) -> None:
        position = row.get("position_local_flu_m")
        now_ns = self._clock_ns()
        sample_ns = row.get("host_monotonic_ns")
        if not isinstance(sample_ns, int):
            sample_ns = now_ns
        with self._lock:
            for key in self._rgbd:
                if key in row:
                    self._rgbd[key] = deepcopy(row[key])
            self._rgbd["connected"] = True
            self._rgbd["updated_monotonic_ns"] = now_ns
            if (
                row.get("tracking")
                and isinstance(position, list)
                and len(position) == 3
            ):
                self._rgbd_path.append([float(value) for value in position])
            if isinstance(position, list) and len(position) == 3:
                self._rtl_shadow.update_visual(
                    sample_ns,
                    position,
                    tracking=bool(row.get("tracking")),
                )

    def update_rgbd_map(
        self, points_m: np.ndarray, colors_rgb: np.ndarray
    ) -> None:
        points = np.asarray(points_m, dtype=np.float32)
        colors = np.asarray(colors_rgb, dtype=np.uint8)
        if points.ndim != 2 or points.shape[1] != 3:
            return
        if colors.shape != (len(points), 3):
            return
        scale_m = 0.01
        quantized = np.clip(
            np.rint(points / scale_m), -32768, 32767
        ).astype("<i2")
        with self._lock:
            self._map_payload = {
                "sequence": int(self._map_payload["sequence"]) + 1,
                "encoding": "int16_le_base64",
                "scale_m": scale_m,
                "point_count": len(points),
                "points_b64": base64.b64encode(quantized.tobytes()).decode(
                    "ascii"
                ),
                "colors_b64": base64.b64encode(colors.tobytes()).decode(
                    "ascii"
                ),
            }
            self._rgbd["map_points"] = len(points)

    def set_rgbd_error(self, detail: str) -> None:
        with self._lock:
            self._rgbd["error"] = detail
            self._rgbd["tracking"] = False

    def request_stop(self) -> None:
        with self._lock:
            self._stop_requested = True

    def should_stop(self) -> bool:
        with self._lock:
            return self._stop_requested

    def flight_complete(self, *, post_disarm_s: float = 3.0) -> bool:
        with self._lock:
            if self._cube_disarm_ns is None:
                return False
            return (
                self._clock_ns() - self._cube_disarm_ns
                >= round(post_disarm_s * 1.0e9)
            )

    def flight_lifecycle(self) -> dict[str, Any]:
        with self._lock:
            return {
                "armed_once": self._cube_armed_once,
                "armed": self._cube_armed,
                "arm_monotonic_ns": self._cube_arm_ns,
                "disarm_monotonic_ns": self._cube_disarm_ns,
                "completed_arm_cycle": bool(
                    self._cube_armed_once and self._cube_disarm_ns is not None
                ),
            }

    def snapshot(self) -> dict[str, Any]:
        now_ns = self._clock_ns()
        with self._lock:
            diagnostics = deepcopy(self._diagnostics)
            rgbd = deepcopy(self._rgbd)
            updated_ns = rgbd.pop("updated_monotonic_ns", None)
            rgbd["age_ms"] = (
                None
                if updated_ns is None
                else max(0.0, (now_ns - updated_ns) / 1.0e6)
            )
            imu = diagnostics.get("imu", {})
            lidar = diagnostics.get("lidar", {})
            ready_for_motion = self.ready_for_motion_locked()
            guide = (
                self._guide_snapshot_locked(now_ns, ready_for_motion)
                if self._guide_enabled
                else {
                    "phase": "flight_shadow",
                    "sequence": 0,
                    "label": "FLIGHT SHADOW",
                    "instruction": "PILOT OWNS THE AIRCRAFT",
                    "detail": "SLAM and obstacle outputs are recording only.",
                    "progress": 0.0,
                    "complete": False,
                }
            )
            return {
                "schema_version": 1,
                "session": self.session_name,
                "elapsed_s": (now_ns - self.started_ns) / 1.0e9,
                "stop_requested": self._stop_requested,
                "pose_output_to_cube": False,
                "stage": (
                    "armed_flight_shadow"
                    if self._allow_armed
                    else "shadow_proof"
                ),
                "rgbd": rgbd,
                "rgbd_path": list(self._rgbd_path),
                "lio": {
                    "publishing": bool(diagnostics.get("publishing")),
                    "synchronized": bool(diagnostics.get("synchronized")),
                    "ever_publishing": self._lio_ever_publishing,
                    "ever_synchronized": self._lio_ever_synchronized,
                    "rows": self._lio_rows,
                    "position_m": deepcopy(self._lio_position),
                    "path": list(self._lio_path),
                    "path_length_m": self._lio_path_length_m,
                },
                "imu": {
                    "connected": bool(imu.get("connected")),
                    "rate_hz": imu.get("rate_hz"),
                    "clock_ready": bool(imu.get("clock", {}).get("ready")),
                    "queue_drops": int(imu.get("queue_drops") or 0),
                },
                "lidar": {
                    "connected": bool(lidar.get("connected")),
                    "rate_hz": lidar.get("rate_hz"),
                    "clock_ready": bool(
                        lidar.get("clock", {}).get("ready")
                    ),
                    "queue_drops": int(lidar.get("queue_drops") or 0),
                },
                "cube_reference_messages": self._cube_messages,
                "cube_armed": self._cube_armed,
                "cube_armed_once": self._cube_armed_once,
                "cube_arm_monotonic_ns": self._cube_arm_ns,
                "cube_disarm_monotonic_ns": self._cube_disarm_ns,
                "ready_for_motion": ready_for_motion,
                "ready_tune_sent": self._ready_tune_sent,
                "guide": guide,
                "rtl_shadow": self._rtl_shadow.snapshot(),
                "map_sequence": int(self._map_payload["sequence"]),
                "map_display_points": int(self._map_payload["point_count"]),
            }

    def ready_for_motion_locked(self) -> bool:
        return bool(
            (self._allow_armed or not self._cube_armed)
            and self._rgbd["connected"]
            and int(self._rgbd["frames"]) >= 30
            and float(self._rgbd["tracking_success_ratio"]) >= 0.70
            and float(self._rgbd["gyro_prior_coverage_ratio"]) >= 0.50
            and self._lio_ever_publishing
            and self._lio_rows >= 5
        )

    @staticmethod
    def _guide_point(value: Any) -> np.ndarray | None:
        point = np.asarray(value, dtype=np.float64)
        if point.shape != (3,) or not np.isfinite(point).all():
            return None
        return point

    def _set_guide_phase_locked(
        self,
        phase: str,
        now_ns: int,
        rgbd_position: np.ndarray | None,
        lio_position: np.ndarray | None,
    ) -> None:
        if phase == self._guide_phase:
            return
        self._guide_phase = phase
        self._guide_sequence += 1
        self._guide_phase_started_ns = now_ns
        if phase in {"settle", "hold_out", "final_hold"}:
            self._guide_rgbd_hold_anchor = (
                None if rgbd_position is None else rgbd_position.copy()
            )
            self._guide_lio_hold_anchor = (
                None if lio_position is None else lio_position.copy()
            )
        if phase == "outbound" and lio_position is not None:
            self._rtl_shadow.capture_launch(
                now_ns,
                lio_position,
                rgbd_position,
            )
        elif phase == "return":
            self._rtl_shadow.begin_return(now_ns)
        elif phase == "complete":
            self._rtl_shadow.finish()

    def _guide_hold_stable_locked(
        self,
        now_ns: int,
        rgbd_position: np.ndarray | None,
        lio_position: np.ndarray | None,
    ) -> bool:
        if rgbd_position is None or lio_position is None:
            self._guide_phase_started_ns = now_ns
            return False
        if (
            self._guide_rgbd_hold_anchor is None
            or self._guide_lio_hold_anchor is None
        ):
            self._guide_rgbd_hold_anchor = rgbd_position.copy()
            self._guide_lio_hold_anchor = lio_position.copy()
            self._guide_phase_started_ns = now_ns
            return False
        rgbd_motion = float(
            np.linalg.norm(rgbd_position - self._guide_rgbd_hold_anchor)
        )
        lio_motion = float(
            np.linalg.norm(lio_position - self._guide_lio_hold_anchor)
        )
        if max(rgbd_motion, lio_motion) > GUIDE_HOLD_MOTION_TOLERANCE_M:
            self._guide_rgbd_hold_anchor = rgbd_position.copy()
            self._guide_lio_hold_anchor = lio_position.copy()
            self._guide_phase_started_ns = now_ns
            return False
        return True

    @staticmethod
    def _guide_offset(
        position: np.ndarray | None,
        origin: np.ndarray | None,
    ) -> tuple[float, float]:
        if position is None or origin is None:
            return 0.0, 0.0
        offset = position - origin
        return float(np.linalg.norm(offset[:2])), float(abs(offset[2]))

    def _guide_snapshot_locked(
        self,
        now_ns: int,
        ready_for_motion: bool,
    ) -> dict[str, Any]:
        rgbd_position = self._guide_point(
            self._rgbd.get("position_local_flu_m")
        )
        lio_position = self._guide_point(self._lio_position)
        phase = self._guide_phase
        elapsed_s = max(
            0.0, (now_ns - self._guide_phase_started_ns) / 1.0e9
        )

        if (
            phase == "synchronizing"
            and ready_for_motion
            and rgbd_position is not None
            and lio_position is not None
        ):
            self._set_guide_phase_locked(
                "settle", now_ns, rgbd_position, lio_position
            )
        elif phase == "settle":
            if self._guide_hold_stable_locked(
                now_ns, rgbd_position, lio_position
            ) and elapsed_s >= GUIDE_SETTLE_S:
                self._guide_rgbd_origin = rgbd_position.copy()
                self._guide_lio_origin = lio_position.copy()
                self._set_guide_phase_locked(
                    "outbound", now_ns, rgbd_position, lio_position
                )
        elif phase == "outbound":
            rgbd_horizontal, rgbd_vertical = self._guide_offset(
                rgbd_position, self._guide_rgbd_origin
            )
            lio_horizontal, lio_vertical = self._guide_offset(
                lio_position, self._guide_lio_origin
            )
            if (
                min(rgbd_horizontal, lio_horizontal) >= GUIDE_TARGET_M
                and max(rgbd_vertical, lio_vertical)
                <= GUIDE_VERTICAL_LIMIT_M
            ):
                self._set_guide_phase_locked(
                    "hold_out", now_ns, rgbd_position, lio_position
                )
        elif phase == "hold_out":
            if self._guide_hold_stable_locked(
                now_ns, rgbd_position, lio_position
            ) and elapsed_s >= GUIDE_HOLD_S:
                self._set_guide_phase_locked(
                    "return", now_ns, rgbd_position, lio_position
                )
        elif phase == "return":
            rgbd_horizontal, rgbd_vertical = self._guide_offset(
                rgbd_position, self._guide_rgbd_origin
            )
            lio_horizontal, lio_vertical = self._guide_offset(
                lio_position, self._guide_lio_origin
            )
            if (
                max(rgbd_horizontal, lio_horizontal)
                <= GUIDE_RETURN_TOLERANCE_M
                and max(rgbd_vertical, lio_vertical)
                <= GUIDE_VERTICAL_LIMIT_M
            ):
                self._set_guide_phase_locked(
                    "final_hold", now_ns, rgbd_position, lio_position
                )
        elif phase == "final_hold":
            if self._guide_hold_stable_locked(
                now_ns, rgbd_position, lio_position
            ) and elapsed_s >= GUIDE_HOLD_S:
                self._set_guide_phase_locked(
                    "complete", now_ns, rgbd_position, lio_position
                )

        phase = self._guide_phase
        elapsed_s = max(
            0.0, (now_ns - self._guide_phase_started_ns) / 1.0e9
        )
        rgbd_horizontal, rgbd_vertical = self._guide_offset(
            rgbd_position, self._guide_rgbd_origin
        )
        lio_horizontal, lio_vertical = self._guide_offset(
            lio_position, self._guide_lio_origin
        )
        hold_remaining_s: float | None = None
        if phase == "synchronizing":
            readiness_gates = (
                bool(self._rgbd["connected"]),
                int(self._rgbd["frames"]) >= 30,
                float(self._rgbd["tracking_success_ratio"]) >= 0.70,
                float(self._rgbd["gyro_prior_coverage_ratio"]) >= 0.50,
                bool(self._lio_ever_publishing and self._lio_rows >= 5),
            )
            progress = sum(readiness_gates) / len(readiness_gates)
        elif phase == "outbound":
            progress = min(
                1.0,
                min(rgbd_horizontal, lio_horizontal) / GUIDE_TARGET_M,
            )
        elif phase == "return":
            progress = min(
                1.0,
                max(
                    0.0,
                    1.0
                    - max(rgbd_horizontal, lio_horizontal)
                    / GUIDE_TARGET_M,
                ),
            )
        elif phase in {"settle", "hold_out", "final_hold"}:
            duration_s = (
                GUIDE_SETTLE_S if phase == "settle" else GUIDE_HOLD_S
            )
            hold_remaining_s = max(0.0, duration_s - elapsed_s)
            progress = min(1.0, elapsed_s / duration_s)
        else:
            progress = 1.0

        copy = GUIDE_COPY[phase]
        detail = copy["detail"]
        vertical_warning = (
            max(rgbd_vertical, lio_vertical) > GUIDE_VERTICAL_LIMIT_M
        )
        estimator_gap_m = abs(rgbd_horizontal - lio_horizontal)
        if vertical_warning and phase in {"outbound", "return"}:
            detail = "Vertical motion detected. Hold height before continuing."
        elif estimator_gap_m > 0.10 and phase in {"outbound", "return"}:
            detail = "The two trajectories disagree. Move slowly and hold steady."

        return {
            "phase": phase,
            "sequence": self._guide_sequence,
            "label": copy["label"],
            "instruction": copy["instruction"],
            "detail": detail,
            "progress": progress,
            "hold_remaining_s": hold_remaining_s,
            "rgbd_horizontal_m": rgbd_horizontal,
            "lio_horizontal_m": lio_horizontal,
            "rgbd_vertical_m": rgbd_vertical,
            "lio_vertical_m": lio_vertical,
            "target_m": GUIDE_TARGET_M,
            "return_tolerance_m": GUIDE_RETURN_TOLERANCE_M,
            "vertical_limit_m": GUIDE_VERTICAL_LIMIT_M,
            "vertical_warning": vertical_warning,
            "estimator_gap_m": estimator_gap_m,
            "complete": phase == "complete",
        }

    def map_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return deepcopy(self._map_payload)

    def rtl_shadow_rows(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._rtl_shadow.rows()

    def rtl_shadow_report(self) -> dict[str, Any]:
        with self._lock:
            return self._rtl_shadow.report()

    def report(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        rtl_shadow = self.rtl_shadow_report()
        rgbd = snapshot["rgbd"]
        lio = snapshot["lio"]
        camera_online = bool(rgbd["connected"] and not rgbd.get("error"))
        visual_tracking = bool(
            int(rgbd["frames"]) >= 30
            and float(rgbd["tracking_success_ratio"]) >= 0.70
        )
        imu_aided = float(rgbd["gyro_prior_coverage_ratio"]) >= 0.50
        map_created = int(rgbd["map_points"]) >= 500
        lio_online = bool(lio["ever_publishing"] and int(lio["rows"]) >= 5)
        rgbd_endpoint = rgbd.get("position_local_flu_m")
        lio_endpoint = lio.get("position_m")
        rgbd_path = np.asarray(snapshot["rgbd_path"], dtype=np.float64)
        lio_path = np.asarray(lio["path"], dtype=np.float64)
        rgbd_norms = (
            np.linalg.norm(rgbd_path, axis=1)
            if rgbd_path.ndim == 2 and rgbd_path.shape[1] == 3
            else np.empty(0)
        )
        lio_norms = (
            np.linalg.norm(lio_path, axis=1)
            if lio_path.ndim == 2 and lio_path.shape[1] == 3
            else np.empty(0)
        )
        rgbd_peak = (
            rgbd_path[int(np.argmax(rgbd_norms))]
            if len(rgbd_norms)
            else None
        )
        lio_peak = (
            lio_path[int(np.argmax(lio_norms))]
            if len(lio_norms)
            else None
        )
        rgbd_maximum_displacement_m = (
            float(np.max(rgbd_norms)) if len(rgbd_norms) else 0.0
        )
        lio_maximum_displacement_m = (
            float(np.max(lio_norms)) if len(lio_norms) else 0.0
        )
        both_trajectories_moved = bool(
            rgbd_maximum_displacement_m >= 0.15
            and lio_maximum_displacement_m >= 0.15
        )
        motion_observed = both_trajectories_moved
        pipeline_pass = all(
            (camera_online, visual_tracking, imu_aided, map_created, lio_online)
        )
        rgbd_end = rgbd.get("position_local_flu_m")
        lio_end = lio.get("position_m")
        endpoint_difference_m = (
            math.dist(rgbd_end, lio_end)
            if isinstance(rgbd_end, list)
            and isinstance(lio_end, list)
            and len(rgbd_end) == 3
            and len(lio_end) == 3
            else None
        )
        peak_difference_m = (
            float(np.linalg.norm(rgbd_peak - lio_peak))
            if rgbd_peak is not None and lio_peak is not None
            else None
        )
        peak_direction_cosine = (
            float(
                np.dot(rgbd_peak, lio_peak)
                / (
                    rgbd_maximum_displacement_m
                    * lio_maximum_displacement_m
                )
            )
            if both_trajectories_moved
            and rgbd_peak is not None
            and lio_peak is not None
            else None
        )
        peak_trajectory_agreement = bool(
            both_trajectories_moved
            and peak_difference_m is not None
            and peak_difference_m <= 0.35
            and peak_direction_cosine is not None
            and peak_direction_cosine >= 0.30
        )
        rtl_time_aligned_agreement = bool(
            rtl_shadow.get("result") == "shadow_pass"
            and rtl_shadow.get("gates", {}).get("visual_consistency")
            and rtl_shadow.get("gates", {}).get(
                "observed_motion_alignment"
            )
        )
        trajectory_agreement = bool(
            peak_trajectory_agreement or rtl_time_aligned_agreement
        )
        trajectory_agreement_source = (
            "rtl_shadow_time_aligned"
            if rtl_time_aligned_agreement
            else "independent_peak_fallback"
        )
        pipeline_pass = bool(pipeline_pass and trajectory_agreement)
        if pipeline_pass and motion_observed:
            result = "pass"
            detail = "Live metric visual-inertial odometry and LIO map proof passed"
        elif all((camera_online, visual_tracking, imu_aided, map_created, lio_online)):
            result = "inconclusive"
            detail = (
                "Both trajectories moved but did not agree within 0.35 m"
                if motion_observed
                else "Pipeline is live; move the disarmed aircraft to prove trajectory"
            )
        else:
            result = "fail"
            detail = "One or more live proof components did not meet the POC gate"
        return {
            "schema_version": 1,
            "kind": "slam_vio_proof_of_concept",
            "result": result,
            "detail": detail,
            "pose_sent_to_cube": False,
            "navigation_enabled": False,
            "loop_closure_enabled": False,
            "local_return_shadow": rtl_shadow,
            "components": {
                "visual_odometry": "OpenCV dense metric RGB-D",
                "visual_rotation_prior": "IM10A gyro",
                "lidar_inertial_odometry": "FAST-LIO2",
                "visual_map": "D415 colored voxel map",
            },
            "gates": {
                "camera_online": camera_online,
                "visual_tracking": visual_tracking,
                "imu_rotation_prior": imu_aided,
                "visual_map_created": map_created,
                "lidar_inertial_odometry": lio_online,
                "motion_observed": motion_observed,
                "trajectory_agreement": trajectory_agreement,
            },
            "metrics": {
                "duration_s": snapshot["elapsed_s"],
                "rgbd_frames": int(rgbd["frames"]),
                "rgbd_tracking_success_ratio": float(
                    rgbd["tracking_success_ratio"]
                ),
                "rgbd_gyro_prior_coverage_ratio": float(
                    rgbd["gyro_prior_coverage_ratio"]
                ),
                "rgbd_path_length_m": float(rgbd["path_length_m"]),
                "rgbd_map_points": int(rgbd["map_points"]),
                "rgbd_map_keyframes": int(rgbd["map_keyframes"]),
                "lio_rows": int(lio["rows"]),
                "lio_path_length_m": float(lio["path_length_m"]),
                "raw_endpoint_difference_m": endpoint_difference_m,
                "rgbd_maximum_displacement_m": (
                    rgbd_maximum_displacement_m
                ),
                "lio_maximum_displacement_m": lio_maximum_displacement_m,
                "peak_position_difference_m": peak_difference_m,
                "peak_direction_cosine": peak_direction_cosine,
                "peak_trajectory_agreement": peak_trajectory_agreement,
                "trajectory_agreement_source": (
                    trajectory_agreement_source
                ),
            },
        }


def make_slam_poc_handler(
    state: SlamPocState,
    static_dir: Path,
) -> type[SimpleHTTPRequestHandler]:
    class SlamPocHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def log_message(self, _format: str, *args: Any) -> None:
            return

        def do_GET(self) -> None:
            if self.path == "/api/slam-poc":
                self._send_json(state.snapshot())
                return
            if self.path == "/api/slam-poc/map":
                self._send_json(state.map_snapshot())
                return
            if self.path in ("/", "/slam-poc"):
                self.path = "/slam-poc.html"
            super().do_GET()

        def do_POST(self) -> None:
            if self.path == "/api/slam-poc/stop":
                state.request_stop()
                self._send_json({"stopping": True})
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def _send_json(
            self,
            payload: dict[str, Any],
            status: HTTPStatus = HTTPStatus.OK,
        ) -> None:
            encoded = json.dumps(payload, separators=(",", ":")).encode(
                "utf-8"
            )
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(encoded)

    return SlamPocHandler


class SlamPocServer:
    def __init__(
        self,
        state: SlamPocState,
        static_dir: Path,
        *,
        host: str,
        port: int,
        open_browser: bool,
    ) -> None:
        handler = make_slam_poc_handler(state, static_dir)
        self._server = ThreadingHTTPServer((host, port), handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=partial(self._server.serve_forever, poll_interval=0.2),
            name="slam-poc-visual",
            daemon=True,
        )
        url_host = "127.0.0.1" if host == "0.0.0.0" else host
        self.url = f"http://{url_host}:{port}/slam-poc"
        self._open_browser = open_browser

    def start(self) -> None:
        self._thread.start()
        if self._open_browser:
            threading.Timer(0.5, partial(webbrowser.open, self.url)).start()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
