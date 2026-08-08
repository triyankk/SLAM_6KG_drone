"""Serve a live Three.js optical-flow visualizer from Cube MAVLink data."""

from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import socket
import threading
import time
from typing import Any
import webbrowser

from .config import (
    ConfigError,
    ObstacleAvoidanceConfig,
    load_config,
)
from .im10a import Im10aDecoder
from .mavlink_compat import install_pymavlink_instance_guard
from .obstacles import (
    ObstacleFusion,
    ObstacleScan,
    obstacle_alert_state,
)
from .paths import CONFIG_DIR, RUNTIME_DIR, VISUALIZER_DIR
from .runtime_lock import RuntimeLockError, cube_mavlink_lock
from .spatial_stream import (
    DEFAULT_SPATIAL_FRAME_DIR,
    DemoSpatialSource,
    HesaiSpatialSource,
    RealSenseSpatialSource,
    SpatialFrameFileSource,
    SpatialFrameStore,
)

DEFAULT_STATIC_DIR = VISUALIZER_DIR / "dist"
DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"
DEFAULT_TRAJECTORY_FILE = RUNTIME_DIR / "slam_navigation_status.json"
TELEMETRY_STREAM_HZ = 60.0
STARTUP_RISING_TUNE = "MFT200L16CEG"
OBSTACLE_BEEP_TUNE = "MFT240L32G"


def _valid_pwm(value: Any) -> int | None:
    pwm = int(value)
    return pwm if 0 < pwm < 65535 else None


def _event_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _event_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_event_safe(item) for item in value]
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value).hex()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


class NavigationTrajectoryStore:
    """Expose the active navigator's atomic status file to the browser."""

    def __init__(self, path: Path, *, demo: bool = False) -> None:
        self.path = path
        self.demo = bool(demo)

    def snapshot(self) -> dict[str, Any]:
        if self.demo:
            return self._demo_snapshot()
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {
                "kind": "trajectory",
                "available": False,
                "detail": "SLAM navigation status has not been created",
                "trajectories": {},
            }
        except (OSError, json.JSONDecodeError) as exc:
            return {
                "kind": "trajectory",
                "available": False,
                "detail": f"SLAM navigation status is unreadable: {exc}",
                "trajectories": {},
            }
        if not isinstance(payload, dict):
            return {
                "kind": "trajectory",
                "available": False,
                "detail": "SLAM navigation status is not an object",
                "trajectories": {},
            }
        updated_unix_ns = payload.get("updated_unix_ns")
        try:
            age_ms = max(
                0.0,
                (time.time_ns() - int(updated_unix_ns)) / 1.0e6,
            )
        except (TypeError, ValueError):
            age_ms = None
        return {
            **payload,
            "kind": "trajectory",
            "available": True,
            "live": bool(age_ms is not None and age_ms <= 1500.0),
            "age_ms": age_ms,
        }


    @staticmethod
    def _demo_snapshot() -> dict[str, Any]:
        phase = time.monotonic() * 0.25
        points = []
        for index in range(160):
            angle = index * 0.035
            points.append(
                [
                    1.7 * math.sin(angle),
                    1.1 * math.sin(angle * 0.55),
                    0.18 + 0.08 * math.sin(angle * 0.4),
                ]
            )
        offset = 0.04 * math.sin(phase)
        rgbd = [[x, y + offset, z] for x, y, z in points]
        return {
            "schema_version": 1,
            "kind": "trajectory",
            "available": True,
            "live": True,
            "age_ms": 0.0,
            "state": "returning_locked",
            "stage": "demo",
            "shadow_only": True,
            "estimator": {
                "frame_yaw_ned_rad": 0.35,
                "pose_reason": "ready",
                "pose_sequence": int(time.monotonic() * 5.0),
            },
            "trajectories": {
                "frame": "launch_local_flu",
                "lio": points,
                "rgbd": rgbd,
                "cube": points[::2],
                "breadcrumbs": points[::8],
                "target": points[max(0, 120 - int(phase) % 80)],
                "launch": points[0],
            },
        }


class VisualCueStore:
    """Thread-safe, display-only instruction cue for connected browsers."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sequence = 0
        self._message = ""
        self._detail = ""
        self._flash_count = 2
        self._expires_monotonic = 0.0

    def trigger(
        self,
        message: str,
        *,
        detail: str = "",
        flash_count: int = 2,
        duration_s: float = 10.0,
    ) -> dict[str, Any]:
        clean_message = str(message).strip()
        clean_detail = str(detail).strip()
        if not clean_message or len(clean_message) > 100:
            raise ValueError("message must contain 1 to 100 characters")
        if len(clean_detail) > 240:
            raise ValueError("detail must contain at most 240 characters")
        flashes = int(flash_count)
        duration = float(duration_s)
        if flashes not in (1, 2, 3):
            raise ValueError("flash_count must be 1, 2, or 3")
        if not math.isfinite(duration) or not 1.0 <= duration <= 30.0:
            raise ValueError("duration_s must be between 1 and 30 seconds")
        with self._lock:
            self._sequence += 1
            self._message = clean_message
            self._detail = clean_detail
            self._flash_count = flashes
            self._expires_monotonic = time.monotonic() + duration
            return self._snapshot_locked()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> dict[str, Any]:
        remaining_s = max(0.0, self._expires_monotonic - time.monotonic())
        return {
            "kind": "visual_cue",
            "sequence": self._sequence,
            "active": remaining_s > 0.0,
            "message": self._message,
            "detail": self._detail,
            "flash_count": self._flash_count,
            "remaining_ms": round(remaining_s * 1000),
        }


class RawEventBus:
    """Bounded event history for loss-detectable sensor recording clients."""

    def __init__(self, max_events: int = 20_000) -> None:
        self._condition = threading.Condition()
        self._events: deque[dict[str, Any]] = deque(maxlen=max_events)
        self._sequence = 0

    def publish(
        self, source: str, event_type: str, data: dict[str, Any]
    ) -> None:
        with self._condition:
            self._sequence += 1
            self._events.append(
                {
                    "sequence": self._sequence,
                    "host_monotonic_ns": time.monotonic_ns(),
                    "host_unix_ns": time.time_ns(),
                    "source": source,
                    "type": event_type,
                    "data": data,
                }
            )
            self._condition.notify_all()

    def latest_sequence(self) -> int:
        with self._condition:
            return self._sequence

    def wait_after(
        self, sequence: int, timeout: float = 1.0, limit: int = 512
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
            ][:limit]
            return events, dropped


class TelemetryStore:
    """Thread-safe latest-value store with per-stream freshness."""

    def __init__(
        self,
        source: str,
        cube_mount: dict[str, Any] | None = None,
        imu_axis_signs: tuple[int, int, int] = (1, 1, 1),
        imu_axis_map_verified: bool = False,
        imu_axis_map_verification: str = "not measured",
    ) -> None:
        now = time.monotonic()
        self._lock = threading.Lock()
        self._imu_axis_signs = imu_axis_signs
        self.raw_events = RawEventBus()
        self._state: dict[str, Any] = {
            "sequence": 0,
            "source": source,
            "link": {
                "connected": False,
                "detail": "Waiting for telemetry",
                "last_packet_monotonic": None,
            },
            "vehicle": {
                "system_id": None,
                "component_id": None,
                "armed": False,
                "mode": "UNKNOWN",
                "landed_state": None,
            },
            "flow": {
                "delta_x_dpix": 0,
                "delta_y_dpix": 0,
                "rate_x_rads": 0.0,
                "rate_y_rads": 0.0,
                "comp_x_mps": 0.0,
                "comp_y_mps": 0.0,
                "quality": 0,
                "updated_monotonic": None,
            },
            "range": {
                "distance_m": 0.0,
                "min_m": 0.08,
                "max_m": 30.0,
                "updated_monotonic": None,
            },
            "attitude": {
                "roll_rad": 0.0,
                "pitch_rad": 0.0,
                "yaw_rad": 0.0,
                "rollspeed_rads": 0.0,
                "pitchspeed_rads": 0.0,
                "yawspeed_rads": 0.0,
                "time_boot_ms": None,
                "updated_monotonic": None,
            },
            "imu": {
                "accel_x_mss": 0.0,
                "accel_y_mss": 0.0,
                "accel_z_mss": 0.0,
                "gyro_x_rads": 0.0,
                "gyro_y_rads": 0.0,
                "gyro_z_rads": 0.0,
                "message": "WAITING",
                "updated_monotonic": None,
            },
            "timing": {
                "time_boot_ms": None,
                "unix_usec": None,
                "updated_monotonic": None,
            },
            "local_position": {
                "x_m": 0.0,
                "y_m": 0.0,
                "z_down_m": 0.0,
                "vx_mps": 0.0,
                "vy_mps": 0.0,
                "vz_mps": 0.0,
                "time_boot_ms": None,
                "updated_monotonic": None,
            },
            "global_position": {
                "lat_deg": None,
                "lon_deg": None,
                "alt_msl_m": None,
                "relative_alt_m": None,
                "vx_mps": None,
                "vy_mps": None,
                "vz_mps": None,
                "heading_deg": None,
                "time_boot_ms": None,
                "updated_monotonic": None,
            },
            "gps": {
                "fix_type": 0,
                "satellites_visible": 0,
                "eph_m": None,
                "epv_m": None,
                "ground_speed_mps": None,
                "course_deg": None,
                "alt_msl_m": None,
                "updated_monotonic": None,
            },
            "power": {
                "source": "WAITING",
                "battery_id": None,
                "voltage_v": None,
                "current_a": None,
                "remaining_pct": None,
                "consumed_mah": None,
                "consumed_wh": None,
                "time_remaining_s": None,
                "updated_monotonic": None,
            },
            "vibration": {
                "x_mss": None,
                "y_mss": None,
                "z_mss": None,
                "clipping_0": 0,
                "clipping_1": 0,
                "clipping_2": 0,
                "updated_monotonic": None,
            },
            "ekf": {
                "flags": 0,
                "velocity_variance": None,
                "horizontal_position_variance": None,
                "vertical_position_variance": None,
                "compass_variance": None,
                "terrain_alt_variance": None,
                "airspeed_variance": None,
                "updated_monotonic": None,
            },
            "rc": {
                "channels_pwm": [None] * 18,
                "channel_count": 0,
                "rssi": None,
                "time_boot_ms": None,
                "updated_monotonic": None,
            },
            "actuators": {
                "servo_pwm": [None] * 16,
                "port": None,
                "time_usec": None,
                "updated_monotonic": None,
            },
            "position_target": {
                "coordinate_frame": None,
                "type_mask": None,
                "x_m": None,
                "y_m": None,
                "z_m": None,
                "vx_mps": None,
                "vy_mps": None,
                "vz_mps": None,
                "afx_mss": None,
                "afy_mss": None,
                "afz_mss": None,
                "yaw_rad": None,
                "yaw_rate_rads": None,
                "time_boot_ms": None,
                "updated_monotonic": None,
            },
            "obstacles": {
                "stage": "disabled",
                "mavlink_output_enabled": False,
                "source": None,
                "valid_sector_count": 0,
                "nearest_distance_m": None,
                "clearance_reference": "aircraft_cg",
                "clearance_distance_metric": "horizontal_xy",
                "hard_cg_clearance_m": None,
                "clearance_status": "unknown",
                "clearance_margin_m": None,
                "clearance_breached": False,
                "violating_sector_count": 0,
                "violating_sector_angles_deg": [],
                "source_stale_timeout_s": None,
                "sector_increment_deg": None,
                "distances_cm": [],
                "messages_sent": 0,
                "rc_toggle_channel": None,
                "rc_toggle_pwm": None,
                "rc_toggle_enabled": False,
                "alert_zone": "inactive",
                "alert_beep_rate_hz": 0.0,
                "alert_beeps_sent": 0,
                "startup_tune_sent": False,
                "updated_monotonic": None,
            },
            "ros_imu": {
                "connected": False,
                "detail": "Waiting for IM10A",
                "transport": "serial-direct",
                "contract": "sensor_msgs/Imu",
                "frame_id": "im10a_link",
                "extrinsics_verified": False,
                "axis_map_verified": imu_axis_map_verified,
                "axis_map_verification": imu_axis_map_verification,
                "axis_map_label": self._axis_map_label(imu_axis_signs),
                "sample_rate_hz": 0.0,
                "checksum_errors": 0,
                "accel_x_mss": 0.0,
                "accel_y_mss": 0.0,
                "accel_z_mss": 0.0,
                "gyro_x_rads": 0.0,
                "gyro_y_rads": 0.0,
                "gyro_z_rads": 0.0,
                "roll_rad": 0.0,
                "pitch_rad": 0.0,
                "yaw_rad": 0.0,
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "updated_monotonic": None,
            },
            "cube_mount": cube_mount
            or {
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.0,
                "yaw_ccw_deg": 0.0,
                "ahrs_orientation": 0,
                "ahrs_orientation_name": "None",
            },
            "started_monotonic": now,
            "visualizer_stream_rate_hz": TELEMETRY_STREAM_HZ,
        }

    @staticmethod
    def _axis_map_label(signs: tuple[int, int, int]) -> str:
        labels = ("X", "Y", "Z")
        return "/".join(
            label if sign > 0 else f"-{label}"
            for label, sign in zip(labels, signs)
        )

    def update(self, section: str, **values: Any) -> None:
        with self._lock:
            target = self._state[section]
            target.update(values)
            self._state["sequence"] += 1

    def mark_packet(self) -> None:
        now = time.monotonic()
        self.update(
            "link",
            connected=True,
            detail="Receiving MAVLink",
            last_packet_monotonic=now,
        )

    def publish_raw(
        self, source: str, event_type: str, data: dict[str, Any]
    ) -> None:
        self.raw_events.publish(source, event_type, data)

    def set_link(self, connected: bool, detail: str) -> None:
        self.update("link", connected=connected, detail=detail)

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            state = deepcopy(self._state)

        def age_ms(value: float | None) -> int | None:
            if value is None:
                return None
            return max(0, round((now - value) * 1000))

        state["server_monotonic_s"] = now
        state["uptime_s"] = round(now - state.pop("started_monotonic"), 3)
        ros_imu = state["ros_imu"]
        sign_x, sign_y, sign_z = self._imu_axis_signs
        ros_imu["body_preview"] = {
            "accel_x_mss": sign_x * ros_imu["accel_x_mss"],
            "accel_y_mss": sign_y * ros_imu["accel_y_mss"],
            "accel_z_mss": sign_z * ros_imu["accel_z_mss"],
            "gyro_x_rads": sign_x * ros_imu["gyro_x_rads"],
            "gyro_y_rads": sign_y * ros_imu["gyro_y_rads"],
            "gyro_z_rads": sign_z * ros_imu["gyro_z_rads"],
            "roll_rad": sign_x * ros_imu["roll_rad"],
            "pitch_rad": sign_y * ros_imu["pitch_rad"],
            "yaw_rad": sign_z * ros_imu["yaw_rad"],
        }
        state["link"]["age_ms"] = age_ms(
            state["link"].pop("last_packet_monotonic")
        )
        for section in (
            "flow",
            "range",
            "attitude",
            "imu",
            "ros_imu",
            "timing",
            "local_position",
            "global_position",
            "gps",
            "power",
            "vibration",
            "ekf",
            "rc",
            "actuators",
            "position_target",
            "obstacles",
        ):
            state[section]["age_ms"] = age_ms(
                state[section].pop("updated_monotonic")
            )
        return state


def _set_message_interval(
    master,
    message_id: int,
    interval_us: int,
    *,
    wait_for_ack: bool = False,
) -> int | None:
    from pymavlink import mavutil

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )
    if not wait_for_ack:
        return None
    acknowledgement = master.recv_match(
        type="COMMAND_ACK",
        condition=(
            "COMMAND_ACK.command=="
            f"{mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL}"
        ),
        blocking=True,
        timeout=1.0,
    )
    if acknowledgement is None:
        return None
    return int(acknowledgement.result)


def _restore_message_intervals(master, message_ids) -> None:
    for message_id in message_ids:
        try:
            # Zero restores the stream default; -1 disables it.
            _set_message_interval(master, message_id, 0)
        except Exception:
            pass


class MavlinkSource(threading.Thread):
    """Reconnectable Cube UART owner with latest-only proximity output."""

    def __init__(
        self,
        store: TelemetryStore,
        stop_event: threading.Event,
        endpoint: str,
        baud: int,
        *,
        source_system: int = 1,
        source_component: int = 191,
        obstacle_max_age_s: float = 0.25,
        obstacle_output_enabled: bool = True,
        obstacle_settings: ObstacleAvoidanceConfig | None = None,
        startup_tune_enabled: bool = False,
    ) -> None:
        super().__init__(name="cube-mavlink", daemon=True)
        self.store = store
        self.stop_event = stop_event
        self.endpoint = endpoint
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.target_system: int | None = None
        self.target_component: int | None = None
        self._last_highres_imu_monotonic: float | None = None
        self._outgoing_lock = threading.Lock()
        self._pending_obstacle: ObstacleScan | None = None
        self._latest_obstacle: ObstacleScan | None = None
        self._obstacle_max_age_ns = round(obstacle_max_age_s * 1.0e9)
        self._obstacle_messages_sent = 0
        self._obstacle_output_enabled = obstacle_output_enabled
        self._obstacle_settings = obstacle_settings
        self._startup_tune_enabled = startup_tune_enabled
        self._startup_tune_sent = False
        self._armed = False
        self._rc_toggle_enabled = False
        self._rc_toggle_pwm: int | None = None
        self._last_obstacle_beep_s = float("-inf")
        self._obstacle_beeps_sent = 0

    def queue_obstacle_scan(self, scan: ObstacleScan) -> None:
        """Replace any unsent scan so stale data can never form a backlog."""

        with self._outgoing_lock:
            self._pending_obstacle = scan
            self._latest_obstacle = scan

    def _send_pending_obstacle(self, master, mavutil) -> bool:
        with self._outgoing_lock:
            scan = self._pending_obstacle
            self._pending_obstacle = None
        if scan is None or not self._obstacle_output_enabled:
            return False

        age_ns = time.monotonic_ns() - scan.monotonic_ns
        if age_ns < 0 or age_ns > self._obstacle_max_age_ns:
            return False
        original_component = getattr(master.mav, "srcComponent", None)
        master.mav.srcComponent = getattr(
            mavutil.mavlink,
            "MAV_COMP_ID_OBSTACLE_AVOIDANCE",
            196,
        )
        try:
            master.mav.obstacle_distance_send(
                scan.monotonic_ns // 1000,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                list(scan.distances_cm),
                round(scan.increment_deg),
                scan.min_distance_cm,
                scan.max_distance_cm,
                increment_f=scan.increment_deg,
                angle_offset=0.0,
                frame=mavutil.mavlink.MAV_FRAME_BODY_FRD,
            )
        finally:
            if original_component is None:
                del master.mav.srcComponent
            else:
                master.mav.srcComponent = original_component
        self._obstacle_messages_sent += 1
        self.store.update(
            "obstacles", messages_sent=self._obstacle_messages_sent
        )
        return True

    def _play_tune(self, master, tune: str) -> bool:
        target_system = self.target_system or 1
        target_component = self.target_component or 1
        try:
            master.mav.play_tune_send(
                target_system,
                target_component,
                tune.encode("ascii"),
            )
        except (AttributeError, OSError, TypeError, ValueError):
            return False
        return True

    def _maybe_send_startup_tune(self, master) -> bool:
        if (
            not self._startup_tune_enabled
            or self._startup_tune_sent
            or not self._play_tune(master, STARTUP_RISING_TUNE)
        ):
            return False
        self._startup_tune_sent = True
        self.store.update("obstacles", startup_tune_sent=True)
        self.store.publish_raw(
            "companion",
            "STARTUP_TUNE",
            {"tune": STARTUP_RISING_TUNE},
        )
        return True

    def _set_rc_toggle_pwm(self, pwm: int | None) -> None:
        settings = self._obstacle_settings
        if settings is None:
            return
        previous = self._rc_toggle_enabled
        self._rc_toggle_pwm = pwm
        if pwm is not None:
            if pwm >= settings.rc_toggle.engage_pwm:
                self._rc_toggle_enabled = True
            elif pwm <= settings.rc_toggle.disengage_pwm:
                self._rc_toggle_enabled = False
        if previous != self._rc_toggle_enabled:
            self._last_obstacle_beep_s = float("-inf")
            self.store.publish_raw(
                "obstacle_alert",
                "RC_TOGGLE",
                {
                    "channel": settings.rc_toggle.channel,
                    "pwm": pwm,
                    "enabled": self._rc_toggle_enabled,
                },
            )
        self.store.update(
            "obstacles",
            rc_toggle_channel=settings.rc_toggle.channel,
            rc_toggle_pwm=pwm,
            rc_toggle_enabled=self._rc_toggle_enabled,
        )

    def _maybe_send_obstacle_beep(self, master) -> bool:
        settings = self._obstacle_settings
        if settings is None:
            return False
        now_ns = time.monotonic_ns()
        now_s = now_ns / 1.0e9
        with self._outgoing_lock:
            scan = self._latest_obstacle
        fresh = (
            scan is not None
            and 0 <= now_ns - scan.monotonic_ns <= self._obstacle_max_age_ns
        )
        distance_m = scan.nearest_distance_m if fresh and scan else None
        alert = obstacle_alert_state(
            distance_m,
            hard_clearance_m=settings.hard_cg_clearance_m,
            full_rate_distance_m=max(
                settings.min_distance_m,
                settings.airframe_radius_m,
            ),
            settings=settings.alerts,
        )
        audible = (
            fresh
            and settings.alerts.enabled
            and self._rc_toggle_enabled
            and (
                self._armed
                or not settings.alerts.only_when_armed
            )
        )
        effective_rate_hz = alert.beep_rate_hz if audible else 0.0
        self.store.update(
            "obstacles",
            alert_zone=alert.zone if fresh else "stale",
            alert_beep_rate_hz=round(effective_rate_hz, 3),
            rc_toggle_channel=settings.rc_toggle.channel,
            rc_toggle_pwm=self._rc_toggle_pwm,
            rc_toggle_enabled=self._rc_toggle_enabled,
        )
        if effective_rate_hz <= 0.0:
            return False
        interval_s = 1.0 / effective_rate_hz
        if now_s - self._last_obstacle_beep_s < interval_s:
            return False
        if not self._play_tune(master, OBSTACLE_BEEP_TUNE):
            return False
        self._last_obstacle_beep_s = now_s
        self._obstacle_beeps_sent += 1
        self.store.update(
            "obstacles",
            alert_beeps_sent=self._obstacle_beeps_sent,
        )
        self.store.publish_raw(
            "obstacle_alert",
            "BEEP",
            {
                "zone": alert.zone,
                "distance_m": distance_m,
                "beep_rate_hz": effective_rate_hz,
                "avoidance_required": alert.avoidance_required,
            },
        )
        return True

    def run(self) -> None:
        try:
            with cube_mavlink_lock("standalone hardware visualizer"):
                self._run_locked()
        except RuntimeLockError as exc:
            self.store.set_link(False, str(exc))

    def _run_locked(self) -> None:
        try:
            from pymavlink import mavutil
            install_pymavlink_instance_guard(mavutil)
        except ImportError as exc:
            self.store.set_link(False, f"pymavlink unavailable: {exc}")
            return

        message_intervals = {
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 50_000,
            mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW: 50_000,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 50_000,
            mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU: 20_000,
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU: 20_000,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 50_000,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 100_000,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 200_000,
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 500_000,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 500_000,
            mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION: 100_000,
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 200_000,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS: 100_000,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 100_000,
            mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: 100_000,
            mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME: 1_000_000,
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 500_000,
        }
        critical_message_ids = frozenset(
            (
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
                mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
                mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
                mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
            )
        )
        while not self.stop_event.is_set():
            master = None
            intervals_requested = False
            try:
                self.store.set_link(
                    False, f"Connecting to {self.endpoint} at {self.baud}"
                )
                master = mavutil.mavlink_connection(
                    self.endpoint,
                    baud=self.baud,
                    source_system=self.source_system,
                    source_component=self.source_component,
                )
                heartbeat = self._wait_for_heartbeat(master, mavutil)
                if heartbeat is None:
                    raise RuntimeError("Cube heartbeat timed out")

                master.target_system = heartbeat.get_srcSystem()
                self.target_system = master.target_system
                self.target_component = heartbeat.get_srcComponent()
                # ArduPilot accepts stream-rate commands at the autopilot-wide
                # component target. Heartbeats are still filtered to the
                # observed flight-controller component above.
                master.target_component = 0
                self._handle_heartbeat(heartbeat, mavutil)
                self._maybe_send_startup_tune(master)
                master.mav.request_data_stream_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    10,
                    1,
                )
                for message_id in critical_message_ids:
                    _set_message_interval(
                        master,
                        message_id,
                        message_intervals[message_id],
                        wait_for_ack=True,
                    )
                for message_id, interval_us in message_intervals.items():
                    if message_id in critical_message_ids:
                        continue
                    _set_message_interval(
                        master,
                        message_id,
                        interval_us,
                        wait_for_ack=False,
                    )
                    time.sleep(0.01)
                intervals_requested = True

                while not self.stop_event.is_set():
                    self._send_pending_obstacle(master, mavutil)
                    self._maybe_send_obstacle_beep(master)
                    message = master.recv_match(blocking=True, timeout=0.05)
                    if message is None:
                        snapshot = self.store.snapshot()
                        age = snapshot["link"]["age_ms"]
                        if age is not None and age > 2000:
                            raise RuntimeError("MAVLink stream stale")
                        continue
                    self.store.mark_packet()
                    try:
                        raw_payload = message.to_dict()
                    except AttributeError:
                        raw_payload = {}
                    self.store.publish_raw(
                        "cube_mavlink",
                        message.get_type(),
                        _event_safe(raw_payload),
                    )
                    self._handle_message(message, mavutil)
            except (
                AttributeError,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                self.store.set_link(False, str(exc))
                self.stop_event.wait(2.0)
            finally:
                if master is not None:
                    if intervals_requested:
                        _restore_message_intervals(
                            master, message_intervals
                        )
                    try:
                        master.close()
                    except Exception:
                        pass

    def _wait_for_heartbeat(self, master, mavutil):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not self.stop_event.is_set():
            message = master.recv_match(
                type="HEARTBEAT", blocking=True, timeout=0.5
            )
            if message is None:
                continue
            if (
                message.autopilot
                == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                and message.get_srcComponent()
                == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            ):
                return message
        return None

    def _handle_heartbeat(self, message, mavutil) -> None:
        armed = bool(
            message.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        if self._armed and not armed:
            self._last_obstacle_beep_s = float("-inf")
        self._armed = armed
        self.store.update(
            "vehicle",
            system_id=message.get_srcSystem(),
            component_id=message.get_srcComponent(),
            armed=armed,
            mode=mavutil.mode_string_v10(message),
        )

    def _handle_message(self, message, mavutil) -> None:
        message_type = message.get_type()
        now = time.monotonic()
        if (
            message_type == "HEARTBEAT"
            and message.get_srcSystem() == self.target_system
            and message.get_srcComponent() == self.target_component
        ):
            self._handle_heartbeat(message, mavutil)
        elif message_type == "ATTITUDE":
            self.store.update(
                "attitude",
                roll_rad=float(message.roll),
                pitch_rad=float(message.pitch),
                yaw_rad=float(message.yaw),
                rollspeed_rads=float(message.rollspeed),
                pitchspeed_rads=float(message.pitchspeed),
                yawspeed_rads=float(message.yawspeed),
                time_boot_ms=int(message.time_boot_ms),
                updated_monotonic=now,
            )
        elif message_type == "OPTICAL_FLOW":
            rate_x = getattr(message, "flow_rate_x", 0.0)
            rate_y = getattr(message, "flow_rate_y", 0.0)
            self.store.update(
                "flow",
                delta_x_dpix=int(message.flow_x),
                delta_y_dpix=int(message.flow_y),
                rate_x_rads=float(rate_x),
                rate_y_rads=float(rate_y),
                comp_x_mps=float(message.flow_comp_m_x),
                comp_y_mps=float(message.flow_comp_m_y),
                quality=int(message.quality),
                updated_monotonic=now,
            )
        elif (
            message_type == "DISTANCE_SENSOR"
            and int(message.orientation) == 25
        ):
            self.store.update(
                "range",
                distance_m=float(message.current_distance) / 100.0,
                min_m=float(message.min_distance) / 100.0,
                max_m=float(message.max_distance) / 100.0,
                updated_monotonic=now,
            )
        elif message_type == "HIGHRES_IMU":
            self._last_highres_imu_monotonic = now
            self.store.update(
                "imu",
                accel_x_mss=float(message.xacc),
                accel_y_mss=float(message.yacc),
                accel_z_mss=float(message.zacc),
                gyro_x_rads=float(message.xgyro),
                gyro_y_rads=float(message.ygyro),
                gyro_z_rads=float(message.zgyro),
                message="HIGHRES_IMU",
                updated_monotonic=now,
            )
        elif message_type == "RAW_IMU" and (
            self._last_highres_imu_monotonic is None
            or now - self._last_highres_imu_monotonic > 0.5
        ):
            self.store.update(
                "imu",
                accel_x_mss=float(message.xacc) * 9.80665 / 1000.0,
                accel_y_mss=float(message.yacc) * 9.80665 / 1000.0,
                accel_z_mss=float(message.zacc) * 9.80665 / 1000.0,
                gyro_x_rads=float(message.xgyro) / 1000.0,
                gyro_y_rads=float(message.ygyro) / 1000.0,
                gyro_z_rads=float(message.zgyro) / 1000.0,
                message="RAW_IMU",
                updated_monotonic=now,
            )
        elif message_type == "SYSTEM_TIME":
            self.store.update(
                "timing",
                time_boot_ms=int(message.time_boot_ms),
                unix_usec=(
                    int(message.time_unix_usec)
                    if int(message.time_unix_usec) > 0
                    else None
                ),
                updated_monotonic=now,
            )
        elif message_type == "LOCAL_POSITION_NED":
            self.store.update(
                "local_position",
                x_m=float(message.x),
                y_m=float(message.y),
                z_down_m=float(message.z),
                vx_mps=float(message.vx),
                vy_mps=float(message.vy),
                vz_mps=float(message.vz),
                time_boot_ms=int(message.time_boot_ms),
                updated_monotonic=now,
            )
        elif message_type == "GLOBAL_POSITION_INT":
            heading = int(message.hdg)
            self.store.update(
                "global_position",
                lat_deg=float(message.lat) / 1.0e7,
                lon_deg=float(message.lon) / 1.0e7,
                alt_msl_m=float(message.alt) / 1000.0,
                relative_alt_m=float(message.relative_alt) / 1000.0,
                vx_mps=float(message.vx) / 100.0,
                vy_mps=float(message.vy) / 100.0,
                vz_mps=float(message.vz) / 100.0,
                heading_deg=(
                    None if heading == 65535 else float(heading) / 100.0
                ),
                time_boot_ms=int(message.time_boot_ms),
                updated_monotonic=now,
            )
        elif message_type == "GPS_RAW_INT":
            eph = int(message.eph)
            epv = int(message.epv)
            velocity = int(message.vel)
            course = int(message.cog)
            self.store.update(
                "gps",
                fix_type=int(message.fix_type),
                satellites_visible=int(message.satellites_visible),
                eph_m=None if eph == 65535 else float(eph) / 100.0,
                epv_m=None if epv == 65535 else float(epv) / 100.0,
                ground_speed_mps=(
                    None if velocity == 65535 else float(velocity) / 100.0
                ),
                course_deg=(
                    None if course == 65535 else float(course) / 100.0
                ),
                alt_msl_m=float(message.alt) / 1000.0,
                updated_monotonic=now,
            )
        elif message_type == "BATTERY_STATUS":
            voltages_mv = [
                int(value)
                for value in message.voltages
                if 0 < int(value) < 65535
            ]
            current = int(message.current_battery)
            remaining = int(message.battery_remaining)
            consumed_mah = int(message.current_consumed)
            consumed_wh = int(getattr(message, "energy_consumed", -1))
            time_remaining = int(
                getattr(message, "time_remaining", 0)
            )
            self.store.update(
                "power",
                source="BATTERY_STATUS",
                battery_id=int(message.id),
                voltage_v=(
                    sum(voltages_mv) / 1000.0 if voltages_mv else None
                ),
                current_a=None if current < 0 else float(current) / 100.0,
                remaining_pct=None if remaining < 0 else remaining,
                consumed_mah=None if consumed_mah < 0 else consumed_mah,
                consumed_wh=None if consumed_wh < 0 else consumed_wh / 36.0,
                time_remaining_s=(
                    None if time_remaining <= 0 else time_remaining
                ),
                updated_monotonic=now,
            )
        elif message_type == "SYS_STATUS":
            voltage = int(message.voltage_battery)
            current = int(message.current_battery)
            remaining = int(message.battery_remaining)
            self.store.update(
                "power",
                source="SYS_STATUS",
                voltage_v=None if voltage == 65535 else voltage / 1000.0,
                current_a=None if current < 0 else current / 100.0,
                remaining_pct=None if remaining < 0 else remaining,
                updated_monotonic=now,
            )
        elif message_type == "VIBRATION":
            self.store.update(
                "vibration",
                x_mss=float(message.vibration_x),
                y_mss=float(message.vibration_y),
                z_mss=float(message.vibration_z),
                clipping_0=int(message.clipping_0),
                clipping_1=int(message.clipping_1),
                clipping_2=int(message.clipping_2),
                updated_monotonic=now,
            )
        elif message_type == "EKF_STATUS_REPORT":
            self.store.update(
                "ekf",
                flags=int(message.flags),
                velocity_variance=float(message.velocity_variance),
                horizontal_position_variance=float(
                    message.pos_horiz_variance
                ),
                vertical_position_variance=float(message.pos_vert_variance),
                compass_variance=float(message.compass_variance),
                terrain_alt_variance=float(message.terrain_alt_variance),
                airspeed_variance=float(message.airspeed_variance),
                updated_monotonic=now,
            )
        elif message_type == "RC_CHANNELS":
            channels = [
                _valid_pwm(getattr(message, f"chan{index}_raw", 65535))
                for index in range(1, 19)
            ]
            self.store.update(
                "rc",
                channels_pwm=channels,
                channel_count=int(message.chancount),
                rssi=(
                    None if int(message.rssi) == 255 else int(message.rssi)
                ),
                time_boot_ms=int(message.time_boot_ms),
                updated_monotonic=now,
            )
            settings = self._obstacle_settings
            if settings is not None:
                channel_index = settings.rc_toggle.channel - 1
                self._set_rc_toggle_pwm(channels[channel_index])
        elif message_type == "SERVO_OUTPUT_RAW":
            outputs = [
                _valid_pwm(getattr(message, f"servo{index}_raw", 0))
                for index in range(1, 17)
            ]
            self.store.update(
                "actuators",
                servo_pwm=outputs,
                port=int(message.port),
                time_usec=int(message.time_usec),
                updated_monotonic=now,
            )
        elif message_type == "POSITION_TARGET_LOCAL_NED":
            self.store.update(
                "position_target",
                coordinate_frame=int(message.coordinate_frame),
                type_mask=int(message.type_mask),
                x_m=float(message.x),
                y_m=float(message.y),
                z_m=float(message.z),
                vx_mps=float(message.vx),
                vy_mps=float(message.vy),
                vz_mps=float(message.vz),
                afx_mss=float(message.afx),
                afy_mss=float(message.afy),
                afz_mss=float(message.afz),
                yaw_rad=float(message.yaw),
                yaw_rate_rads=float(message.yaw_rate),
                time_boot_ms=int(message.time_boot_ms),
                updated_monotonic=now,
            )
        elif message_type == "EXTENDED_SYS_STATE":
            self.store.update(
                "vehicle",
                landed_state=int(message.landed_state),
            )


class Im10aSource(threading.Thread):
    """Read the external IM10A into the future ROS IMU contract."""

    def __init__(
        self,
        store: TelemetryStore,
        stop_event: threading.Event,
        endpoint: str,
        baud: int,
    ) -> None:
        super().__init__(name="im10a-serial", daemon=True)
        self.store = store
        self.stop_event = stop_event
        self.endpoint = endpoint
        self.baud = baud

    def run(self) -> None:
        try:
            import serial
        except ImportError as exc:
            self.store.update(
                "ros_imu",
                connected=False,
                detail=f"pyserial unavailable: {exc}",
            )
            return

        while not self.stop_event.is_set():
            port = None
            try:
                self.store.update(
                    "ros_imu",
                    connected=False,
                    detail=f"Connecting to {self.endpoint} at {self.baud}",
                )
                port = serial.Serial(
                    self.endpoint,
                    self.baud,
                    timeout=0.25,
                    exclusive=True,
                )
                port.reset_input_buffer()
                decoder = Im10aDecoder()
                rate_frames = 0
                rate_started = time.monotonic()
                current_sensor_time_s: float | None = None
                while not self.stop_event.is_set():
                    data = port.read(max(1, port.in_waiting))
                    if not data:
                        continue
                    for measurement in decoder.feed(data):
                        now = time.monotonic()
                        if measurement.kind == "sensor_time":
                            current_sensor_time_s = measurement.sensor_time_s
                        self.store.publish_raw(
                            "external_imu",
                            measurement.kind,
                            {
                                "values": list(measurement.values),
                                "sensor_time_s": current_sensor_time_s,
                                "checksum_errors": decoder.checksum_errors,
                            },
                        )
                        common = {
                            "connected": True,
                            "detail": "Receiving IM10A serial",
                            "checksum_errors": decoder.checksum_errors,
                            "updated_monotonic": now,
                        }
                        if measurement.kind == "accel_mss":
                            x, y, z = measurement.values
                            self.store.update(
                                "ros_imu",
                                **common,
                                accel_x_mss=x,
                                accel_y_mss=y,
                                accel_z_mss=z,
                            )
                        elif measurement.kind == "gyro_rads":
                            x, y, z = measurement.values
                            rate_frames += 1
                            elapsed = now - rate_started
                            rate_hz = 0.0
                            if elapsed >= 1.0:
                                rate_hz = rate_frames / elapsed
                                rate_frames = 0
                                rate_started = now
                            values: dict[str, Any] = {
                                **common,
                                "gyro_x_rads": x,
                                "gyro_y_rads": y,
                                "gyro_z_rads": z,
                                "sensor_time_s": current_sensor_time_s,
                            }
                            if rate_hz > 0.0:
                                values["sample_rate_hz"] = rate_hz
                            self.store.update(
                                "ros_imu",
                                **values,
                            )
                        elif measurement.kind == "euler_rad":
                            roll, pitch, yaw = measurement.values
                            self.store.update(
                                "ros_imu",
                                **common,
                                roll_rad=roll,
                                pitch_rad=pitch,
                                yaw_rad=yaw,
                            )
                        elif measurement.kind == "quaternion_wxyz":
                            values: dict[str, Any] = {
                                **common,
                                "quaternion_wxyz": list(measurement.values),
                            }
                            self.store.update("ros_imu", **values)
            except (OSError, serial.SerialException, ValueError) as exc:
                self.store.update(
                    "ros_imu",
                    connected=False,
                    detail=str(exc),
                )
                self.stop_event.wait(2.0)
            finally:
                if port is not None:
                    port.close()


class NavigationRuntimeSource(threading.Thread):
    """Mirror navigator status without opening any flight hardware."""

    def __init__(
        self,
        store: TelemetryStore,
        trajectory_store: NavigationTrajectoryStore,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name="navigation-runtime-source", daemon=True)
        self.store = store
        self.trajectory_store = trajectory_store
        self.stop_event = stop_event

    def run(self) -> None:
        while not self.stop_event.is_set():
            payload = self.trajectory_store.snapshot()
            if not payload.get("available") or not payload.get("live"):
                self.store.set_link(
                    False,
                    str(payload.get("detail") or "SLAM runtime is offline"),
                )
                self.stop_event.wait(0.20)
                continue
            now = time.monotonic()
            cube = payload.get("cube", {})
            estimator = payload.get("estimator", {})
            obstacles = payload.get("obstacles", {})
            self.store.update(
                "link",
                connected=True,
                detail="SLAM navigation runtime",
                last_packet_monotonic=now,
            )
            self.store.update(
                "vehicle",
                armed=bool(cube.get("armed")),
                mode=str(cube.get("mode") or "UNKNOWN"),
            )
            if all(
                isinstance(cube.get(key), (int, float))
                for key in ("roll_rad", "pitch_rad", "yaw_rad")
            ):
                self.store.update(
                    "attitude",
                    roll_rad=float(cube["roll_rad"]),
                    pitch_rad=float(cube["pitch_rad"]),
                    yaw_rad=float(cube["yaw_rad"]),
                    updated_monotonic=now,
                )
            monitor_flu = estimator.get("monitor_position_local_flu_m")
            monitor_age_ms = estimator.get("monitor_pose_age_ms")
            local_flu = (
                monitor_flu
                if isinstance(monitor_flu, list)
                and len(monitor_flu) == 3
                and isinstance(monitor_age_ms, (int, float))
                and 0.0 <= float(monitor_age_ms) <= 1000.0
                else estimator.get("position_local_flu_m")
            )
            if isinstance(local_flu, list) and len(local_flu) == 3:
                self.store.update(
                    "local_position",
                    x_m=float(local_flu[0]),
                    y_m=-float(local_flu[1]),
                    z_down_m=-float(local_flu[2]),
                    source=(
                        "lio_monitor"
                        if local_flu is monitor_flu
                        else "lio_guarded"
                    ),
                    updated_monotonic=now,
                )
            else:
                local_ned = cube.get("local_position_ned_m")
                if isinstance(local_ned, list) and len(local_ned) == 3:
                    self.store.update(
                        "local_position",
                        x_m=float(local_ned[0]),
                        y_m=float(local_ned[1]),
                        z_down_m=float(local_ned[2]),
                        updated_monotonic=now,
                    )
            flow_quality = cube.get("flow_quality")
            if isinstance(flow_quality, (int, float)):
                self.store.update(
                    "flow",
                    quality=int(flow_quality),
                    updated_monotonic=now,
                )
            distance_m = cube.get("downward_range_m")
            if isinstance(distance_m, (int, float)):
                self.store.update(
                    "range",
                    distance_m=float(distance_m),
                    updated_monotonic=now,
                )
            voltage_v = cube.get("battery_voltage_v")
            if isinstance(voltage_v, (int, float)):
                self.store.update(
                    "power",
                    source="SLAM_RUNTIME",
                    voltage_v=float(voltage_v),
                    updated_monotonic=now,
                )
            nearest_m = obstacles.get("nearest_distance_m")
            breached = bool(obstacles.get("clearance_breached"))
            hard_clearance_m = obstacles.get("hard_clearance_m")
            self.store.update(
                "obstacles",
                stage=str(payload.get("stage") or "unknown"),
                source="slam_runtime",
                nearest_distance_m=nearest_m,
                hard_cg_clearance_m=hard_clearance_m,
                clearance_status=(
                    "unknown"
                    if nearest_m is None
                    else "breach" if breached else "clear"
                ),
                clearance_margin_m=(
                    None
                    if not isinstance(nearest_m, (int, float))
                    or not isinstance(hard_clearance_m, (int, float))
                    else float(nearest_m) - float(hard_clearance_m)
                ),
                clearance_breached=breached,
                updated_monotonic=now,
            )
            self.stop_event.wait(0.05)


class DemoSource(threading.Thread):
    """Deterministic animated telemetry for UI and screenshot validation."""

    def __init__(
        self, store: TelemetryStore, stop_event: threading.Event
    ) -> None:
        super().__init__(name="demo-telemetry", daemon=True)
        self.store = store
        self.stop_event = stop_event

    def run(self) -> None:
        started = time.monotonic()
        self.store.update(
            "vehicle",
            system_id=1,
            component_id=1,
            armed=False,
            mode="DEMO",
        )
        while not self.stop_event.wait(0.025):
            now = time.monotonic()
            phase = now - started
            flow_x = 0.38 * math.sin(phase * 0.72)
            flow_y = 0.24 * math.cos(phase * 0.91)
            distance = 1.35 + 0.22 * math.sin(phase * 0.23)
            self.store.mark_packet()
            self.store.update(
                "flow",
                delta_x_dpix=round(flow_x * 100),
                delta_y_dpix=round(flow_y * 100),
                rate_x_rads=flow_x,
                rate_y_rads=flow_y,
                comp_x_mps=flow_x * 0.86,
                comp_y_mps=flow_y * 0.86,
                quality=round(176 + 44 * math.sin(phase * 0.31)),
                updated_monotonic=now,
            )
            self.store.update(
                "range",
                distance_m=distance,
                min_m=0.08,
                max_m=30.0,
                updated_monotonic=now,
            )
            self.store.update(
                "attitude",
                roll_rad=math.radians(7.0) * math.sin(phase * 0.52),
                pitch_rad=math.radians(5.0) * math.cos(phase * 0.43),
                yaw_rad=(phase * 0.08) % (2.0 * math.pi),
                rollspeed_rads=0.064 * math.cos(phase * 0.52),
                pitchspeed_rads=-0.038 * math.sin(phase * 0.43),
                yawspeed_rads=0.08,
                updated_monotonic=now,
            )
            self.store.update(
                "local_position",
                x_m=1.25 * math.sin(phase * 0.13),
                y_m=0.85 * math.cos(phase * 0.11),
                z_down_m=-distance,
                vx_mps=0.1625 * math.cos(phase * 0.13),
                vy_mps=-0.0935 * math.sin(phase * 0.11),
                vz_mps=-0.0506 * math.cos(phase * 0.23),
                time_boot_ms=round(phase * 1000),
                updated_monotonic=now,
            )
            self.store.update(
                "imu",
                accel_x_mss=0.42 * math.sin(phase * 0.8),
                accel_y_mss=0.31 * math.cos(phase * 0.67),
                accel_z_mss=-9.80665 + 0.18 * math.sin(phase * 0.44),
                gyro_x_rads=0.064 * math.cos(phase * 0.52),
                gyro_y_rads=-0.038 * math.sin(phase * 0.43),
                gyro_z_rads=0.08,
                message="DEMO_IMU",
                updated_monotonic=now,
            )
            self.store.update(
                "ros_imu",
                connected=True,
                detail="Demo ROS IMU",
                sample_rate_hz=40.0,
                checksum_errors=0,
                accel_x_mss=0.5 * math.sin(phase * 0.74),
                accel_y_mss=0.3 * math.cos(phase * 0.61),
                accel_z_mss=9.80665,
                gyro_x_rads=0.12 * math.cos(phase * 0.36),
                gyro_y_rads=0.08 * math.sin(phase * 0.41),
                gyro_z_rads=0.1,
                roll_rad=math.radians(14.0) * math.sin(phase * 0.36),
                pitch_rad=math.radians(9.0) * math.cos(phase * 0.41),
                yaw_rad=(phase * 0.1) % (2.0 * math.pi),
                updated_monotonic=now,
            )


def make_handler(
    store: TelemetryStore,
    static_dir: Path,
    spatial_store: SpatialFrameStore | None = None,
    trajectory_store: NavigationTrajectoryStore | None = None,
    cue_store: VisualCueStore | None = None,
) -> type[SimpleHTTPRequestHandler]:
    class VisualizerHandler(SimpleHTTPRequestHandler):
        server_version = "OptFlowVisualizer/0.1"

        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def do_GET(self) -> None:
            if self.path == "/api/snapshot":
                self._send_json(store.snapshot())
                return
            if self.path == "/api/stream":
                self._send_event_stream()
                return
            if self.path == "/api/events":
                self._send_raw_event_stream()
                return
            if self.path == "/api/spatial":
                self._send_spatial_stream()
                return
            if self.path == "/api/trajectory":
                self._send_trajectory_stream()
                return
            if self.path == "/api/cue":
                self._send_visual_cue_stream()
                return
            if self.path == "/healthz":
                self._send_json(
                    {
                        "ok": True,
                        "spatial": (
                            None
                            if spatial_store is None
                            else spatial_store.snapshot()
                        ),
                        "trajectory": (
                            None
                            if trajectory_store is None
                            else trajectory_store.snapshot()
                        ),
                        "cue": (
                            None if cue_store is None else cue_store.snapshot()
                        ),
                    }
                )
                return
            super().do_GET()

        def do_POST(self) -> None:
            if self.path != "/api/cue":
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            if cue_store is None:
                self._send_json(
                    {"error": "visual cue channel is unavailable"},
                    status=HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                if not 1 <= content_length <= 4096:
                    raise ValueError("request body must contain 1 to 4096 bytes")
                payload = json.loads(self.rfile.read(content_length))
                if not isinstance(payload, dict):
                    raise ValueError("request body must be a JSON object")
                cue = cue_store.trigger(
                    payload.get("message", ""),
                    detail=payload.get("detail", ""),
                    flash_count=payload.get("flash_count", 2),
                    duration_s=payload.get("duration_s", 10.0),
                )
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self._send_json(
                    {"error": str(exc)}, status=HTTPStatus.BAD_REQUEST
                )
                return
            self._send_json(cue, status=HTTPStatus.ACCEPTED)

        def end_headers(self) -> None:
            if self.path.startswith("/api/"):
                self.send_header("Cache-Control", "no-store")
            super().end_headers()

        def log_message(self, format: str, *args: Any) -> None:
            if self.path not in (
                "/api/stream",
                "/api/events",
                "/api/spatial",
                "/api/trajectory",
                "/api/cue",
                "/api/snapshot",
            ):
                super().log_message(format, *args)

        def _send_json(
            self,
            payload: dict[str, Any],
            *,
            status: HTTPStatus = HTTPStatus.OK,
        ) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_event_stream(self) -> None:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
            )
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(
                        store.snapshot(), separators=(",", ":")
                    )
                    self.wfile.write(f"data:{payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(1.0 / TELEMETRY_STREAM_HZ)
            except (BrokenPipeError, ConnectionResetError):
                return

        def _send_spatial_stream(self) -> None:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
            )
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            if spatial_store is None:
                payload = json.dumps(
                    {
                        "kind": "snapshot",
                        "sequence": 0,
                        "sources": {},
                        "disabled": True,
                    },
                    separators=(",", ":"),
                )
                try:
                    self.wfile.write(
                        f"data:{payload}\n\n".encode("utf-8")
                    )
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return

            sequence = spatial_store.latest_sequence()
            initial = json.dumps(
                spatial_store.snapshot(), separators=(",", ":")
            )
            try:
                self.wfile.write(
                    f"data:{initial}\n\n".encode("utf-8")
                )
                self.wfile.flush()
                while True:
                    events, dropped = spatial_store.wait_after(sequence)
                    if not events:
                        self.wfile.write(b":keepalive\n\n")
                        self.wfile.flush()
                        continue
                    for index, event in enumerate(events):
                        payload = dict(event)
                        if index == 0 and dropped:
                            payload["dropped_before"] = dropped
                        encoded = json.dumps(
                            payload, separators=(",", ":")
                        )
                        self.wfile.write(
                            f"data:{encoded}\n\n".encode("utf-8")
                        )
                        sequence = int(event["sequence"])
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

        def _send_trajectory_stream(self) -> None:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
            )
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            try:
                while True:
                    payload = (
                        {
                            "kind": "trajectory",
                            "available": False,
                            "detail": "trajectory monitor disabled",
                            "trajectories": {},
                        }
                        if trajectory_store is None
                        else trajectory_store.snapshot()
                    )
                    encoded = json.dumps(payload, separators=(",", ":"))
                    self.wfile.write(
                        f"data:{encoded}\n\n".encode("utf-8")
                    )
                    self.wfile.flush()
                    time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError):
                return

        def _send_visual_cue_stream(self) -> None:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
            )
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            try:
                while True:
                    payload = (
                        {
                            "kind": "visual_cue",
                            "sequence": 0,
                            "active": False,
                        }
                        if cue_store is None
                        else cue_store.snapshot()
                    )
                    encoded = json.dumps(payload, separators=(",", ":"))
                    self.wfile.write(
                        f"data:{encoded}\n\n".encode("utf-8")
                    )
                    self.wfile.flush()
                    time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError):
                return

        def _send_raw_event_stream(self) -> None:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
            )
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            sequence = store.raw_events.latest_sequence()
            try:
                while True:
                    events, dropped = store.raw_events.wait_after(sequence)
                    if not events:
                        self.wfile.write(b":keepalive\n\n")
                        self.wfile.flush()
                        continue
                    for index, event in enumerate(events):
                        payload = dict(event)
                        if index == 0 and dropped:
                            payload["dropped_before"] = dropped
                        encoded = json.dumps(
                            payload, separators=(",", ":")
                        )
                        self.wfile.write(
                            f"data:{encoded}\n\n".encode("utf-8")
                        )
                        sequence = int(event["sequence"])
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

    return VisualizerHandler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--serial")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--external-imu")
    parser.add_argument("--external-imu-baud", type=int)
    parser.add_argument("--no-external-imu", action="store_true")
    parser.add_argument("--no-spatial", action="store_true")
    parser.add_argument("--no-depth-cloud", action="store_true")
    parser.add_argument("--no-lidar-cloud", action="store_true")
    parser.add_argument(
        "--trajectory-file",
        type=Path,
        default=DEFAULT_TRAJECTORY_FILE,
        help="Atomic status file written by the SLAM return runtime",
    )
    parser.add_argument(
        "--trajectory-monitor",
        action="store_true",
        help="display the live SLAM runtime without opening hardware devices",
    )
    parser.add_argument(
        "--spatial-frame-dir",
        type=Path,
        default=DEFAULT_SPATIAL_FRAME_DIR,
        help="atomic D415/JT16 frames shared by the SLAM runtime",
    )
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--static-dir", type=Path, default=DEFAULT_STATIC_DIR)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
    except (ConfigError, OSError) as exc:
        print(f"Configuration error: {exc}")
        return 2
    if args.demo and args.trajectory_monitor:
        print("--demo and --trajectory-monitor are mutually exclusive")
        return 2
    static_dir = args.static_dir.resolve()
    if not (static_dir / "index.html").exists():
        print(
            f"Visualizer build not found at {static_dir}. "
            "Run: npm --prefix visualizer run build"
        )
        return 2

    stop_event = threading.Event()
    spatial_store = SpatialFrameStore()
    trajectory_store = NavigationTrajectoryStore(
        args.trajectory_file,
        demo=args.demo,
    )
    cue_store = VisualCueStore()
    mount = config.flight_controller.cube_mount
    store = TelemetryStore(
        (
            "demo"
            if args.demo
            else "slam_runtime" if args.trajectory_monitor else "cube_uart"
        ),
        cube_mount={
            "x_m": mount.x_m,
            "y_m": mount.y_m,
            "z_m": mount.z_m,
            "yaw_ccw_deg": mount.yaw_ccw_deg,
            "ahrs_orientation": mount.ahrs_orientation,
            "ahrs_orientation_name": mount.ahrs_orientation_name,
        },
        imu_axis_signs=(
            config.external_imu.body_axis_signs.x,
            config.external_imu.body_axis_signs.y,
            config.external_imu.body_axis_signs.z,
        ),
        imu_axis_map_verified=config.external_imu.axis_map_verified,
        imu_axis_map_verification=(
            config.external_imu.axis_map_verification
        ),
    )
    obstacle_settings = config.obstacle_avoidance
    store.update(
        "obstacles",
        stage=obstacle_settings.stage,
        mavlink_output_enabled=False,
        clearance_reference="aircraft_cg",
        clearance_distance_metric="horizontal_xy",
        hard_cg_clearance_m=(
            obstacle_settings.hard_cg_clearance_m
        ),
        source_stale_timeout_s=(
            obstacle_settings.source_stale_timeout_s
        ),
        clearance_status="unknown",
        sector_increment_deg=(
            obstacle_settings.sector_increment_deg
        ),
    )
    obstacle_fusion = ObstacleFusion(obstacle_settings)

    def receive_obstacle_scan(scan: ObstacleScan) -> None:
        obstacle_fusion.update(scan)
        fused = obstacle_fusion.fused(monotonic_ns=scan.monotonic_ns)
        if fused is None:
            return
        clearance = fused.assess_clearance(
            obstacle_settings.hard_cg_clearance_m
        )
        store.update(
            "obstacles",
            source=fused.source,
            valid_sector_count=fused.valid_sector_count,
            nearest_distance_m=fused.nearest_distance_m,
            clearance_status=clearance.status,
            clearance_margin_m=clearance.margin_m,
            clearance_breached=clearance.breached,
            violating_sector_count=(
                clearance.violating_sector_count
            ),
            violating_sector_angles_deg=list(
                clearance.violating_sector_angles_deg
            ),
            sector_increment_deg=fused.increment_deg,
            distances_cm=list(fused.distances_cm),
            updated_monotonic=time.monotonic(),
        )
        store.publish_raw(
            "obstacle_fusion",
            "CG_CLEARANCE_SHADOW",
            {
                "source": fused.source,
                "clearance": clearance.as_dict(),
                "mavlink_output_enabled": False,
            },
        )

    if args.trajectory_monitor:
        sources = [
            NavigationRuntimeSource(
                store,
                trajectory_store,
                stop_event,
            ),
            SpatialFrameFileSource(
                spatial_store,
                stop_event,
                args.spatial_frame_dir,
            ),
        ]
    elif args.demo:
        sources: list[threading.Thread] = [
            DemoSource(store, stop_event),
            DemoSpatialSource(spatial_store, store, stop_event),
        ]
    else:
        sources = [
            MavlinkSource(
                store,
                stop_event,
                endpoint=args.serial or config.flight_controller.endpoint,
                baud=args.baud or config.flight_controller.baud,
                source_system=(
                    config.flight_controller.companion_system_id
                ),
                source_component=(
                    config.flight_controller.companion_component_id
                ),
            )
        ]
        if not args.no_external_imu:
            sources.append(
                Im10aSource(
                    store,
                    stop_event,
                    endpoint=args.external_imu
                    or config.external_imu.symlink,
                    baud=args.external_imu_baud
                    or config.external_imu.baud,
                )
            )
        if not args.no_spatial and not args.no_depth_cloud:
            sources.append(
                RealSenseSpatialSource(
                    spatial_store,
                    stop_event,
                    config,
                    obstacle_sink=receive_obstacle_scan,
                )
            )
        else:
            spatial_store.publish_status(
                "depth_camera",
                connected=False,
                detail="D415 spatial stream disabled",
            )
        if not args.no_spatial and not args.no_lidar_cloud:
            sources.append(
                HesaiSpatialSource(
                    spatial_store,
                    stop_event,
                    config,
                    obstacle_sink=receive_obstacle_scan,
                )
            )
        else:
            spatial_store.publish_status(
                "lidar",
                connected=False,
                detail="JT16 spatial stream disabled",
            )
    for source in sources:
        source.start()

    handler = make_handler(
        store,
        static_dir,
        spatial_store,
        trajectory_store,
        cue_store,
    )
    server = ThreadingHTTPServer((args.host, args.port), handler)
    server.daemon_threads = True

    def stop_server(_signum=None, _frame=None) -> None:
        stop_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)

    url_host = "127.0.0.1" if args.host == "0.0.0.0" else args.host
    url = f"http://{url_host}:{args.port}"
    print(
        f"Optical-flow visualizer ({store.snapshot()['source']}) running at {url}"
    )
    print("This server does not arm the vehicle or send movement commands.")
    if not args.no_browser:
        threading.Timer(0.5, partial(webbrowser.open, url)).start()

    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        stop_event.set()
        server.server_close()
        for source in sources:
            source.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
