"""Run and record the JT16 + IM10A estimator with Cube output disabled."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback
from typing import Any

import numpy as np
import yaml

from .config import ConfigError, ProjectConfig, load_config
from .cube_odometry import CubeOdometryShadowLink, OdometryShadowState
from .flight_guide import FlightShadowGuide
from .lio_bridge import FAST_LIO_POINT_DTYPE
from .lio_validation import validate_lio_session
from .lio_visual_assist import LioVisualServer, LioVisualState
from .mavlink_compat import install_pymavlink_instance_guard
from .mavlink_proximity import send_horizontal_distance_sensors
from .obstacles import (
    DepthObstacleExtractor,
    ObstacleFusion,
    ObstacleScan,
    PointObstacleExtractor,
    obstacle_alert_state,
)
from .paths import PROJECT_ROOT
from .rgbd_odometry import GyroPriorBuffer, RgbdOdometryWorker
from .runtime_lock import cube_mavlink_lock
from .rtl_shadow import (
    add_control_approval_gates,
    replay_session,
    settings_from_config,
)
from .slam_poc_visual import SlamPocServer, SlamPocState
from .slam_navigation import (
    CubeGuidedVelocityLink,
    SlamReturnController,
    live_control_approval,
)
from .spatial_stream import (
    SpatialFrameFilePublisher,
    lidar_point_colors,
    voxel_sample,
)


CUBE_CONNECTED_TUNE = "MFT200L16CEG"
POC_READY_TUNE = "MFT220L16G"
FLIGHT_GUIDE_TUNE = "MFT200L32C"
OBSTACLE_BEEP_TUNE = "MFT240L32G"
FLIGHT_MESSAGE_RATES_HZ = (
    ("MAVLINK_MSG_ID_ATTITUDE", 10.0),
    ("MAVLINK_MSG_ID_LOCAL_POSITION_NED", 10.0),
    ("MAVLINK_MSG_ID_OPTICAL_FLOW", 10.0),
    ("MAVLINK_MSG_ID_DISTANCE_SENSOR", 10.0),
    ("MAVLINK_MSG_ID_RC_CHANNELS", 10.0),
    ("MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 2.0),
    ("MAVLINK_MSG_ID_GPS_RAW_INT", 2.0),
    ("MAVLINK_MSG_ID_VFR_HUD", 2.0),
    ("MAVLINK_MSG_ID_SYS_STATUS", 2.0),
    ("MAVLINK_MSG_ID_BATTERY_STATUS", 2.0),
    ("MAVLINK_MSG_ID_VIBRATION", 5.0),
    ("MAVLINK_MSG_ID_EKF_STATUS_REPORT", 5.0),
    ("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW", 5.0),
    ("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED", 5.0),
)


class NdjsonWriter:
    def __init__(self, path: Path, *, flush_every: int = 1) -> None:
        if flush_every < 1:
            raise ValueError("flush_every must be positive")
        self._output = path.open("w", encoding="utf-8")
        self._lock = threading.Lock()
        self._flush_every = flush_every
        self._since_flush = 0
        self.rows = 0

    def write(self, row: dict[str, Any]) -> None:
        with self._lock:
            self._output.write(json.dumps(row, sort_keys=True) + "\n")
            self._since_flush += 1
            if self._since_flush >= self._flush_every:
                self._output.flush()
                self._since_flush = 0
            self.rows += 1

    def close(self) -> None:
        with self._lock:
            if not self._output.closed:
                self._output.flush()
                os.fsync(self._output.fileno())
                self._output.close()


class ObstacleShadowRecorder:
    """Record source and fused sectors without exposing a Cube output path."""

    def __init__(self, config: ProjectConfig, output: NdjsonWriter) -> None:
        self.settings = config.obstacle_avoidance
        self.output = output
        self.fusion = ObstacleFusion(self.settings)
        self._lock = threading.Lock()
        self._source_counts: dict[str, int] = {}
        self._fused_count = 0
        self._breach_count = 0
        self._nearest_distance_m: float | None = None

    def receive(self, scan: ObstacleScan) -> None:
        now_ns = time.monotonic_ns()
        self.fusion.update(scan)
        fused = self.fusion.fused(monotonic_ns=now_ns)
        with self._lock:
            self._source_counts[scan.source] = (
                self._source_counts.get(scan.source, 0) + 1
            )
            self.output.write(self._row("source", scan))
            if fused is None:
                return
            clearance = fused.assess_clearance(
                self.settings.hard_cg_clearance_m
            )
            self._fused_count += 1
            if clearance.breached:
                self._breach_count += 1
            nearest = fused.nearest_distance_m
            if nearest is not None:
                self._nearest_distance_m = (
                    nearest
                    if self._nearest_distance_m is None
                    else min(self._nearest_distance_m, nearest)
                )
            self.output.write(
                {
                    **self._row("fused", fused),
                    "clearance": clearance.as_dict(),
                }
            )

    @staticmethod
    def _row(kind: str, scan: ObstacleScan) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "kind": kind,
            "host_monotonic_ns": time.monotonic_ns(),
            "host_unix_ns": time.time_ns(),
            "source": scan.source,
            "scan_monotonic_ns": scan.monotonic_ns,
            "increment_deg": scan.increment_deg,
            "min_distance_cm": scan.min_distance_cm,
            "max_distance_cm": scan.max_distance_cm,
            "valid_sector_count": scan.valid_sector_count,
            "nearest_distance_m": scan.nearest_distance_m,
            "distances_cm": list(scan.distances_cm),
        }

    def report(
        self,
        *,
        mavlink_messages_sent: int = 0,
    ) -> dict[str, Any]:
        expected = {
            name
            for name, enabled in (
                ("depth_camera", self.settings.depth_camera_enabled),
                ("lidar", self.settings.lidar_enabled),
            )
            if enabled
        }
        with self._lock:
            observed = {
                name for name, count in self._source_counts.items() if count > 0
            }
            return {
                "mode": (
                    "active_lidar_proximity"
                    if mavlink_messages_sent > 0
                    else "shadow_only"
                ),
                "mavlink_output_sent": mavlink_messages_sent > 0,
                "mavlink_messages_sent": mavlink_messages_sent,
                "expected_sources": sorted(expected),
                "source_counts": dict(sorted(self._source_counts.items())),
                "missing_sources": sorted(expected - observed),
                "fused_scans": self._fused_count,
                "breach_scans": self._breach_count,
                "nearest_distance_m": self._nearest_distance_m,
                "all_sources_observed": expected <= observed,
            }


class CubeReferenceReader(threading.Thread):
    """Record Cube telemetry and host an optional guarded link handler."""

    MESSAGE_TYPES = (
        "HEARTBEAT",
        "ATTITUDE",
        "LOCAL_POSITION_NED",
        "GLOBAL_POSITION_INT",
        "GPS_RAW_INT",
        "VFR_HUD",
        "SYS_STATUS",
        "BATTERY_STATUS",
        "STATUSTEXT",
        "OPTICAL_FLOW",
        "DISTANCE_SENSOR",
        "VIBRATION",
        "EKF_STATUS_REPORT",
        "RC_CHANNELS",
        "SERVO_OUTPUT_RAW",
        "POSITION_TARGET_LOCAL_NED",
        "GPS_GLOBAL_ORIGIN",
    )
    MAXIMUM_RECEIVE_SILENCE_S = 2.0
    RECONNECT_RETRY_S = 0.5

    def __init__(
        self,
        config: ProjectConfig,
        output: NdjsonWriter,
        stop_event: threading.Event,
        visual_state: LioVisualState | SlamPocState | None = None,
        link_handler: Any | None = None,
        request_flight_streams: bool = False,
        flight_guide: FlightShadowGuide | None = None,
    ) -> None:
        super().__init__(name="lio-cube-reference", daemon=True)
        self.config = config
        self.output = output
        self.stop_event = stop_event
        self.visual_state = visual_state
        self.link_handler = link_handler
        self.request_flight_streams = bool(request_flight_streams)
        self.flight_guide = flight_guide
        self.error: str | None = None
        self.recoverable_errors = 0
        self.last_recoverable_error: str | None = None
        self.ready_tune_sent = False
        self.ready_tune_error: str | None = None
        self.connection_tunes_sent = 0
        self.connection_tune_error: str | None = None
        self._connection_tune_pending = True
        self.stream_request_sent = False
        self.message_counts: dict[str, int] = {}
        self.guidance_messages_sent = 0
        self.guidance_tunes_sent = 0
        self.guidance_send_errors = 0
        self.reconnect_attempts = 0
        self.successful_reconnects = 0
        self.receive_stale_events = 0
        obstacle = config.obstacle_avoidance
        self._obstacle_output_enabled = bool(
            obstacle.stage == "active" and obstacle.mavlink_output_enabled
        )
        self._obstacle_max_age_ns = round(
            obstacle.source_stale_timeout_s * 1.0e9
        )
        self._obstacle_send_period_ns = round(
            1.0e9 / obstacle.target_rate_hz
        )
        self._last_obstacle_send_ns = 0
        self._obstacle_lock = threading.Lock()
        self._pending_obstacle: ObstacleScan | None = None
        self._latest_obstacle: ObstacleScan | None = None
        self._obstacle_mav: Any | None = None
        self._armed = False
        self._rc_toggle_enabled = False
        self._rc_toggle_pwm: int | None = None
        self._last_obstacle_beep_s = float("-inf")
        self.obstacle_messages_sent = 0
        self.obstacle_wire_packets_sent = 0
        self.obstacle_messages_dropped_stale = 0
        self.obstacle_beeps_sent = 0
        self.obstacle_wire_writes = 0
        self.obstacle_wire_bytes = 0
        self.obstacle_last_wire_result: int | None = None
        self.obstacle_last_wire_source: tuple[int, int] | None = None
        self.obstacle_last_wire_packet_hex: str | None = None
        self.obstacle_heartbeats_sent = 0
        self._next_obstacle_heartbeat_ns = 0

    def queue_obstacle_scan(self, scan: ObstacleScan) -> None:
        """Keep only the newest scan so proximity can never backlog."""
        with self._obstacle_lock:
            self._pending_obstacle = scan
            self._latest_obstacle = scan

    def _request_flight_message_intervals(
        self,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
    ) -> None:
        for constant_name, rate_hz in FLIGHT_MESSAGE_RATES_HZ:
            message_id = getattr(mavutil.mavlink, constant_name)
            connection.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                round(1_000_000.0 / rate_hz),
                0,
                0,
                0,
                0,
                0,
            )
        self.stream_request_sent = True

    def _send_pending_obstacle(self, connection: Any, mavutil: Any) -> bool:
        with self._obstacle_lock:
            scan = self._pending_obstacle or self._latest_obstacle
            self._pending_obstacle = None
        if scan is None or not self._obstacle_output_enabled:
            return False
        now_ns = time.monotonic_ns()
        if now_ns - self._last_obstacle_send_ns < self._obstacle_send_period_ns:
            return False
        age_ns = now_ns - scan.monotonic_ns
        if age_ns < 0 or age_ns > self._obstacle_max_age_ns:
            self.obstacle_messages_dropped_stale += 1
            return False

        obstacle_mav = self._obstacle_mav or connection.mav
        packets_sent = send_horizontal_distance_sensors(
            obstacle_mav,
            mavutil.mavlink,
            scan,
        )
        if packets_sent == 0:
            return False
        serial_port = getattr(connection, "port", None)
        flush = getattr(serial_port, "flush", None)
        if callable(flush):
            flush()
        self._last_obstacle_send_ns = now_ns
        self.obstacle_messages_sent += 1
        self.obstacle_wire_packets_sent += packets_sent
        self.output.write(
            {
                "schema_version": 1,
                "host_monotonic_ns": time.monotonic_ns(),
                "host_unix_ns": time.time_ns(),
                "type": "COMPANION_OBSTACLE_DISTANCE",
                "data": {
                    "source": scan.source,
                    "nearest_distance_m": scan.nearest_distance_m,
                    "valid_sector_count": scan.valid_sector_count,
                    "messages_sent": self.obstacle_messages_sent,
                    "transport": "DISTANCE_SENSOR_8_FACE",
                    "wire_packets_sent": self.obstacle_wire_packets_sent,
                    "wire_packets_this_scan": packets_sent,
                    "wire_protocol_version": getattr(
                        connection, "WIRE_PROTOCOL_VERSION", None
                    ),
                    "source_system": getattr(
                        obstacle_mav, "srcSystem", None
                    ),
                    "source_component": getattr(
                        obstacle_mav, "srcComponent", None
                    ),
                    "wire_writes": self.obstacle_wire_writes,
                    "wire_bytes": self.obstacle_wire_bytes,
                    "last_wire_result": self.obstacle_last_wire_result,
                    "last_wire_source": self.obstacle_last_wire_source,
                    "last_wire_packet_hex": (
                        self.obstacle_last_wire_packet_hex
                    ),
                },
            }
        )
        return True

    def _send_obstacle_heartbeat(self, mavutil: Any) -> bool:
        if self._obstacle_mav is None:
            return False
        now_ns = time.monotonic_ns()
        if now_ns < self._next_obstacle_heartbeat_ns:
            return False
        self._obstacle_mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        self.obstacle_heartbeats_sent += 1
        self._next_obstacle_heartbeat_ns = now_ns + 1_000_000_000
        return True

    def _set_obstacle_rc_pwm(self, pwm: int | None) -> None:
        settings = self.config.obstacle_avoidance.rc_toggle
        self._rc_toggle_pwm = pwm
        if pwm is None:
            return
        if pwm >= settings.engage_pwm:
            self._rc_toggle_enabled = True
        elif pwm <= settings.disengage_pwm:
            self._rc_toggle_enabled = False

    def _maybe_send_obstacle_beep(self, connection: Any) -> bool:
        settings = self.config.obstacle_avoidance
        now_ns = time.monotonic_ns()
        with self._obstacle_lock:
            scan = self._latest_obstacle
        fresh = bool(
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
        audible = bool(
            fresh
            and settings.alerts.enabled
            and self._rc_toggle_enabled
            and (self._armed or not settings.alerts.only_when_armed)
        )
        if not audible or alert.beep_rate_hz <= 0.0:
            return False
        now_s = now_ns / 1.0e9
        if now_s - self._last_obstacle_beep_s < 1.0 / alert.beep_rate_hz:
            return False
        target_system = self.config.flight_controller.system_id
        try:
            connection.mav.play_tune_send(
                target_system,
                1,
                OBSTACLE_BEEP_TUNE.encode("ascii"),
            )
        except (AttributeError, OSError, TypeError, ValueError):
            return False
        self._last_obstacle_beep_s = now_s
        self.obstacle_beeps_sent += 1
        return True

    def _mark_link_disconnected(self, reason: str) -> None:
        self.recoverable_errors += 1
        self.last_recoverable_error = reason
        self._connection_tune_pending = True
        handler = self.link_handler
        if handler is not None and hasattr(handler, "mark_disconnected"):
            handler.mark_disconnected(reason)
        self.output.write(
            {
                "schema_version": 1,
                "host_monotonic_ns": time.monotonic_ns(),
                "host_unix_ns": time.time_ns(),
                "type": "CUBE_LINK_RECOVERY",
                "data": {"state": "disconnected", "reason": reason},
            }
        )

    def _send_connection_tune(
        self,
        connection: Any,
        target_system: int,
        target_component: int,
    ) -> bool:
        if not self._connection_tune_pending:
            return False
        try:
            connection.mav.play_tune_send(
                target_system,
                target_component,
                CUBE_CONNECTED_TUNE.encode("ascii"),
            )
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            self.connection_tune_error = str(exc)
            return False

        self._connection_tune_pending = False
        self.connection_tune_error = None
        self.connection_tunes_sent += 1
        self.output.write(
            {
                "schema_version": 1,
                "host_monotonic_ns": time.monotonic_ns(),
                "host_unix_ns": time.time_ns(),
                "type": "CUBE_CONNECTION_TUNE",
                "data": {
                    "state": "sent",
                    "count": self.connection_tunes_sent,
                },
            }
        )
        return True

    def _update_and_send_flight_guide(
        self,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
        *,
        message_type: str | None = None,
        message_data: dict[str, Any] | None = None,
        mode_name: str | None = None,
    ) -> None:
        guide = self.flight_guide
        if guide is None:
            return
        now_ns = time.monotonic_ns()
        if message_type is not None and message_data is not None:
            guide.observe_cube(
                message_type,
                message_data,
                mode_name=mode_name,
                now_ns=now_ns,
            )
        ready = bool(
            isinstance(self.visual_state, SlamPocState)
            and self.visual_state.ready_for_motion()
        )
        guide.set_pipeline_ready(ready, now_ns=now_ns)
        for prompt in guide.drain_messages():
            severity_name = prompt.severity.upper()
            severity = getattr(
                mavutil.mavlink,
                f"MAV_SEVERITY_{severity_name}",
                mavutil.mavlink.MAV_SEVERITY_NOTICE,
            )
            message_sent = False
            tune_sent = False
            error: str | None = None
            try:
                connection.mav.statustext_send(
                    severity,
                    prompt.text.encode("ascii"),
                )
                message_sent = True
                self.guidance_messages_sent += 1
                if prompt.beep:
                    connection.mav.play_tune_send(
                        target_system,
                        target_component,
                        FLIGHT_GUIDE_TUNE.encode("ascii"),
                    )
                    tune_sent = True
                    self.guidance_tunes_sent += 1
            except Exception as exc:
                self.guidance_send_errors += 1
                error = str(exc)
            self.output.write(
                {
                    "schema_version": 1,
                    "host_monotonic_ns": time.monotonic_ns(),
                    "host_unix_ns": time.time_ns(),
                    "type": "JETSON_GUIDANCE",
                    "data": {
                        **prompt.as_dict(),
                        "statustext_sent": message_sent,
                        "tune_sent": tune_sent,
                        "error": error,
                    },
                }
            )

    def run(self) -> None:
        connection = None
        try:
            from pymavlink import mavutil

            install_pymavlink_instance_guard(mavutil)

            connection = mavutil.mavlink_connection(
                self.config.flight_controller.endpoint,
                baud=self.config.flight_controller.baud,
                source_system=(
                    self.config.flight_controller.companion_system_id
                ),
                source_component=(
                    self.config.flight_controller.companion_component_id
                ),
                autoreconnect=True,
                robust_parsing=True,
            )
            serial_write = connection.write

            def track_serial_write(buffer: bytes) -> int:
                payload = bytes(buffer)
                result = serial_write(buffer)
                if len(payload) >= 10 and payload[0] == 0xFD:
                    message_id = (
                        payload[7]
                        | payload[8] << 8
                        | payload[9] << 16
                    )
                    if message_id == 132:
                        self.obstacle_wire_writes += 1
                        self.obstacle_wire_bytes += len(payload)
                        self.obstacle_last_wire_result = (
                            None if result is None else int(result)
                        )
                        self.obstacle_last_wire_source = (
                            int(payload[5]),
                            int(payload[6]),
                        )
                        self.obstacle_last_wire_packet_hex = payload.hex()
                return result

            connection.write = track_serial_write
            self._obstacle_mav = mavutil.mavlink.MAVLink(
                connection,
                srcSystem=self.config.flight_controller.companion_system_id,
                srcComponent=getattr(
                    mavutil.mavlink,
                    "MAV_COMP_ID_OBSTACLE_AVOIDANCE",
                    196,
                ),
            )
            target_system = self.config.flight_controller.system_id
            target_component = 1
            consecutive_errors = 0
            autopilot_seen = False
            connection_stale = False
            last_received_ns = time.monotonic_ns()
            next_reconnect_ns = 0
            receive_silence_ns = round(
                min(
                    self.MAXIMUM_RECEIVE_SILENCE_S,
                    self.config.flight_controller.heartbeat_timeout_s,
                )
                * 1.0e9
            )
            message_types = list(self.MESSAGE_TYPES)
            if self.link_handler is not None:
                message_types.append("PARAM_VALUE")
            while not self.stop_event.is_set():
                try:
                    self._send_obstacle_heartbeat(mavutil)
                    self._send_pending_obstacle(connection, mavutil)
                    self._maybe_send_obstacle_beep(connection)
                    message = connection.recv_match(
                        type=message_types,
                        blocking=True,
                        timeout=0.05 if self.link_handler is not None else 0.25,
                    )
                except OSError as exc:
                    if not connection_stale:
                        connection_stale = True
                        self._mark_link_disconnected(
                            f"Cube receive error: {exc}"
                        )
                    message = None
                except (TypeError, ValueError) as exc:
                    self.recoverable_errors += 1
                    self.last_recoverable_error = str(exc)
                    consecutive_errors += 1
                    if consecutive_errors >= 5:
                        raise RuntimeError(
                            "Cube parser failed five consecutive reads"
                        ) from exc
                    continue
                if message is None:
                    now_ns = time.monotonic_ns()
                    if (
                        not connection_stale
                        and now_ns - last_received_ns >= receive_silence_ns
                    ):
                        connection_stale = True
                        self.receive_stale_events += 1
                        self._mark_link_disconnected(
                            "Cube receive stream silent for "
                            f"{receive_silence_ns / 1.0e9:.1f}s"
                        )
                    if connection_stale:
                        if now_ns >= next_reconnect_ns:
                            next_reconnect_ns = now_ns + round(
                                self.RECONNECT_RETRY_S * 1.0e9
                            )
                            self.reconnect_attempts += 1
                            reset = getattr(connection, "reset", None)
                            if callable(reset) and bool(reset()):
                                self.successful_reconnects += 1
                                connection_stale = False
                                autopilot_seen = False
                                last_received_ns = time.monotonic_ns()
                                self.stream_request_sent = False
                                self.output.write(
                                    {
                                        "schema_version": 1,
                                        "host_monotonic_ns": last_received_ns,
                                        "host_unix_ns": time.time_ns(),
                                        "type": "CUBE_LINK_RECOVERY",
                                        "data": {"state": "port_reopened"},
                                    }
                                )
                        self.stop_event.wait(0.02)
                        continue
                    if not autopilot_seen:
                        continue
                    self._update_and_send_flight_guide(
                        connection,
                        mavutil,
                        target_system,
                        target_component,
                    )
                    if self.link_handler is not None:
                        self.link_handler.tick(
                            connection,
                            mavutil,
                            target_system,
                            target_component,
                        )
                    continue
                last_received_ns = time.monotonic_ns()
                if connection_stale:
                    connection_stale = False
                    self.successful_reconnects += 1
                    self.stream_request_sent = False
                message_type = message.get_type()
                try:
                    message_data = message.to_dict()
                except (TypeError, ValueError) as exc:
                    self.recoverable_errors += 1
                    self.last_recoverable_error = str(exc)
                    consecutive_errors += 1
                    if consecutive_errors >= 5:
                        raise RuntimeError(
                            "Cube message conversion failed five times"
                        ) from exc
                    continue
                consecutive_errors = 0
                self.message_counts[message_type] = (
                    self.message_counts.get(message_type, 0) + 1
                )
                self.output.write(
                    {
                        "schema_version": 1,
                        "host_monotonic_ns": time.monotonic_ns(),
                        "host_unix_ns": time.time_ns(),
                        "type": message_type,
                        "data": message_data,
                    }
                )
                handler_message = True
                if message_type == "HEARTBEAT":
                    handler_message = bool(
                        message.autopilot
                        == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                        and message.get_srcComponent()
                        == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                    )
                    if handler_message:
                        autopilot_seen = True
                        target_system = message.get_srcSystem()
                        target_component = message.get_srcComponent()
                        if (
                            self.request_flight_streams
                            and not self.stream_request_sent
                            and (
                                self.link_handler is None
                                or not hasattr(
                                    self.link_handler,
                                    "ready_for_stream_request",
                                )
                                or self.link_handler.ready_for_stream_request()
                            )
                        ):
                            self._request_flight_message_intervals(
                                connection,
                                mavutil,
                                target_system,
                                target_component,
                            )
                        self._armed = bool(
                            int(message.base_mode)
                            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )
                        if not self._armed:
                            self._send_connection_tune(
                                connection,
                                target_system,
                                target_component,
                            )
                elif message_type == "PARAM_VALUE":
                    handler_message = bool(
                        message.get_srcComponent()
                        == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                    )
                elif message_type == "STATUSTEXT":
                    handler_message = bool(
                        message.get_srcSystem() == target_system
                        and message.get_srcComponent()
                        == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                    )
                elif message_type == "RC_CHANNELS":
                    handler_message = bool(
                        message.get_srcComponent()
                        == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                    )
                    if handler_message:
                        channel = (
                            self.config.obstacle_avoidance.rc_toggle.channel
                        )
                        raw_pwm = message_data.get(f"chan{channel}_raw")
                        pwm = int(raw_pwm) if raw_pwm is not None else None
                        self._set_obstacle_rc_pwm(
                            pwm
                            if pwm is not None and 0 < pwm < 65535
                            else None
                        )
                if self.visual_state is not None and handler_message:
                    self.visual_state.update_cube(message_type, message_data)
                guide_message = bool(
                    handler_message
                    and message.get_srcSystem() == target_system
                    and message.get_srcComponent() == target_component
                )
                self._update_and_send_flight_guide(
                    connection,
                    mavutil,
                    target_system,
                    target_component,
                    message_type=message_type if guide_message else None,
                    message_data=message_data if guide_message else None,
                    mode_name=(
                        mavutil.mode_string_v10(message)
                        if guide_message and message_type == "HEARTBEAT"
                        else None
                    ),
                )
                if self.link_handler is not None and handler_message:
                    handler_data = dict(message_data)
                    if message_type == "HEARTBEAT":
                        handler_data["_mode_name"] = (
                            mavutil.mode_string_v10(message)
                        )
                    self.link_handler.observe_message(
                        message_type,
                        handler_data,
                    )
                if (
                    isinstance(self.visual_state, SlamPocState)
                    and not self.ready_tune_sent
                    and self.visual_state.ready_for_motion()
                ):
                    try:
                        connection.mav.play_tune_send(
                            target_system,
                            target_component,
                            POC_READY_TUNE.encode("ascii"),
                        )
                        self.ready_tune_sent = True
                        self.visual_state.mark_ready_tune_sent()
                    except Exception as exc:
                        self.ready_tune_error = str(exc)
                if self.link_handler is not None:
                    self.link_handler.tick(
                        connection,
                        mavutil,
                        target_system,
                        target_component,
                    )
        except Exception as exc:
            self.error = f"{exc}\n{traceback.format_exc(limit=5)}"
            if self.link_handler is not None:
                self.stop_event.set()
        finally:
            if connection is not None:
                connection.close()


def _runtime_path(config: ProjectConfig) -> Path:
    path = Path(config.lidar_inertial_odometry.runtime_dir)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _resolved_fast_lio_config(
    config: ProjectConfig,
    session: Path,
    *,
    map_output_enabled: bool | None = None,
) -> Path:
    template_path = PROJECT_ROOT / "config" / "lio" / "fast_lio_jt16_shadow.yaml"
    payload = yaml.safe_load(template_path.read_text(encoding="ascii"))
    parameters = payload["/**"]["ros__parameters"]
    lio = config.lidar_inertial_odometry
    parameters["common"]["lid_topic"] = lio.pointcloud_topic
    parameters["common"]["imu_topic"] = lio.imu_topic
    parameters["common"]["time_offset_lidar_to_imu"] = (
        lio.clock_sync.time_offset_lidar_to_imu_s
    )
    save_map = (
        lio.map_output_enabled
        if map_output_enabled is None
        else bool(map_output_enabled)
    )
    parameters["publish"]["map_en"] = save_map
    parameters["pcd_save"]["pcd_save_en"] = save_map
    lidar_position = config.lidar.position_from_cg_frd_m
    imu_position = config.external_imu.position_from_cg_frd_m
    parameters["mapping"]["extrinsic_T"] = [
        lidar_position.x - imu_position.x,
        lidar_position.y - imu_position.y,
        lidar_position.z - imu_position.z,
    ]
    if save_map:
        parameters["map_file_path"] = str(session / "map.pcd")
    resolved_path = session / "fast_lio_resolved.yaml"
    resolved_path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="ascii",
    )
    return resolved_path


def _stop_process(process: subprocess.Popen[Any] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=7.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def _read_ndjson(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            rows.append(value)
    return rows


def _flight_window_rows(
    rows: list[dict[str, Any]],
    start_ns: int | None,
    end_ns: int | None,
) -> list[dict[str, Any]]:
    if start_ns is None or end_ns is None or end_ns <= start_ns:
        return []
    selected = []
    for row in rows:
        try:
            timestamp_ns = int(row["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        if start_ns <= timestamp_ns <= end_ns:
            selected.append(row)
    return selected


def _flight_shadow_metrics(
    session: Path,
    lifecycle: dict[str, Any],
) -> dict[str, Any]:
    start_ns = lifecycle.get("arm_monotonic_ns")
    end_ns = lifecycle.get("disarm_monotonic_ns")
    lio_rows = _flight_window_rows(
        _read_ndjson(session / "lio_odometry.ndjson"), start_ns, end_ns
    )
    rgbd_rows = _flight_window_rows(
        _read_ndjson(session / "rgbd_odometry.ndjson"), start_ns, end_ns
    )
    obstacle_rows = _flight_window_rows(
        _read_ndjson(session / "obstacles.ndjson"), start_ns, end_ns
    )
    cube_rows = _flight_window_rows(
        _read_ndjson(session / "cube_reference.ndjson"), start_ns, end_ns
    )

    duration_s = (
        0.0
        if start_ns is None or end_ns is None or end_ns <= start_ns
        else (end_ns - start_ns) / 1.0e9
    )
    lio_rate_hz = len(lio_rows) / duration_s if duration_s > 0.0 else 0.0
    positions = []
    for row in lio_rows:
        try:
            point = np.asarray(row["position_m"], dtype=np.float64)
        except (KeyError, TypeError, ValueError):
            continue
        if point.shape == (3,) and np.isfinite(point).all():
            positions.append(point)
    maximum_lio_jump_m = (
        max(
            float(np.linalg.norm(current - previous))
            for previous, current in zip(positions, positions[1:])
        )
        if len(positions) >= 2
        else None
    )
    tracked_rgbd = sum(bool(row.get("tracking")) for row in rgbd_rows)
    rgbd_tracking_ratio = tracked_rgbd / len(rgbd_rows) if rgbd_rows else 0.0
    obstacle_source_counts: dict[str, int] = {}
    obstacle_source_times: dict[str, list[int]] = {}
    for row in obstacle_rows:
        if row.get("kind") != "source":
            continue
        source = str(row.get("source", "unknown"))
        obstacle_source_counts[source] = (
            obstacle_source_counts.get(source, 0) + 1
        )
        try:
            obstacle_source_times.setdefault(source, []).append(
                int(row["host_monotonic_ns"])
            )
        except (KeyError, TypeError, ValueError):
            pass
    obstacle_source_timing = {}
    for source, timestamps in obstacle_source_times.items():
        timestamps.sort()
        gaps_s = np.diff(np.asarray(timestamps, dtype=np.int64)) / 1.0e9
        span_s = (timestamps[-1] - timestamps[0]) / 1.0e9
        obstacle_source_timing[source] = {
            "rate_hz": (
                (len(timestamps) - 1) / span_s if span_s > 0.0 else 0.0
            ),
            "gap_p95_s": (
                float(np.percentile(gaps_s, 95)) if len(gaps_s) else None
            ),
            "maximum_gap_s": float(np.max(gaps_s)) if len(gaps_s) else None,
        }
    cube_message_counts: dict[str, int] = {}
    flow_qualities = []
    range_distances_m = []
    for row in cube_rows:
        message_type = str(row.get("type", "unknown"))
        cube_message_counts[message_type] = (
            cube_message_counts.get(message_type, 0) + 1
        )
        data = row.get("data")
        if not isinstance(data, dict):
            continue
        if message_type == "OPTICAL_FLOW":
            try:
                flow_qualities.append(int(data["quality"]))
            except (KeyError, TypeError, ValueError):
                pass
        elif message_type == "DISTANCE_SENSOR":
            try:
                if int(data.get("orientation", -1)) == 25:
                    range_distances_m.append(
                        float(data["current_distance"]) / 100.0
                    )
            except (KeyError, TypeError, ValueError):
                pass
    return {
        "duration_s": duration_s,
        "lio_samples": len(lio_rows),
        "lio_rate_hz": lio_rate_hz,
        "maximum_lio_jump_m": maximum_lio_jump_m,
        "rgbd_frames": len(rgbd_rows),
        "rgbd_tracking_ratio": rgbd_tracking_ratio,
        "obstacle_source_counts": dict(sorted(obstacle_source_counts.items())),
        "obstacle_source_timing": dict(sorted(obstacle_source_timing.items())),
        "cube_message_counts": dict(sorted(cube_message_counts.items())),
        "flow_samples": len(flow_qualities),
        "minimum_flow_quality": min(flow_qualities) if flow_qualities else None,
        "flow_quality_p10": (
            float(np.percentile(flow_qualities, 10))
            if flow_qualities
            else None
        ),
        "range_samples": len(range_distances_m),
        "range_minimum_m": min(range_distances_m) if range_distances_m else None,
        "range_maximum_m": max(range_distances_m) if range_distances_m else None,
    }


def run_shadow(
    config: ProjectConfig,
    config_path: Path,
    *,
    output_root: Path,
    duration_s: float,
    visual_assist: bool = False,
    visual_host: str = "127.0.0.1",
    visual_port: int = 8766,
    open_browser: bool = True,
    visual_guide: str = "full",
    slam_poc: bool = False,
    cube_odometry_shadow: bool = False,
    flight_shadow: bool = False,
    slam_navigation: bool = False,
    initial_cube_parameters: dict[str, float] | None = None,
) -> tuple[Path, dict[str, Any], str]:
    purpose = (
        "SLAM navigation"
        if slam_navigation
        else "Cube odometry shadow"
        if cube_odometry_shadow
        else "SLAM flight shadow"
        if flight_shadow
        else "LIO shadow"
    )
    with cube_mavlink_lock(purpose):
        return _run_shadow_locked(
            config,
            config_path,
            output_root=output_root,
            duration_s=duration_s,
            visual_assist=visual_assist,
            visual_host=visual_host,
            visual_port=visual_port,
            open_browser=open_browser,
            visual_guide=visual_guide,
            slam_poc=slam_poc,
            cube_odometry_shadow=cube_odometry_shadow,
            flight_shadow=flight_shadow,
            slam_navigation=slam_navigation,
            initial_cube_parameters=initial_cube_parameters,
        )


def _run_shadow_locked(
    config: ProjectConfig,
    config_path: Path,
    *,
    output_root: Path,
    duration_s: float,
    visual_assist: bool = False,
    visual_host: str = "127.0.0.1",
    visual_port: int = 8766,
    open_browser: bool = True,
    visual_guide: str = "full",
    slam_poc: bool = False,
    cube_odometry_shadow: bool = False,
    flight_shadow: bool = False,
    slam_navigation: bool = False,
    initial_cube_parameters: dict[str, float] | None = None,
) -> tuple[Path, dict[str, Any], str]:
    lio = config.lidar_inertial_odometry
    if lio.stage != "shadow" or lio.pose_output_to_cube_enabled:
        raise ConfigError("LIO shadow runner requires Cube pose output disabled")
    if duration_s < 0.0:
        raise ValueError("duration must be zero (manual stop) or positive")
    if slam_navigation:
        if visual_assist or cube_odometry_shadow or flight_shadow:
            raise ValueError(
                "SLAM navigation cannot share a calibration, odometry-shadow, "
                "or flight-shadow run"
            )
        slam_poc = True
    if flight_shadow:
        if visual_assist or cube_odometry_shadow:
            raise ValueError(
                "flight shadow cannot use calibration or Cube odometry output"
            )
        if (
            config.navigation.autonomous_control_enabled
            or config.obstacle_avoidance.mavlink_output_enabled
        ):
            raise ConfigError(
                "flight shadow requires navigation and obstacle output disabled"
            )
        slam_poc = True
    if slam_poc and visual_assist:
        raise ValueError("SLAM POC and guided calibration displays are exclusive")
    if cube_odometry_shadow:
        if slam_poc or visual_assist:
            raise ValueError(
                "Cube odometry shadow is a dedicated non-visual bench run"
            )
        if not lio.odometry_shadow_to_cube_enabled:
            raise ConfigError("Cube odometry shadow output is disabled in config")
        if (
            config.navigation.autonomous_control_enabled
            or config.navigation.external_nav_to_cube_enabled
            or lio.pose_output_to_cube_enabled
        ):
            raise ConfigError(
                "Cube odometry shadow requires every active control gate disabled"
            )
        os.environ["MAVLINK20"] = "1"

    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import Imu, PointCloud2
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(
            f"ROS 2 Python runtime is unavailable: {exc}; "
            "run ./optflow build-lio"
        ) from exc

    runtime = _runtime_path(config)
    imu_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1000,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    fast_lio_binary = (
        runtime / "fastlio" / "fast_lio" / "lib" / "fast_lio" / "fastlio_mapping"
    )
    if not fast_lio_binary.is_file() or not os.access(fast_lio_binary, os.X_OK):
        raise RuntimeError(
            f"FAST-LIO2 runtime is missing: {fast_lio_binary}; "
            "run ./optflow build-lio"
        )

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_suffix = (
        "slam-navigation"
        if slam_navigation
        else "slam-flight-shadow" if flight_shadow
        else "slam-poc" if slam_poc
        else "cube-odom-shadow" if cube_odometry_shadow else "lio-shadow"
    )
    session = output_root / f"{timestamp}_{session_suffix}"
    session.mkdir(parents=True, exist_ok=False)
    odometry_output = NdjsonWriter(session / "lio_odometry.ndjson")
    diagnostics_output = NdjsonWriter(session / "lio_diagnostics.ndjson")
    cube_output = NdjsonWriter(session / "cube_reference.ndjson")
    imu_output = NdjsonWriter(
        session / "lio_imu.ndjson",
        flush_every=20,
    )
    lidar_frames_output = NdjsonWriter(
        session / "lio_lidar_frames.ndjson",
        flush_every=5,
    )
    cube_odometry_output = (
        NdjsonWriter(session / "cube_odometry_shadow.ndjson")
        if cube_odometry_shadow or slam_navigation
        else None
    )
    rgbd_output = (
        NdjsonWriter(session / "rgbd_odometry.ndjson", flush_every=10)
        if slam_poc
        else None
    )
    obstacle_output = (
        NdjsonWriter(session / "obstacles.ndjson", flush_every=5)
        if flight_shadow or slam_navigation
        else None
    )
    navigation_output = (
        NdjsonWriter(session / "slam_return.ndjson")
        if slam_navigation
        else None
    )
    obstacle_recorder = (
        ObstacleShadowRecorder(config, obstacle_output)
        if obstacle_output is not None
        else None
    )
    config_bytes = config_path.resolve().read_bytes()
    config_snapshot = session / "system_snapshot.yaml"
    config_snapshot.write_bytes(config_bytes)
    resolved_config = _resolved_fast_lio_config(
        config,
        session,
        map_output_enabled=None,
    )
    resolved_config_bytes = resolved_config.read_bytes()
    manifest_path = session / "manifest.json"
    manifest = {
        "schema_version": 1,
        "kind": "lio_shadow",
        "session_purpose": (
            "gps_denied_slam_return_runtime"
            if slam_navigation
            else "armed_slam_obstacle_shadow_flight" if flight_shadow
            else "slam_vio_proof_of_concept" if slam_poc else "lio_validation"
        ),
        "status": "recording",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "backend": lio.backend,
        "backend_revision": lio.backend_revision,
        "pose_sent_to_cube": False,
        "odometry_shadow_to_cube": cube_odometry_shadow,
        "cube_external_nav_fusion_enabled": False,
        "flight_shadow": flight_shadow,
        "obstacle_output_to_cube": bool(
            slam_navigation
            and config.obstacle_avoidance.stage == "active"
            and config.obstacle_avoidance.mavlink_output_enabled
        ),
        "velocity_output_to_cube": False,
        "cube_link_direction": (
            "guarded_guided_velocity_plus_telemetry"
            if slam_navigation
            else "telemetry_stream_request_plus_qgc_guidance_and_tunes"
            if flight_shadow
            else "telemetry_plus_ready_tune" if slam_poc
            else (
                "telemetry_plus_odometry_shadow"
                if cube_odometry_shadow
                else "read_only"
            )
        ),
        "config_source": str(config_path.resolve()),
        "config_snapshot": config_snapshot.name,
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "resolved_fast_lio_config": str(resolved_config),
        "resolved_fast_lio_config_sha256": hashlib.sha256(
            resolved_config_bytes
        ).hexdigest(),
        "visual_assist": visual_assist,
        "slam_poc": slam_poc,
        "rgbd_odometry": "opencv_dense_rgbd" if slam_poc else None,
        "rgbd_imu_rotation_prior": "im10a_gyro" if slam_poc else None,
        "navigation_enabled": slam_navigation,
        "arming_allowed": flight_shadow or slam_navigation,
        "visual_guide": visual_guide if visual_assist else None,
        "guide_result_required": visual_assist,
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    lio_visual_state = (
        LioVisualState(
            session.name,
            maximum_position_jump_m=(
                lio.validation.maximum_position_jump_m
            ),
            maximum_speed_mps=lio.validation.maximum_speed_mps,
            maximum_attitude_jump_deg=(
                lio.validation.maximum_attitude_jump_deg
            ),
            guide_kind=visual_guide,
        )
        if visual_assist
        else None
    )
    poc_state = (
        SlamPocState(
            session.name,
            return_settings=settings_from_config(config),
            allow_armed=flight_shadow or slam_navigation,
            guide_enabled=not flight_shadow and not slam_navigation,
        )
        if slam_poc
        else None
    )
    visual_state: LioVisualState | SlamPocState | None = (
        poc_state if poc_state is not None else lio_visual_state
    )
    visual_server: LioVisualServer | SlamPocServer | None = None
    if lio_visual_state is not None:
        static_dir = PROJECT_ROOT / "visualizer" / "dist"
        if not (static_dir / "lio-assist.html").is_file():
            raise RuntimeError(
                "LIO visual assist build is missing; run ./optflow build"
            )
        visual_server = LioVisualServer(
            lio_visual_state,
            static_dir,
            host=visual_host,
            port=visual_port,
            open_browser=open_browser,
        )
    elif poc_state is not None and not flight_shadow and not slam_navigation:
        static_dir = PROJECT_ROOT / "visualizer" / "dist"
        if not (static_dir / "slam-poc.html").is_file():
            raise RuntimeError("SLAM POC display is missing; run ./optflow build")
        visual_server = SlamPocServer(
            poc_state,
            static_dir,
            host=visual_host,
            port=visual_port,
            open_browser=open_browser,
        )

    gyro_buffer = GyroPriorBuffer() if slam_poc else None
    realtime_minus_monotonic_ns = time.time_ns() - time.monotonic_ns()
    cube_odometry_state = (
        OdometryShadowState(
            stale_timeout_s=config.navigation.local_pose_stale_timeout_s,
            maximum_position_jump_m=lio.validation.maximum_position_jump_m,
            maximum_attitude_jump_deg=(
                lio.validation.maximum_attitude_jump_deg
            ),
        )
        if cube_odometry_shadow or slam_navigation
        else None
    )
    cube_odometry_link = (
        CubeOdometryShadowLink(
            cube_odometry_state,
            cube_odometry_output,
            heartbeat_timeout_s=config.flight_controller.heartbeat_timeout_s,
        )
        if cube_odometry_state is not None
        and cube_odometry_output is not None
        else None
    )
    slam_navigation_link: CubeGuidedVelocityLink | None = None
    if slam_navigation:
        assert cube_odometry_state is not None
        assert navigation_output is not None
        control_permitted, approval_reason = live_control_approval(
            config, config_path
        )
        navigation_controller = SlamReturnController(
            config,
            control_permitted=control_permitted,
            approval_reason=approval_reason,
        )
        status_path = Path(config.navigation.slam_return.status_file)
        if not status_path.is_absolute():
            status_path = PROJECT_ROOT / status_path
        slam_navigation_link = CubeGuidedVelocityLink(
            navigation_controller,
            cube_odometry_state,
            navigation_output,
            status_path,
            heartbeat_timeout_s=(
                config.flight_controller.heartbeat_timeout_s
            ),
        )
        if initial_cube_parameters:
            slam_navigation_link.parameters.update(
                {
                    name: float(value)
                    for name, value in initial_cube_parameters.items()
                    if name in slam_navigation_link.audit_parameters
                    and math.isfinite(float(value))
                }
            )
        manifest.update(
            navigation_control_permitted=control_permitted,
            navigation_approval_reason=approval_reason,
            velocity_output_to_cube=control_permitted,
            navigation_status_file=str(status_path),
        )
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    cube_link_handler = slam_navigation_link or cube_odometry_link
    active_obstacle_fusion = (
        ObstacleFusion(config.obstacle_avoidance)
        if slam_navigation
        and config.obstacle_avoidance.stage == "active"
        and config.obstacle_avoidance.mavlink_output_enabled
        else None
    )

    class NavigationRgbdSink:
        def update_rgbd(self, row: dict[str, Any]) -> None:
            if poc_state is not None:
                poc_state.update_rgbd(row)
            if slam_navigation_link is not None:
                slam_navigation_link.observe_visual(row)

        def update_rgbd_map(
            self, points_m: np.ndarray, colors_rgb: np.ndarray
        ) -> None:
            if poc_state is not None:
                poc_state.update_rgbd_map(points_m, colors_rgb)

        def set_rgbd_error(self, detail: str) -> None:
            if poc_state is not None:
                poc_state.set_rgbd_error(detail)

    rgbd_state_sink = (
        NavigationRgbdSink() if slam_navigation_link is not None else poc_state
    )

    def receive_obstacle_scan(scan: ObstacleScan) -> None:
        if obstacle_recorder is not None:
            obstacle_recorder.receive(scan)
        if slam_navigation_link is not None:
            slam_navigation_link.observe_obstacle(scan)
        if active_obstacle_fusion is not None:
            active_obstacle_fusion.update(scan)
            fused = active_obstacle_fusion.fused(
                monotonic_ns=time.monotonic_ns()
            )
            if fused is not None:
                cube_reader.queue_obstacle_scan(fused)

    depth_obstacle_extractor = (
        DepthObstacleExtractor(
            config.obstacle_avoidance,
            config.depth_camera,
        )
        if obstacle_recorder is not None
        and config.obstacle_avoidance.depth_camera_enabled
        else None
    )
    lidar_obstacle_extractor = (
        PointObstacleExtractor(
            config.obstacle_avoidance,
            source="lidar",
        )
        if obstacle_recorder is not None
        and config.obstacle_avoidance.lidar_enabled
        else None
    )
    lidar_position = config.lidar.position_from_cg_frd_m
    lidar_translation_frd_m = np.asarray(
        (lidar_position.x, lidar_position.y, lidar_position.z),
        dtype=np.float64,
    )
    spatial_publisher = (
        SpatialFrameFilePublisher() if slam_navigation else None
    )
    if spatial_publisher is not None:
        spatial_publisher.reset()

    class ShadowRecorderNode(Node):
        def __init__(self) -> None:
            super().__init__("optflow_lio_shadow_recorder")
            self.lidar_spatial_started_s = time.monotonic()
            self.lidar_spatial_frames = 0
            self.safety_callback_group = MutuallyExclusiveCallbackGroup()
            self.imu_callback_group = MutuallyExclusiveCallbackGroup()
            self.lidar_callback_group = MutuallyExclusiveCallbackGroup()
            self.create_subscription(
                Odometry,
                lio.odometry_topic,
                self.record_odometry,
                qos_profile_sensor_data,
                callback_group=self.safety_callback_group,
            )
            self.create_subscription(
                String,
                lio.diagnostics_topic,
                self.record_diagnostics,
                20,
                callback_group=self.safety_callback_group,
            )
            self.create_subscription(
                Imu,
                lio.imu_topic,
                self.record_imu,
                imu_qos,
                callback_group=self.imu_callback_group,
            )
            self.create_subscription(
                PointCloud2,
                lio.pointcloud_topic,
                self.record_lidar_frame,
                qos_profile_sensor_data,
                callback_group=self.lidar_callback_group,
            )

        def record_odometry(self, message: Any) -> None:
            stamp = message.header.stamp
            ros_time_ns = (
                int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            )
            position_m = [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ]
            quaternion_xyzw = [
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ]
            odometry_output.write(
                {
                    "schema_version": 1,
                    "host_monotonic_ns": time.monotonic_ns(),
                    "host_unix_ns": time.time_ns(),
                    "ros_time_ns": ros_time_ns,
                    "frame_id": message.header.frame_id,
                    "child_frame_id": message.child_frame_id,
                    "position_m": position_m,
                    "quaternion_xyzw": quaternion_xyzw,
                    "linear_velocity_mps": [
                        message.twist.twist.linear.x,
                        message.twist.twist.linear.y,
                        message.twist.twist.linear.z,
                    ],
                    "angular_velocity_rads": [
                        message.twist.twist.angular.x,
                        message.twist.twist.angular.y,
                        message.twist.twist.angular.z,
                    ],
                    "pose_covariance": list(message.pose.covariance),
                    "twist_covariance": list(message.twist.covariance),
                }
            )
            if cube_odometry_state is not None:
                cube_odometry_state.update_odometry(
                    host_monotonic_ns=time.monotonic_ns(),
                    ros_time_ns=ros_time_ns,
                    frame_id=message.header.frame_id,
                    child_frame_id=message.child_frame_id,
                    position_m=position_m,
                    quaternion_xyzw=quaternion_xyzw,
                    pose_covariance=message.pose.covariance,
                )
            if visual_state is not None:
                visual_state.update_odometry(
                    position_m,
                    timestamp_ns=ros_time_ns,
                    quaternion_xyzw=quaternion_xyzw,
                )

        def record_diagnostics(self, message: Any) -> None:
            try:
                diagnostics = json.loads(message.data)
            except json.JSONDecodeError:
                diagnostics = {"parse_error": True, "raw": message.data}
            diagnostics_output.write(
                {
                    "schema_version": 1,
                    "host_monotonic_ns": time.monotonic_ns(),
                    "host_unix_ns": time.time_ns(),
                    "diagnostics": diagnostics,
                }
            )
            if cube_odometry_state is not None:
                cube_odometry_state.update_diagnostics(
                    time.monotonic_ns(), diagnostics
                )
            if visual_state is not None:
                visual_state.update_diagnostics(diagnostics)

        def record_imu(self, message: Any) -> None:
            stamp = message.header.stamp
            host_monotonic_ns = time.monotonic_ns()
            angular_velocity_rads = [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ]
            imu_output.write(
                {
                    "schema_version": 1,
                    "host_monotonic_ns": host_monotonic_ns,
                    "host_unix_ns": time.time_ns(),
                    "ros_time_ns": (
                        int(stamp.sec) * 1_000_000_000
                        + int(stamp.nanosec)
                    ),
                    "linear_acceleration_mpss": [
                        message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z,
                    ],
                    "angular_velocity_rads": angular_velocity_rads,
                }
            )
            if cube_odometry_state is not None:
                cube_odometry_state.update_imu(
                    host_monotonic_ns,
                    angular_velocity_rads,
                )
            if gyro_buffer is not None:
                ros_time_ns = (
                    int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                )
                gyro_buffer.add(
                    ros_time_ns - realtime_minus_monotonic_ns,
                    angular_velocity_rads,
                )

        def record_lidar_frame(self, message: Any) -> None:
            stamp = message.header.stamp
            host_monotonic_ns = time.monotonic_ns()
            point_count = int(message.width) * int(message.height)
            lidar_frames_output.write(
                {
                    "schema_version": 1,
                    "host_monotonic_ns": host_monotonic_ns,
                    "host_unix_ns": time.time_ns(),
                    "ros_time_ns": (
                        int(stamp.sec) * 1_000_000_000
                        + int(stamp.nanosec)
                    ),
                    "points": point_count,
                    "point_step": int(message.point_step),
                    "row_step": int(message.row_step),
                }
            )
            if int(message.point_step) != FAST_LIO_POINT_DTYPE.itemsize:
                return
            expected_bytes = point_count * FAST_LIO_POINT_DTYPE.itemsize
            if len(message.data) < expected_bytes:
                return
            records = np.frombuffer(
                bytes(message.data),
                dtype=FAST_LIO_POINT_DTYPE,
                count=point_count,
            )
            points_body_frd = np.column_stack(
                (records["x"], records["y"], records["z"])
            ).astype(np.float64, copy=False)
            points_body_frd += lidar_translation_frd_m
            if (
                lidar_obstacle_extractor is not None
                and obstacle_recorder is not None
            ):
                receive_obstacle_scan(
                    lidar_obstacle_extractor.extract(
                        points_body_frd,
                        monotonic_ns=host_monotonic_ns,
                    )
                )
            if spatial_publisher is None:
                return
            try:
                finite = np.isfinite(points_body_frd).all(axis=1)
                distance = np.linalg.norm(points_body_frd, axis=1)
                valid = finite & (distance >= 0.25) & (distance <= 20.0)
                display_points = points_body_frd[valid].astype(
                    np.float32, copy=False
                )
                intensities = np.clip(
                    records["intensity"][valid], 0.0, 255.0
                )
                colors = lidar_point_colors(display_points, intensities)
                input_points = len(display_points)
                display_points, colors = voxel_sample(
                    display_points,
                    colors,
                    voxel_size_m=0.06,
                    max_points=8_000,
                )
                self.lidar_spatial_frames += 1
                rate_hz = self.lidar_spatial_frames / max(
                    0.001,
                    time.monotonic() - self.lidar_spatial_started_s,
                )
                spatial_publisher.publish_frame(
                    "lidar",
                    display_points,
                    colors,
                    input_points=input_points,
                    frame_rate_hz=rate_hz,
                    frame_monotonic_ns=host_monotonic_ns,
                    detail="JT16 cloud shared by SLAM runtime",
                )
            except (OSError, TypeError, ValueError):
                pass

    stop_event = threading.Event()

    def publish_depth_cloud(
        points_body_frd: np.ndarray,
        colors_rgb: np.ndarray,
        input_points: int,
        frame_rate_hz: float,
        frame_monotonic_ns: int,
    ) -> None:
        if spatial_publisher is None:
            return
        points, colors = voxel_sample(
            points_body_frd,
            colors_rgb,
            voxel_size_m=0.04,
            max_points=6_000,
        )
        spatial_publisher.publish_frame(
            "depth_camera",
            points,
            colors,
            input_points=input_points,
            frame_rate_hz=frame_rate_hz,
            frame_monotonic_ns=frame_monotonic_ns,
            detail="D415 cloud shared by SLAM runtime",
        )

    flight_guide = FlightShadowGuide() if flight_shadow else None
    cube_reader = CubeReferenceReader(
        config,
        cube_output,
        stop_event,
        visual_state,
        cube_link_handler,
        request_flight_streams=flight_shadow or slam_navigation,
        flight_guide=flight_guide,
    )
    rgbd_worker = (
        RgbdOdometryWorker(
            config.depth_camera,
            rgbd_output,
            session / "rgbd_map.ply",
            stop_event,
            gyro_buffer=gyro_buffer,
            state_sink=rgbd_state_sink,
            obstacle_extractor=depth_obstacle_extractor,
            obstacle_sink=(
                receive_obstacle_scan
                if obstacle_recorder is not None
                and depth_obstacle_extractor is not None
                else None
            ),
            obstacle_rate_hz=config.obstacle_avoidance.target_rate_hz,
            cloud_sink=(
                publish_depth_cloud
                if spatial_publisher is not None
                else None
            ),
        )
        if slam_poc and rgbd_output is not None and poc_state is not None
        else None
    )
    fast_lio_process: subprocess.Popen[Any] | None = None
    bridge_process: subprocess.Popen[Any] | None = None
    fast_lio_log = (session / "fast_lio.log").open("wb")
    bridge_process_log = (session / "sensor_bridge.log").open("wb")
    rclpy.init()
    node = ShadowRecorderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def request_stop(_signum: int | None = None, _frame: Any = None) -> None:
        stop_event.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    started_s = time.monotonic()
    failure: str | None = None
    visual_failure: dict[str, Any] | None = None
    flight_ready_announced = False
    try:
        if visual_server is not None:
            visual_server.start()
            print(
                (
                    f"SLAM/VIO proof running at {visual_server.url}"
                    if slam_poc
                    else f"LIO visual assist running at {visual_server.url}"
                ),
                flush=True,
            )
        fast_lio_process = subprocess.Popen(
            (
                str(fast_lio_binary),
                "--ros-args",
                "--params-file",
                str(resolved_config),
            ),
            stdin=subprocess.DEVNULL,
            stdout=fast_lio_log,
            stderr=subprocess.STDOUT,
        )
        bridge_command = [
                sys.executable,
                "-m",
                "optflow_slam.lio_bridge",
                "--config",
                str(config_path),
                "--bridge-log",
                str(session / "jt16_bridge.log"),
        ]
        if slam_poc:
            bridge_command.append("--proof-mode")
        bridge_process = subprocess.Popen(
            bridge_command,
            stdin=subprocess.DEVNULL,
            stdout=bridge_process_log,
            stderr=subprocess.STDOUT,
        )
        cube_reader.start()
        if rgbd_worker is not None:
            rgbd_worker.start()
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
            if (
                flight_shadow
                and poc_state is not None
                and not flight_ready_announced
                and poc_state.ready_for_motion()
            ):
                flight_ready_announced = True
                print(
                    "SLAM FLIGHT SHADOW READY: GPS Loiter only; "
                    "all navigation and obstacle outputs remain disabled.",
                    flush=True,
                )
            if (
                flight_shadow
                and poc_state is not None
                and poc_state.flight_complete(post_disarm_s=3.0)
            ):
                print("Disarm detected; saving the flight shadow.", flush=True)
                stop_event.set()
            if (
                slam_navigation
                and poc_state is not None
                and not flight_ready_announced
                and poc_state.ready_for_motion()
            ):
                flight_ready_announced = True
                assert slam_navigation_link is not None
                status = slam_navigation_link.controller.snapshot()
                print(
                    "SLAM RETURN PIPELINE READY: "
                    + (
                        "LIVE OUTPUT APPROVED"
                        if status["live_control_permitted"]
                        else "LOCKED MONITOR MODE"
                    ),
                    flush=True,
                )
            if duration_s > 0.0 and time.monotonic() - started_s >= duration_s:
                stop_event.set()
            if visual_state is not None and visual_state.should_stop():
                visual_failure = (
                    lio_visual_state.failure_detail()
                    if lio_visual_state is not None
                    else None
                )
                if visual_failure is not None and lio_visual_state is not None:
                    failure = (
                        "visual shadow safety stop: "
                        f"{visual_failure.get('detail', 'trajectory rejected')}"
                    )
                stop_event.set()
            if fast_lio_process.poll() is not None:
                failure = (
                    f"FAST-LIO2 exited with {fast_lio_process.returncode}"
                )
                stop_event.set()
            if bridge_process.poll() is not None:
                failure = (
                    f"sensor bridge exited with {bridge_process.returncode}"
                )
                stop_event.set()
            if cube_reader.error is not None:
                failure = f"Cube link exited: {cube_reader.error.splitlines()[0]}"
                stop_event.set()
            if rgbd_worker is not None and not rgbd_worker.is_alive():
                if rgbd_worker.error is not None:
                    failure = f"RGB-D odometry exited: {rgbd_worker.error}"
                    stop_event.set()
    finally:
        stop_event.set()
        _stop_process(bridge_process)
        _stop_process(fast_lio_process)
        if rgbd_worker is not None and rgbd_worker.is_alive():
            rgbd_worker.join(timeout=5.0)
        if visual_server is not None:
            visual_server.close()
        if cube_reader.is_alive():
            cube_reader.join(timeout=3.0)
        if slam_navigation_link is not None:
            slam_navigation_link.close()
        executor.remove_node(node)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        fast_lio_log.close()
        bridge_process_log.close()
        odometry_output.close()
        diagnostics_output.close()
        cube_output.close()
        imu_output.close()
        lidar_frames_output.close()
        if rgbd_output is not None:
            rgbd_output.close()
        if cube_odometry_output is not None:
            cube_odometry_output.close()
        if obstacle_output is not None:
            obstacle_output.close()
        if navigation_output is not None:
            navigation_output.close()

    if lio_visual_state is not None:
        guide_result_path = session / "guide_result.json"
        guide_result_bytes = (
            json.dumps(
                lio_visual_state.guide_result(),
                indent=2,
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
        guide_result_path.write_bytes(guide_result_bytes)
        manifest.update(
            guide_result=guide_result_path.name,
            guide_result_sha256=hashlib.sha256(
                guide_result_bytes
            ).hexdigest(),
        )

    if rgbd_worker is not None and rgbd_worker.error is not None and failure is None:
        failure = f"RGB-D odometry failed: {rgbd_worker.error}"

    cube_odometry_report: dict[str, Any] | None = None
    cube_odometry_report_path: Path | None = None
    cube_odometry_digest: str | None = None
    if cube_odometry_link is not None:
        cube_odometry_report = cube_odometry_link.report()
        analysis_dir = session / "analysis"
        analysis_dir.mkdir(parents=True, exist_ok=True)
        cube_odometry_report_bytes = (
            json.dumps(cube_odometry_report, indent=2, sort_keys=True) + "\n"
        ).encode("utf-8")
        cube_odometry_report_path = (
            analysis_dir / "cube_odometry_shadow.json"
        )
        cube_odometry_report_path.write_bytes(cube_odometry_report_bytes)
        cube_odometry_digest = hashlib.sha256(
            cube_odometry_report_bytes
        ).hexdigest()
        (analysis_dir / "cube_odometry_shadow.sha256").write_text(
            f"{cube_odometry_digest}  {cube_odometry_report_path.name}\n",
            encoding="ascii",
        )

    obstacle_report = (
        obstacle_recorder.report(
            mavlink_messages_sent=cube_reader.obstacle_messages_sent
        )
        if obstacle_recorder is not None
        else None
    )
    flight_lifecycle = (
        poc_state.flight_lifecycle()
        if (flight_shadow or slam_navigation) and poc_state is not None
        else None
    )
    slam_navigation_report = (
        slam_navigation_link.report()
        if slam_navigation_link is not None
        else None
    )

    manifest.update(
        status="complete" if failure is None else "failed",
        ended_utc=datetime.now(timezone.utc).isoformat(),
        failure=failure,
        visual_failure=visual_failure,
        cube_reference_error=cube_reader.error,
        cube_reference_recoverable_errors=(
            cube_reader.recoverable_errors
        ),
        cube_reference_last_recoverable_error=(
            cube_reader.last_recoverable_error
        ),
        cube_receive_stale_events=cube_reader.receive_stale_events,
        cube_reconnect_attempts=cube_reader.reconnect_attempts,
        cube_successful_reconnects=cube_reader.successful_reconnects,
        cube_connection_tunes_sent=cube_reader.connection_tunes_sent,
        cube_connection_tune_error=cube_reader.connection_tune_error,
        cube_ready_tune_sent=cube_reader.ready_tune_sent,
        cube_ready_tune_error=cube_reader.ready_tune_error,
        cube_stream_request_sent=cube_reader.stream_request_sent,
        cube_obstacle_messages_sent=cube_reader.obstacle_messages_sent,
        cube_obstacle_messages_dropped_stale=(
            cube_reader.obstacle_messages_dropped_stale
        ),
        cube_obstacle_beeps_sent=cube_reader.obstacle_beeps_sent,
        qgc_guidance=(
            {
                "statustext_messages_sent": (
                    cube_reader.guidance_messages_sent
                ),
                "instruction_tunes_sent": cube_reader.guidance_tunes_sent,
                "send_errors": cube_reader.guidance_send_errors,
                "guide": flight_guide.report(),
            }
            if flight_guide is not None
            else None
        ),
        cube_message_counts=dict(sorted(cube_reader.message_counts.items())),
        flight_lifecycle=flight_lifecycle,
        obstacle_shadow=obstacle_report,
        slam_navigation=slam_navigation_report,
        velocity_output_to_cube=bool(
            slam_navigation_link is not None
            and slam_navigation_link.commands_sent > 0
        ),
        pose_sent_to_cube=(
            cube_odometry_link is not None
            and cube_odometry_link.packets_sent > 0
        ),
        cube_odometry_shadow_report=(
            str(cube_odometry_report_path.relative_to(session))
            if cube_odometry_report_path is not None
            else None
        ),
        cube_odometry_shadow_report_sha256=cube_odometry_digest,
        rows={
            "odometry": odometry_output.rows,
            "diagnostics": diagnostics_output.rows,
            "cube_reference": cube_output.rows,
            "imu": imu_output.rows,
            "lidar_frames": lidar_frames_output.rows,
            "rgbd_odometry": rgbd_output.rows if rgbd_output is not None else 0,
            "obstacles": obstacle_output.rows if obstacle_output is not None else 0,
            "slam_return": (
                navigation_output.rows
                if navigation_output is not None
                else 0
            ),
            "cube_odometry_shadow": (
                cube_odometry_output.rows
                if cube_odometry_output is not None
                else 0
            ),
        },
        rgbd={
            "frames": rgbd_worker.frames,
            "tracked_frames": rgbd_worker.tracked_frames,
            "gyro_prior_frames": rgbd_worker.gyro_prior_frames,
            "map_keyframes": rgbd_worker.map_keyframes,
            "map_points": rgbd_worker.map_points,
            "path_length_m": rgbd_worker.path_length_m,
            "obstacle_frames": rgbd_worker.obstacle_frames,
            "obstacle_errors": rgbd_worker.obstacle_errors,
            "error": rgbd_worker.error,
        }
        if rgbd_worker is not None
        else None,
    )
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    lio_report_path, lio_report, lio_digest = validate_lio_session(
        session, config
    )
    if flight_shadow and poc_state is not None:
        lifecycle = poc_state.flight_lifecycle()
        metrics = _flight_shadow_metrics(session, lifecycle)
        rtl_replay_report: dict[str, Any] | None = None
        rtl_replay_path: Path | None = None
        rtl_replay_digest: str | None = None
        rtl_replay_error: str | None = None
        if lifecycle["completed_arm_cycle"]:
            try:
                (
                    rtl_replay_path,
                    rtl_replay_report,
                    rtl_replay_digest,
                ) = replay_session(session, config)
            except (OSError, ValueError) as exc:
                rtl_replay_error = str(exc)

        expected_obstacle_sources = {
            name
            for name, enabled in (
                (
                    "depth_camera",
                    config.obstacle_avoidance.depth_camera_enabled,
                ),
                ("lidar", config.obstacle_avoidance.lidar_enabled),
            )
            if enabled
        }
        observed_obstacle_sources = {
            name
            for name, count in metrics["obstacle_source_counts"].items()
            if count > 0
        }
        primary_obstacle_source = (
            "lidar"
            if config.obstacle_avoidance.lidar_enabled
            else "depth_camera"
        )
        primary_obstacle_timing = metrics[
            "obstacle_source_timing"
        ].get(primary_obstacle_source, {})
        primary_obstacle_gap_p95_s = primary_obstacle_timing.get(
            "gap_p95_s"
        )
        maximum_jump = metrics["maximum_lio_jump_m"]
        flow_quality_p10 = metrics["flow_quality_p10"]
        gates = {
            "complete_arm_disarm_cycle": lifecycle["completed_arm_cycle"],
            "pilot_guidance_sequence_completed": bool(
                flight_guide is not None
                and flight_guide.status().get("phase") == "complete"
            ),
            "minimum_20_second_flight": metrics["duration_s"] >= 20.0,
            "lio_live_in_flight": metrics["lio_rate_hz"] >= 4.0,
            "lio_continuity_in_flight": bool(
                maximum_jump is not None
                and maximum_jump
                <= config.lidar_inertial_odometry.validation.maximum_position_jump_m
            ),
            "visual_tracking_in_flight": (
                metrics["rgbd_frames"] >= 30
                and metrics["rgbd_tracking_ratio"] >= 0.70
            ),
            "obstacle_sources_live_in_flight": (
                expected_obstacle_sources <= observed_obstacle_sources
            ),
            "primary_obstacle_source_fresh_in_flight": bool(
                primary_obstacle_gap_p95_s is not None
                and primary_obstacle_gap_p95_s
                <= config.obstacle_avoidance.source_stale_timeout_s
            ),
            "optical_flow_live_in_flight": bool(
                metrics["flow_samples"] >= 5
                and flow_quality_p10 is not None
                and flow_quality_p10
                >= config.flight_controller.hflow_min_bench_quality
            ),
            "downward_range_live_in_flight": metrics["range_samples"] >= 5,
            "local_return_shadow_passed": bool(
                rtl_replay_report is not None
                and rtl_replay_report.get("result") == "shadow_pass"
            ),
            "zero_flight_control_output": bool(
                manifest["pose_sent_to_cube"] is False
                and manifest["obstacle_output_to_cube"] is False
                and manifest["velocity_output_to_cube"] is False
                and manifest["navigation_enabled"] is False
            ),
        }
        passed = all(gates.values())
        report = {
            "schema_version": 1,
            "kind": "armed_slam_obstacle_shadow_flight",
            "result": "flight_shadow_pass" if passed else "incomplete",
            "detail": (
                "Airborne SLAM, obstacle sensing, and local-return shadow passed"
                if passed
                else "Flight shadow saved; one or more airborne gates remain open"
            ),
            "session": str(session),
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "active_control_sent": False,
            "pose_sent_to_cube": False,
            "obstacle_output_sent_to_cube": False,
            "velocity_sent_to_cube": False,
            "flight_lifecycle": lifecycle,
            "qgc_guidance": (
                {
                    "statustext_messages_sent": (
                        cube_reader.guidance_messages_sent
                    ),
                    "instruction_tunes_sent": (
                        cube_reader.guidance_tunes_sent
                    ),
                    "send_errors": cube_reader.guidance_send_errors,
                    "guide": flight_guide.report(),
                }
                if flight_guide is not None
                else None
            ),
            "metrics": metrics,
            "obstacle_shadow": obstacle_report,
            "gates": gates,
            "failed_gates": [name for name, passed in gates.items() if not passed],
            "ready_for_guarded_control_bridge_bench": passed,
            "rtl_shadow": rtl_replay_report,
            "rtl_shadow_error": rtl_replay_error,
            "lio_validation_result": lio_report.get("result"),
            "artifacts": {
                "manifest": str(manifest_path),
                "obstacles": str(session / "obstacles.ndjson"),
                "rgbd_odometry": str(session / "rgbd_odometry.ndjson"),
                "rgbd_map": str(session / "rgbd_map.ply"),
                "fast_lio_map": str(session / "map.pcd"),
                "fast_lio_validation": str(lio_report_path),
                "fast_lio_validation_sha256": lio_digest,
                "rtl_shadow_report": (
                    str(rtl_replay_path) if rtl_replay_path is not None else None
                ),
                "rtl_shadow_report_sha256": rtl_replay_digest,
            },
        }
        report_bytes = (
            json.dumps(report, indent=2, sort_keys=True) + "\n"
        ).encode("utf-8")
        analysis_dir = session / "analysis"
        analysis_dir.mkdir(parents=True, exist_ok=True)
        report_path = analysis_dir / "slam_flight_shadow.json"
        report_path.write_bytes(report_bytes)
        digest = hashlib.sha256(report_bytes).hexdigest()
        (analysis_dir / "slam_flight_shadow.sha256").write_text(
            f"{digest}  {report_path.name}\n",
            encoding="ascii",
        )
    elif slam_navigation_report is not None:
        report = slam_navigation_report
        report["session"] = str(session)
        report["flight_lifecycle"] = flight_lifecycle
        report["obstacle_shadow"] = obstacle_report
        report["lio_validation_result"] = lio_report.get("result")
        report["artifacts"] = {
            "manifest": str(manifest_path),
            "decisions": str(session / "slam_return.ndjson"),
            "obstacles": str(session / "obstacles.ndjson"),
            "rgbd_odometry": str(session / "rgbd_odometry.ndjson"),
            "fast_lio_validation": str(lio_report_path),
            "fast_lio_validation_sha256": lio_digest,
        }
        report_bytes = (
            json.dumps(report, indent=2, sort_keys=True) + "\n"
        ).encode("utf-8")
        analysis_dir = session / "analysis"
        analysis_dir.mkdir(parents=True, exist_ok=True)
        report_path = analysis_dir / "slam_navigation.json"
        report_path.write_bytes(report_bytes)
        digest = hashlib.sha256(report_bytes).hexdigest()
        (analysis_dir / "slam_navigation.sha256").write_text(
            f"{digest}  {report_path.name}\n",
            encoding="ascii",
        )
    elif poc_state is not None:
        report = poc_state.report()
        rtl_shadow_report = add_control_approval_gates(
            poc_state.rtl_shadow_report(),
            config,
        )
        analysis_dir = session / "analysis"
        analysis_dir.mkdir(parents=True, exist_ok=True)
        rtl_commands_path = analysis_dir / "rtl_shadow_commands.ndjson"
        rtl_commands_path.write_text(
            "".join(
                json.dumps(row, sort_keys=True) + "\n"
                for row in poc_state.rtl_shadow_rows()
            ),
            encoding="utf-8",
        )
        rtl_report_bytes = (
            json.dumps(rtl_shadow_report, indent=2, sort_keys=True) + "\n"
        ).encode("utf-8")
        rtl_report_path = analysis_dir / "rtl_shadow_live.json"
        rtl_report_path.write_bytes(rtl_report_bytes)
        rtl_report_digest = hashlib.sha256(rtl_report_bytes).hexdigest()
        (analysis_dir / "rtl_shadow_live.sha256").write_text(
            f"{rtl_report_digest}  {rtl_report_path.name}\n",
            encoding="utf-8",
        )
        report["local_return_shadow"] = rtl_shadow_report
        report["artifacts"] = {
            "session": str(session),
            "rgbd_odometry": "rgbd_odometry.ndjson",
            "rgbd_map": "rgbd_map.ply",
            "fast_lio_map": "map.pcd",
            "fast_lio_validation": str(lio_report_path),
            "fast_lio_validation_sha256": lio_digest,
            "rtl_shadow_commands": str(rtl_commands_path),
            "rtl_shadow_report": str(rtl_report_path),
            "rtl_shadow_report_sha256": rtl_report_digest,
        }
        if rgbd_worker is not None:
            report["metrics"]["rgbd_map_points"] = rgbd_worker.map_points
            report["metrics"]["rgbd_map_keyframes"] = (
                rgbd_worker.map_keyframes
            )
        report_bytes = (
            json.dumps(report, indent=2, sort_keys=True) + "\n"
        ).encode("utf-8")
        report_path = session / "slam_poc_report.json"
        report_path.write_bytes(report_bytes)
        digest = hashlib.sha256(report_bytes).hexdigest()
    elif (
        cube_odometry_report is not None
        and cube_odometry_report_path is not None
        and cube_odometry_digest is not None
    ):
        report_path = cube_odometry_report_path
        report = cube_odometry_report
        digest = cube_odometry_digest
    else:
        report_path, report, digest = (
            lio_report_path,
            lio_report,
            lio_digest,
        )
    if failure is not None:
        raise RuntimeError(f"{failure}; report: {report_path}")
    return report_path, report, digest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record synchronized FAST-LIO2 in shadow mode",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "system.yaml",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PROJECT_ROOT / "data" / "recordings" / "lio",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to record; zero records until Ctrl+C",
    )
    parser.add_argument(
        "--visual-assist",
        action="store_true",
        help="Open the guided carry-test display and stop after its sequence",
    )
    parser.add_argument("--visual-host", default="127.0.0.1")
    parser.add_argument("--visual-port", type=int, default=8766)
    parser.add_argument(
        "--visual-guide",
        choices=("full", "yaw", "translation"),
        default="full",
        help="Guided motion sequence shown by the visual assist",
    )
    parser.add_argument(
        "--cube-odometry-shadow",
        action="store_true",
        help=(
            "send disarmed MAVLink2 ODOMETRY only after proving Cube "
            "ExternalNav fusion is unavailable"
        ),
    )
    parser.add_argument("--no-browser", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
        report_path, report, digest = run_shadow(
            config,
            args.config,
            output_root=args.output_root,
            duration_s=args.duration,
            visual_assist=args.visual_assist,
            visual_host=args.visual_host,
            visual_port=args.visual_port,
            open_browser=not args.no_browser,
            visual_guide=args.visual_guide,
            cube_odometry_shadow=args.cube_odometry_shadow,
        )
        print(
            json.dumps(
                {
                    "result": report["result"],
                    "report": str(report_path),
                    "sha256": digest,
                    "pose_sent_to_cube": bool(
                        args.cube_odometry_shadow
                        and report.get("packets_sent", 0) > 0
                    ),
                    "cube_external_nav_fusion_enabled": False,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0 if report["result"] == "pass" else 1
    except (ConfigError, OSError, RuntimeError, ValueError) as exc:
        print(f"LIO shadow error: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
