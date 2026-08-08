"""Minimal JT16-to-Cube obstacle-avoidance runtime.

This service deliberately excludes camera capture, LIO, SLAM, pose output,
trajectory recording, and companion-generated movement commands.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import threading
import time
from typing import Any, Callable

import numpy as np

from .config import ConfigError, ProjectConfig, load_config
from .mavlink_compat import install_pymavlink_instance_guard
from .mavlink_proximity import (
    horizontal_transport_distances_cm,
    send_horizontal_distance_sensor,
)
from .obstacles import (
    LidarObstacleExtractor,
    ObstacleScan,
    obstacle_alert_state,
)
from .paths import CONFIG_DIR, PROJECT_ROOT, RUNTIME_DIR
from .runtime_lock import (
    RuntimeLockError,
    RuntimeResourceLock,
    cube_mavlink_lock,
)


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"
DEFAULT_STATUS = RUNTIME_DIR / "obstacle_avoidance_status.json"
JT16_FRAME_HEADER = struct.Struct("<8sIIQQ")
JT16_FRAME_MAGIC = b"OFJT16P1"
JT16_FRAME_VERSION = 2
JT16_MAXIMUM_POINTS = 1_000_000
JT16_POINT_DTYPE = np.dtype(
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
STARTUP_RISING_TUNE = "MFT200L16CEG"
OBSTACLE_BEEP_TUNE = "MFT240L32G"
FACE_PACKET_GAP_NS = 12_000_000


def _project_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_status(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="ascii",
    )
    temporary.replace(path)


class PacedFaceScheduler:
    """Repeat only the newest fresh scan without serial packet bursts."""

    def __init__(self, *, rate_hz: float, stale_timeout_s: float) -> None:
        self._cycle_period_ns = round(1.0e9 / rate_hz)
        self._stale_timeout_ns = round(stale_timeout_s * 1.0e9)
        self._lock = threading.Lock()
        self._latest: ObstacleScan | None = None
        self._cycle: deque[tuple[ObstacleScan, int, int]] = deque()
        self._next_cycle_ns = 0
        self._next_packet_ns = 0

    def queue(self, scan: ObstacleScan) -> None:
        with self._lock:
            self._latest = scan

    def latest(self) -> ObstacleScan | None:
        with self._lock:
            return self._latest

    def next_packet(
        self, now_ns: int
    ) -> tuple[ObstacleScan, int, int] | None:
        if self._cycle and now_ns >= self._next_packet_ns:
            packet = self._cycle.popleft()
            self._next_packet_ns = now_ns + FACE_PACKET_GAP_NS
            return packet
        if self._cycle or now_ns < self._next_cycle_ns:
            return None

        self._next_cycle_ns = now_ns + self._cycle_period_ns
        scan = self.latest()
        if scan is None:
            return None
        age_ns = now_ns - scan.monotonic_ns
        if age_ns < 0 or age_ns > self._stale_timeout_ns:
            return None
        self._cycle.extend(
            (scan, orientation, distance_cm)
            for orientation, distance_cm in enumerate(
                horizontal_transport_distances_cm(scan)
            )
        )
        if not self._cycle:
            return None
        packet = self._cycle.popleft()
        self._next_packet_ns = now_ns + FACE_PACKET_GAP_NS
        return packet


class Jt16ObstacleSource(threading.Thread):
    """Decode JT16 point frames and emit CG-referenced horizontal sectors."""

    def __init__(
        self,
        config: ProjectConfig,
        stop_event: threading.Event,
        sink: Callable[[ObstacleScan], None],
    ) -> None:
        super().__init__(name="jt16-obstacle-source", daemon=True)
        self.config = config
        self.stop_event = stop_event
        self.sink = sink
        self.extractor = LidarObstacleExtractor(
            config.obstacle_avoidance,
            config.lidar,
        )
        self._lock = threading.Lock()
        self._started_ns = time.monotonic_ns()
        self._frames = 0
        self._points = 0
        self._latest_points = 0
        self._last_frame_ns: int | None = None
        self._valid_sectors = 0
        self._nearest_distance_m: float | None = None
        self.error: str | None = None

    def _read_exact(
        self,
        process: subprocess.Popen[bytes],
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

    def _command(self) -> list[str]:
        lidar = self.config.lidar
        return [
            str(_project_path(lidar.bridge_binary)),
            "--device",
            lidar.symlink,
            "--baud",
            str(lidar.baud),
            "--correction",
            str(_project_path(lidar.correction_file)),
            "--startup-timeout",
            "5",
        ]

    def run(self) -> None:
        process: subprocess.Popen[bytes] | None = None
        log_output = None
        try:
            command = self._command()
            bridge = Path(command[0])
            correction = Path(command[6])
            if not bridge.is_file() or not os.access(bridge, os.X_OK):
                raise OSError(
                    "JT16 bridge is missing; run ./optflow build-jt16"
                )
            if not correction.is_file():
                raise OSError(f"JT16 correction is missing: {correction}")
            if not Path(self.config.lidar.symlink).exists():
                raise OSError(
                    f"JT16 device is missing: {self.config.lidar.symlink}"
                )
            RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
            log_output = (RUNTIME_DIR / "jt16_oa_bridge.log").open("ab")
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=log_output,
                bufsize=0,
            )
            while not self.stop_event.is_set():
                header = self._read_exact(process, JT16_FRAME_HEADER.size)
                if header is None:
                    break
                magic, version, count, frame_ns, _index = (
                    JT16_FRAME_HEADER.unpack(header)
                )
                if magic != JT16_FRAME_MAGIC or version != JT16_FRAME_VERSION:
                    raise ValueError("JT16 bridge frame header is invalid")
                if count <= 0 or count > JT16_MAXIMUM_POINTS:
                    raise ValueError("JT16 point count is outside limits")
                payload = self._read_exact(
                    process,
                    count * JT16_POINT_DTYPE.itemsize,
                )
                if payload is None:
                    break
                records = np.frombuffer(payload, dtype=JT16_POINT_DTYPE)
                points = np.column_stack(
                    (records["x"], records["y"], records["z"])
                )
                scan = self.extractor.extract(
                    points,
                    monotonic_ns=frame_ns,
                )
                self.sink(scan)
                with self._lock:
                    self._frames += 1
                    self._points += count
                    self._latest_points = count
                    self._last_frame_ns = time.monotonic_ns()
                    self._valid_sectors = scan.valid_sector_count
                    self._nearest_distance_m = scan.nearest_distance_m
            if (
                not self.stop_event.is_set()
                and process.poll() is not None
            ):
                raise RuntimeError(
                    f"JT16 bridge exited with code {process.returncode}"
                )
        except (OSError, RuntimeError, ValueError) as exc:
            self.error = str(exc)
            self.stop_event.set()
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
            if process is not None and process.stdout is not None:
                process.stdout.close()
            if log_output is not None:
                log_output.close()

    def snapshot(self, now_ns: int) -> dict[str, Any]:
        with self._lock:
            frames = self._frames
            elapsed_s = max(0.001, (now_ns - self._started_ns) / 1.0e9)
            last_frame_age_ms = (
                None
                if self._last_frame_ns is None
                else max(0, round((now_ns - self._last_frame_ns) / 1.0e6))
            )
            return {
                "connected": self._last_frame_ns is not None,
                "frames": frames,
                "frame_rate_hz": round(frames / elapsed_s, 3),
                "points": self._points,
                "latest_points": self._latest_points,
                "last_frame_age_ms": last_frame_age_ms,
                "valid_sectors": self._valid_sectors,
                "nearest_distance_m": self._nearest_distance_m,
                "error": self.error,
            }


class CubeProximityLink(threading.Thread):
    """Own the Cube UART and send only proximity, alerts, and a heartbeat."""

    def __init__(
        self,
        config: ProjectConfig,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name="cube-proximity-link", daemon=True)
        self.config = config
        self.stop_event = stop_event
        obstacle = config.obstacle_avoidance
        self.scheduler = PacedFaceScheduler(
            rate_hz=obstacle.target_rate_hz,
            stale_timeout_s=obstacle.source_stale_timeout_s,
        )
        self._lock = threading.Lock()
        self._connected = False
        self._last_heartbeat_ns: int | None = None
        self._armed = False
        self._mode: str | None = None
        self._rc_pwm: int | None = None
        self._rc_enabled = False
        self._packets_sent = 0
        self._cycles_started = 0
        self._last_packet_ns: int | None = None
        self._health: bool | None = None
        self._health_samples = 0
        self._healthy_samples = 0
        self._startup_tune_sent = False
        self._alert_beeps_sent = 0
        self._last_alert_beep_s = float("-inf")
        self._last_statustext: str | None = None
        self._prearm_messages: deque[str] = deque(maxlen=12)
        self._last_error: str | None = None
        self._reconnects = 0

    def queue_scan(self, scan: ObstacleScan) -> None:
        self.scheduler.queue(scan)

    def _set_connected(self, connected: bool, error: str | None = None) -> None:
        with self._lock:
            self._connected = connected
            if error is not None:
                self._last_error = error

    def _request_minimal_streams(
        self,
        connection: Any,
        mavutil: Any,
        target_system: int,
        target_component: int,
    ) -> None:
        for message_id, rate_hz in (
            (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 5.0),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2.0),
        ):
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

    def _handle_message(self, message: Any, mavutil: Any) -> None:
        message_type = message.get_type()
        now_ns = time.monotonic_ns()
        if message_type == "HEARTBEAT":
            if (
                message.autopilot
                != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                or message.get_srcComponent()
                != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            ):
                return
            with self._lock:
                self._last_heartbeat_ns = now_ns
                self._armed = bool(
                    int(message.base_mode)
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._mode = mavutil.mode_string_v10(message)
        elif message_type == "RC_CHANNELS":
            channel = self.config.obstacle_avoidance.rc_toggle.channel
            raw_pwm = getattr(message, f"chan{channel}_raw", None)
            pwm = (
                int(raw_pwm)
                if raw_pwm is not None and 0 < int(raw_pwm) < 65535
                else None
            )
            settings = self.config.obstacle_avoidance.rc_toggle
            with self._lock:
                self._rc_pwm = pwm
                if pwm is not None and pwm >= settings.engage_pwm:
                    self._rc_enabled = True
                elif pwm is None or pwm <= settings.disengage_pwm:
                    self._rc_enabled = False
        elif message_type == "SYS_STATUS":
            sensor_bit = getattr(
                mavutil.mavlink,
                "MAV_SYS_STATUS_SENSOR_PROXIMITY",
                0,
            )
            healthy = bool(
                sensor_bit
                and int(message.onboard_control_sensors_present) & sensor_bit
                and int(message.onboard_control_sensors_enabled) & sensor_bit
                and int(message.onboard_control_sensors_health) & sensor_bit
            )
            with self._lock:
                self._health = healthy
                self._health_samples += 1
                self._healthy_samples += int(healthy)
        elif message_type == "STATUSTEXT":
            text = str(getattr(message, "text", "")).strip("\x00 ")
            if not text:
                return
            with self._lock:
                self._last_statustext = text
                if "PreArm" in text or "PRX" in text:
                    self._prearm_messages.append(text)

    def _send_companion_heartbeat(self, connection: Any, mavutil: Any) -> None:
        connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _send_next_proximity(self, connection: Any, mavutil: Any) -> None:
        now_ns = time.monotonic_ns()
        packet = self.scheduler.next_packet(now_ns)
        if packet is None:
            return
        scan, orientation, distance_cm = packet
        send_horizontal_distance_sensor(
            connection.mav,
            mavutil.mavlink,
            scan,
            orientation,
            distance_cm,
        )
        with self._lock:
            self._packets_sent += 1
            self._last_packet_ns = now_ns
            if orientation == 0:
                self._cycles_started += 1

    def _send_startup_tune(
        self,
        connection: Any,
        target_system: int,
        target_component: int,
    ) -> None:
        with self._lock:
            if self._startup_tune_sent:
                return
        connection.mav.play_tune_send(
            target_system,
            target_component,
            STARTUP_RISING_TUNE.encode("ascii"),
        )
        with self._lock:
            self._startup_tune_sent = True

    def _maybe_send_alert(
        self,
        connection: Any,
        target_system: int,
        target_component: int,
    ) -> None:
        scan = self.scheduler.latest()
        settings = self.config.obstacle_avoidance
        now_ns = time.monotonic_ns()
        fresh = bool(
            scan is not None
            and 0
            <= now_ns - scan.monotonic_ns
            <= round(settings.source_stale_timeout_s * 1.0e9)
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
        with self._lock:
            audible = bool(
                fresh
                and settings.alerts.enabled
                and self._rc_enabled
                and (self._armed or not settings.alerts.only_when_armed)
            )
        if not audible or alert.beep_rate_hz <= 0.0:
            return
        now_s = now_ns / 1.0e9
        if now_s - self._last_alert_beep_s < 1.0 / alert.beep_rate_hz:
            return
        connection.mav.play_tune_send(
            target_system,
            target_component,
            OBSTACLE_BEEP_TUNE.encode("ascii"),
        )
        self._last_alert_beep_s = now_s
        with self._lock:
            self._alert_beeps_sent += 1

    def run(self) -> None:
        try:
            from pymavlink import mavutil

            install_pymavlink_instance_guard(mavutil)
        except ImportError as exc:
            self._set_connected(False, f"pymavlink unavailable: {exc}")
            self.stop_event.set()
            return

        fc = self.config.flight_controller
        while not self.stop_event.is_set():
            connection = None
            try:
                connection = mavutil.mavlink_connection(
                    fc.endpoint,
                    baud=fc.baud,
                    source_system=fc.companion_system_id,
                    source_component=getattr(
                        mavutil.mavlink,
                        "MAV_COMP_ID_OBSTACLE_AVOIDANCE",
                        196,
                    ),
                    robust_parsing=True,
                )
                deadline = time.monotonic() + fc.heartbeat_timeout_s
                heartbeat = None
                while (
                    heartbeat is None
                    and time.monotonic() < deadline
                    and not self.stop_event.is_set()
                ):
                    candidate = connection.recv_match(
                        type="HEARTBEAT",
                        blocking=True,
                        timeout=0.25,
                    )
                    if (
                        candidate is not None
                        and candidate.autopilot
                        == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                        and candidate.get_srcComponent()
                        == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                    ):
                        heartbeat = candidate
                if heartbeat is None:
                    raise RuntimeError("Cube autopilot heartbeat timed out")
                target_system = heartbeat.get_srcSystem()
                target_component = heartbeat.get_srcComponent()
                self._handle_message(heartbeat, mavutil)
                self._set_connected(True)
                self._request_minimal_streams(
                    connection,
                    mavutil,
                    target_system,
                    target_component,
                )
                self._send_startup_tune(
                    connection,
                    target_system,
                    target_component,
                )
                next_heartbeat_ns = 0
                while not self.stop_event.is_set():
                    now_ns = time.monotonic_ns()
                    if now_ns >= next_heartbeat_ns:
                        self._send_companion_heartbeat(connection, mavutil)
                        next_heartbeat_ns = now_ns + 1_000_000_000
                    self._send_next_proximity(connection, mavutil)
                    self._maybe_send_alert(
                        connection,
                        target_system,
                        target_component,
                    )
                    message = connection.recv_match(
                        type=[
                            "HEARTBEAT",
                            "RC_CHANNELS",
                            "SYS_STATUS",
                            "STATUSTEXT",
                        ],
                        blocking=True,
                        timeout=0.01,
                    )
                    if message is not None:
                        self._handle_message(message, mavutil)
                    with self._lock:
                        last_heartbeat_ns = self._last_heartbeat_ns
                    if (
                        last_heartbeat_ns is None
                        or now_ns - last_heartbeat_ns
                        > round(fc.heartbeat_timeout_s * 1.0e9)
                    ):
                        raise RuntimeError("Cube heartbeat stream became stale")
            except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
                with self._lock:
                    self._reconnects += 1
                self._set_connected(False, str(exc))
                self.stop_event.wait(1.0)
            finally:
                self._set_connected(False)
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass

    def snapshot(self, now_ns: int) -> dict[str, Any]:
        scan = self.scheduler.latest()
        scan_age_ms = (
            None
            if scan is None
            else max(0, round((now_ns - scan.monotonic_ns) / 1.0e6))
        )
        with self._lock:
            heartbeat_age_ms = (
                None
                if self._last_heartbeat_ns is None
                else max(
                    0,
                    round((now_ns - self._last_heartbeat_ns) / 1.0e6),
                )
            )
            last_packet_age_ms = (
                None
                if self._last_packet_ns is None
                else max(
                    0,
                    round((now_ns - self._last_packet_ns) / 1.0e6),
                )
            )
            return {
                "connected": self._connected,
                "heartbeat_age_ms": heartbeat_age_ms,
                "armed": self._armed,
                "mode": self._mode,
                "rc_channel": self.config.obstacle_avoidance.rc_toggle.channel,
                "rc_pwm": self._rc_pwm,
                "rc_avoidance_enabled": self._rc_enabled,
                "proximity_packets_sent": self._packets_sent,
                "proximity_cycles_started": self._cycles_started,
                "last_packet_age_ms": last_packet_age_ms,
                "latest_scan_age_ms": scan_age_ms,
                "proximity_healthy": self._health,
                "health_samples": self._health_samples,
                "healthy_samples": self._healthy_samples,
                "startup_tune_sent": self._startup_tune_sent,
                "alert_beeps_sent": self._alert_beeps_sent,
                "last_statustext": self._last_statustext,
                "prearm_messages": list(self._prearm_messages),
                "reconnects": self._reconnects,
                "last_error": self._last_error,
            }


def _validate(config: ProjectConfig) -> None:
    obstacle = config.obstacle_avoidance
    fc = config.flight_controller
    if obstacle.stage != "active" or not obstacle.mavlink_output_enabled:
        raise ConfigError("active MAVLink obstacle output is not enabled")
    if not obstacle.lidar_enabled:
        raise ConfigError("JT16 obstacle input is disabled")
    if obstacle.depth_camera_enabled:
        raise ConfigError("OA-only runtime requires depth camera input disabled")
    if not config.lidar.correction_verified:
        raise ConfigError("JT16 correction is not verified")
    if not config.calibration.lidar_to_body_extrinsics_verified:
        raise ConfigError("JT16 body extrinsics are not verified")
    if fc.router.enabled:
        raise ConfigError("OA-only runtime requires the UART router disabled")
    if not fc.endpoint.startswith("/dev/"):
        raise ConfigError("OA-only Cube endpoint must be a direct serial device")


def _status_payload(
    *,
    running: bool,
    started_at: str,
    config: ProjectConfig,
    lidar: Jt16ObstacleSource,
    cube: CubeProximityLink,
    reason: str | None = None,
) -> dict[str, Any]:
    now_ns = time.monotonic_ns()
    lidar_state = lidar.snapshot(now_ns)
    cube_state = cube.snapshot(now_ns)
    maximum_age_ms = round(
        config.obstacle_avoidance.source_stale_timeout_s * 1000
    )
    fresh_lidar = bool(
        lidar_state["last_frame_age_ms"] is not None
        and lidar_state["last_frame_age_ms"] <= maximum_age_ms
    )
    ready = bool(
        running
        and lidar_state["connected"]
        and fresh_lidar
        and cube_state["connected"]
        and cube_state["proximity_healthy"] is True
    )
    return {
        "schema_version": 1,
        "updated_at": _utc_now(),
        "started_at": started_at,
        "running": running,
        "mode": "obstacle_avoidance_only",
        "ready": ready,
        "reason": reason,
        "active_components": ["jt16", "cube_proximity", "rc7_alerts"],
        "disabled_components": [
            "d415",
            "external_imu",
            "fast_lio",
            "slam",
            "trajectory",
            "navigation",
            "companion_movement_commands",
        ],
        "companion_movement_commands_sent": False,
        "lidar": lidar_state,
        "cube": cube_state,
    }


def run_service(config_path: Path, status_path: Path) -> int:
    config = load_config(config_path)
    _validate(config)
    stop_event = threading.Event()
    previous_handlers: dict[int, Any] = {}

    def stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, stop)

    started_at = _utc_now()
    cube = CubeProximityLink(config, stop_event)
    lidar = Jt16ObstacleSource(config, stop_event, cube.queue_scan)
    cube_lock = cube_mavlink_lock("JT16 obstacle avoidance only")
    lidar_lock = RuntimeResourceLock(
        "jt16",
        purpose="JT16 obstacle avoidance only",
    )
    reason: str | None = None
    cube_started = False
    lidar_started = False
    try:
        cube_lock.acquire()
        lidar_lock.acquire()
        cube.start()
        cube_started = True
        lidar.start()
        lidar_started = True
        print(
            "OA-only runtime started: JT16 -> paced Cube proximity; "
            "SLAM and movement output disabled",
            flush=True,
        )
        while not stop_event.wait(0.5):
            _write_status(
                status_path,
                _status_payload(
                    running=True,
                    started_at=started_at,
                    config=config,
                    lidar=lidar,
                    cube=cube,
                ),
            )
        reason = lidar.error
    finally:
        stop_event.set()
        if lidar_started:
            lidar.join(timeout=8.0)
        if cube_started:
            cube.join(timeout=8.0)
        if (
            (lidar_started and lidar.is_alive())
            or (cube_started and cube.is_alive())
        ):
            reason = reason or "runtime thread did not stop cleanly"
        _write_status(
            status_path,
            _status_payload(
                running=False,
                started_at=started_at,
                config=config,
                lidar=lidar,
                cube=cube,
                reason=reason or "service stopped",
            ),
        )
        lidar_lock.release()
        cube_lock.release()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    if reason:
        print(f"OA-only runtime stopped: {reason}", flush=True)
        return 2
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        return run_service(args.config, args.status_file)
    except (ConfigError, OSError, RuntimeLockError, ValueError) as exc:
        print(f"OA-only service failed: {exc}", flush=True)
        return 2


def status_main() -> int:
    parser = argparse.ArgumentParser(
        description="Show the OA-only service and Cube proximity state"
    )
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS)
    args = parser.parse_args()
    try:
        payload = json.loads(args.status_file.read_text(encoding="ascii"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"Obstacle status unavailable: {exc}")
        return 2
    lidar = payload.get("lidar", {})
    cube = payload.get("cube", {})
    print("MODE=OBSTACLE_AVOIDANCE_ONLY")
    print(f"RUNNING={str(bool(payload.get('running'))).lower()}")
    print(f"READY={str(bool(payload.get('ready'))).lower()}")
    print(f"JT16_FRAMES={lidar.get('frames')}")
    print(f"JT16_LAST_FRAME_AGE_MS={lidar.get('last_frame_age_ms')}")
    print(f"CUBE_LINK={str(bool(cube.get('connected'))).lower()}")
    print(f"CUBE_ARMED={str(bool(cube.get('armed'))).lower()}")
    print(f"CUBE_MODE={cube.get('mode')}")
    print(f"RC7_PWM={cube.get('rc_pwm')}")
    print(
        "RC7_AVOIDANCE_ENABLED="
        f"{str(bool(cube.get('rc_avoidance_enabled'))).lower()}"
    )
    print(f"PRX_HEALTH={cube.get('proximity_healthy')}")
    print(f"PRX_PACKETS_SENT={cube.get('proximity_packets_sent')}")
    print(f"LAST_STATUSTEXT={cube.get('last_statustext')}")
    for message in cube.get("prearm_messages", []):
        print(f"PREARM_MESSAGE={message}")
    if payload.get("reason"):
        print(f"REASON={payload['reason']}")
    return 0 if payload.get("running") else 2


if __name__ == "__main__":
    raise SystemExit(main())
