"""Serve a live Three.js optical-flow visualizer from Cube MAVLink data."""

from __future__ import annotations

import argparse
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

from .config import ConfigError, load_config
from .im10a import Im10aDecoder
from .paths import CONFIG_DIR, VISUALIZER_DIR


DEFAULT_STATIC_DIR = VISUALIZER_DIR / "dist"
DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"


class TelemetryStore:
    """Thread-safe latest-value store with per-stream freshness."""

    def __init__(
        self,
        source: str,
        cube_mount: dict[str, Any] | None = None,
    ) -> None:
        now = time.monotonic()
        self._lock = threading.Lock()
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
            },
            "flow": {
                "rate_x_rads": 0.0,
                "rate_y_rads": 0.0,
                "comp_x": 0.0,
                "comp_y": 0.0,
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
            "ros_imu": {
                "connected": False,
                "detail": "Waiting for IM10A",
                "transport": "serial-direct",
                "contract": "sensor_msgs/Imu",
                "frame_id": "im10a_link",
                "extrinsics_verified": False,
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
        }

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
        state["link"]["age_ms"] = age_ms(
            state["link"].pop("last_packet_monotonic")
        )
        for section in ("flow", "range", "attitude", "imu", "ros_imu"):
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


class MavlinkSource(threading.Thread):
    """Reconnectable, read-only telemetry source for the Cube UART."""

    def __init__(
        self,
        store: TelemetryStore,
        stop_event: threading.Event,
        endpoint: str,
        baud: int,
    ) -> None:
        super().__init__(name="cube-mavlink", daemon=True)
        self.store = store
        self.stop_event = stop_event
        self.endpoint = endpoint
        self.baud = baud
        self.target_system: int | None = None
        self.target_component: int | None = None
        self._last_highres_imu_monotonic: float | None = None

    def run(self) -> None:
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            self.store.set_link(False, f"pymavlink unavailable: {exc}")
            return

        message_intervals = {
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 50_000,
            mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW: 50_000,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 50_000,
            mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU: 20_000,
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU: 20_000,
        }
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
                    source_system=1,
                    source_component=191,
                )
                heartbeat = self._wait_for_heartbeat(master, mavutil)
                if heartbeat is None:
                    raise RuntimeError("Cube heartbeat timed out")

                master.target_system = heartbeat.get_srcSystem()
                master.target_component = heartbeat.get_srcComponent()
                self.target_system = master.target_system
                self.target_component = master.target_component
                self._handle_heartbeat(heartbeat, mavutil)
                for message_id, interval_us in message_intervals.items():
                    result = _set_message_interval(
                        master,
                        message_id,
                        interval_us,
                        wait_for_ack=True,
                    )
                    if result not in (
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    ):
                        self.store.set_link(
                            True,
                            f"Message {message_id} interval ACK: {result}",
                        )
                intervals_requested = True

                while not self.stop_event.is_set():
                    message = master.recv_match(blocking=True, timeout=0.5)
                    if message is None:
                        snapshot = self.store.snapshot()
                        age = snapshot["link"]["age_ms"]
                        if age is not None and age > 2000:
                            raise RuntimeError("MAVLink stream stale")
                        continue
                    self.store.mark_packet()
                    self._handle_message(message, mavutil)
            except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
                self.store.set_link(False, str(exc))
                self.stop_event.wait(2.0)
            finally:
                if master is not None:
                    if intervals_requested:
                        for message_id in message_intervals:
                            try:
                                _set_message_interval(master, message_id, -1)
                            except Exception:
                                pass
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
            ):
                return message
        return None

    def _handle_heartbeat(self, message, mavutil) -> None:
        armed = bool(
            message.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
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
                updated_monotonic=now,
            )
        elif message_type == "OPTICAL_FLOW":
            rate_x = getattr(message, "flow_rate_x", message.flow_x)
            rate_y = getattr(message, "flow_rate_y", message.flow_y)
            self.store.update(
                "flow",
                rate_x_rads=float(rate_x),
                rate_y_rads=float(rate_y),
                comp_x=float(message.flow_comp_m_x),
                comp_y=float(message.flow_comp_m_y),
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
                quaternion_frames = 0
                rate_started = time.monotonic()
                while not self.stop_event.is_set():
                    data = port.read(max(1, port.in_waiting))
                    if not data:
                        continue
                    for measurement in decoder.feed(data):
                        now = time.monotonic()
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
                            self.store.update(
                                "ros_imu",
                                **common,
                                gyro_x_rads=x,
                                gyro_y_rads=y,
                                gyro_z_rads=z,
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
                            quaternion_frames += 1
                            elapsed = now - rate_started
                            rate_hz = 0.0
                            if elapsed >= 1.0:
                                rate_hz = quaternion_frames / elapsed
                                quaternion_frames = 0
                                rate_started = now
                            values: dict[str, Any] = {
                                **common,
                                "quaternion_wxyz": list(measurement.values),
                            }
                            if rate_hz > 0:
                                values["sample_rate_hz"] = rate_hz
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
                rate_x_rads=flow_x,
                rate_y_rads=flow_y,
                comp_x=flow_x * 0.86,
                comp_y=flow_y * 0.86,
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
    store: TelemetryStore, static_dir: Path
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
            if self.path == "/healthz":
                self._send_json({"ok": True})
                return
            super().do_GET()

        def end_headers(self) -> None:
            if self.path.startswith("/api/"):
                self.send_header("Cache-Control", "no-store")
            super().end_headers()

        def log_message(self, format: str, *args: Any) -> None:
            if self.path not in ("/api/stream", "/api/snapshot"):
                super().log_message(format, *args)

        def _send_json(self, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(HTTPStatus.OK)
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
                    time.sleep(1.0 / 30.0)
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
    static_dir = args.static_dir.resolve()
    if not (static_dir / "index.html").exists():
        print(
            f"Visualizer build not found at {static_dir}. "
            "Run: npm --prefix visualizer run build"
        )
        return 2

    stop_event = threading.Event()
    mount = config.flight_controller.cube_mount
    store = TelemetryStore(
        "demo" if args.demo else "cube_uart",
        cube_mount={
            "x_m": mount.x_m,
            "y_m": mount.y_m,
            "z_m": mount.z_m,
            "yaw_ccw_deg": mount.yaw_ccw_deg,
            "ahrs_orientation": mount.ahrs_orientation,
            "ahrs_orientation_name": mount.ahrs_orientation_name,
        },
    )
    if args.demo:
        sources: list[threading.Thread] = [DemoSource(store, stop_event)]
    else:
        sources = [
            MavlinkSource(
                store,
                stop_event,
                endpoint=args.serial or config.flight_controller.endpoint,
                baud=args.baud or config.flight_controller.baud,
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
    for source in sources:
        source.start()

    handler = make_handler(store, static_dir)
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
