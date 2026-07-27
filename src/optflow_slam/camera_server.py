"""Serve the RealSense forward RGB stream over the local network."""

from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import signal
import socket
import threading
import time
from typing import Any
from urllib.parse import urlsplit
import webbrowser

from .config import ConfigError, DepthCameraConfig, load_config
from .paths import CONFIG_DIR


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"
MJPEG_BOUNDARY = "rgbframe"

CAMERA_PAGE = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Forward RGB Camera</title>
    <style>
      :root {
        color-scheme: dark;
        font-family: Inter, ui-sans-serif, system-ui, sans-serif;
        --bg: #0d0f0e;
        --surface: rgba(15, 18, 16, 0.88);
        --line: #343a36;
        --text: #f1f4f2;
        --muted: #9da49f;
        --live: #8de06f;
        --stale: #f0b44b;
        --offline: #ef6258;
      }
      * { box-sizing: border-box; }
      html, body {
        width: 100%;
        height: 100%;
        margin: 0;
        overflow: hidden;
        background: var(--bg);
        color: var(--text);
      }
      main {
        position: fixed;
        inset: 0;
        overflow: hidden;
      }
      #feed {
        position: absolute;
        inset: 0;
        width: 100%;
        height: 100%;
        min-width: 0;
        min-height: 0;
        object-fit: contain;
        background: #050605;
      }
      header, footer {
        position: fixed;
        z-index: 2;
        left: 0;
        right: 0;
        min-height: 54px;
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 18px;
        padding: 0 18px;
        border-color: var(--line);
        background: var(--surface);
        backdrop-filter: blur(12px);
      }
      header {
        top: 0;
        border-bottom: 1px solid var(--line);
      }
      footer {
        bottom: 0;
        border-top: 1px solid var(--line);
      }
      .identity {
        min-width: 0;
        overflow: hidden;
        font-size: 15px;
        font-weight: 760;
        text-overflow: ellipsis;
        white-space: nowrap;
      }
      .status, .metrics {
        display: flex;
        align-items: center;
        gap: 12px;
        min-width: 0;
        color: var(--muted);
        font-size: 11px;
        font-variant-numeric: tabular-nums;
      }
      .status-dot {
        width: 8px;
        height: 8px;
        flex: 0 0 auto;
        border-radius: 50%;
        background: var(--offline);
      }
      .status-dot.live { background: var(--live); }
      .status-dot.stale { background: var(--stale); }
      .metric {
        overflow: hidden;
        text-overflow: ellipsis;
        white-space: nowrap;
      }
      .metric strong {
        color: var(--text);
        font-weight: 650;
      }
      @media (max-width: 620px) {
        header, footer {
          min-height: 48px;
          padding: 0 11px;
        }
        .identity { font-size: 13px; }
        .metrics { gap: 8px; }
        #deviceMetric, #viewerMetric { display: none; }
      }
    </style>
  </head>
  <body>
    <main>
      <img id="feed" src="/stream.mjpg" alt="Live forward RGB camera">
    </main>
    <header>
      <div id="identity" class="identity">FORWARD RGB</div>
      <div class="status">
        <span id="statusDot" class="status-dot"></span>
        <span id="statusText">Connecting</span>
      </div>
    </header>
    <footer class="metrics">
      <span id="deviceMetric" class="metric">SERIAL <strong>--</strong></span>
      <span class="metric">FRAME <strong id="frameMetric">--</strong></span>
      <span class="metric">RATE <strong id="fpsMetric">--</strong></span>
      <span class="metric">AGE <strong id="ageMetric">--</strong></span>
      <span id="viewerMetric" class="metric">VIEWERS <strong>--</strong></span>
    </footer>
    <script>
      const fields = {
        identity: document.querySelector("#identity"),
        statusDot: document.querySelector("#statusDot"),
        statusText: document.querySelector("#statusText"),
        device: document.querySelector("#deviceMetric strong"),
        frame: document.querySelector("#frameMetric"),
        fps: document.querySelector("#fpsMetric"),
        age: document.querySelector("#ageMetric"),
        viewers: document.querySelector("#viewerMetric strong")
      };
      async function updateStatus() {
        try {
          const response = await fetch("/api/status", { cache: "no-store" });
          const data = await response.json();
          const fresh = data.connected && data.frame_age_ms !== null
            && data.frame_age_ms < 1500;
          fields.identity.textContent = data.model || "FORWARD RGB";
          fields.statusDot.className = fresh
            ? "status-dot live"
            : data.connected ? "status-dot stale" : "status-dot";
          fields.statusText.textContent = fresh
            ? "Live" : data.connected ? "Stale" : "Offline";
          fields.device.textContent = data.serial || "--";
          fields.frame.textContent = `${data.width || "--"} x ${data.height || "--"}`;
          fields.fps.textContent = `${(data.measured_fps || 0).toFixed(1)} fps`;
          fields.age.textContent = data.frame_age_ms === null
            ? "--" : `${data.frame_age_ms} ms`;
          fields.viewers.textContent = data.viewers;
        } catch {
          fields.statusDot.className = "status-dot";
          fields.statusText.textContent = "Disconnected";
        }
      }
      updateStatus();
      setInterval(updateStatus, 1000);
    </script>
  </body>
</html>
""".encode("utf-8")


class CameraStore:
    """Thread-safe latest-frame and camera-health store."""

    def __init__(self, model: str, serial: str | None) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._frame_monotonic: float | None = None
        self._timestamps: deque[float] = deque(maxlen=90)
        self._state: dict[str, Any] = {
            "connected": False,
            "detail": "Waiting for camera",
            "model": model,
            "serial": serial,
            "firmware": None,
            "usb_type": None,
            "width": 0,
            "height": 0,
            "sequence": 0,
            "viewers": 0,
        }

    def set_state(self, **values: Any) -> None:
        with self._condition:
            self._state.update(values)
            self._condition.notify_all()

    def publish(self, jpeg: bytes, width: int, height: int) -> None:
        now = time.monotonic()
        with self._condition:
            self._jpeg = jpeg
            self._frame_monotonic = now
            self._timestamps.append(now)
            self._state.update(
                connected=True,
                detail="Receiving RGB frames",
                width=width,
                height=height,
                sequence=self._state["sequence"] + 1,
            )
            self._condition.notify_all()

    def wait_for_frame(
        self, after_sequence: int, timeout: float
    ) -> tuple[bytes | None, int]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._state["sequence"] > after_sequence,
                timeout=timeout,
            )
            sequence = int(self._state["sequence"])
            if sequence <= after_sequence:
                return None, sequence
            return self._jpeg, sequence

    def latest_frame(self) -> bytes | None:
        with self._condition:
            return self._jpeg

    def add_viewer(self, amount: int) -> None:
        with self._condition:
            self._state["viewers"] = max(
                0, int(self._state["viewers"]) + amount
            )

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._condition:
            state = deepcopy(self._state)
            frame_time = self._frame_monotonic
            timestamps = tuple(self._timestamps)

        state["frame_age_ms"] = (
            None
            if frame_time is None
            else max(0, round((now - frame_time) * 1000))
        )
        state["measured_fps"] = (
            0.0
            if len(timestamps) < 2 or timestamps[-1] <= timestamps[0]
            else round(
                (len(timestamps) - 1) / (timestamps[-1] - timestamps[0]),
                2,
            )
        )
        return state


class RealSenseColorSource(threading.Thread):
    """Reconnectable RGB-only RealSense capture source."""

    def __init__(
        self,
        store: CameraStore,
        stop_event: threading.Event,
        camera: DepthCameraConfig,
    ) -> None:
        super().__init__(name="realsense-rgb", daemon=True)
        self.store = store
        self.stop_event = stop_event
        self.camera = camera

    def run(self) -> None:
        try:
            import numpy as np
            from PIL import Image
            import pyrealsense2 as rs
        except ImportError as exc:
            self.store.set_state(
                connected=False, detail=f"Camera dependency unavailable: {exc}"
            )
            return

        while not self.stop_event.is_set():
            pipeline = None
            try:
                self.store.set_state(
                    connected=False,
                    detail=f"Connecting to {self.camera.model}",
                )
                pipeline = rs.pipeline()
                stream_config = rs.config()
                if self.camera.serial:
                    stream_config.enable_device(self.camera.serial)
                stream_config.enable_stream(
                    rs.stream.color,
                    self.camera.width,
                    self.camera.height,
                    rs.format.rgb8,
                    self.camera.fps,
                )
                profile = pipeline.start(stream_config)
                device = profile.get_device()
                self.store.set_state(
                    connected=True,
                    detail="Camera connected; waiting for RGB frames",
                    model=device.get_info(rs.camera_info.name),
                    serial=device.get_info(rs.camera_info.serial_number),
                    firmware=device.get_info(rs.camera_info.firmware_version),
                    usb_type=device.get_info(
                        rs.camera_info.usb_type_descriptor
                    ),
                )

                while not self.stop_event.is_set():
                    frames = pipeline.wait_for_frames(timeout_ms=2000)
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue
                    array = np.asanyarray(color_frame.get_data())
                    image = Image.fromarray(array)
                    output = io.BytesIO()
                    image.save(
                        output,
                        format="JPEG",
                        quality=self.camera.jpeg_quality,
                        optimize=False,
                        subsampling=2,
                    )
                    self.store.publish(
                        output.getvalue(),
                        color_frame.get_width(),
                        color_frame.get_height(),
                    )
            except (OSError, RuntimeError, ValueError) as exc:
                self.store.set_state(
                    connected=False, detail=f"Camera stream error: {exc}"
                )
                self.stop_event.wait(1.5)
            finally:
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except RuntimeError:
                        pass


def make_handler(
    store: CameraStore, stop_event: threading.Event
) -> type[SimpleHTTPRequestHandler]:
    class CameraHandler(SimpleHTTPRequestHandler):
        server_version = "OptFlowRGB/0.1"

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                self._send_bytes(
                    CAMERA_PAGE, "text/html; charset=utf-8"
                )
                return
            if path == "/api/status":
                self._send_json(store.snapshot())
                return
            if path == "/healthz":
                status = store.snapshot()
                fresh = (
                    status["connected"]
                    and status["frame_age_ms"] is not None
                    and status["frame_age_ms"] < 2000
                )
                self._send_json(
                    {"ok": fresh, "camera": status},
                    HTTPStatus.OK
                    if fresh
                    else HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            if path == "/snapshot.jpg":
                frame = store.latest_frame()
                if frame is None:
                    self.send_error(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        "No RGB frame available",
                    )
                    return
                self._send_bytes(
                    frame,
                    "image/jpeg",
                    extra_headers={
                        "Content-Disposition": (
                            'inline; filename="forward-rgb.jpg"'
                        )
                    },
                )
                return
            if path == "/stream.mjpg":
                self._send_stream()
                return
            if path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def end_headers(self) -> None:
            self.send_header(
                "Cache-Control",
                "no-store, no-cache, must-revalidate, max-age=0",
            )
            self.send_header("Pragma", "no-cache")
            super().end_headers()

        def log_message(self, format: str, *args: Any) -> None:
            if urlsplit(self.path).path not in (
                "/stream.mjpg",
                "/api/status",
            ):
                super().log_message(format, *args)

        def _send_json(
            self,
            payload: dict[str, Any],
            status: HTTPStatus = HTTPStatus.OK,
        ) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(
            self,
            body: bytes,
            content_type: str,
            *,
            extra_headers: dict[str, str] | None = None,
        ) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            for key, value in (extra_headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def _send_stream(self) -> None:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
            )
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}",
            )
            self.send_header("Connection", "close")
            self.end_headers()
            sequence = -1
            store.add_viewer(1)
            try:
                while not stop_event.is_set():
                    frame, sequence = store.wait_for_frame(
                        sequence, timeout=1.0
                    )
                    if frame is None:
                        continue
                    header = (
                        f"--{MJPEG_BOUNDARY}\r\n"
                        "Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(frame)}\r\n\r\n"
                    ).encode("ascii")
                    self.wfile.write(header)
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except (
                BrokenPipeError,
                ConnectionResetError,
                ConnectionAbortedError,
            ):
                return
            finally:
                store.add_viewer(-1)

    return CameraHandler


class CameraHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 16


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--serial")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fps", type=int)
    parser.add_argument("--jpeg-quality", type=int)
    parser.add_argument("--no-browser", action="store_true")
    return parser


def camera_from_args(
    configured: DepthCameraConfig, args: argparse.Namespace
) -> DepthCameraConfig:
    camera = DepthCameraConfig(
        model=configured.model,
        backend=configured.backend,
        mounting=configured.mounting,
        serial=(
            args.serial if args.serial is not None else configured.serial
        ),
        width=args.width if args.width is not None else configured.width,
        height=(
            args.height if args.height is not None else configured.height
        ),
        fps=args.fps if args.fps is not None else configured.fps,
        stream_host=(
            args.host if args.host is not None else configured.stream_host
        ),
        stream_port=(
            args.port if args.port is not None else configured.stream_port
        ),
        jpeg_quality=(
            args.jpeg_quality
            if args.jpeg_quality is not None
            else configured.jpeg_quality
        ),
    )
    if camera.width <= 0 or camera.height <= 0 or camera.fps <= 0:
        raise ConfigError("camera width, height, and fps must be positive")
    if not camera.stream_host:
        raise ConfigError("camera stream_host must not be empty")
    if not 1 <= camera.stream_port <= 65535:
        raise ConfigError("camera stream_port must be between 1 and 65535")
    if not 1 <= camera.jpeg_quality <= 100:
        raise ConfigError("camera jpeg_quality must be between 1 and 100")
    return camera


def main() -> int:
    args = build_parser().parse_args()
    try:
        project_config = load_config(args.config)
        camera = camera_from_args(project_config.depth_camera, args)
    except (OSError, ConfigError) as exc:
        print(f"Camera configuration error: {exc}")
        return 2
    if camera.backend.lower() != "realsense":
        print(f"Unsupported camera backend: {camera.backend}")
        return 2

    stop_event = threading.Event()
    store = CameraStore(camera.model, camera.serial)
    source = RealSenseColorSource(store, stop_event, camera)
    source.start()

    server = CameraHTTPServer(
        (camera.stream_host, camera.stream_port),
        make_handler(store, stop_event),
    )

    def stop_server(_signum=None, _frame=None) -> None:
        stop_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)

    url_host = (
        "127.0.0.1"
        if camera.stream_host == "0.0.0.0"
        else camera.stream_host
    )
    url = f"http://{url_host}:{camera.stream_port}"
    print(f"Forward RGB camera stream running at {url}")
    print(
        f"Requested {camera.model} serial={camera.serial or 'any'} "
        f"{camera.width}x{camera.height}@{camera.fps}"
    )
    print("This service does not communicate with or command the flight controller.")
    if not args.no_browser:
        threading.Timer(0.5, partial(webbrowser.open, url)).start()

    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        stop_event.set()
        server.server_close()
        source.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
