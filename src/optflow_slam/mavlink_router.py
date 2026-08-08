"""Own the Cube UART and relay raw MAVLink bytes to one localhost client."""

from __future__ import annotations

import argparse
from collections import deque
import json
import os
from pathlib import Path
import select
import signal
import socket
import threading
import time

from .config import ConfigError, load_config
from .paths import PROJECT_ROOT


DEFAULT_CONFIG = PROJECT_ROOT / "config" / "system.yaml"
MAX_CHUNK_BYTES = 8192
STATUS_PERIOD_S = 1.0
SERIAL_INTERFRAME_GAP_S = 0.002
MAX_TX_QUEUE_DATAGRAMS = 512


def _status_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _write_status(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="ascii",
    )
    temporary.replace(path)


def run_router(config_path: Path, stop_event: threading.Event) -> None:
    import serial

    config = load_config(config_path)
    flight_controller = config.flight_controller
    router = flight_controller.router
    if not router.enabled:
        raise ConfigError("Cube UART router is disabled")

    bind_address = (router.bind_host, router.bind_port)
    client_address = (router.client_host, router.client_port)
    status_path = _status_path(router.status_file)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1_048_576)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1_048_576)
    udp.bind(bind_address)
    udp.setblocking(False)
    port = serial.Serial(
        router.serial_endpoint,
        flight_controller.baud,
        timeout=0,
        write_timeout=1.0,
        dsrdtr=False,
        rtscts=False,
        xonxoff=False,
        exclusive=True,
    )
    started_monotonic_ns = time.monotonic_ns()
    counters = {
        "serial_rx_bytes": 0,
        "serial_rx_chunks": 0,
        "serial_tx_bytes": 0,
        "serial_tx_datagrams": 0,
        "serial_tx_max_queue_depth": 0,
        "serial_tx_max_queue_age_ms": 0.0,
        "rejected_udp_datagrams": 0,
    }
    tx_queue: deque[tuple[bytes, int]] = deque()
    next_serial_tx_s = time.monotonic()
    last_serial_rx_ns = None
    last_udp_rx_ns = None
    next_status_s = 0.0
    print(
        "Cube MAVLink router ready: "
        f"{router.serial_endpoint}@{flight_controller.baud} <-> "
        f"udp://{router.client_host}:{router.client_port}",
        flush=True,
    )
    try:
        while not stop_event.is_set():
            now_s = time.monotonic()
            select_timeout_s = 0.1
            if tx_queue:
                select_timeout_s = min(
                    select_timeout_s,
                    max(0.0, next_serial_tx_s - now_s),
                )
            readable, _, _ = select.select(
                [port.fileno(), udp.fileno()], [], [], select_timeout_s
            )
            if port.fileno() in readable:
                waiting = max(1, min(int(port.in_waiting), MAX_CHUNK_BYTES))
                payload = port.read(waiting)
                if payload:
                    udp.sendto(payload, client_address)
                    counters["serial_rx_bytes"] += len(payload)
                    counters["serial_rx_chunks"] += 1
                    last_serial_rx_ns = time.monotonic_ns()
            if udp.fileno() in readable:
                while True:
                    try:
                        payload, source = udp.recvfrom(MAX_CHUNK_BYTES)
                    except BlockingIOError:
                        break
                    if source[0] != router.client_host:
                        counters["rejected_udp_datagrams"] += 1
                        continue
                    if not payload:
                        continue
                    if len(tx_queue) >= MAX_TX_QUEUE_DATAGRAMS:
                        raise OSError("Cube UART transmit queue overflow")
                    tx_queue.append((payload, time.monotonic_ns()))
                    counters["serial_tx_max_queue_depth"] = max(
                        counters["serial_tx_max_queue_depth"],
                        len(tx_queue),
                    )
                    last_udp_rx_ns = time.monotonic_ns()
            now_s = time.monotonic()
            if tx_queue and now_s >= next_serial_tx_s:
                payload, queued_ns = tx_queue.popleft()
                written = port.write(payload)
                if written != len(payload):
                    raise OSError(
                        f"short Cube UART write: {written}/{len(payload)}"
                    )
                port.flush()
                counters["serial_tx_bytes"] += written
                counters["serial_tx_datagrams"] += 1
                counters["serial_tx_max_queue_age_ms"] = max(
                    counters["serial_tx_max_queue_age_ms"],
                    round((time.monotonic_ns() - queued_ns) / 1.0e6, 3),
                )
                next_serial_tx_s = time.monotonic() + SERIAL_INTERFRAME_GAP_S
            now_s = time.monotonic()
            if now_s >= next_status_s:
                now_ns = time.monotonic_ns()
                _write_status(
                    status_path,
                    {
                        "schema_version": 1,
                        "pid": os.getpid(),
                        "live": True,
                        "updated_unix_ns": time.time_ns(),
                        "uptime_s": round(
                            (now_ns - started_monotonic_ns) / 1.0e9, 3
                        ),
                        "serial_endpoint": router.serial_endpoint,
                        "baud": flight_controller.baud,
                        "bind": f"{router.bind_host}:{router.bind_port}",
                        "client": f"{router.client_host}:{router.client_port}",
                        "serial_rx_age_s": (
                            None
                            if last_serial_rx_ns is None
                            else round((now_ns - last_serial_rx_ns) / 1.0e9, 3)
                        ),
                        "udp_rx_age_s": (
                            None
                            if last_udp_rx_ns is None
                            else round((now_ns - last_udp_rx_ns) / 1.0e9, 3)
                        ),
                        "serial_tx_queue_depth": len(tx_queue),
                        "serial_tx_oldest_queue_age_ms": (
                            None
                            if not tx_queue
                            else round(
                                (now_ns - tx_queue[0][1]) / 1.0e6,
                                3,
                            )
                        ),
                        **counters,
                    },
                )
                next_status_s = now_s + STATUS_PERIOD_S
    finally:
        port.close()
        udp.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        run_router(args.config, stop_event)
        return 0
    except (ConfigError, OSError, ValueError) as exc:
        print(f"Cube MAVLink router error: {exc}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
